/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0

 TWAI and TWIA_HAL have been modified to work with LEVCAN by Viktor Sverdlyuk.
 Access to interrupts is required for operation.

 */

#include "levcan.h"
#include <math.h>
#include <string.h>


#include "can_hal.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "driver/twai.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_rom_gpio.h"
#include "esp_types.h"
#include "hal/twai_ll.h"
#include "soc/GPIO_SIG_MAP.h"
#include "soc/soc_caps.h"
#include "soc/twai_periph.h"
#include "soc/twai_struct.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

/* ---------------------------- Definitions --------------------------------- */
// Internal Macros
#define TWAI_CHECK(cond, ret_val)                                              \
  ({                                                                           \
    if (!(cond)) {                                                             \
      return (ret_val);                                                        \
    }                                                                          \
  })
#define TWAI_CHECK_FROM_CRIT(cond, ret_val)                                    \
  ({                                                                           \
    if (!(cond)) {                                                             \
      TWAI_EXIT_CRITICAL();                                                    \
      return ret_val;                                                          \
    }                                                                          \
  })
#define TWAI_SET_FLAG(var, mask) ((var) |= (mask))
#define TWAI_RESET_FLAG(var, mask) ((var) &= ~(mask))
#ifdef CONFIG_TWAI_ISR_IN_IRAM
#define TWAI_ISR_ATTR IRAM_ATTR
#define TWAI_MALLOC_CAPS (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
#else
#define TWAI_TAG "TWAI"
#define TWAI_ISR_ATTR
#define TWAI_MALLOC_CAPS MALLOC_CAP_DEFAULT
#endif // CONFIG_TWAI_ISR_IN_IRAM

#define DRIVER_DEFAULT_INTERRUPTS                                              \
  0xE7 // Exclude data overrun (bit[3]) and brp_div (bit[4])

#define ALERT_LOG_LEVEL_WARNING                                                \
  TWAI_ALERT_ARB_LOST // Alerts above and including this level use ESP_LOGW
#define ALERT_LOG_LEVEL_ERROR                                                  \
  TWAI_ALERT_TX_FAILED // Alerts above and including this level use ESP_LOGE

/* ------------------ Typedefs, structures, and variables ------------------- */

#define TXQUEUE_SIZE 30
#define TWAI_HAL_INIT_TEC 0
#define TWAI_HAL_INIT_REC 0
#define TWAI_HAL_INIT_EWL 96

// twai_dev_t TWAI;

// Control structure for TWAI driver
typedef struct {
  // Control and status members
  twai_state_t state;
  twai_mode_t mode;
  uint32_t rx_missed_count;
  uint32_t rx_overrun_count;
  uint32_t tx_failed_count;
  uint32_t arb_lost_count;
  uint32_t bus_error_count;
  intr_handle_t isr_handle;
  // TX and RX
#ifdef CONFIG_TWAI_ISR_IN_IRAM
  void *tx_queue_buff;
  void *tx_queue_struct;
  void *rx_queue_buff;
  void *rx_queue_struct;
  void *semphr_struct;
#endif
  QueueHandle_t tx_queue;
  QueueHandle_t rx_queue;
  int tx_msg_count;
  int rx_msg_count;
  // Alerts
  SemaphoreHandle_t alert_semphr;
  uint32_t alerts_enabled;
  uint32_t alerts_triggered;
#ifdef CONFIG_PM_ENABLE
  // Power Management
  esp_pm_lock_handle_t pm_lock;
#endif
} twai_obj_t;

static twai_obj_t *p_twai_obj = NULL;
static portMUX_TYPE twai_spinlock = portMUX_INITIALIZER_UNLOCKED;
#define TWAI_ENTER_CRITICAL_ISR() portENTER_CRITICAL_ISR(&twai_spinlock)
#define TWAI_EXIT_CRITICAL_ISR() portEXIT_CRITICAL_ISR(&twai_spinlock)
#define TWAI_ENTER_CRITICAL() portENTER_CRITICAL(&twai_spinlock)
#define TWAI_EXIT_CRITICAL() portEXIT_CRITICAL(&twai_spinlock)

static twai_hal_context_t twai_context;

#define TWAI_RX_IDX 74
#define TWAI_TX_IDX 74
#define TX_GPIO_NUM 2
#define RX_GPIO_NUM                                                            \
  3 //.acceptance_code = (125 << 7) << 3, .acceptance_mask = 0xFFFE0BFF,
    //.single_filter = true

static const twai_filter_config_t f_config = {.acceptance_code = (125 << 7)
                                                                 << 3,
                                              .acceptance_mask = 0xFFFE0BFF,
                                              .single_filter = true};
static const twai_timing_config_t t_config ={.brp = 4, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false};// {.clk_src = TWAI_CLK_SRC_DEFAULT, .quanta_resolution_hz = 20000000, .brp = 0, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false};//{.brp = 4, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false};
static twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT_user(
    TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);//TWAI_TIMING_CONFIG_1MBITS()  

const float accuracy = 1.e-3; // minimum required accuracy of the bit time
uint8_t _getFreeTX();
extern void LC_ReceiveHandler(LC_NodeDescriptor_t *node,
                              LC_HeaderPacked_t header, uint32_t *data,
                              uint8_t length);
#ifdef TRACE
extern int trace_printf(const char *format, ...);
#endif
void txCanTask(void *pvParameters);

#define CAN_ForceEXID

uint8_t USB_ID = 0xFF;
TaskHandle_t txCantask = 0;
QueueHandle_t txCanqueue = 0;
volatile uint32_t can_tx_cnt = 0;
volatile uint32_t can_rx_cnt = 0;

typedef struct {
  LC_HeaderPacked_t Header;
  uint32_t data[2];
  uint8_t length;
} can_packet_t;

enum {
  FIFO0,
  FIFO1,
};


 uint8_t FilterENABLE;
 uint32_t FilterTarget;

void CAN_Init() {

  twai_driver_install_user(&g_config, &t_config, &f_config);

  txCanqueue = xQueueCreate(300, sizeof(can_packet_t));
}

/// Begin CAN operation
void CAN_Start(void) {
  twai_start_user();
  xTaskCreate(txCanTask, "ctx", 4096, NULL, 4, &txCantask);
}

LC_Return_t LC_HAL_TxHalfFull() {
  int size = uxQueueMessagesWaiting(txCanqueue);

  if (size * 4 < TXQUEUE_SIZE * 3)
    return LC_BufferEmpty;
  else
    return LC_BufferFull;
}

void CAN_tx_buffer_and_transmit(twai_hal_context_t *hal_ctx,
                                twai_hal_frame_t *tx_frame) {
  twai_ll_set_tx_buffer(hal_ctx->dev, tx_frame);
  twai_ll_set_cmd_tx(hal_ctx->dev);
  TWAI_HAL_SET_BITS(hal_ctx->state_flags, TWAI_HAL_STATE_FLAG_TX_BUFF_OCCUPIED);
}

CAN_Status CAN_Send(CAN_IR index, uint32_t *data, uint16_t length) {

#ifdef CAN_ForceEXID
  index.ExtensionID = 1;
#endif
#ifdef CAN_ForceSTID
  index.ExtensionID = 0;
#endif

  if (length > 8)
    return CANH_Fail;
  if (twai_context.state_flags & TWAI_HAL_STATE_FLAG_BUS_OFF) {
    twai_driver_uninstall_user();
    twai_driver_install_user(&g_config, &t_config, &f_config);
    twai_start_user();
  }

  if (TWAI.status_reg.tbs) {
    twai_message_t tx_message = {0};
    tx_message.identifier = index.EXID;
    tx_message.data_length_code = length;
    tx_message.extd = index.ExtensionID;
    tx_message.rtr = index.Request;
    for (uint8_t i = 0; i < length; i++) {
      tx_message.data[i] = *(((uint8_t *)data) + i);
    }
    twai_hal_frame_t tx_frame;
    twai_hal_format_frame_user(&tx_message, &tx_frame);

    CAN_tx_buffer_and_transmit(&twai_context, &tx_frame);

    return CANH_Ok;
  } else
    return CANH_QueueFull;
}

LC_Return_t LC_HAL_Send(LC_HeaderPacked_t header, uint32_t *data,
                        uint8_t length) {
  // CAN send
  uint8_t state = LC_Ok;

  can_packet_t packet;
  packet.Header = header;
  if (length) {
    packet.data[0] = (uint32_t)data[0];
    packet.data[1] = (uint32_t)data[1];
    packet.length = length;
  }

  state =
      (xQueueSend(txCanqueue, &packet, 0) == pdTRUE) ? LC_Ok : LC_BufferFull;
  if (state == LC_BufferFull) {
    //ESP_LOGE("xQueueSend(txCanqueue", "LC_BufferFull");
  }
  return state;
}

// controller filter init
LC_Return_t LC_HAL_CreateFilterMasks(LC_HeaderPacked_t *reg,
                                     LC_HeaderPacked_t *mask, uint16_t count) {
  if (count == 2) {
    FilterTarget = reg[1].Target;
    FilterENABLE = 1;
  }
  return LC_Ok;
}

void txCanTask(void *pvParameters) {
  (void)pvParameters;

  static can_packet_t packet_from_q = {0};
  for (;;) {
    if (xQueueReceive(txCanqueue, &packet_from_q, portMAX_DELAY) == pdTRUE) {
      CAN_IR sindex;
      sindex.EXID = packet_from_q.Header.ToUint32;
      sindex.ExtensionID = 1;
      sindex.Request = packet_from_q.Header.Request;
      if (sindex.Request)
        packet_from_q.length = 0;
      // counter
      can_tx_cnt++;
      // try to send till its actually sent
      int i = 0;

      while (CAN_Send(sindex, packet_from_q.data, packet_from_q.length) !=
             CANH_Ok) {
        if (i > 5) {

          break; // cant CAN send :P
        }
        // cant send, wait for TX empty
        ulTaskNotifyTake(pdTRUE, 50); // 0.25s wait
        i++;
      }
    }
  }
}

/* -------------------- Interrupt and Alert Handlers ------------------------ */

TWAI_ISR_ATTR static void twai_alert_handler_user(uint32_t alert_code,
                                                  int *alert_req) {
  if (p_twai_obj->alerts_enabled & alert_code) {
    // Signify alert has occurred
    TWAI_SET_FLAG(p_twai_obj->alerts_triggered, alert_code);
    *alert_req = 1;
#ifndef CONFIG_TWAI_ISR_IN_IRAM // Only log if ISR is not in IRAM
    if (p_twai_obj->alerts_enabled & TWAI_ALERT_AND_LOG) {
      if (alert_code >= ALERT_LOG_LEVEL_ERROR) {
        ESP_EARLY_LOGE(TWAI_TAG, "Alert %d", alert_code);
      } else if (alert_code >= ALERT_LOG_LEVEL_WARNING) {
        ESP_EARLY_LOGW(TWAI_TAG, "Alert %d", alert_code);
      } else {
        ESP_EARLY_LOGI(TWAI_TAG, "Alert %d", alert_code);
      }
    }
#endif // CONFIG_TWAI_ISR_IN_IRAM
  }
}

static inline void twai_handle_rx_buffer_frames_user(BaseType_t *task_woken,
                                                     int *alert_req) {
  uint32_t msg_count = twai_hal_get_rx_msg_count_user(&twai_context);
  static uint8_t data_eror = 0;
  for (uint32_t i = 0; i < msg_count; i++) {
    twai_hal_frame_t frame = {0};
    uint16_t rlength;
    uint32_t data[2];
    if (twai_hal_read_rx_buffer_and_clear_user(&twai_context, &frame)) {
      // Valid frame copied from RX buffer
      LC_HeaderPacked_t header = {0};
      header.ToUint32 = *((uint32_t *)frame.extended.id); // 29b
      header.ToUint32 = HAL_SWAP32(header.ToUint32) >> 3;
      header.Request = frame.rtr; // 30b
      // data[0]=*((uint32_t*) frame.extended.data) ;
      // data[1]=*((uint32_t*) frame.extended.data+1) ;
      rlength = frame.dlc;
      uint8_t *data_buffer =
          (frame.frame_format) ? frame.extended.data : frame.standard.data;
      // Only copy data if the frame is a data frame (i.e. not a remote frame)
      int data_length =
          (frame.rtr) ? 0
                      : ((frame.dlc > TWAI_FRAME_MAX_DLC) ? TWAI_FRAME_MAX_DLC
                                                          : frame.dlc);
      for (int i = 0; i < data_length; i++) {
        *(((uint8_t *)data) + i) = data_buffer[i];
      }
      // Set remaining bytes of data to 0
      for (int i = data_length; i < TWAI_FRAME_MAX_DLC; i++) {
        *(((uint8_t *)data) + i) = 0;
      }
      if (((header.Target == FilterTarget) && FilterENABLE) ||
          header.Target == LC_Broadcast_Address)
        LC_ReceiveHandler(mynode, header, data, rlength); // todo
      else
        data_eror++;

    } else { // Failed to read from RX buffer because message is overrun
      p_twai_obj->rx_overrun_count++;
      twai_alert_handler_user(TWAI_ALERT_RX_FIFO_OVERRUN, alert_req);
    }
  }
}

static inline void twai_handle_tx_buffer_frame_user(BaseType_t *task_woken,
                                                    int *alert_req) {
  // Handle previously transmitted frame
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(txCantask, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR();
}

TWAI_ISR_ATTR static void twai_intr_handler_main_user(void *arg) {
  BaseType_t task_woken = pdFALSE;
  int alert_req = 0;
  uint32_t events;
  TWAI_ENTER_CRITICAL_ISR();
  if (p_twai_obj ==
      NULL) { // In case intr occurs whilst driver is being uninstalled
    TWAI_EXIT_CRITICAL_ISR();
    return;
  }
  events = twai_hal_get_events_user(
      &twai_context); // Get the events that triggered the interrupt

#if defined(CONFIG_TWAI_ERRATA_FIX_RX_FRAME_INVALID) ||                        \
    defined(CONFIG_TWAI_ERRATA_FIX_RX_FIFO_CORRUPT)
  if (events & TWAI_HAL_EVENT_NEED_PERIPH_RESET) {
    twai_hal_prepare_for_reset(&twai_context);
    periph_module_reset(PERIPH_TWAI_MODULE);
    twai_hal_recover_from_reset(&twai_context);
    p_twai_obj->rx_missed_count +=
        twai_hal_get_reset_lost_rx_cnt(&twai_context);
    twai_alert_handler(TWAI_ALERT_PERIPH_RESET, &alert_req);
  }
#endif
  if (events & TWAI_HAL_EVENT_RX_BUFF_FRAME) {
    // Note: This event will never occur if there is a periph reset event
    twai_handle_rx_buffer_frames_user(&task_woken, &alert_req);
  }
  if (events & TWAI_HAL_EVENT_TX_BUFF_FREE) {
    twai_handle_tx_buffer_frame_user(&task_woken, &alert_req);
  }

  // Handle events that only require alerting (i.e. no handler)
  if (events & TWAI_HAL_EVENT_BUS_OFF) {
    p_twai_obj->state = TWAI_STATE_BUS_OFF;
    twai_alert_handler_user(TWAI_ALERT_BUS_OFF, &alert_req);
  }
  if (events & TWAI_HAL_EVENT_BUS_RECOV_CPLT) {
    p_twai_obj->state = TWAI_STATE_STOPPED;
    twai_alert_handler_user(TWAI_ALERT_BUS_RECOVERED, &alert_req);
  }
  if (events & TWAI_HAL_EVENT_BUS_ERR) {
    p_twai_obj->bus_error_count++;
    twai_alert_handler_user(TWAI_ALERT_BUS_ERROR, &alert_req);
  }
  if (events & TWAI_HAL_EVENT_ARB_LOST) {
    p_twai_obj->arb_lost_count++;
   // twai_alert_handler_user(TWAI_ALERT_ARB_LOST, &alert_req);
  }
  if (events & TWAI_HAL_EVENT_BUS_RECOV_PROGRESS) {
    // Bus-recovery in progress. TEC has dropped below error warning limit
    twai_alert_handler_user(TWAI_ALERT_RECOVERY_IN_PROGRESS, &alert_req);
  }
  if (events & TWAI_HAL_EVENT_ERROR_PASSIVE) {
    // Entered error passive
    twai_alert_handler_user(TWAI_ALERT_ERR_PASS, &alert_req);
  }
  if (events & TWAI_HAL_EVENT_ERROR_ACTIVE) {
    // Returned to error active
    twai_alert_handler_user(TWAI_ALERT_ERR_ACTIVE, &alert_req);
  }
  if (events & TWAI_HAL_EVENT_ABOVE_EWL) {
    // TEC or REC surpassed error warning limit
    twai_alert_handler_user(TWAI_ALERT_ABOVE_ERR_WARN, &alert_req);
  }
  if (events & TWAI_HAL_EVENT_BELOW_EWL) {
    // TEC and REC are both below error warning
    twai_alert_handler_user(TWAI_ALERT_BELOW_ERR_WARN, &alert_req);
  }

  TWAI_EXIT_CRITICAL_ISR();

  if (p_twai_obj->alert_semphr != NULL && alert_req) {
    // Give semaphore if alerts were triggered
    xSemaphoreGiveFromISR(p_twai_obj->alert_semphr, &task_woken);
  }
  if (task_woken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

/* -------------------------- Helper functions  ----------------------------- */

static void twai_configure_gpio_user(gpio_num_t tx, gpio_num_t rx,
                                     gpio_num_t clkout, gpio_num_t bus_status) {
  // Set TX pin
  gpio_set_pull_mode(tx, GPIO_FLOATING);
  esp_rom_gpio_connect_out_signal(tx, TWAI_TX_IDX, false, false);
  esp_rom_gpio_pad_select_gpio(tx);

  // Set RX pin
  gpio_set_pull_mode(rx, GPIO_FLOATING);
  esp_rom_gpio_connect_in_signal(rx, TWAI_RX_IDX, false);
  esp_rom_gpio_pad_select_gpio(rx);
  gpio_set_direction(rx, GPIO_MODE_INPUT);

  // Configure output clock pin (Optional)
  if (clkout >= 0 && clkout < GPIO_NUM_MAX) {
    gpio_set_pull_mode(clkout, GPIO_FLOATING);
    esp_rom_gpio_connect_out_signal(clkout, TWAI_CLKOUT_IDX, false, false);
    esp_rom_gpio_pad_select_gpio(clkout);
  }

  // Configure bus status pin (Optional)
  if (bus_status >= 0 && bus_status < GPIO_NUM_MAX) {
    gpio_set_pull_mode(bus_status, GPIO_FLOATING);
    esp_rom_gpio_connect_out_signal(bus_status, TWAI_BUS_OFF_ON_IDX, false,
                                    false);
    esp_rom_gpio_pad_select_gpio(bus_status);
  }
}

static void twai_free_driver_obj_user(twai_obj_t *p_obj) {
  // Free driver object and any dependent SW resources it uses (queues,
  // semaphores etc)
#ifdef CONFIG_PM_ENABLE
  if (p_obj->pm_lock != NULL) {
    ESP_ERROR_CHECK(esp_pm_lock_delete(p_obj->pm_lock));
  }
#endif
  // Delete queues and semaphores
  if (p_obj->tx_queue != NULL) {
    vQueueDelete(p_obj->tx_queue);
  }
  if (p_obj->rx_queue != NULL) {
    vQueueDelete(p_obj->rx_queue);
  }
  if (p_obj->alert_semphr != NULL) {
    vSemaphoreDelete(p_obj->alert_semphr);
  }
#ifdef CONFIG_TWAI_ISR_IN_IRAM
  // Free memory used by static queues and semaphores. free() allows freeing
  // NULL pointers
  free(p_obj->tx_queue_buff);
  free(p_obj->tx_queue_struct);
  free(p_obj->rx_queue_buff);
  free(p_obj->rx_queue_struct);
  free(p_obj->semphr_struct);
#endif // CONFIG_TWAI_ISR_IN_IRAM
  free(p_obj);
}

static twai_obj_t *twai_alloc_driver_obj_user(uint32_t tx_queue_len,
                                              uint32_t rx_queue_len) {
  // Allocates driver object and any dependent SW resources it uses (queues,
  // semaphores etc) Create a TWAI driver object
  twai_obj_t *p_obj = heap_caps_calloc(1, sizeof(twai_obj_t), TWAI_MALLOC_CAPS);
  if (p_obj == NULL) {
    return NULL;
  }
#ifdef CONFIG_TWAI_ISR_IN_IRAM
  // Allocate memory for queues and semaphores in DRAM
  if (tx_queue_len > 0) {
    p_obj->tx_queue_buff = heap_caps_calloc(
        tx_queue_len, sizeof(twai_hal_frame_t), TWAI_MALLOC_CAPS);
    p_obj->tx_queue_struct =
        heap_caps_calloc(1, sizeof(StaticQueue_t), TWAI_MALLOC_CAPS);
    if (p_obj->tx_queue_buff == NULL || p_obj->tx_queue_struct == NULL) {
      goto cleanup;
    }
  }
  p_obj->rx_queue_buff = heap_caps_calloc(
      rx_queue_len, sizeof(twai_hal_frame_t), TWAI_MALLOC_CAPS);
  p_obj->rx_queue_struct =
      heap_caps_calloc(1, sizeof(StaticQueue_t), TWAI_MALLOC_CAPS);
  p_obj->semphr_struct =
      heap_caps_calloc(1, sizeof(StaticSemaphore_t), TWAI_MALLOC_CAPS);
  if (p_obj->rx_queue_buff == NULL || p_obj->rx_queue_struct == NULL ||
      p_obj->semphr_struct == NULL) {
    goto cleanup;
  }
  // Create static queues and semaphores
  if (tx_queue_len > 0) {
    p_obj->tx_queue =
        xQueueCreateStatic(tx_queue_len, sizeof(twai_hal_frame_t),
                           p_obj->tx_queue_buff, p_obj->tx_queue_struct);
    if (p_obj->tx_queue == NULL) {
      goto cleanup;
    }
  }
  p_obj->rx_queue =
      xQueueCreateStatic(rx_queue_len, sizeof(twai_hal_frame_t),
                         p_obj->rx_queue_buff, p_obj->rx_queue_struct);
  p_obj->alert_semphr = xSemaphoreCreateBinaryStatic(p_obj->semphr_struct);
  if (p_obj->rx_queue == NULL || p_obj->alert_semphr == NULL) {
    goto cleanup;
  }
#else  // CONFIG_TWAI_ISR_IN_IRAM
  if (tx_queue_len > 0) {
    p_obj->tx_queue = xQueueCreate(tx_queue_len, sizeof(twai_hal_frame_t));
  }
  p_obj->rx_queue = xQueueCreate(rx_queue_len, sizeof(twai_hal_frame_t));
  p_obj->alert_semphr = xSemaphoreCreateBinary();
  if ((tx_queue_len > 0 && p_obj->tx_queue == NULL) ||
      p_obj->rx_queue == NULL || p_obj->alert_semphr == NULL) {
    goto cleanup;
  }
#endif // CONFIG_TWAI_ISR_IN_IRAM

#ifdef CONFIG_PM_ENABLE
  esp_err_t pm_err =
      esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "twai", &(p_obj->pm_lock));
  if (pm_err != ESP_OK) {
    goto cleanup;
  }
#endif
  return p_obj;

cleanup:
  twai_free_driver_obj_user(p_obj);
  return NULL;
}

/* ---------------------------- Public Functions ---------------------------- */

esp_err_t twai_driver_install_user(const twai_general_config_t *g_config,
                                   const twai_timing_config_t *t_config,
                                   const twai_filter_config_t *f_config) {
  // Check arguments
  TWAI_CHECK(g_config != NULL, ESP_ERR_INVALID_ARG);
  TWAI_CHECK(t_config != NULL, ESP_ERR_INVALID_ARG);
  TWAI_CHECK(f_config != NULL, ESP_ERR_INVALID_ARG);
  TWAI_CHECK(g_config->rx_queue_len > 0, ESP_ERR_INVALID_ARG);
  TWAI_CHECK(g_config->tx_io >= 0 && g_config->tx_io < GPIO_NUM_MAX,
             ESP_ERR_INVALID_ARG);
  TWAI_CHECK(g_config->rx_io >= 0 && g_config->rx_io < GPIO_NUM_MAX,
             ESP_ERR_INVALID_ARG);
  TWAI_CHECK(t_config->brp >= SOC_TWAI_BRP_MIN &&
                 t_config->brp <= SOC_TWAI_BRP_MAX,
             ESP_ERR_INVALID_ARG);
#ifndef CONFIG_TWAI_ISR_IN_IRAM
  TWAI_CHECK(!(g_config->intr_flags & ESP_INTR_FLAG_IRAM), ESP_ERR_INVALID_ARG);
#endif
  TWAI_ENTER_CRITICAL();
  TWAI_CHECK_FROM_CRIT(p_twai_obj == NULL, ESP_ERR_INVALID_STATE);
  TWAI_EXIT_CRITICAL();

  esp_err_t ret;
  twai_obj_t *p_twai_obj_dummy;

  // Create a TWAI object (including queues and semaphores)
  p_twai_obj_dummy = twai_alloc_driver_obj_user(g_config->tx_queue_len,
                                                g_config->rx_queue_len);
  TWAI_CHECK(p_twai_obj_dummy != NULL, ESP_ERR_NO_MEM);

  // Initialize flags and variables. All other members are already set to zero
  // by twai_alloc_driver_obj()
  p_twai_obj_dummy->state = TWAI_STATE_STOPPED;
  p_twai_obj_dummy->mode = g_config->mode;
  p_twai_obj_dummy->alerts_enabled = g_config->alerts_enabled;

  // Initialize TWAI peripheral registers, and allocate interrupt
  TWAI_ENTER_CRITICAL();
  if (p_twai_obj == NULL) {
    p_twai_obj = p_twai_obj_dummy;
  } else {
    // Check if driver is already installed
    TWAI_EXIT_CRITICAL();
    ret = ESP_ERR_INVALID_STATE;
    goto err;
  }
  periph_module_reset(PERIPH_TWAI_MODULE);
  periph_module_enable(PERIPH_TWAI_MODULE); // Enable APB CLK to TWAI peripheral
  bool init = twai_hal_init_user(&twai_context);
  assert(init);
  (void)init;
  twai_hal_configure_user(&twai_context, t_config, f_config,
                          DRIVER_DEFAULT_INTERRUPTS, g_config->clkout_divider);
  TWAI_EXIT_CRITICAL();

  // Allocate GPIO and Interrupts
  twai_configure_gpio_user(g_config->tx_io, g_config->rx_io,
                           g_config->clkout_io, g_config->bus_off_io);
  ESP_ERROR_CHECK(esp_intr_alloc(ETS_TWAI_INTR_SOURCE, g_config->intr_flags,
                                 twai_intr_handler_main_user, NULL,
                                 &p_twai_obj->isr_handle));

#ifdef CONFIG_PM_ENABLE
  ESP_ERROR_CHECK(esp_pm_lock_acquire(
      p_twai_obj->pm_lock)); // Acquire pm_lock to keep APB clock at 80MHz
#endif
  return ESP_OK; // TWAI module is still in reset mode, users need to call
                 // twai_start() afterwards

err:
  twai_free_driver_obj_user(p_twai_obj_dummy);
  return ret;
}

esp_err_t twai_driver_uninstall_user(void) {
  twai_obj_t *p_twai_obj_dummy;

  TWAI_ENTER_CRITICAL();
  // Check state
  TWAI_CHECK_FROM_CRIT(p_twai_obj != NULL, ESP_ERR_INVALID_STATE);
  TWAI_CHECK_FROM_CRIT(p_twai_obj->state == TWAI_STATE_STOPPED ||
                           p_twai_obj->state == TWAI_STATE_BUS_OFF,
                       ESP_ERR_INVALID_STATE);
  // Clear registers by reading
  twai_hal_deinit_user(&twai_context);
  periph_module_disable(PERIPH_TWAI_MODULE); // Disable TWAI peripheral
  p_twai_obj_dummy = p_twai_obj; // Use dummy to shorten critical section
  p_twai_obj = NULL;
  TWAI_EXIT_CRITICAL();

  ESP_ERROR_CHECK(esp_intr_free(p_twai_obj_dummy->isr_handle)); // Free
                                                                // interrupt

#ifdef CONFIG_PM_ENABLE
  // Release and delete power management lock
  ESP_ERROR_CHECK(esp_pm_lock_release(p_twai_obj_dummy->pm_lock));
#endif
  // Free can driver object
  twai_free_driver_obj_user(p_twai_obj_dummy);
  return ESP_OK;
}

esp_err_t twai_start_user(void) {
  // Check state
  TWAI_ENTER_CRITICAL();
  TWAI_CHECK_FROM_CRIT(p_twai_obj != NULL, ESP_ERR_INVALID_STATE);
  TWAI_CHECK_FROM_CRIT(p_twai_obj->state == TWAI_STATE_STOPPED,
                       ESP_ERR_INVALID_STATE);

  // Reset RX queue, RX message count, amd TX queue
  xQueueReset(p_twai_obj->rx_queue);
  if (p_twai_obj->tx_queue != NULL) {
    xQueueReset(p_twai_obj->tx_queue);
  }
  p_twai_obj->rx_msg_count = 0;
  p_twai_obj->tx_msg_count = 0;
  twai_hal_start_user(&twai_context, p_twai_obj->mode);

  p_twai_obj->state = TWAI_STATE_RUNNING;
  TWAI_EXIT_CRITICAL();
  return ESP_OK;
}

esp_err_t twai_stop_user(void) {
  // Check state
  TWAI_ENTER_CRITICAL();
  TWAI_CHECK_FROM_CRIT(p_twai_obj != NULL, ESP_ERR_INVALID_STATE);
  TWAI_CHECK_FROM_CRIT(p_twai_obj->state == TWAI_STATE_RUNNING,
                       ESP_ERR_INVALID_STATE);

  twai_hal_stop_user(&twai_context);

  // Reset TX Queue and message count
  if (p_twai_obj->tx_queue != NULL) {
    xQueueReset(p_twai_obj->tx_queue);
  }
  p_twai_obj->tx_msg_count = 0;
  p_twai_obj->state = TWAI_STATE_STOPPED;

  TWAI_EXIT_CRITICAL();

  return ESP_OK;
}

esp_err_t twai_transmit_user(const twai_message_t *message,
                             TickType_t ticks_to_wait) {
  // Check arguments
  TWAI_CHECK(p_twai_obj != NULL, ESP_ERR_INVALID_STATE);
  TWAI_CHECK(message != NULL, ESP_ERR_INVALID_ARG);
  TWAI_CHECK((message->data_length_code <= TWAI_FRAME_MAX_DLC) ||
                 message->dlc_non_comp,
             ESP_ERR_INVALID_ARG);

  // TWAI_ENTER_CRITICAL();
  // Check State
  TWAI_CHECK_FROM_CRIT(!(p_twai_obj->mode == TWAI_MODE_LISTEN_ONLY),
                       ESP_ERR_NOT_SUPPORTED);
  TWAI_CHECK_FROM_CRIT(p_twai_obj->state == TWAI_STATE_RUNNING,
                       ESP_ERR_INVALID_STATE);
  // Format frame
  esp_err_t ret = ESP_FAIL;
  twai_hal_frame_t tx_frame;
  twai_hal_format_frame_user(message, &tx_frame);

  // Check if frame can be sent immediately
  if (p_twai_obj->tx_msg_count == 0) {
    // No other frames waiting to transmit. Bypass queue and transmit
    // immediately
    twai_hal_set_tx_buffer_and_transmit_user(&twai_context, &tx_frame); // todo
    p_twai_obj->tx_msg_count++;
    ret = ESP_OK;
  }
  // TWAI_EXIT_CRITICAL();

  if (ret != ESP_OK) {
    if (p_twai_obj->tx_queue == NULL) {
      // TX Queue is disabled and TX buffer is occupied, message was not sent
      ret = ESP_FAIL;
    } else if (xQueueSend(p_twai_obj->tx_queue, &tx_frame, ticks_to_wait) ==
               pdTRUE) {
      // Copied to TX Queue
      TWAI_ENTER_CRITICAL();
      if ((!twai_hal_check_state_flags_user(
              &twai_context, TWAI_HAL_STATE_FLAG_TX_BUFF_OCCUPIED)) &&
          uxQueueMessagesWaiting(p_twai_obj->tx_queue) > 0) {
        // If the TX buffer is free but the TX queue is not empty. Check if we
        // need to manually start a transmission
        if (twai_hal_check_state_flags_user(&twai_context,
                                            TWAI_HAL_STATE_FLAG_BUS_OFF) ||
            !twai_hal_check_state_flags_user(&twai_context,
                                             TWAI_HAL_STATE_FLAG_RUNNING)) {
          // TX buffer became free due to bus-off or is no longer running. No
          // need to start a transmission
          ret = ESP_ERR_INVALID_STATE;
        } else {
          // Manually start a transmission
          int res = xQueueReceive(p_twai_obj->tx_queue, &tx_frame, 0);
          assert(res == pdTRUE);
          (void)res;
          twai_hal_set_tx_buffer_and_transmit_user(&twai_context, &tx_frame);
          p_twai_obj->tx_msg_count++;
          ret = ESP_OK;
        }
      } else {
        // Frame was copied to queue, waiting to be transmitted
        p_twai_obj->tx_msg_count++;
        ret = ESP_OK;
      }
      TWAI_EXIT_CRITICAL();
    } else {
      // Timed out waiting for free space on TX queue
      ret = ESP_ERR_TIMEOUT;
    }
  }
  return ret;
}

esp_err_t twai_receive_user(twai_message_t *message, TickType_t ticks_to_wait) {
  // Check arguments and state
  TWAI_CHECK(p_twai_obj != NULL, ESP_ERR_INVALID_STATE);
  TWAI_CHECK(message != NULL, ESP_ERR_INVALID_ARG);

  // Get frame from RX Queue or RX Buffer
  twai_hal_frame_t rx_frame;
  if (xQueueReceive(p_twai_obj->rx_queue, &rx_frame, ticks_to_wait) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }

  TWAI_ENTER_CRITICAL();
  p_twai_obj->rx_msg_count--;
  TWAI_EXIT_CRITICAL();

  // Decode frame
  twai_hal_parse_frame_user(&rx_frame, message);
  return ESP_OK;
}

esp_err_t twai_read_alerts_user(uint32_t *alerts, TickType_t ticks_to_wait) {
  // Check arguments and state
  TWAI_CHECK(p_twai_obj != NULL, ESP_ERR_INVALID_STATE);
  TWAI_CHECK(alerts != NULL, ESP_ERR_INVALID_ARG);

  // Wait for an alert to occur
  if (xSemaphoreTake(p_twai_obj->alert_semphr, ticks_to_wait) == pdTRUE) {
    TWAI_ENTER_CRITICAL();
    *alerts = p_twai_obj->alerts_triggered;
    p_twai_obj->alerts_triggered = 0; // Clear triggered alerts
    TWAI_EXIT_CRITICAL();
    return ESP_OK;
  } else {
    *alerts = 0;
    return ESP_ERR_TIMEOUT;
  }
}

esp_err_t twai_reconfigure_alerts_user(uint32_t alerts_enabled,
                                       uint32_t *current_alerts) {
  TWAI_CHECK(p_twai_obj != NULL, ESP_ERR_INVALID_STATE);

  TWAI_ENTER_CRITICAL();
  // Clear any unhandled alerts
  if (current_alerts != NULL) {
    *current_alerts = p_twai_obj->alerts_triggered;
    ;
  }
  p_twai_obj->alerts_triggered = 0;
  p_twai_obj->alerts_enabled = alerts_enabled; // Update enabled alerts
  TWAI_EXIT_CRITICAL();

  return ESP_OK;
}

esp_err_t twai_initiate_recovery_user(void) {
  TWAI_ENTER_CRITICAL();
  // Check state
  TWAI_CHECK_FROM_CRIT(p_twai_obj != NULL, ESP_ERR_INVALID_STATE);
  TWAI_CHECK_FROM_CRIT(p_twai_obj->state == TWAI_STATE_BUS_OFF,
                       ESP_ERR_INVALID_STATE);

  // Reset TX Queue/Counters
  if (p_twai_obj->tx_queue != NULL) {
    xQueueReset(p_twai_obj->tx_queue);
  }
  p_twai_obj->tx_msg_count = 0;

  // Trigger start of recovery process
  twai_hal_start_bus_recovery_user(&twai_context);
  p_twai_obj->state = TWAI_STATE_RECOVERING;
  TWAI_EXIT_CRITICAL();

  return ESP_OK;
}

esp_err_t twai_get_status_info_user(twai_status_info_t *status_info) {
  // Check parameters and state
  TWAI_CHECK(p_twai_obj != NULL, ESP_ERR_INVALID_STATE);
  TWAI_CHECK(status_info != NULL, ESP_ERR_INVALID_ARG);

  TWAI_ENTER_CRITICAL();
  status_info->tx_error_counter = twai_hal_get_tec_user(&twai_context);
  status_info->rx_error_counter = twai_hal_get_rec_user(&twai_context);
  status_info->msgs_to_tx = p_twai_obj->tx_msg_count;
  status_info->msgs_to_rx = p_twai_obj->rx_msg_count;
  status_info->tx_failed_count = p_twai_obj->tx_failed_count;
  status_info->rx_missed_count = p_twai_obj->rx_missed_count;
  status_info->rx_overrun_count = p_twai_obj->rx_overrun_count;
  status_info->arb_lost_count = p_twai_obj->arb_lost_count;
  status_info->bus_error_count = p_twai_obj->bus_error_count;
  status_info->state = p_twai_obj->state;
  TWAI_EXIT_CRITICAL();

  return ESP_OK;
}

esp_err_t twai_clear_transmit_queue_user(void) {
  // Check State
  TWAI_CHECK(p_twai_obj != NULL, ESP_ERR_INVALID_STATE);
  TWAI_CHECK(p_twai_obj->tx_queue != NULL, ESP_ERR_NOT_SUPPORTED);

  TWAI_ENTER_CRITICAL();
  // If a message is currently undergoing transmission, the tx interrupt handler
  // will decrement tx_msg_count
  p_twai_obj->tx_msg_count =
      twai_hal_check_state_flags_user(&twai_context,
                                      TWAI_HAL_STATE_FLAG_TX_BUFF_OCCUPIED)
          ? 1
          : 0;
  xQueueReset(p_twai_obj->tx_queue);
  TWAI_EXIT_CRITICAL();

  return ESP_OK;
}

esp_err_t twai_clear_receive_queue_user(void) {
  // Check State
  TWAI_CHECK(p_twai_obj != NULL, ESP_ERR_INVALID_STATE);

  TWAI_ENTER_CRITICAL();
  p_twai_obj->rx_msg_count = 0;
  xQueueReset(p_twai_obj->rx_queue);
  TWAI_EXIT_CRITICAL();

  return ESP_OK;
}

bool twai_hal_init_user(twai_hal_context_t *hal_ctx) {
  // Initialize HAL context
  hal_ctx->dev = &TWAI;
  hal_ctx->state_flags = 0;
  // Initialize TWAI controller, and set default values to registers
  twai_ll_enter_reset_mode(hal_ctx->dev);
  if (!twai_ll_is_in_reset_mode(
          hal_ctx->dev)) { // Must enter reset mode to write to config registers
    return false;
  }
#if SOC_TWAI_SUPPORT_MULTI_ADDRESS_LAYOUT
  twai_ll_enable_extended_reg_layout(
      hal_ctx->dev); // Changes the address layout of the registers
#endif
  twai_ll_set_mode(hal_ctx->dev,
                   TWAI_MODE_LISTEN_ONLY); // Freeze REC by changing to LOM mode
  // Both TEC and REC should start at 0
  twai_ll_set_tec(hal_ctx->dev, TWAI_HAL_INIT_TEC);
  twai_ll_set_rec(hal_ctx->dev, TWAI_HAL_INIT_REC);
  twai_ll_set_err_warn_lim(hal_ctx->dev,
                           TWAI_HAL_INIT_EWL); // Set default value of for EWL
  return true;
}

void twai_hal_deinit_user(twai_hal_context_t *hal_ctx) {
  // Clear any pending registers
  (void)twai_ll_get_and_clear_intrs(hal_ctx->dev);
  twai_ll_set_enabled_intrs(hal_ctx->dev, 0);
  twai_ll_clear_arb_lost_cap(hal_ctx->dev);
  twai_ll_clear_err_code_cap(hal_ctx->dev);
  hal_ctx->dev = NULL;
}

void twai_hal_configure_user(twai_hal_context_t *hal_ctx,
                             const twai_timing_config_t *t_config,
                             const twai_filter_config_t *f_config,
                             uint32_t intr_mask, uint32_t clkout_divider) {
  // Configure bus timing, acceptance filter, CLKOUT, and interrupts
  twai_ll_set_bus_timing(hal_ctx->dev, t_config->brp, t_config->sjw,
                         t_config->tseg_1, t_config->tseg_2,
                         t_config->triple_sampling);
  twai_ll_set_acc_filter(hal_ctx->dev, f_config->acceptance_code,
                         f_config->acceptance_mask, f_config->single_filter);
  twai_ll_set_clkout(hal_ctx->dev, clkout_divider);
  twai_ll_set_enabled_intrs(hal_ctx->dev, intr_mask);
  (void)twai_ll_get_and_clear_intrs(
      hal_ctx->dev); // Clear any latched interrupts
}

/* -------------------------------- Actions --------------------------------- */

void twai_hal_start_user(twai_hal_context_t *hal_ctx, twai_mode_t mode) {
  twai_ll_set_mode(hal_ctx->dev, mode); // Set operating mode
  // Clear the TEC and REC
  twai_ll_set_tec(hal_ctx->dev, 0);
#ifdef CONFIG_TWAI_ERRATA_FIX_LISTEN_ONLY_DOM
  /*
  Errata workaround: Prevent transmission of dominant error frame while in
  listen only mode by setting REC to 128 before exiting reset mode. This forces
  the controller to be error passive (thus only transmits recessive bits). The
  TEC/REC remain frozen in listen only mode thus ensuring we remain error
  passive.
  */
  if (mode == TWAI_MODE_LISTEN_ONLY) {
    twai_ll_set_rec(hal_ctx->dev, 128);
  } else
#endif
  {
    twai_ll_set_rec(hal_ctx->dev, 0);
  }
  (void)twai_ll_get_and_clear_intrs(
      hal_ctx->dev); // Clear any latched interrupts
  TWAI_HAL_SET_BITS(hal_ctx->state_flags, TWAI_HAL_STATE_FLAG_RUNNING);
  twai_ll_exit_reset_mode(hal_ctx->dev);
}

void twai_hal_stop_user(twai_hal_context_t *hal_ctx) {
  twai_ll_enter_reset_mode(hal_ctx->dev);
  (void)twai_ll_get_and_clear_intrs(hal_ctx->dev);
  twai_ll_set_mode(hal_ctx->dev,
                   TWAI_MODE_LISTEN_ONLY); // Freeze REC by changing to LOM mode
  // Any TX is immediately halted on entering reset mode
  TWAI_HAL_CLEAR_BITS(hal_ctx->state_flags,
                      TWAI_HAL_STATE_FLAG_TX_BUFF_OCCUPIED |
                          TWAI_HAL_STATE_FLAG_RUNNING);
}

#ifdef CONFIG_TWAI_ERRATA_FIX_RX_FIFO_CORRUPT
// Errata condition occurs at 64 messages. Threshold set to 62 to prevent the
// chance of failing to detect errata condition.
#define TWAI_RX_FIFO_CORRUPT_THRESH 62
#endif

/* ----------------------------- Event Handling ----------------------------- */

/**
 * Helper functions that can decode what events have been triggered based on
 * the values of the interrupt, status, TEC and REC registers. The HAL context's
 * state flags are also updated based on the events that have triggered.
 */
static inline uint32_t
twai_hal_decode_interrupt_user(twai_hal_context_t *hal_ctx) {
  uint32_t events = 0;
  uint32_t interrupts = twai_ll_get_and_clear_intrs(hal_ctx->dev);
  uint32_t status = twai_ll_get_status(hal_ctx->dev);
  uint32_t tec = twai_ll_get_tec(hal_ctx->dev);
  uint32_t rec = twai_ll_get_rec(hal_ctx->dev);
  uint32_t state_flags = hal_ctx->state_flags;

  // Error Warning Interrupt set whenever Error or Bus Status bit changes
  if (interrupts & TWAI_LL_INTR_EI) {
    if (status & TWAI_LL_STATUS_BS) {   // Currently in BUS OFF state
      if (status & TWAI_LL_STATUS_ES) { // EWL is exceeded, thus must have
                                        // entered BUS OFF
        TWAI_HAL_SET_BITS(events, TWAI_HAL_EVENT_BUS_OFF);
        TWAI_HAL_SET_BITS(state_flags, TWAI_HAL_STATE_FLAG_BUS_OFF);
        // Any TX would have been halted by entering bus off. Reset its flag
        TWAI_HAL_CLEAR_BITS(state_flags,
                            TWAI_HAL_STATE_FLAG_RUNNING |
                                TWAI_HAL_STATE_FLAG_TX_BUFF_OCCUPIED);
      } else {
        // Below EWL. Therefore TEC is counting down in bus recovery
        TWAI_HAL_SET_BITS(events, TWAI_HAL_EVENT_BUS_RECOV_PROGRESS);
      }
    } else {                            // Not in BUS OFF
      if (status & TWAI_LL_STATUS_ES) { // Just Exceeded EWL
        TWAI_HAL_SET_BITS(events, TWAI_HAL_EVENT_ABOVE_EWL);
        TWAI_HAL_SET_BITS(state_flags, TWAI_HAL_STATE_FLAG_ERR_WARN);
      } else if (hal_ctx->state_flags & TWAI_HAL_STATE_FLAG_RECOVERING) {
        // Previously undergoing bus recovery. Thus means bus recovery complete
        TWAI_HAL_SET_BITS(events, TWAI_HAL_EVENT_BUS_RECOV_CPLT);
        TWAI_HAL_CLEAR_BITS(state_flags, TWAI_HAL_STATE_FLAG_RECOVERING |
                                             TWAI_HAL_STATE_FLAG_BUS_OFF);
      } else { // Just went below EWL
        TWAI_HAL_SET_BITS(events, TWAI_HAL_EVENT_BELOW_EWL);
        TWAI_HAL_CLEAR_BITS(state_flags, TWAI_HAL_STATE_FLAG_ERR_WARN);
      }
    }
  }
  // Receive Interrupt set whenever RX FIFO is not empty
  if (interrupts & TWAI_LL_INTR_RI) {
    TWAI_HAL_SET_BITS(events, TWAI_HAL_EVENT_RX_BUFF_FRAME);
  }
  // Transmit interrupt set whenever TX buffer becomes free
#ifdef CONFIG_TWAI_ERRATA_FIX_TX_INTR_LOST
  if ((interrupts & TWAI_LL_INTR_TI ||
       hal_ctx->state_flags & TWAI_HAL_STATE_FLAG_TX_BUFF_OCCUPIED) &&
      status & TWAI_LL_STATUS_TBS) {
#else
  if (interrupts & TWAI_LL_INTR_TI) {
#endif
    TWAI_HAL_SET_BITS(events, TWAI_HAL_EVENT_TX_BUFF_FREE);
    TWAI_HAL_CLEAR_BITS(state_flags, TWAI_HAL_STATE_FLAG_TX_BUFF_OCCUPIED);
  }
  // Error Passive Interrupt on transition from error active to passive or vice
  // versa
  if (interrupts & TWAI_LL_INTR_EPI) {
    if (tec >= TWAI_ERR_PASS_THRESH || rec >= TWAI_ERR_PASS_THRESH) {
      TWAI_HAL_SET_BITS(events, TWAI_HAL_EVENT_ERROR_PASSIVE);
      TWAI_HAL_SET_BITS(state_flags, TWAI_HAL_STATE_FLAG_ERR_PASSIVE);
    } else {
      TWAI_HAL_SET_BITS(events, TWAI_HAL_EVENT_ERROR_ACTIVE);
      TWAI_HAL_CLEAR_BITS(state_flags, TWAI_HAL_STATE_FLAG_ERR_PASSIVE);
    }
  }
  // Bus error interrupt triggered on a bus error (e.g. bit, ACK, stuff etc)
  if (interrupts & TWAI_LL_INTR_BEI) {
    TWAI_HAL_SET_BITS(events, TWAI_HAL_EVENT_BUS_ERR);
  }
  // Arbitration Lost Interrupt triggered on losing arbitration
  if (interrupts & TWAI_LL_INTR_ALI) {
    TWAI_HAL_SET_BITS(events, TWAI_HAL_EVENT_ARB_LOST);
  }
  hal_ctx->state_flags = state_flags;
  return events;
}

uint32_t twai_hal_get_events_user(twai_hal_context_t *hal_ctx) {
  uint32_t events = twai_hal_decode_interrupt_user(hal_ctx);

  // Handle low latency events
  if (events & TWAI_HAL_EVENT_BUS_OFF) {
    twai_ll_set_mode(hal_ctx->dev,
                     TWAI_MODE_LISTEN_ONLY); // Freeze TEC/REC by entering LOM
#ifdef CONFIG_TWAI_ERRATA_FIX_BUS_OFF_REC
    // Errata workaround: Force REC to 0 by re-triggering bus-off (by setting
    // TEC to 0 then 255)
    twai_ll_set_tec(hal_ctx->dev, 0);
    twai_ll_set_tec(hal_ctx->dev, 255);
    (void)twai_ll_get_and_clear_intrs(
        hal_ctx->dev); // Clear the re-triggered bus-off interrupt
#endif
  }
  if (events & TWAI_HAL_EVENT_BUS_RECOV_CPLT) {
    twai_ll_enter_reset_mode(
        hal_ctx->dev); // Enter reset mode to stop the controller
  }
  if (events & TWAI_HAL_EVENT_BUS_ERR) {
#ifdef CONFIG_TWAI_ERRATA_FIX_RX_FRAME_INVALID
    twai_ll_err_type_t type;
    twai_ll_err_dir_t dir;
    twai_ll_err_seg_t seg;
    twai_ll_parse_err_code_cap(hal_ctx->dev, &type, &dir,
                               &seg); // Decode error interrupt
    // Check for errata condition (RX message has bus error at particular
    // segments)
    if (dir == TWAI_LL_ERR_DIR_RX &&
        ((seg == TWAI_LL_ERR_SEG_DATA || seg == TWAI_LL_ERR_SEG_CRC_SEQ) ||
         (seg == TWAI_LL_ERR_SEG_ACK_DELIM && type == TWAI_LL_ERR_OTHER))) {
      TWAI_HAL_SET_BITS(events, TWAI_HAL_EVENT_NEED_PERIPH_RESET);
    }
#endif
    twai_ll_clear_err_code_cap(hal_ctx->dev);
  }
  if (events & TWAI_HAL_EVENT_ARB_LOST) {
    twai_ll_clear_arb_lost_cap(hal_ctx->dev);
  }
#ifdef CONFIG_TWAI_ERRATA_FIX_RX_FIFO_CORRUPT
  // Check for errata condition (rx_msg_count >= corruption_threshold)
  if (events & TWAI_HAL_EVENT_RX_BUFF_FRAME &&
      twai_ll_get_rx_msg_count(hal_ctx->dev) >= TWAI_RX_FIFO_CORRUPT_THRESH) {
    TWAI_HAL_SET_BITS(events, TWAI_HAL_EVENT_NEED_PERIPH_RESET);
  }
#endif
#if defined(CONFIG_TWAI_ERRATA_FIX_RX_FRAME_INVALID) ||                        \
    defined(CONFIG_TWAI_ERRATA_FIX_RX_FIFO_CORRUPT)
  if (events & TWAI_HAL_EVENT_NEED_PERIPH_RESET) {
    // A peripheral reset will invalidate an RX event;
    TWAI_HAL_CLEAR_BITS(events, (TWAI_HAL_EVENT_RX_BUFF_FRAME));
  }
#endif
  return events;
}

#if defined(CONFIG_TWAI_ERRATA_FIX_RX_FRAME_INVALID) ||                        \
    defined(CONFIG_TWAI_ERRATA_FIX_RX_FIFO_CORRUPT)
void twai_hal_prepare_for_reset_user(twai_hal_context_t *hal_ctx) {
  uint32_t status = twai_ll_get_status(hal_ctx->dev);
  if (!(status &
        TWAI_LL_STATUS_TBS)) { // Transmit buffer is NOT free, indicating an
                               // Ongoing TX will be cancelled by the HW reset
    TWAI_HAL_SET_BITS(hal_ctx->state_flags, TWAI_HAL_STATE_FLAG_TX_NEED_RETRY);
    // Note: Even if the TX completes right after this, we still consider it
    // will be retried. Worst case the same message will get sent twice.
  }
  // Some register must saved before entering reset mode
  hal_ctx->rx_msg_cnt_save = (uint8_t)twai_ll_get_rx_msg_count(hal_ctx->dev);
  twai_ll_enter_reset_mode(
      hal_ctx->dev); // Enter reset mode to stop the controller
  twai_ll_save_reg(
      hal_ctx->dev,
      &hal_ctx->reg_save); // Save remaining registers after entering reset mode
}

void twai_hal_recover_from_reset(twai_hal_context_t *hal_ctx) {
  twai_ll_enter_reset_mode(hal_ctx->dev);
  twai_ll_enable_extended_reg_layout(hal_ctx->dev);
  twai_ll_restore_reg(hal_ctx->dev, &hal_ctx->reg_save);
  twai_ll_exit_reset_mode(hal_ctx->dev);
  (void)twai_ll_get_and_clear_intrs(hal_ctx->dev);

  if (hal_ctx->state_flags & TWAI_HAL_STATE_FLAG_TX_NEED_RETRY) {
    // HW reset has cancelled a TX. Re-transmit here
    twai_hal_set_tx_buffer_and_transmit(hal_ctx, &hal_ctx->tx_frame_save);
    TWAI_HAL_CLEAR_BITS(hal_ctx->state_flags,
                        TWAI_HAL_STATE_FLAG_TX_NEED_RETRY);
  }
}
#endif // defined(CONFIG_TWAI_ERRATA_FIX_RX_FRAME_INVALID) ||
       // defined(CONFIG_TWAI_ERRATA_FIX_RX_FIFO_CORRUPT)

void twai_hal_set_tx_buffer_and_transmit_user(twai_hal_context_t *hal_ctx,
                                              twai_hal_frame_t *tx_frame) {
  // Copy frame into tx buffer
  twai_ll_set_tx_buffer(hal_ctx->dev, tx_frame);
  // Hit the send command
  if (tx_frame->self_reception) {
    if (tx_frame->single_shot) {
      twai_ll_set_cmd_self_rx_single_shot(hal_ctx->dev);
    } else {
      twai_ll_set_cmd_self_rx_request(hal_ctx->dev);
    }
  } else if (tx_frame->single_shot) {
    twai_ll_set_cmd_tx_single_shot(hal_ctx->dev);
  } else {
    twai_ll_set_cmd_tx(hal_ctx->dev);
  }
  TWAI_HAL_SET_BITS(hal_ctx->state_flags, TWAI_HAL_STATE_FLAG_TX_BUFF_OCCUPIED);
#if defined(CONFIG_TWAI_ERRATA_FIX_RX_FRAME_INVALID) ||                        \
    defined(CONFIG_TWAI_ERRATA_FIX_RX_FIFO_CORRUPT)
  // Save transmitted frame in case we need to retry
  memcpy(&hal_ctx->tx_frame_save, tx_frame, sizeof(twai_hal_frame_t));
#endif // defined(CONFIG_TWAI_ERRATA_FIX_RX_FRAME_INVALID) ||
       // defined(CONFIG_TWAI_ERRATA_FIX_RX_FIFO_CORRUPT)
}
