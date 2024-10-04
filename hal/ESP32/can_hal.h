/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 
 TWAI and TWIA_HAL have been modified to work with Levñan by Viktor Sverdlyuk. Access to interrupts is required for operation.
 */


#include "stdint.h"
#include "levcan.h"
#include "soc/soc_caps.h"


#include "freertos/FreeRTOS.h"
#include "esp_types.h"
#include "esp_intr_alloc.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "hal/twai_types.h"
#include "hal/twai_ll.h"
#include "driver/twai.h"




#define TWAI_GENERAL_CONFIG_DEFAULT_user(tx_io_num, rx_io_num, op_mode) {.mode = op_mode, .tx_io = tx_io_num, .rx_io = rx_io_num,        \
                                                                    .clkout_io = TWAI_IO_UNUSED, .bus_off_io = TWAI_IO_UNUSED,      \
                                                                    .tx_queue_len = 5, .rx_queue_len = 5,                           \
                                                                    .alerts_enabled = TWAI_ALERT_NONE,  .clkout_divider = 0,        \
                                                                    .intr_flags = ESP_INTR_FLAG_LEVEL1}



/* ------------------------------ Public API -------------------------------- */

/**
 * @brief   Install TWAI driver
 *
 * This function installs the TWAI driver using three configuration structures.
 * The required memory is allocated and the TWAI driver is placed in the stopped
 * state after running this function.
 *
 * @param[in]   g_config    General configuration structure
 * @param[in]   t_config    Timing configuration structure
 * @param[in]   f_config    Filter configuration structure
 *
 * @note    Macro initializers are available for the configuration structures (see documentation)
 *
 * @note    To reinstall the TWAI driver, call twai_driver_uninstall() first
 *
 * @return
 *      - ESP_OK: Successfully installed TWAI driver
 *      - ESP_ERR_INVALID_ARG: Arguments are invalid
 *      - ESP_ERR_NO_MEM: Insufficient memory
 *      - ESP_ERR_INVALID_STATE: Driver is already installed
 */
esp_err_t twai_driver_install_user(const twai_general_config_t *g_config, const twai_timing_config_t *t_config, const twai_filter_config_t *f_config);

/**
 * @brief   Uninstall the TWAI driver
 *
 * This function uninstalls the TWAI driver, freeing the memory utilized by the
 * driver. This function can only be called when the driver is in the stopped
 * state or the bus-off state.
 *
 * @warning The application must ensure that no tasks are blocked on TX/RX
 *          queues or alerts when this function is called.
 *
 * @return
 *      - ESP_OK: Successfully uninstalled TWAI driver
 *      - ESP_ERR_INVALID_STATE: Driver is not in stopped/bus-off state, or is not installed
 */
esp_err_t twai_driver_uninstall_user(void);

/**
 * @brief   Start the TWAI driver
 *
 * This function starts the TWAI driver, putting the TWAI driver into the running
 * state. This allows the TWAI driver to participate in TWAI bus activities such
 * as transmitting/receiving messages. The TX and RX queue are reset in this function,
 * clearing any messages that are unread or pending transmission. This function
 * can only be called when the TWAI driver is in the stopped state.
 *
 * @return
 *      - ESP_OK: TWAI driver is now running
 *      - ESP_ERR_INVALID_STATE: Driver is not in stopped state, or is not installed
 */
esp_err_t twai_start_user(void);

/**
 * @brief   Stop the TWAI driver
 *
 * This function stops the TWAI driver, preventing any further message from being
 * transmitted or received until twai_start() is called. Any messages in the TX
 * queue are cleared. Any messages in the RX queue should be read by the
 * application after this function is called. This function can only be called
 * when the TWAI driver is in the running state.
 *
 * @warning A message currently being transmitted/received on the TWAI bus will
 *          be ceased immediately. This may lead to other TWAI nodes interpreting
 *          the unfinished message as an error.
 *
 * @return
 *      - ESP_OK: TWAI driver is now Stopped
 *      - ESP_ERR_INVALID_STATE: Driver is not in running state, or is not installed
 */
esp_err_t twai_stop_user(void);

/**
 * @brief   Transmit a TWAI message
 *
 * This function queues a TWAI message for transmission. Transmission will start
 * immediately if no other messages are queued for transmission. If the TX queue
 * is full, this function will block until more space becomes available or until
 * it times out. If the TX queue is disabled (TX queue length = 0 in configuration),
 * this function will return immediately if another message is undergoing
 * transmission. This function can only be called when the TWAI driver is in the
 * running state and cannot be called under Listen Only Mode.
 *
 * @param[in]   message         Message to transmit
 * @param[in]   ticks_to_wait   Number of FreeRTOS ticks to block on the TX queue
 *
 * @note    This function does not guarantee that the transmission is successful.
 *          The TX_SUCCESS/TX_FAILED alert can be enabled to alert the application
 *          upon the success/failure of a transmission.
 *
 * @note    The TX_IDLE alert can be used to alert the application when no other
 *          messages are awaiting transmission.
 *
 * @return
 *      - ESP_OK: Transmission successfully queued/initiated
 *      - ESP_ERR_INVALID_ARG: Arguments are invalid
 *      - ESP_ERR_TIMEOUT: Timed out waiting for space on TX queue
 *      - ESP_FAIL: TX queue is disabled and another message is currently transmitting
 *      - ESP_ERR_INVALID_STATE: TWAI driver is not in running state, or is not installed
 *      - ESP_ERR_NOT_SUPPORTED: Listen Only Mode does not support transmissions
 */
esp_err_t twai_transmit_user(const twai_message_t *message, TickType_t ticks_to_wait);

/**
 * @brief   Receive a TWAI message
 *
 * This function receives a message from the RX queue. The flags field of the
 * message structure will indicate the type of message received. This function
 * will block if there are no messages in the RX queue
 *
 * @param[out]  message         Received message
 * @param[in]   ticks_to_wait   Number of FreeRTOS ticks to block on RX queue
 *
 * @warning The flags field of the received message should be checked to determine
 *          if the received message contains any data bytes.
 *
 * @return
 *      - ESP_OK: Message successfully received from RX queue
 *      - ESP_ERR_TIMEOUT: Timed out waiting for message
 *      - ESP_ERR_INVALID_ARG: Arguments are invalid
 *      - ESP_ERR_INVALID_STATE: TWAI driver is not installed
 */
esp_err_t twai_receive_user(twai_message_t *message, TickType_t ticks_to_wait);

/**
 * @brief   Read TWAI driver alerts
 *
 * This function will read the alerts raised by the TWAI driver. If no alert has
 * been issued when this function is called, this function will block until an alert
 * occurs or until it timeouts.
 *
 * @param[out]  alerts          Bit field of raised alerts (see documentation for alert flags)
 * @param[in]   ticks_to_wait   Number of FreeRTOS ticks to block for alert
 *
 * @note    Multiple alerts can be raised simultaneously. The application should
 *          check for all alerts that have been enabled.
 *
 * @return
 *      - ESP_OK: Alerts read
 *      - ESP_ERR_TIMEOUT: Timed out waiting for alerts
 *      - ESP_ERR_INVALID_ARG: Arguments are invalid
 *      - ESP_ERR_INVALID_STATE: TWAI driver is not installed
 */
esp_err_t twai_read_alerts_user(uint32_t *alerts, TickType_t ticks_to_wait);

/**
 * @brief   Reconfigure which alerts are enabled
 *
 * This function reconfigures which alerts are enabled. If there are alerts
 * which have not been read whilst reconfiguring, this function can read those
 * alerts.
 *
 * @param[in]   alerts_enabled  Bit field of alerts to enable (see documentation for alert flags)
 * @param[out]  current_alerts  Bit field of currently raised alerts. Set to NULL if unused
 *
 * @return
 *      - ESP_OK: Alerts reconfigured
 *      - ESP_ERR_INVALID_STATE: TWAI driver is not installed
 */
esp_err_t twai_reconfigure_alerts_user(uint32_t alerts_enabled, uint32_t *current_alerts);

/**
 * @brief   Start the bus recovery process
 *
 * This function initiates the bus recovery process when the TWAI driver is in
 * the bus-off state. Once initiated, the TWAI driver will enter the recovering
 * state and wait for 128 occurrences of the bus-free signal on the TWAI bus
 * before returning to the stopped state. This function will reset the TX queue,
 * clearing any messages pending transmission.
 *
 * @note    The BUS_RECOVERED alert can be enabled to alert the application when
 *          the bus recovery process completes.
 *
 * @return
 *      - ESP_OK: Bus recovery started
 *      - ESP_ERR_INVALID_STATE: TWAI driver is not in the bus-off state, or is not installed
 */
esp_err_t twai_initiate_recovery_user(void);

/**
 * @brief   Get current status information of the TWAI driver
 *
 * @param[out]  status_info     Status information
 *
 * @return
 *      - ESP_OK: Status information retrieved
 *      - ESP_ERR_INVALID_ARG: Arguments are invalid
 *      - ESP_ERR_INVALID_STATE: TWAI driver is not installed
 */
esp_err_t twai_get_status_info_user(twai_status_info_t *status_info);

/**
 * @brief   Clear the transmit queue
 *
 * This function will clear the transmit queue of all messages.
 *
 * @note    The transmit queue is automatically cleared when twai_stop() or
 *          twai_initiate_recovery() is called.
 *
 * @return
 *      - ESP_OK: Transmit queue cleared
 *      - ESP_ERR_INVALID_STATE: TWAI driver is not installed or TX queue is disabled
 */
esp_err_t twai_clear_transmit_queue_user(void);

/**
 * @brief   Clear the receive queue
 *
 * This function will clear the receive queue of all messages.
 *
 * @note    The receive queue is automatically cleared when twai_start() is
 *          called.
 *
 * @return
 *      - ESP_OK: Transmit queue cleared
 *      - ESP_ERR_INVALID_STATE: TWAI driver is not installed
 */
esp_err_t twai_clear_receive_queue_user(void);

#ifdef __cplusplus
}
#endif

/* ------------------------- Defines and Typedefs --------------------------- */

#define TWAI_HAL_SET_BITS(var, flag)            ((var) |= (flag))
#define TWAI_HAL_CLEAR_BITS(var, flag)          ((var) &= ~(flag))

//HAL state flags
#define TWAI_HAL_STATE_FLAG_RUNNING             (1 << 0)    //Controller is active (not in reset mode)
#define TWAI_HAL_STATE_FLAG_RECOVERING          (1 << 1)    //Bus is undergoing bus recovery
#define TWAI_HAL_STATE_FLAG_ERR_WARN            (1 << 2)    //TEC or REC is >= error warning limit
#define TWAI_HAL_STATE_FLAG_ERR_PASSIVE         (1 << 3)    //TEC or REC is >= 128
#define TWAI_HAL_STATE_FLAG_BUS_OFF             (1 << 4)    //Bus-off due to TEC >= 256
#define TWAI_HAL_STATE_FLAG_TX_BUFF_OCCUPIED    (1 << 5)    //Transmit buffer is occupied
#if defined(CONFIG_TWAI_ERRATA_FIX_RX_FRAME_INVALID) || defined(CONFIG_TWAI_ERRATA_FIX_RX_FIFO_CORRUPT)
#define TWAI_HAL_STATE_FLAG_TX_NEED_RETRY       (1 << 7)    //TX needs to be restarted due to errata workarounds
#endif

//Interrupt Events
#define TWAI_HAL_EVENT_BUS_OFF                  (1 << 0)
#define TWAI_HAL_EVENT_BUS_RECOV_CPLT           (1 << 1)
#define TWAI_HAL_EVENT_BUS_RECOV_PROGRESS       (1 << 2)
#define TWAI_HAL_EVENT_ABOVE_EWL                (1 << 3)
#define TWAI_HAL_EVENT_BELOW_EWL                (1 << 4)
#define TWAI_HAL_EVENT_ERROR_PASSIVE            (1 << 5)
#define TWAI_HAL_EVENT_ERROR_ACTIVE             (1 << 6)
#define TWAI_HAL_EVENT_BUS_ERR                  (1 << 7)
#define TWAI_HAL_EVENT_ARB_LOST                 (1 << 8)
#define TWAI_HAL_EVENT_RX_BUFF_FRAME            (1 << 9)
#define TWAI_HAL_EVENT_TX_BUFF_FREE             (1 << 10)
#if defined(CONFIG_TWAI_ERRATA_FIX_RX_FRAME_INVALID) || defined(CONFIG_TWAI_ERRATA_FIX_RX_FIFO_CORRUPT)
#define TWAI_HAL_EVENT_NEED_PERIPH_RESET        (1 << 11)
#endif

typedef twai_ll_frame_buffer_t twai_hal_frame_t;

typedef struct {
    twai_dev_t *dev;
    uint32_t state_flags;
#if defined(CONFIG_TWAI_ERRATA_FIX_RX_FRAME_INVALID) || defined(CONFIG_TWAI_ERRATA_FIX_RX_FIFO_CORRUPT)
    twai_hal_frame_t tx_frame_save;
    twai_ll_reg_save_t reg_save;
    uint8_t rx_msg_cnt_save;
#endif
} twai_hal_context_t;

/* ---------------------------- Init and Config ----------------------------- */

/**
 * @brief Initialize TWAI peripheral and HAL context
 *
 * Sets HAL context, puts TWAI peripheral into reset mode, then sets some
 * registers with default values.
 *
 * @param hal_ctx Context of the HAL layer
 * @return True if successfully initialized, false otherwise.
 */
bool twai_hal_init_user(twai_hal_context_t *hal_ctx);

/**
 * @brief Deinitialize the TWAI peripheral and HAL context
 *
 * Clears any unhandled interrupts and unsets HAL context
 *
 * @param hal_ctx Context of the HAL layer
 */
void twai_hal_deinit_user(twai_hal_context_t *hal_ctx);

/**
 * @brief Configure the TWAI peripheral
 *
 * @param hal_ctx Context of the HAL layer
 * @param t_config Pointer to timing configuration structure
 * @param f_config Pointer to filter configuration structure
 * @param intr_mask Mask of interrupts to enable
 * @param clkout_divider Clock divider value for CLKOUT. Set to -1 to disable CLKOUT
 */
void twai_hal_configure_user(twai_hal_context_t *hal_ctx, const twai_timing_config_t *t_config, const twai_filter_config_t *f_config, uint32_t intr_mask, uint32_t clkout_divider);

/* -------------------------------- Actions --------------------------------- */

/**
 * @brief Start the TWAI peripheral
 *
 * Start the TWAI peripheral by configuring its operating mode, then exiting
 * reset mode so that the TWAI peripheral can participate in bus activities.
 *
 * @param hal_ctx Context of the HAL layer
 * @param mode Operating mode
 */
void twai_hal_start_user(twai_hal_context_t *hal_ctx, twai_mode_t mode);

/**
 * @brief Stop the TWAI peripheral
 *
 * Stop the TWAI peripheral by entering reset mode to stop any bus activity, then
 * setting the operating mode to Listen Only so that REC is frozen.
 *
 * @param hal_ctx Context of the HAL layer
 */
void twai_hal_stop_user(twai_hal_context_t *hal_ctx);

/**
 * @brief Start bus recovery
 *
 * @param hal_ctx Context of the HAL layer
 */
static inline void twai_hal_start_bus_recovery_user(twai_hal_context_t *hal_ctx)
{
    TWAI_HAL_SET_BITS(hal_ctx->state_flags, TWAI_HAL_STATE_FLAG_RECOVERING);
    twai_ll_exit_reset_mode(hal_ctx->dev);
}

/**
 * @brief Get the value of the TX Error Counter
 *
 * @param hal_ctx Context of the HAL layer
 * @return TX Error Counter Value
 */
static inline uint32_t twai_hal_get_tec_user(twai_hal_context_t *hal_ctx)
{
    return twai_ll_get_tec((hal_ctx)->dev);
}

/**
 * @brief Get the value of the RX Error Counter
 *
 * @param hal_ctx Context of the HAL layer
 * @return RX Error Counter Value
 */
static inline uint32_t twai_hal_get_rec_user(twai_hal_context_t *hal_ctx)
{
    return twai_ll_get_rec((hal_ctx)->dev);
}

/**
 * @brief Get the RX message count register
 *
 * @param hal_ctx Context of the HAL layer
 * @return RX message count
 */
static inline uint32_t twai_hal_get_rx_msg_count_user(twai_hal_context_t *hal_ctx)
{
    return twai_ll_get_rx_msg_count((hal_ctx)->dev);
}

/**
 * @brief Check if the last transmitted frame was successful
 *
 * @param hal_ctx Context of the HAL layer
 * @return True if successful
 */
static inline bool twai_hal_check_last_tx_successful_user(twai_hal_context_t *hal_ctx)
{
    return twai_ll_is_last_tx_successful((hal_ctx)->dev);
}

/**
 * @brief Check if certain HAL state flags are set
 *
 * The HAL will maintain a record of the controller's state via a set of flags.
 * These flags are automatically maintained (i.e., set and reset) inside various
 * HAL function calls. This function checks if certain flags are currently set.
 *
 * @param hal_ctx Context of the HAL layer
 * @param check_flags Bit mask of flags to check
 * @return True if one or more of the flags in check_flags are set
 */

static inline bool twai_hal_check_state_flags_user(twai_hal_context_t *hal_ctx, uint32_t check_flags)
{
    return hal_ctx->state_flags & check_flags;
}

/* ----------------------------- Event Handling ----------------------------- */

/**
 * @brief Get a bit mask of the events that triggered that triggered an interrupt
 *
 * This function should be called at the beginning of an interrupt. This function will do the following:
 * - Read and clear interrupt register
 * - Calculate what events have triggered the interrupt
 * - Respond to low latency interrupt events
 *      - Bus off: Change to LOM to freeze TEC/REC. Errata 1 Fix
 *      - Recovery complete: Enter reset mode
 *      - Clear ECC and ALC so that their interrupts are re-armed
 * - Update HAL state flags based on interrupts that have occurred.
 * - For the ESP32, check for errata conditions. If a HW reset is required, this function
 *   will set the TWAI_HAL_EVENT_NEED_PERIPH_RESET event.
 *
 * @param hal_ctx Context of the HAL layer
 * @return Bit mask of events that have occurred
 */
uint32_t twai_hal_get_events_user(twai_hal_context_t *hal_ctx);

/* ------------------------------- TX and RX -------------------------------- */

/**
 * @brief Format a TWAI Frame
 *
 * This function takes a TWAI message structure (containing ID, DLC, data, and
 * flags) and formats it to match the layout of the TX frame buffer.
 *
 * @param message Pointer to TWAI message
 * @param frame Pointer to empty frame structure
 */
static inline void twai_hal_format_frame_user(const twai_message_t *message, twai_hal_frame_t *frame)
{
    //Direct call to ll function
    twai_ll_format_frame_buffer(message->identifier, message->data_length_code, message->data,
                               message->flags, frame);
}

/**
 * @brief Parse a TWAI Frame
 *
 * This function takes a TWAI frame (in the format of the RX frame buffer) and
 * parses it to a TWAI message (containing ID, DLC, data and flags).
 *
 * @param frame Pointer to frame structure
 * @param message Pointer to empty message structure
 */
static inline void twai_hal_parse_frame_user(twai_hal_frame_t *frame, twai_message_t *message)
{
    //Direct call to ll function
    twai_ll_parse_frame_buffer(frame, &message->identifier, &message->data_length_code,
                              message->data, &message->flags);
}

/**
 * @brief Copy a frame into the TX buffer and transmit
 *
 * This function copies a formatted TX frame into the TX buffer, and the
 * transmit by setting the correct transmit command (e.g. normal, single shot,
 * self RX) in the command register.
 *
 * @param hal_ctx Context of the HAL layer
 * @param tx_frame Pointer to structure containing formatted TX frame
 */
void twai_hal_set_tx_buffer_and_transmit_user(twai_hal_context_t *hal_ctx, twai_hal_frame_t *tx_frame);

/**
 * @brief Copy a frame from the RX buffer and release
 *
 * This function copies a frame from the RX buffer, then release the buffer (so
 * that it loads the next frame in the RX FIFO). False is returned under the
 * following conditions:
 * - On the ESP32S2, false is returned if the RX buffer points to an overrun frame
 * - On the ESP32, false is returned if the RX buffer points to the first overrun
 * frame in the RX FIFO
 *
 * @param hal_ctx Context of the HAL layer
 * @param rx_frame Pointer to structure to store RX frame
 * @return True if a valid frame was copied and released. False if overrun.
 */
static inline bool twai_hal_read_rx_buffer_and_clear_user(twai_hal_context_t *hal_ctx, twai_hal_frame_t *rx_frame)
{
#ifdef SOC_TWAI_SUPPORTS_RX_STATUS
    if (twai_ll_get_status(hal_ctx->dev) & TWAI_LL_STATUS_MS) {
        //Release the buffer for this particular overrun frame
        twai_ll_set_cmd_release_rx_buffer(hal_ctx->dev);
        return false;
    }
#else
    if (twai_ll_get_status(hal_ctx->dev) & TWAI_LL_STATUS_DOS) {
        //No need to release RX buffer as we'll be releaseing all RX frames in continuously later
        return false;
    }
#endif
    twai_ll_get_rx_buffer(hal_ctx->dev, rx_frame);
    twai_ll_set_cmd_release_rx_buffer(hal_ctx->dev);
    return true;
}

#ifndef SOC_TWAI_SUPPORTS_RX_STATUS
/**
 * @brief Clear the RX FIFO of overrun frames
 *
 * This function will clear the RX FIFO of overrun frames. The RX message count
 * will return to 0 after calling this function.
 *
 * @param hal_ctx Context of the HAL layer
 * @return Number of overrun messages cleared from RX FIFO
 */
static inline uint32_t twai_hal_clear_rx_fifo_overrun_user(twai_hal_context_t *hal_ctx)
{
    uint32_t msg_cnt = 0;
    //Note: Need to keep polling th rx message counter incase another message arrives whilst clearing
    while (twai_ll_get_rx_msg_count(hal_ctx->dev) > 0) {
        twai_ll_set_cmd_release_rx_buffer(hal_ctx->dev);
        msg_cnt++;
    }
    //Set a clear data overrun command to clear the data overrun status bit
    twai_ll_set_cmd_clear_data_overrun(hal_ctx->dev);

    return msg_cnt;
}
#endif  //SOC_TWAI_SUPPORTS_RX_STATUS

/* --------------------------- Errata Workarounds --------------------------- */

#if defined(CONFIG_TWAI_ERRATA_FIX_RX_FRAME_INVALID) || defined(CONFIG_TWAI_ERRATA_FIX_RX_FIFO_CORRUPT)
/**
 * @brief Prepare the peripheral for a HW reset
 *
 * Some HW erratas will require the peripheral be reset. This function should be
 * called if twai_hal_get_events() returns the TWAI_HAL_EVENT_NEED_PERIPH_RESET event.
 * Preparing for a reset involves the following:
 * - Checking if a reset will cancel a TX. If so, mark that we need to retry that message after the reset
 * - Save how many RX messages were lost due to this reset
 * - Enter reset mode to stop any the peripheral from receiving any bus activity
 * - Store the regsiter state of the peripheral
 *
 * @param hal_ctx Context of the HAL layer
 */
void twai_hal_prepare_for_reset_user(twai_hal_context_t *hal_ctx);

/**
 * @brief Recover the peripheral after a HW reset
 *
 * This should be called after calling twai_hal_prepare_for_reset() and then
 * executing the HW reset.
 * Recovering the peripheral from a HW reset involves the following:
 * - Restoring the previously saved register state
 * - Exiting reset mode to allow receiving of bus activity
 * - Retrying any TX message that was cancelled by the HW reset
 *
 * @param hal_ctx Context of the HAL layer
 */
void twai_hal_recover_from_reset_user(twai_hal_context_t *hal_ctx);

/**
 * @brief Get how many RX messages were lost due to HW reset
 *
 * @note The number of lost RX messages are saved during twai_hal_prepare_for_reset()
 *
 * @param hal_ctx Context of the HAL layer
 * @return uint32_t Number of RX messages lost due to HW reset
 */
static inline uint32_t twai_hal_get_reset_lost_rx_cnt_user(twai_hal_context_t *hal_ctx)
{
    return hal_ctx->rx_msg_cnt_save;
}
#endif  //defined(CONFIG_TWAI_ERRATA_FIX_RX_FRAME_INVALID) || defined(CONFIG_TWAI_ERRATA_FIX_RX_FIFO_CORRUPT)

#ifdef __cplusplus
}
#endif




#pragma once
//#include "levcan.h"
#ifdef ConnectivityLine
#define CAN_FilterSize 28
#else
#define CAN_FilterSize 14
#endif

#define CAN_ForceEXID
//#define CAN_ForceSTID

#define CAN_FIFO_0		0
#define CAN_FIFO_1		1

typedef union {
	//union
	uint32_t ToUint32;
	//11 bit
	struct {
		uint32_t Transmit11b :1;
		uint32_t Request11b :1;
		uint32_t ExtensionID11b :1;
		uint32_t reserved :18; //just skip those, use next one
		uint32_t STID :11;
	} __attribute__((packed));
	//29 bit
	struct {
		uint32_t Transmit :1;
		uint32_t Request :1;
		uint32_t ExtensionID :1;
		uint32_t EXID :29;
	} __attribute__((packed));
} CAN_IR; //identifier register

enum {
	CAN_Request, CAN_Data
};

typedef enum {
	CANH_Ok, CANH_QueueFull, CANH_QueueEmpty, CANH_Fail
} CAN_Status;

enum {
	CANH_EC_No_Error,
	CANH_EC_Stuff_Error,
	CANH_EC_Form_Error,
	CANH_EC_Acknowledgment_Error,
	CANH_EC_Bit_recessive_Error,
	CANH_EC_Bit_dominant_Error,
	CANH_EC_CRC_Error,
	CANH_EC_Set_by_software
};


extern LC_NodeDescriptor_t *mynode;

void CAN_InitFromClock(uint32_t PCLK, uint32_t bitrate_khz, uint16_t sjw, uint16_t sample_point);
void CAN_Init();
void CAN_Start(void);

void CAN_FiltersClear(void);
void CAN_FilterEditOn(void);
CAN_Status CAN_CreateFilterIndex(CAN_IR reg, uint16_t fifo);
CAN_Status CAN_CreateFilterMask(CAN_IR reg, CAN_IR mask, uint8_t fifo);
CAN_Status CAN_CreateFilterMaskPosition(CAN_IR reg, CAN_IR mask, uint8_t fifo, uint8_t position);
//void CAN_SetupUSBFilters(LC_HeaderPacked_t reg, LC_HeaderPacked_t mask, uint16_t index);
void CAN_FilterEditOff(void);
void Init_LEVCAN(void);
CAN_Status CAN_Send(CAN_IR index, uint32_t *data, uint16_t length);
CAN_Status CAN_Receive(CAN_IR *index, uint32_t *data, uint16_t *length);
CAN_Status CAN_ReceiveN(CAN_IR *index, uint32_t *data, uint16_t *length, int fifo);
//AN_Status CAN_USB_Send(LC_HeaderPacked_t header, uint32_t *data, uint8_t length);

LC_Return_t LC_HAL_CreateFilterMasks(LC_HeaderPacked_t *reg, LC_HeaderPacked_t *mask, uint16_t count);
LC_Return_t LC_HAL_Send(LC_HeaderPacked_t header, uint32_t *data, uint8_t length);
LC_Return_t LC_HAL_TxHalfFull();


