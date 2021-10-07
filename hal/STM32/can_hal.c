/*******************************************************************************
 * LEVCAN: Light Electric Vehicle CAN protocol [LC]
 * Copyright (C) 2020 Vasiliy Sukhoparov
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/

#include <string.h>
#include <math.h>
#include "can_hal.h"

#ifdef  STM32F10X_MD
#include "stm32f10x.h"
#endif
#if  defined(STM32F446xx) || defined(STM32F405xx)
#include "stm32f4xx.h"
#endif
#ifdef  STM32F30X
#include "stm32f30x.h"
#endif
#include "can_hal.h"

// TYPEDEFS
typedef struct {
	LC_HeaderPacked_t Header;
	uint32_t data[2];
	uint8_t length;
} can_packet_t;

enum {
	FIFO0, FIFO1,
};

//EXTERN FUNCTIONS
extern void LC_ReceiveHandler(LC_NodeDescriptor_t *node, LC_HeaderPacked_t header, uint32_t *data, uint8_t length);
#ifdef TRACE
extern int trace_printf(const char *format, ...);
#endif

//PRIVATE FUNCTIONS
uint8_t _getFreeTX();
uint8_t _anyTX();
void receiveIRQ(void);
void transmitIRQ(void);
#ifndef LEVCAN_USE_RTOS_QUEUE
void txFifoProceed(void);
#endif

//PRIVATE VARIABLES
const float accuracy = 1.e-3;	// minimum required accuracy of the bit time
volatile int CAN_ERR = 0;			
#ifdef LEVCAN_USE_RTOS_QUEUE
TaskHandle_t txCantask = 0;
QueueHandle_t txCanqueue = 0;
volatile uint32_t can_tx_cnt = 0;
volatile uint32_t can_rx_cnt = 0;
#else
volatile uint32_t txFIFO_in = 0;
volatile uint32_t txFIFO_out = 0;
volatile can_packet_t txFIFO[LEVCAN_TX_SIZE];
#endif

//EXTERN VARIABLES
void *LEVCAN_Node_Drv;

/// Initialize CAN core using desired freq and bus freq
/// @param PCLK - prescaler input clock
/// @param bitrate_khzn - CAN bus bitrate
/// @param sjw - Synchronization Jump Width, 1..4
/// @param sample_point in %, for default is 87%
void CAN_InitFromClock(uint32_t PCLK, uint32_t bitrate_khz, uint16_t sjw, uint16_t sample_point) {
	uint16_t max_brp = 1024, min_tq = 8, max_tq = 25, max_tq1 = 16, max_sjw = 4; //stm32 specific
	float thisBittime = 1.0f / (bitrate_khz * 1000);
	uint32_t btr = 0;

	if (sjw > max_sjw)
		sjw = max_sjw;
	else if (sjw <= 0)
		sjw = 1;

	//trace_printf("CAN init, clock: %d, bitrate: %dkHz, SJW: %d, sample point: %d%%\n", PCLK, bitrate_khz, sjw, sample_point);
	// for all possible BRP - bit rate prescaler registers do
	for (int brpreg = 0; brpreg < max_brp; brpreg++) {
		int brp = brpreg + 1;
		float t_scl = 1.0f / PCLK * brp;
		float ratio = thisBittime / t_scl; // real ratio
		// get rratio, the integer part of ratio
		int rratio = roundf(ratio);  // to integer rounded ratio
		float accy;	// accuracy

		if (rratio < min_tq)
			break; // end
		if (rratio <= max_tq) {
			accy = rratio - ratio;
			if (fabsf(accy) < accuracy) {
				//round
				if (fabsf(accy) < 1.e-13) {
					accy = 0.0;
				}
				//trace_printf("Accuracy OK: %.4f", accy);

				// ok, found a good divisor
				// set t_seg1 to sample_point (in %)
				/* comes from input */
				int t_seg1 = roundf(rratio * sample_point / 100.f) - 1;
				int t_seg2 = rratio - 1 - t_seg1;
				float real_sp = (t_seg1 + 1) * 100.f / rratio;

				//Do some generic simple tests on the results got so far.
				//sample point at the beginning of a bit makes no sense
				if (real_sp < 50)
					continue;
				if (t_seg1 > max_tq1)
					continue;
				if (t_seg2 < 1)
					continue;

				// -1 for register value
				brp = brp - 1;
				// last part is CAN controller specific
				btr = brp | ((t_seg1 - 1) << 16) | ((t_seg2 - 1) << 20) | ((sjw - 1) << 24);
				// end for an entry with good accuracy
				//trace_printf("Prescaler: %d, ratio: %.2f, rratio: %d, accy: %.4f\n", brp, ratio, rratio, ((float) rratio - ratio));
				//trace_printf("tseg1: %d, tseg2: %d, sp: %.1f%%, brp: %d, tq num: %d \n", t_seg1, t_seg2, real_sp, brp, (1 + t_seg1 + t_seg2));
				break;//done
			}
		}
	}

	if (btr)
		CAN_Init(btr);
	else {
		//trace_printf("Can init failed, no divider found");
#ifdef DEBUG
		__BKPT();
#endif
	}
}

/// Initialize CAN core
/// @param BTR bitrate setting, calculate at http://www.bittiming.can-wiki.info/
void CAN_Init(uint32_t BTR) {
	//mask mode must match FF...F so nothing matches
	CAN1->MCR |= CAN_MCR_RESET;
	CAN1->MCR |= CAN_MCR_INRQ; //init can
	while ((CAN1->MSR & CAN_MSR_INAK) == 0)
		;
	CAN1->MCR &= ~CAN_MCR_SLEEP;

	CAN1->BTR = BTR; //calculated for 1mhz http://www.bittiming.can-wiki.info/
	CAN1->MCR = CAN_MCR_TXFP; // normal mode, transmit by message order

	CAN1->IER = CAN_IER_BOFIE | CAN_IER_EPVIE | CAN_IER_ERRIE | CAN_IER_FMPIE0 /*| CAN_IER_FMPIE1 */| CAN_IER_TMEIE;

#if defined(STM32F405xx) || defined(STM32F446xx)
	NVIC_SetPriority(CAN1_RX0_IRQn, 14);
	NVIC_EnableIRQ(CAN1_RX0_IRQn);
	//NVIC_SetPriority(CAN1_RX1_IRQn, 14);
	//NVIC_EnableIRQ(CAN1_RX1_IRQn);
	NVIC_SetPriority(CAN1_SCE_IRQn, 15);
	NVIC_EnableIRQ(CAN1_SCE_IRQn);
	NVIC_SetPriority(CAN1_TX_IRQn, 15);
	NVIC_EnableIRQ(CAN1_TX_IRQn);
#endif
#if defined(STM32F10X_MD) || defined(STM32F30X)
	NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 2);
	NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
	//NVIC_SetPriority(CAN1_RX1_IRQn, 2);
	//NVIC_EnableIRQ(CAN1_RX1_IRQn);
	NVIC_SetPriority(CAN1_SCE_IRQn, 3);
	NVIC_EnableIRQ(CAN1_SCE_IRQn);
	NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, 3);
	NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);

#endif
	//run CAN
	CAN_Start();
	CAN_FiltersClear();
	
#ifdef LEVCAN_USE_RTOS_QUEUE
	txCanqueue = xQueueCreate(TXQUEUE_SIZE, sizeof(can_packet_t));
	xTaskCreate(txCanTask, "ctx", configMINIMAL_STACK_SIZE, NULL, OS_PRIORITY_HIGH, &txCantask);
#else
	txFIFO_in = 0;
	txFIFO_out = 0;
	memset((void*)&txFIFO, 0, sizeof(txFIFO));
#endif		
}

/// Begin CAN operation
void CAN_Start(void) {
	CAN1->MSR |= CAN_MSR_ERRI; //clear errors
	CAN1->TSR |= CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_ABRQ2;
	CAN1->MCR &= ~CAN_MCR_SLEEP;

	while ((CAN1->MSR & CAN_MSR_SLAK) != 0)
		;
	//TODO add timeout here to catch broken CAN driver
	CAN1->MCR |= CAN_MCR_INRQ; //init can
	while ((CAN1->MSR & CAN_MSR_INAK) == 0)
		;
	CAN1->MCR &= ~CAN_MCR_INRQ;
	while ((CAN1->MSR & CAN_MSR_INAK) != 0)
		;
}

uint32_t FilterActivation;
/// Run this function before filter editing
void CAN_FiltersClear(void) {
	CAN1->FMR = CAN_FMR_FINIT; //init filters
	CAN1->FA1R = 0; //filter all inactive
	CAN1->FM1R = 0; //mask mode for all filters
	CAN1->FS1R = 0; //filter all in 16bit
	CAN1->FFA1R = 0; //filter all to fifo-0
	FilterActivation = 0; //CAN1->FA1R

	uint16_t filter_bank = 0;
	for (; filter_bank < CAN_FilterSize; filter_bank++) {
		CAN1->sFilterRegister[filter_bank].FR1 = 0xFFFFFFFF;
		CAN1->sFilterRegister[filter_bank].FR2 = 0xFFFFFFFF;
	}
	//CAN_CreateFilterMask((CAN_IR ) { .ExtensionID=1 }, (CAN_IR ) { .IdentReg = 0 }, 1);
	CAN1->FMR &= ~CAN_FMR_FINIT;
}

void CAN_FilterEditOn() {
	//CAN1->FA1R = 0; //filter all inactive
	CAN1->FMR = CAN_FMR_FINIT; //init filters
}

/// Filter single message
/// @param index Message index to pass
/// @param fifo Select fifo number
/// @return Returns 1 if something wrong
LC_Return_t CAN_CreateFilterIndex(CAN_IR reg, uint16_t fifo) {

	//get position in filter bank
	uint16_t filter_bank = 0;
	uint16_t relative = 0;
#ifdef CAN_ForceEXID
	reg.ExtensionID = 1;
#endif
#ifdef CAN_ForceSTID
	reg.ExtensionID=0;
#endif
	for (; filter_bank < CAN_FilterSize; filter_bank++) {
		if ((FilterActivation & (1 << filter_bank)) == 0) {
			//bind empty register to new type
			FilterActivation |= (1 << filter_bank);
			//filter size
			CAN1->FS1R = (reg.ExtensionID << filter_bank) | (CAN1->FS1R & ~(1 << filter_bank)); //16b or 32b?
			//FIFO
			CAN1->FFA1R = (fifo << filter_bank) | (CAN1->FFA1R & ~(1 << filter_bank));
			//mode - index
			CAN1->FM1R |= (1 << filter_bank);
			//fill register with invalid value
			CAN1->sFilterRegister[filter_bank].FR1 = 0xFFFFFFFF;
			CAN1->sFilterRegister[filter_bank].FR2 = 0xFFFFFFFF;
			break;
		} else {
			//already active
			relative = 0;
			//check same filtersize, fifo and index mode
			if (((CAN1->FS1R & (1 << filter_bank)) == ((uint32_t) reg.ExtensionID << filter_bank))
					&& ((CAN1->FFA1R & (1 << filter_bank)) == ((uint32_t) fifo << filter_bank)) && ((CAN1->FM1R & (1 << filter_bank)) != 0)) {
				if ((reg.ExtensionID && (CAN1->sFilterRegister[filter_bank].FR2 & 1) == 1))
					//extended index - next reg
					relative = 1;
				else {
					//look for empty filter
					if ((CAN1->sFilterRegister[filter_bank].FR1 & 0xFFFF0000) == 0xFFFF0000)
						relative = 1;
					else if ((CAN1->sFilterRegister[filter_bank].FR2 & 0xFFFF) == 0xFFFF)
						relative = 2;
					else if ((CAN1->sFilterRegister[filter_bank].FR2 & 0xFFFF0000) == 0xFFFF0000)
						relative = 3;
				}
			}
			if (relative)
				break; //found!
		}
	}
	if (filter_bank == CAN_FilterSize)
		return LC_BufferFull;

	if (reg.ExtensionID) {
		if (relative == 0)
			CAN1->sFilterRegister[filter_bank].FR1 = reg.ToUint32 & ~1; //0bit =0
		else
			CAN1->sFilterRegister[filter_bank].FR2 = reg.ToUint32 & ~1;

	} else {
		switch (relative) {
		case 0:
			CAN1->sFilterRegister[filter_bank].FR1 = (reg.STID << 5) | (reg.Request << 4) | (CAN1->sFilterRegister[filter_bank].FR1 & ~0xFFFF);
			break;
		case 1:
			CAN1->sFilterRegister[filter_bank].FR1 = ((reg.STID << 5) | (reg.Request << 4)) << 16 | (CAN1->sFilterRegister[filter_bank].FR1 & 0xFFFF);
			break;
		case 2:
			CAN1->sFilterRegister[filter_bank].FR2 = (reg.STID << 5) | (reg.Request << 4) | (CAN1->sFilterRegister[filter_bank].FR2 & ~0xFFFF);
			break;
		case 3:
			CAN1->sFilterRegister[filter_bank].FR2 = ((reg.STID << 5) | (reg.Request << 4)) << 16 | (CAN1->sFilterRegister[filter_bank].FR2 & 0xFFFF);
			break;
		}
	}

	return LC_Ok;
}

/// Filter messages by mask
/// @param reg Message index to check
/// @param mask Mask filtration for index, 1 - care, 0 - don't care
/// @param fifo Select fifo number
/// @return Returns 1 if something wrong
LC_Return_t CAN_CreateFilterMask(CAN_IR reg, CAN_IR mask, uint8_t fifo) {
#ifdef CAN_ForceEXID
	reg.ExtensionID = 1;
	mask.ExtensionID = 1;
#endif
#ifdef CAN_ForceSTID
	reg.ExtensionID=0;
	mask.ExtensionID=1;
#endif
//get position in filter bank
	uint8_t filter_bank = 0;
	for (; filter_bank < CAN_FilterSize; filter_bank++) {

		if ((FilterActivation & (1 << filter_bank)) == 0) {
			//bind empty register to new type
			FilterActivation |= (1 << filter_bank);
			//filter size
			CAN1->FS1R = (reg.ExtensionID << filter_bank) | (CAN1->FS1R & ~(1 << filter_bank)); //16b or 32b?
			//FIFO
			CAN1->FFA1R = (fifo << filter_bank) | (CAN1->FFA1R & ~(1 << filter_bank));
			//mode - mask
			CAN1->FM1R &= ~(1 << filter_bank);
			//fill register with invalid value
			if (reg.ExtensionID) { //only one here
				CAN1->sFilterRegister[filter_bank].FR1 = reg.ToUint32 & ~1; //must match impossible
				CAN1->sFilterRegister[filter_bank].FR2 = mask.ToUint32 & ~1; //must match impossible
			} else {
				CAN1->sFilterRegister[filter_bank].FR1 = ((reg.STID << 5) | (reg.Request << 4)) | (((mask.STID << 5) | (mask.Request << 4)) << 16);
				CAN1->sFilterRegister[filter_bank].FR2 = 0xFFFFFFFF; //must match impossible
			}
			break;
		} else {
			//check same filtersize, fifo and index mode
			if (((CAN1->FS1R & (1 << filter_bank)) == ((uint32_t) reg.ExtensionID << filter_bank))
					&& ((CAN1->FFA1R & (1 << filter_bank)) == ((uint32_t) fifo << filter_bank)) && ((CAN1->FM1R & (1 << filter_bank)) == 0)) {
				if (!reg.ExtensionID && CAN1->sFilterRegister[filter_bank].FR2 == 0xFFFFFFFF) {
					//extended only one filter
					CAN1->sFilterRegister[filter_bank].FR2 = ((reg.STID << 5) | (reg.Request << 4)) | (((mask.STID << 5) | (mask.Request << 4)) << 16);
					break;
				}
			}
		}
	}
	if (filter_bank == CAN_FilterSize)
		return LC_BufferFull;

	return LC_Ok;
}
///  Filter messages by mask at specified position in filter bank
/// @param reg Message index to check
/// @param mask Mask filtration for index, 1 - care, 0 - don't care
/// @param fifo Select fifo number
/// @param position Filter bank position
/// @return Returns 1 if something wrong
LC_Return_t CAN_CreateFilterMaskPosition(CAN_IR reg, CAN_IR mask, uint8_t fifo, uint8_t position) {
#ifdef CAN_ForceEXID
	reg.ExtensionID = 1;
	mask.ExtensionID = 1;
#endif
#ifdef CAN_ForceSTID
	reg.ExtensionID=0;
	mask.ExtensionID=1;
#endif
	if (position >= CAN_FilterSize)
		return LC_BufferFull;
	//get position in filter bank
	uint8_t filter_bank = position;

	//force active mode
	FilterActivation |= (1 << filter_bank);
	//filter size
	CAN1->FS1R = (reg.ExtensionID << filter_bank) | (CAN1->FS1R & ~(1 << filter_bank)); //16b or 32b?
	//FIFO
	CAN1->FFA1R = (fifo << filter_bank) | (CAN1->FFA1R & ~(1 << filter_bank));
	//mode - mask
	CAN1->FM1R &= ~(1 << filter_bank);
	//fill register with invalid value
	if (reg.ExtensionID) { //only one here
		CAN1->sFilterRegister[filter_bank].FR1 = reg.ToUint32 & ~1; //must match impossible
		CAN1->sFilterRegister[filter_bank].FR2 = mask.ToUint32 & ~1; //must match impossible
	} else {
		CAN1->sFilterRegister[filter_bank].FR1 = ((reg.STID << 5) | (reg.Request << 4)) | (((mask.STID << 5) | (mask.Request << 4)) << 16);
		CAN1->sFilterRegister[filter_bank].FR2 = 0xFFFFFFFF; //must match impossible
	}

	return LC_Ok;
}

void CAN_FilterEditOff() {
	CAN1->FA1R = FilterActivation;
	CAN1->FMR &= ~CAN_FMR_FINIT;
}

LC_Return_t CAN_Send(CAN_IR index, uint32_t *data, uint16_t length) {
	uint8_t txBox;
#ifdef CAN_ForceEXID
	index.ExtensionID = 1;
#endif
#ifdef CAN_ForceSTID
	index.ExtensionID = 0;
#endif
	if (length > 8)
		return LC_DataError;

	txBox = _getFreeTX();
	if (txBox > 2)
		return LC_BufferFull;

	CAN1->sTxMailBox[txBox].TDTR = length; //data length
	CAN1->sTxMailBox[txBox].TDLR = data[0];
	CAN1->sTxMailBox[txBox].TDHR = data[1];
	CAN1->sTxMailBox[txBox].TIR = index.ToUint32 | 1; //transmit

	return LC_Ok;
}

LC_Return_t CAN_Receive(CAN_IR *index, uint32_t *data, uint16_t *length) {
	LC_Return_t status = LC_BufferEmpty;
	//check is there something
	uint16_t fifo = 0;
	while (status == LC_BufferEmpty) {
		if ((CAN1->RF0R & CAN_RF0R_FMP0) == 0) {
			if ((CAN1->RF1R & CAN_RF1R_FMP1) == 0)
				return LC_BufferEmpty; //nothing
			else {
				fifo = 1;
				status = LC_Ok; //we found a valid message! yaay!
			}
		} else {
			fifo = 0;
			status = LC_Ok; //we found a valid message! yaay!
		}
	}

	index->ToUint32 = CAN1->sFIFOMailBox[fifo].RIR; //get index
	*length = CAN1->sFIFOMailBox[fifo].RDTR & CAN_RDT0R_DLC;
	//don't care if it is 8 byte or 1, you should read only what needed
	if (data) {
		data[0] = CAN1->sFIFOMailBox[fifo].RDLR;
		data[1] = CAN1->sFIFOMailBox[fifo].RDHR;
	}
	if (fifo == 0)
		CAN1->RF0R = CAN_RF0R_RFOM0;
	else
		CAN1->RF1R = CAN_RF1R_RFOM1;
	return status;
}

LC_Return_t CAN_ReceiveN(CAN_IR *index, uint32_t *data, uint16_t *length, int fifo) {
	LC_Return_t status = LC_BufferEmpty;

	if (fifo == 0) {
		if ((CAN1->RF0R & CAN_RF0R_FMP0) == 0) {
			return LC_BufferEmpty; //nothing
		}
	} else if (fifo == 1) {
		if ((CAN1->RF1R & CAN_RF1R_FMP1) == 0) {
			return LC_BufferEmpty; //nothing
		}
	} else {
		return LC_BufferEmpty; //nothing
	}
	status = LC_Ok;

	index->ToUint32 = CAN1->sFIFOMailBox[fifo].RIR; //get index
	*length = CAN1->sFIFOMailBox[fifo].RDTR & CAN_RDT0R_DLC;
	//don't care if it is 8 byte or 1, you should read only what needed
	if (data) {
		data[0] = CAN1->sFIFOMailBox[fifo].RDLR;
		data[1] = CAN1->sFIFOMailBox[fifo].RDHR;
	}
	if (fifo == 0)
		CAN1->RF0R = CAN_RF0R_RFOM0;
	else
		CAN1->RF1R = CAN_RF1R_RFOM1;
	return status;
}

//Returns free transmission buffer
uint8_t _getFreeTX() {

	uint8_t txBox;
	for (txBox = 0; txBox < 3; txBox++) { //look for free transmit mailbox
		if ((CAN1->TSR & (CAN_TSR_TME0 << txBox)) != 0) {
			break;
		}
	}
	return txBox;
}

uint8_t _anyTX() {
	//invert empty bits and shit it to beginning
	return ((~(CAN1->TSR)) & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) >> 26;
}

void CAN1_SCE_IRQHandler(void) {
	CAN1->MSR |= CAN_MSR_ERRI;
	CAN_ERR = 1;
	//CAN_Start();
	//trace_printf("CAN bus RESET, error has occurred");
}

LC_Return_t LC_HAL_Receive(LC_HeaderPacked_t *header, uint32_t *data, uint8_t *length) {
	//Small hal overlay for converting LC header packed to CAN specific data
	CAN_IR rindex;
	uint16_t rlength;
	uint8_t state = CAN_Receive(&rindex, data, &rlength);
	if (state == LC_Ok) {
		header->ToUint32 = rindex.EXID; //29b
		header->Request = rindex.Request; //30b
		*length = rlength;
		return LC_Ok;
	}
	return LC_BufferEmpty;
}

LC_Return_t LC_HAL_Send(LC_HeaderPacked_t header, uint32_t *data, uint8_t length) {
	uint8_t state = 0;
	can_packet_t packet;
	packet.Header = header;
	packet.data[0] = data[0];
	packet.data[1] = data[1];
	packet.length = length;
#ifdef LEVCAN_USE_RTOS_QUEUE
	uint8_t state = (xQueueSend(txCanqueue, &packet, 0) == pdTRUE) ? LC_Ok : LC_BufferFull;
#else
	lc_disable_irq(); //critical section
	if (txFIFO_in == ((txFIFO_out - 1 + LEVCAN_TX_SIZE) % LEVCAN_TX_SIZE)) {
		state = LC_BufferFull;
	} else {
		txFIFO[txFIFO_in] = packet;
		txFIFO_in = (txFIFO_in + 1) % LEVCAN_TX_SIZE;
		state = LC_Ok;
		//uint8_t state = CAN_Send(txmsg, data, length);
	}
	//check for no transmission happening
	if (_anyTX() == 0) {
		txFifoProceed(); //critical section
	}
	lc_enable_irq();

#endif
	return (state == LC_Ok) ? LC_Ok : LC_BufferFull;
}

LC_Return_t LC_HAL_CreateFilterMasks(LC_HeaderPacked_t *reg, LC_HeaderPacked_t *mask, uint16_t count) {
	
	CAN_FilterEditOn();

	CAN_IR can_reg, can_mask;

	for (int i = 0; i < count; i++) {
		can_reg.ToUint32 = 0;
		can_reg.ExtensionID = 1; //force 29b
		can_mask.ToUint32 = can_reg.ToUint32;
		//convert data
		can_reg.EXID = reg[i].ToUint32;
		can_reg.Request = reg[i].Request;
		can_mask.EXID = mask[i].ToUint32;
		can_mask.Request = mask[i].Request;
		CAN_CreateFilterMaskPosition(can_reg, can_mask, 0, i);
	}

	CAN_FilterEditOff();
	return LC_Ok;
}

#if defined(STM32F405xx) || defined(STM32F446xx)
void CAN1_RX0_IRQHandler(void) {
	receiveIRQ();
}
/*void CAN1_RX1_IRQHandler(void) {
	receiveIRQ();
 }*/
void CAN1_TX_IRQHandler(void) {
	transmitIRQ();
}
#endif

#if defined(STM32F10X_MD) || defined(STM32F30X)
void USB_LP_CAN1_RX0_IRQHandler(void) {
	 receiveIRQ() ;
}
void USB_HP_CAN1_TX_IRQHandler(void) {
	transmitIRQ();
}
#endif
#ifdef LEVCAN_USE_RTOS_QUEUE
void txCanTask(void *pvParameters) {
	(void) pvParameters;

	static can_packet_t packet_from_q = { 0 };
	for (;;) {
		if (xQueueReceive(txCanqueue, &packet_from_q, portMAX_DELAY ) == pdTRUE) {
			CAN_IR sindex;
			sindex.EXID = packet_from_q.Header.ToUint32;
			sindex.ExtensionID = 1;
			sindex.Request = packet_from_q.Header.Request;
			//counter
			can_tx_cnt++;
			//try to send till its actually sent
			int i = 0;

			while (CAN_Send(sindex, packet_from_q.data, packet_from_q.length) != LC_Ok) {
				if (i > 3)
					break; //cant CAN send :P
				//cant send, wait for TX empty
				ulTaskNotifyTake(pdTRUE, 250); // 0.25s wait
				i++;
			}
		}
	}
}

#endif

LC_Return_t LC_HAL_TxHalfFull() {
#ifdef LEVCAN_USE_RTOS_QUEUE
	int size = uxQueueMessagesWaiting(txCanqueue);

	if (size * 4 < TXQUEUE_SIZE * 3)
		return LC_BufferEmpty;
	else
		return LC_BufferFull;
#else
	int32_t stored = 0;
	if (txFIFO_in >= txFIFO_out)
		stored = txFIFO_in - txFIFO_out;
	else
		stored = txFIFO_in + (LEVCAN_TX_SIZE - txFIFO_out);

	if (stored > LEVCAN_TX_SIZE / 2)
		return LC_BufferFull;
	else
		return LC_BufferEmpty;
#endif

}

void transmitIRQ(void) {
	//remove interrupts
#ifdef LEVCAN_USE_RTOS_QUEUE
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(txCantask, &xHigherPriorityTaskWoken);

	CAN1->TSR |= CAN_TSR_TXOK0 | CAN_TSR_TXOK1 | CAN_TSR_TXOK2;

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
#else
	txFifoProceed();
	CAN1->TSR |= CAN_TSR_TXOK0 | CAN_TSR_TXOK1 | CAN_TSR_TXOK2;
#endif
}

void receiveIRQ(void) {
	CAN_IR rindex;
	uint16_t rlength;
	uint32_t data[2];
	uint8_t state = CAN_ReceiveN(&rindex, data, &rlength, FIFO0);
	//receive everything
	for (; state == LC_Ok; state = CAN_ReceiveN(&rindex, data, &rlength, FIFO0)) {
		LC_HeaderPacked_t header = { 0 };
		header.ToUint32 = rindex.EXID; //29b
		header.Request = rindex.Request; //30b
				
		LC_ReceiveHandler(LEVCAN_Node_Drv, header, data, rlength);
	}
}

#ifndef LEVCAN_USE_RTOS_QUEUE
void txFifoProceed(void) {
	while (1) {
		if (txFIFO_in == txFIFO_out)
			break; /* Queue Empty - nothing to send*/

		CAN_IR sindex;
		sindex.EXID = txFIFO[txFIFO_out].Header.ToUint32;
		sindex.ExtensionID = 1;
		sindex.Request = txFIFO[txFIFO_out].Header.Request;

		if (CAN_Send(sindex, (void*)&txFIFO[txFIFO_out].data, txFIFO[txFIFO_out].length) != LC_Ok)
			break; //CAN full
		txFIFO_out = (txFIFO_out + 1) % LEVCAN_TX_SIZE; //successful sent
	}
}
#endif
