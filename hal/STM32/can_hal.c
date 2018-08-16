#include "can_hal.h"
#include <string.h>
#include <math.h>

#ifdef  STM32F10X_MD
#include "stm32f10x.h"
#endif
#if  defined(STM32F446xx) || defined(STM32F405xx)
#include "stm32f4xx.h"
#endif
#ifdef  STM32F30X
#include "stm32f30x.h"
#endif

const float accuracy = 1.e-3;	// minimum required accuracy of the bit time
uint8_t _getFreeTX();
extern void LC_ReceiveHandler(void);
#ifdef TRACE
extern int trace_printf(const char* format, ...);
#endif
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
				break; //done
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
}

/// Begin CAN operation
void CAN_Start(void) {
	CAN1->MSR |= CAN_MSR_ERRI; //clear errors
	CAN1->TSR |= CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_ABRQ2;
	CAN1->MCR &= ~CAN_MCR_SLEEP;

	while ((CAN1->MSR & CAN_MSR_SLAK) != 0)
		;
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
/// @param RTR Request or data?
/// @return Returns 1 if something wrong
CAN_Status CAN_CreateFilterIndex(CAN_IR reg, uint16_t fifo) {

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
			if (((CAN1->FS1R & (1 << filter_bank)) == (reg.ExtensionID << filter_bank)) && ((CAN1->FFA1R & (1 << filter_bank)) == (fifo << filter_bank))
					&& ((CAN1->FM1R & (1 << filter_bank)) != 0)) {
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
		return CANH_QueueFull;

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

	return CANH_Ok;
}

/// Filter messages by mask
/// @param index Message index to pass
/// @param mask Mask filtration for index, 1 - care, 0 - don't care
/// @param RTR Request (1) or data (0)?
/// @return Returns 1 if something wrong
CAN_Status CAN_CreateFilterMask(CAN_IR reg, CAN_IR mask, uint8_t fifo) {
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
			if (((CAN1->FS1R & (1 << filter_bank)) == (reg.ExtensionID << filter_bank)) && ((CAN1->FFA1R & (1 << filter_bank)) == (fifo << filter_bank))
					&& ((CAN1->FM1R & (1 << filter_bank)) == 0)) {
				if (!reg.ExtensionID && CAN1->sFilterRegister[filter_bank].FR2 == 0xFFFFFFFF) {
					//extended only one filter
					CAN1->sFilterRegister[filter_bank].FR2 = ((reg.STID << 5) | (reg.Request << 4)) | (((mask.STID << 5) | (mask.Request << 4)) << 16);
					break;
				}
			}
		}
	}
	if (filter_bank == CAN_FilterSize)
		return CANH_QueueFull;

	return CANH_Ok;
}

void CAN_FilterEditOff() {
	CAN1->FA1R = FilterActivation;
	CAN1->FMR &= ~CAN_FMR_FINIT;
}

CAN_Status CAN_Send(uint32_t index32, uint32_t* data, uint16_t length) {
	CAN_IR index = { .ToUint32 = index32 };
	uint8_t txBox;
#ifdef CAN_ForceEXID
	index.ExtensionID = 1;
#endif
#ifdef CAN_ForceSTID
	index.ExtensionID=0;
#endif
	//this is request? forward it to other function.
//	if (index.Request)
//		return CAN_SendIndex(index32);

	if (length > 8)
		return CANH_Fail;

	txBox = _getFreeTX();
	if (txBox > 2)
		return CANH_QueueFull;

	CAN1->sTxMailBox[txBox].TDTR = length; //data length
	CAN1->sTxMailBox[txBox].TDLR = data[0];
	CAN1->sTxMailBox[txBox].TDHR = data[1];
	CAN1->sTxMailBox[txBox].TIR = index.ToUint32 | 1; //transmit

	return CANH_Ok;
}

CAN_Status CAN_Receive(uint32_t* index32, uint32_t* data, uint16_t* length) {
	CAN_Status status = CANH_QueueEmpty;
//check is there something
	uint16_t fifo = 0;
	while (status == CANH_QueueEmpty) {
		if ((CAN1->RF0R & CAN_RF0R_FMP0) == 0) {
			if ((CAN1->RF1R & CAN_RF1R_FMP1) == 0)
				return CANH_QueueEmpty; //nothing
			else {
				fifo = 1;
				status = CANH_Ok; //we found a valid message! yaay!
			}
		} else {
			fifo = 0;
			status = CANH_Ok; //we found a valid message! yaay!
		}
	}

	*index32 = CAN1->sFIFOMailBox[fifo].RIR; //get index
	*length = CAN1->sFIFOMailBox[fifo].RDTR & CAN_RDT0R_DLC;
//don't care if it is 8 byte or 1, you should read only what needed
	if (data) {
		data[0] = CAN1->sFIFOMailBox[fifo].RDLR;
		data[1] = CAN1->sFIFOMailBox[fifo].RDHR;
	}
	CAN1->RF0R = CAN_RF0R_RFOM0;
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

void CAN1_SCE_IRQHandler(void) {
	CAN1->MSR |= CAN_MSR_ERRI;
	CAN_Start();
	//RuntimeData.Flags.CANErr = 1;
	//trace_printf("CAN bus RESET, error has occurred");
}
void CAN1_RX1_IRQHandler(void) {
	LC_ReceiveHandler();

}

#if defined(STM32F405xx) || defined(STM32F446xx)
void CAN1_RX0_IRQHandler(void) {
	LC_ReceiveHandler();
}
void CAN1_TX_IRQHandler(void) {
	//remove interrupts
	LC_TransmitHandler();

	CAN1->TSR |= CAN_TSR_TXOK0 | CAN_TSR_TXOK1 | CAN_TSR_TXOK2;
}

#endif

#if defined(STM32F10X_MD) || defined(STM32F30X)
void USB_LP_CAN1_RX0_IRQHandler(void) {
	LC_ReceiveHandler();
}
void USB_HP_CAN1_TX_IRQHandler(void) {
	//remove interrupts
	LC_TransmitHandler();
	CAN1->TSR |= CAN_TSR_TXOK0 | CAN_TSR_TXOK1 | CAN_TSR_TXOK2;
}
#endif

