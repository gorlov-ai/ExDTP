/*
 * DataTransferProt.c
 *
 *  Created on: 26 ����. 2016 �.
 *      Author: �����
 */

#include "DataTransferProt.h"
#include <string.h>
#include <stdlib.h>

#define CON_SINC_STAT_SEARCH		0
#define CON_SINC_STAT_CONNECTED		1
#define CON_SINC_STAT_RESYNC		3

#define COM_ACK_CODE_MASC			0x00FF
#define COM_ACK_CODE_NON			0x0000
#define COM_ACK_CODE_ACCEPTED		0x0001	// ������������ ��� ������ ������ (� ����� ������ ���� ���� ������)
#define COM_ACK_CODE_INVALID		0x0002	// ������������ ���� �������� ���� ���������
#define COM_ACK_CODE_BUSY			0x0004	// ������������ ���� ����� �� ����� ������� ������
#define COM_ACK_CODE_FR_LOST		0x0008	// ������������ ���� ��������� ������ ������
#define COM_ACK_CODE_RESYNC			0x0010	// ������������ ���� ������� ����� ���������������


#define RX_SHFR_BUF_NOT_EMPTY		0x01	// ����� ������ �������� ������ �� ����

#define TXRX_FLAG_RX_BUF_NOT_EMPTY	0x00000001	// �������� ����� ������ �� ������
#define TXRX_FLAG_TX_BUF_BLOKED		0x00010000	// ���� ����� ��� �������� ������ ������ ������������ �� ������� ��������� ������������� � ������
#define TXRX_FLAG_FRAME_RESYNC_F	0x10001000	// ��� ������ ������ �� ���������������

typedef enum
{
  DTP_PORT_STATE_RESET             = 0x00U,    /*!< Peripheral is not yet Initialized
                                                   Value is allowed for gState and RxState */
  DTP_PORT_STATE_READY             = 0x20U,    /*!< Peripheral Initialized and ready for use
                                                   Value is allowed for gState and RxState */
  DTP_PORT_STATE_BUSY              = 0x04U,    /*!< an internal process is ongoing
                                                   Value is allowed for gState only */
  DTP_PORT_STATE_BUSY_TX           = 0x01U,    /*!< Data Transmission process is ongoing
                                                   Value is allowed for gState only */
  DTP_PORT_STATE_BUSY_RX           = 0x02U,    /*!< Data Reception process is ongoing
                                                   Value is allowed for RxState only */
  DTP_PORT_STATE_BUSY_TX_RX        = 0x03U,    /*!< Data Transmission and Reception process is ongoing
                                                   Not to be used for neither gState nor RxState.
                                                   Value is result of combination (Or) between gState and RxState values */
  DTP_PORT_STATE_TIMEOUT           = 0x80U,    /*!< Timeout state
                                                   Value is allowed for gState only */
  DTP_PORT_STATE_ERROR             = 0x40U     /*!< Error
                                                   Value is allowed for gState only */
}DTP_PORT_StateTypeDef;

const uint8_t startSq[5] = {0xAA, 0xAF, 0x55, 0x00, 0xAA};
const uint8_t commSq[2] = {0xAA, 0x55};

// ��������� ���������� ����������
uint8_t DTP_InitFlag = 0;
uint8_t conSincStat;
DTP_PORT_StateTypeDef commState;

uint8_t reciverShFrBuf[16];
uint8_t reciverShFrBufFlag;

uint16_t rxtxBufMaxLen;
uint8_t* reciverBuf;
uint16_t reciverBufLen;
uint8_t* transmitBuf;
uint16_t transmitBufLen;
uint32_t rxtxBufFlags;

uint8_t* WriteSmBuf;
uint16_t WriteSmBufLen;
uint16_t WriteSmBufPos;
uint8_t* ReadSmBuf;
uint16_t ReadSmBufLen;
uint16_t ReadSmBufPos;

int tmp;
uint32_t crc32, crc32ret, tOutTicks;
uint32_t txPeriodTicks;
uint32_t rxframeLen, txframeLen, rxframeFlags;
uint16_t rxframeID, rxframeAckCode; // rxframeAckCode - ����������� �� ��������� ��������� ������ ���� �������
uint16_t txframeID, txframeAckCode;	// txframeAckCode - ����������� �� ���������� ����� (��������� �� ��������� ��������� ������ � ������ �������)
uint16_t rxframeIDPrev;

uint8_t rxFailFrameCntr;
uint8_t rxTrueResyncFrCntr;

uint32_t (*CRC32CalcFunc)(uint32_t pBuffer[], uint32_t BufferLength);
void (*DTP_Delay)(uint32_t period);
uint32_t (*DTP_GetSysTicks)(void);
uint8_t (*DTP_ReceiveBuf)(uint8_t *pRxData, uint16_t Size);
uint8_t (*DTP_TransmitBuf)(uint8_t *pRxData, uint16_t Size);
void (*DTP_CommAbort)(void);
void (*DTP_RxTxBufCommEnd)(uint8_t);

// ��������� ���������� �������
inline void DTP_SearchAdnCon(void);
uint32_t DTP_CalcCRC32(uint32_t pBuffer[], uint32_t BufferLength);
void DTP_DelayInternal(uint32_t period);

//////////// ����������� ���������� ������� ////////////////
int DTP_Init(DTP_HandlerTDef* initHandler)
{
	if(DTP_InitFlag != 0) return 1;

	if(initHandler->rxtxBufLen_W < (32 / 4)) return -1;	// ���� ������ ������ ������ 32 ����
	if((initHandler->pReciverBuf == NULL) || (initHandler->pTransmitBuf == NULL)) return -1;
	if(initHandler->GetSysTicks == NULL) return -1;
	if((initHandler->ReceiveBuf == NULL) || (initHandler->TransmitBuf == NULL) || (initHandler->CommAbort == NULL)) return -1;

	reciverBuf = (uint8_t*)initHandler->pReciverBuf;
	transmitBuf = (uint8_t*)initHandler->pTransmitBuf;
	rxtxBufMaxLen = (initHandler->rxtxBufLen_W * 4);

	if(initHandler->CRC32CalcFunc != NULL)	// �������� ����������� CRC32
		CRC32CalcFunc = initHandler->CRC32CalcFunc;	// �������
	else
		CRC32CalcFunc = DTP_CalcCRC32;				// ����������

	if(initHandler->Delay != NULL)	// �������� ������� ��������
		DTP_Delay = initHandler->Delay;
	else
		DTP_Delay = DTP_DelayInternal;

	if(initHandler->RxTxBufCommEnd != NULL)	// �������� ������� ���������� ���������� ������/�������� ��� ��������/����������� �����
		DTP_RxTxBufCommEnd = initHandler->RxTxBufCommEnd;
	else
		DTP_RxTxBufCommEnd = NULL;

	DTP_GetSysTicks = initHandler->GetSysTicks;	// �������� ������� ������� �������� ����� � ����
	DTP_ReceiveBuf = initHandler->ReceiveBuf;
	DTP_TransmitBuf = initHandler->TransmitBuf;
	DTP_CommAbort = initHandler->CommAbort;

	memset(reciverBuf, 0, rxtxBufMaxLen);
	memset(transmitBuf, 0, rxtxBufMaxLen);
	reciverBufLen = 0;
	transmitBufLen = 0;
	conSincStat = CON_SINC_STAT_SEARCH;
	commState = DTP_PORT_STATE_RESET;
	reciverShFrBufFlag = 0;
	WriteSmBuf = NULL;
	WriteSmBufLen = 0;
	WriteSmBufPos = 0;
	ReadSmBuf = NULL;
	ReadSmBufLen = 0;
	ReadSmBufPos = 0;
	rxtxBufFlags = 0;
	rxframeLen = txframeLen = rxframeFlags = 0;
	rxframeID = 0;
	rxframeAckCode = 0;
	txframeID = 1;
	txframeAckCode = 0;
	rxframeIDPrev = 0;
	rxFailFrameCntr = rxTrueResyncFrCntr = 0;

//	osThreadTerminate(procRxTxTaskHandle);
//	osThreadDef(RxTxPr, DTP_RxTxProc, osPriorityHigh, 0, 256);
//	procRxTxTaskHandle = osThreadCreate(osThread(RxTxPr), NULL);
//	if(procRxTxTaskHandle == NULL)
//	{
//		osThreadTerminate(procRxTxTaskHandle);
//		return HAL_ERROR;
//	}

	DTP_InitFlag = 1;
	return 0;
}

/*
 * ��� ������� ���������� ������������ �������� ��� ������ ������� ������ �������
 */
void DTP_RxTxProc(void)
{
	if(DTP_InitFlag != 0)
	{
		switch(conSincStat)
		{
		case CON_SINC_STAT_SEARCH:		// ����� ������ ������������
			DTP_SearchAdnCon();
			if(conSincStat == CON_SINC_STAT_CONNECTED)
			{
				reciverBufLen = 0;
				transmitBufLen = 0;
				conSincStat = CON_SINC_STAT_SEARCH;
				commState = DTP_PORT_STATE_RESET;
				reciverShFrBufFlag = 0;
				WriteSmBuf = NULL;
				WriteSmBufLen = 0;
				WriteSmBufPos = 0;
				ReadSmBuf = NULL;
				ReadSmBufLen = 0;
				ReadSmBufPos = 0;
				rxtxBufFlags = 0;
				rxframeLen = txframeLen = rxframeFlags = 0;
				rxframeID = 0;
				rxframeAckCode = 0;
				txframeAckCode = 0;
				rxFailFrameCntr = rxTrueResyncFrCntr = 0;
				txframeID = 1;	// ������ ���
				rxframeIDPrev = 0;	// ������ ���
			}
			break;


		case CON_SINC_STAT_CONNECTED:	// ����� ������ �������
			///////////////// �������� ������ ������ ////////////////////
			if((commState & DTP_PORT_STATE_ERROR) != 0)
			{
				if(WriteSmBuf != NULL)
				{
					WriteSmBufPos = 0;
					WriteSmBufLen = 0;
					WriteSmBuf = NULL;
				}
				if(ReadSmBuf != NULL)
				{
					ReadSmBufPos = 0;
					ReadSmBufLen = 0;
					ReadSmBuf = NULL;
				}
				break;
			}

			///////////////// ��������� ������ ������ ////////////////////
			if(((commState & DTP_PORT_STATE_BUSY_RX) != DTP_PORT_STATE_BUSY_RX) && ((rxframeAckCode & COM_ACK_CODE_BUSY) == 0))
			{	// ���� ����� �������� � ����� ������ ������ ����� � ������ - �������� ����� �����
				commState |= DTP_PORT_STATE_BUSY_RX;
				rxtxBufFlags &= ~TXRX_FLAG_RX_BUF_NOT_EMPTY;
				while(DTP_ReceiveBuf(reciverBuf, reciverBufLen + 4) != 0); // �������� ����� ������ ����� ������ + CRC32
				tOutTicks = DTP_GetSysTicks() + 5000;
				rxframeLen = 0;
			}
			else if((commState & DTP_PORT_STATE_BUSY_RX) != DTP_PORT_STATE_BUSY_RX) // ���� ����� ������ ������ ����� � ����� ��� ��������
			{	// �������� ����� ��������
				if((reciverShFrBufFlag & RX_SHFR_BUF_NOT_EMPTY) == 0)
				{
					commState |= DTP_PORT_STATE_BUSY_RX;
					while(DTP_ReceiveBuf(reciverShFrBuf, 16) != 0); // �������� ����� ������ ����� ������ + CRC32
					tOutTicks = DTP_GetSysTicks() + 5000;
					reciverShFrBufFlag |= RX_SHFR_BUF_NOT_EMPTY;	// ��������� ���� �������� � �������
				}
				else // ��������� ��������
				{
					memcpy(&crc32, &reciverShFrBuf[12], 4);
					crc32ret = CRC32CalcFunc((uint32_t*)reciverShFrBuf, 12 / 4);
					if((crc32ret == crc32) && (memcmp(reciverShFrBuf, commSq, 2) == 0))
					{
						memcpy(&rxframeFlags, &reciverShFrBuf[2], 4); 	// ������� ������-�������� �������	(Bit 31..16 - txframeAckCode, Bit 15..0 - ExData)
						memcpy(&rxframeID, &reciverShFrBuf[6], 2);		// ������� ������������� ����� ������
						memcpy(&rxframeLen, &reciverShFrBuf[8], 4);		// ������� ����� ������ ������
						if(rxframeLen != 0)	// ���! �������� ����� � �������, � �������� �������
						{
							rxframeAckCode = (rxframeAckCode & ~COM_ACK_CODE_MASC) | (COM_ACK_CODE_ACCEPTED | COM_ACK_CODE_INVALID); // �� ����������� ������ ��� ��������� ��������
						}
						rxFailFrameCntr = 0;
					}
					else
					{
						rxFailFrameCntr++;
						if(rxFailFrameCntr > RX_FAIL_FRAME_COUNTER_MAX)
						{
							conSincStat = CON_SINC_STAT_RESYNC;
							rxtxBufFlags |= TXRX_FLAG_FRAME_RESYNC_F;
						}
					}
					reciverShFrBufFlag &= ~RX_SHFR_BUF_NOT_EMPTY; // ������ ���� �������� � �������
					rxframeLen = 0;
				}
			}
			if((rxtxBufFlags & TXRX_FLAG_RX_BUF_NOT_EMPTY) != 0) // ���� ����� �������� - �������� �������� ����
			{	// ���������� ��������� ����� ������
				if(rxframeLen == 0) // ���� �������� ����� ��� �� ��� ��������
				{
					memcpy(&crc32, &reciverBuf[reciverBufLen], 4);
					crc32ret = CRC32CalcFunc((uint32_t*)reciverBuf, reciverBufLen / 4);
					if((crc32ret == crc32) && (memcmp(reciverBuf, startSq, 2) == 0))
					{	// ������ ���������� ������
						memcpy(&rxframeFlags, &reciverBuf[2], 4); 	// ������� ������-�������� �������	(Bit 31..16 - txframeAckCode, Bit 15..0 - ExData)
						memcpy(&rxframeID, &reciverBuf[6], 2);		// ������� ������������� ����� ������
						memcpy(&rxframeLen, &reciverBuf[8], 4);		// ������� ����� ������ ������
							// ��������� CRC32_1 ��� �����������
						txframeAckCode = (uint16_t)((rxframeFlags >> 16) & 0x00FF); // ��������� ��������� ������ � ������ �������
						rxframeAckCode = (rxframeAckCode & ~COM_ACK_CODE_MASC) | (COM_ACK_CODE_ACCEPTED | COM_ACK_CODE_BUSY);	// ����������� ������ � ������, ��� ����� �����
						if(rxframeID != (rxframeIDPrev + 1))
						{	// ���� ���� ������ ����������� ������� ����� � ������
							rxframeAckCode = (rxframeAckCode & ~COM_ACK_CODE_MASC) | (COM_ACK_CODE_ACCEPTED | COM_ACK_CODE_FR_LOST); // �� ����������� ������ � ������� � �������� ����� ������
							rxframeIDPrev = 0;	// ��������� ID ������� ������, � ��������������� ������� txframeID = 1
							if(ReadSmBuf != NULL) // ���� ��� ����� ������ � ����������� �����
							{
								ReadSmBufPos = 0;	// ������� ��� � ������
							}
							rxframeLen = 0;
							rxtxBufFlags &= ~TXRX_FLAG_RX_BUF_NOT_EMPTY; // ������ ���� ������ ����� ������
						}
						else
						{
							rxframeIDPrev = rxframeID;
						}
						rxFailFrameCntr = 0;	// ����� �������� ������ ������
					}
					else
					{	// ������ ������������ ������ (�������� ����������� �����)
						rxframeAckCode = (rxframeAckCode & ~COM_ACK_CODE_MASC) | (COM_ACK_CODE_ACCEPTED | COM_ACK_CODE_INVALID); // �� ����������� ������
						rxFailFrameCntr++;
						if(rxFailFrameCntr > RX_FAIL_FRAME_COUNTER_MAX)
						{
							conSincStat = CON_SINC_STAT_RESYNC;
							rxtxBufFlags |= TXRX_FLAG_FRAME_RESYNC_F;
						}
					}
				}
				if(rxframeLen != 0)	// ���� �������� ����� ��� ��� �������� ��� � ��� ���������� ������ - ������ �������� ���
				{
					if(ReadSmBuf != NULL)
					{	//���� ����� ������ ������ � ����� ��� �� �����
						if((ReadSmBufLen - ReadSmBufPos) > rxframeLen)	// ���� � ��������� ������ ����� �����, ��� ������ � �������� ������
						{
							memcpy(&ReadSmBuf[ReadSmBufPos], &reciverBuf[12], rxframeLen);
							ReadSmBufPos += rxframeLen;
							rxframeLen = 0;
							rxtxBufFlags &= ~TXRX_FLAG_RX_BUF_NOT_EMPTY; // ������ ���� ������ ����� ������
							rxframeAckCode = (rxframeAckCode & ~COM_ACK_CODE_MASC) | COM_ACK_CODE_ACCEPTED;	// ����������� ������ � ������, ��� ����� ��������
						}
						else		// ���� � ��������� ������ ����� ������ ��� ������� ��, ������� ������ � �������� ������
						{
							memcpy(&ReadSmBuf[ReadSmBufPos], &reciverBuf[12], ReadSmBufLen - ReadSmBufPos);
							ReadSmBuf = NULL;
							if(DTP_RxTxBufCommEnd != NULL) DTP_RxTxBufCommEnd(DTP_RXTX_COMM_RET_READ_END); // �������� ��������� �� �������� ������
							rxframeLen -= ReadSmBufLen - ReadSmBufPos;
							if(rxframeLen != 0) // ���� � �������� ������ ��� �������� ������
							{
								memcpy(&reciverBuf[12], &reciverBuf[12+(ReadSmBufLen-ReadSmBufPos)], rxframeLen);
								ReadSmBufPos = ReadSmBufLen;
								if(DTP_RxTxBufCommEnd != NULL) DTP_RxTxBufCommEnd(DTP_RXTX_COMM_RET_RXBUF_NOT_EMPTY); // �������� ���������, ��� �������� ����� �� ����
							}
							else
							{
								rxframeAckCode = (rxframeAckCode & ~COM_ACK_CODE_MASC) | COM_ACK_CODE_ACCEPTED;	// ����������� ������ � ������, ��� ����� ��������
								rxtxBufFlags &= ~TXRX_FLAG_RX_BUF_NOT_EMPTY; // ������ ���� ������ ����� ������
							}
						}
					}
				}
			}

			if((DTP_GetSysTicks() > tOutTicks)) // ���� �� ���� ������ ������� ����� 5��� - ������ ��� ����� ���������
			{
				DTP_CommAbort();
				conSincStat = CON_SINC_STAT_SEARCH;
				commState = DTP_PORT_STATE_TIMEOUT;
				if(DTP_RxTxBufCommEnd != NULL) DTP_RxTxBufCommEnd(DTP_RXTX_COMM_RET_LINK_LOST); // �������� ��������� - ���������� ��������
				break;
			}
			if(conSincStat != CON_SINC_STAT_CONNECTED) break; // ���� ����� �������

			///////////////// ��������� �������� ������ ////////////////////
			if(((txframeAckCode & COM_ACK_CODE_ACCEPTED) != 0) && ((txframeAckCode & COM_ACK_CODE_RESYNC) == 0)) // ���� ��������������� ������� �������� � ������ ������ � ��� ���������������
			{
				rxtxBufFlags &= ~TXRX_FLAG_TX_BUF_BLOKED;
			}
			else if((txframeAckCode & COM_ACK_CODE_RESYNC) != 0)	// ��� ������� ��������������� � ��������������� �������
			{
				rxtxBufFlags |= TXRX_FLAG_TX_BUF_BLOKED;
			}
			if(((commState & DTP_PORT_STATE_BUSY_TX) != DTP_PORT_STATE_BUSY_TX) && ((rxtxBufFlags & TXRX_FLAG_TX_BUF_BLOKED) == 0))
			{	// ���� �������� ��������� � ����� ������ ������ � ������ ������� ����� � ������ - �������� ����� ��������
				memcpy(transmitBuf, startSq, 2);				// �������� ���������
				memcpy(&transmitBuf[2], &rxframeIDPrev, 2); 	// �������� ������-�������� �������	(Bit 15..0 - ExData)
				memcpy(&transmitBuf[4], &rxframeAckCode, 2); 	// �������� ������-�������� �������	(Bit 15..0  - txframeAckCode)
				if((txframeAckCode & COM_ACK_CODE_FR_LOST) != 0)	// ���� ���� ��������� �� �������� ����� ������
				{	// �������� �������� ����� ������������� ������ ������ ������
					txframeID = 1;	// = 1 �.�. �������� �������, ��� ���������� = 0
					if(WriteSmBuf != NULL)
					{
						WriteSmBufPos = 0;
					}
				}
				else if((txframeAckCode & COM_ACK_CODE_INVALID) == 0)
				{	// �������� �� ���������� ������������ �������� ����������� ������ (�������� �������� ���� �� ������ ���� � �������)
					txframeID++;
//					txframeAckCode &= ~COM_ACK_CODE_MASC;
				}
				memcpy(&transmitBuf[6], &txframeID, 2);			// �������� ������������� ����� ������
				if((WriteSmBuf != NULL) && ((txframeAckCode & COM_ACK_CODE_INVALID) == 0))
				{	// ���� ������� ����� ������ ��� �� ������� ��������� � ���������� �������� ����� �������� (�������� ������ �������)
					if(WriteSmBufPos < WriteSmBufLen)
					{	// ���� ���� ��� ������ ��� ��������
						memset(&transmitBuf[12], 0, (rxtxBufMaxLen - 12));
						if((WriteSmBufLen - WriteSmBufPos) < (transmitBufLen - (12+4)))
							txframeLen = WriteSmBufLen - WriteSmBufPos;
						else
							txframeLen = transmitBufLen - (12+4);
						memcpy(&transmitBuf[12], &WriteSmBuf[WriteSmBufPos], txframeLen);
						WriteSmBufPos += txframeLen;
					}
					else if((txframeAckCode & COM_ACK_CODE_ACCEPTED) != 0)
					{	// ���� � ������������ ������ ������ �� �������� ������ � ������ ������������ � ������ �������
						WriteSmBuf = NULL;
						if(DTP_RxTxBufCommEnd != NULL) DTP_RxTxBufCommEnd(DTP_RXTX_COMM_RET_WRITE_END); // �������� ��������� �� �������� ��������
					}
				}
				memcpy(&transmitBuf[8], &txframeLen, 4);		// �������� ����� ������ ������
				crc32 = CRC32CalcFunc((uint32_t*)transmitBuf, transmitBufLen / 4);
				memcpy(&transmitBuf[transmitBufLen], &crc32, 4);
				// ������ ��������
				rxtxBufFlags |= TXRX_FLAG_TX_BUF_BLOKED;
				commState |= DTP_PORT_STATE_BUSY_TX;
				while(DTP_TransmitBuf(transmitBuf, transmitBufLen + 4) != 0); // �������� �������� ������ ����� ������ + CRC32
				txPeriodTicks = DTP_GetSysTicks() + TX_SHORT_FRAME_PERIOD;
			}
			else if((commState & DTP_PORT_STATE_BUSY_TX) != DTP_PORT_STATE_BUSY_TX)	// ��������� �������� �������
			{
				if(txPeriodTicks < DTP_GetSysTicks())	// ���� ������ ����� ��������� ��������� ����� - ��������� ��������� �������� ����
				{
					memcpy(transmitBuf, startSq, 2);				// �������� ���������
					memcpy(&transmitBuf[2], &rxframeIDPrev, 2); 	// �������� ������-�������� �������	(Bit 31..16 - txframeAckCode, Bit 15..0 - ExData)
					memcpy(&transmitBuf[4], &rxframeAckCode, 2); 	// �������� ������-�������� �������	(Bit 31..16 - txframeAckCode, Bit 15..0 - ExData)
					memcpy(&transmitBuf[6], &txframeID, 2);			// �������� ������������� ����� ������
					txframeLen = 0;
					memcpy(&transmitBuf[8], &txframeLen, 4);		// �������� ����� ������ ������
					crc32 = CRC32CalcFunc((uint32_t*)transmitBuf, 12 / 4);
					memcpy(&transmitBuf[12], &crc32, 4);
					// ������ ��������
					commState |= DTP_PORT_STATE_BUSY_TX;
					while(DTP_TransmitBuf(transmitBuf, 12 + 4) != 0); // �������� �������� ��������� ����� ������ + CRC32
					txPeriodTicks = DTP_GetSysTicks() + TX_SHORT_FRAME_PERIOD;
				}
			}
			break;


		case CON_SINC_STAT_RESYNC:		// ����� ���������������
			if(rxtxBufFlags & TXRX_FLAG_FRAME_RESYNC_F)
			{	// ������������� ��������� � ����������
				rxTrueResyncFrCntr = 0;
				rxFailFrameCntr = 0;
				reciverShFrBufFlag &= ~RX_SHFR_BUF_NOT_EMPTY;
				if((commState & DTP_PORT_STATE_BUSY_TX_RX) == 0)	// ������ ���������� ������ � �������� ������ � ������ ���� ������� ����� � ���������������
					rxtxBufFlags &= ~TXRX_FLAG_FRAME_RESYNC_F;
			}
			else
			{
				if((commState & DTP_PORT_STATE_BUSY_RX) != DTP_PORT_STATE_BUSY_RX)
				{
					if((reciverShFrBufFlag & RX_SHFR_BUF_NOT_EMPTY) == 0)	// ����� �������� ������ ��� �������������
					{
						commState |= DTP_PORT_STATE_BUSY_RX;
						while(DTP_ReceiveBuf(reciverShFrBuf, 16) != 0); // �������� ����� ������ ����� ������ + CRC32
						tOutTicks = DTP_GetSysTicks() + 5000;
						reciverShFrBufFlag |= RX_SHFR_BUF_NOT_EMPTY;	// ��������� ���� �������� � �������
					}
					else // ��������� ��������
					{
						memcpy(&crc32, &reciverShFrBuf[12], 4);
						crc32ret = CRC32CalcFunc((uint32_t*)reciverShFrBuf, 12 / 4);
						if((crc32ret == crc32) && (memcmp(reciverShFrBuf, commSq, 2) == 0))
						{
							memcpy(&rxframeFlags, &reciverShFrBuf[2], 4); 	// ������� ������-�������� �������	(Bit 31..16 - txframeAckCode, Bit 15..0 - ExData)
							memcpy(&rxframeID, &reciverShFrBuf[6], 2);		// ������� ������������� ����� ������
							memcpy(&rxframeLen, &reciverShFrBuf[8], 4);		// ������� ����� ������ ������
							rxframeAckCode |= COM_ACK_CODE_RESYNC; // ������� � ������ ���������������
							rxFailFrameCntr = 0;
							rxTrueResyncFrCntr++;
							if(rxTrueResyncFrCntr > RX_TRUE_RESYNC_COUNTER_MAX)
							{
								conSincStat = CON_SINC_STAT_CONNECTED;
								commState = DTP_PORT_STATE_READY;
								rxframeAckCode &= ~COM_ACK_CODE_RESYNC; // ������� � ������ �� ������ ���������������
							}
						}
						else
						{
							rxTrueResyncFrCntr = 0;
							rxFailFrameCntr++;
							if(rxFailFrameCntr > RX_FAIL_RESYNC_COUNTER_MAX)
							{
								DTP_CommAbort();
								conSincStat = CON_SINC_STAT_SEARCH;
								commState = DTP_PORT_STATE_TIMEOUT;
								if(DTP_RxTxBufCommEnd != NULL) DTP_RxTxBufCommEnd(DTP_RXTX_COMM_RET_LINK_LOST); // �������� ��������� - ���������� ��������
								break;
							}
						}
						reciverShFrBufFlag &= ~RX_SHFR_BUF_NOT_EMPTY; // ������ ���� �������� � �������
						rxframeLen = 0;
					}
				}

				// �������� �������� ������ ��� ��������������
				if(txPeriodTicks < DTP_GetSysTicks())	// ���� ������ ����� ��������� ��������� ����� - ��������� ��������� �������� ����
				{
					memcpy(transmitBuf, startSq, 2);				// �������� ���������
					memcpy(&transmitBuf[2], &rxframeIDPrev, 2); 	// �������� ������-�������� �������	(Bit 31..16 - txframeAckCode, Bit 15..0 - ExData)
					memcpy(&transmitBuf[4], &rxframeAckCode, 2); 	// �������� ������-�������� �������	(Bit 31..16 - txframeAckCode, Bit 15..0 - ExData)
					memcpy(&transmitBuf[6], &txframeID, 2);			// �������� ������������� ����� ������
					txframeLen = 0;
					memcpy(&transmitBuf[8], &txframeLen, 4);		// �������� ����� ������ ������
					crc32 = CRC32CalcFunc((uint32_t*)transmitBuf, 12 / 4);
					memcpy(&transmitBuf[12], &crc32, 4);
					// ������ ��������
					commState |= DTP_PORT_STATE_BUSY_TX;
					while(DTP_TransmitBuf(transmitBuf, 12 + 4) != 0); // �������� �������� ��������� ����� ������ + CRC32
					txPeriodTicks = DTP_GetSysTicks() + TX_SHORT_FRAME_PERIOD;
				}
			}

			if((DTP_GetSysTicks() > tOutTicks)) // ���� �� ���� ������ ������� ����� 5��� - ������ ��� ����� ���������
			{
				DTP_CommAbort();
				conSincStat = CON_SINC_STAT_SEARCH;
				commState = DTP_PORT_STATE_TIMEOUT;
				if(DTP_RxTxBufCommEnd != NULL) DTP_RxTxBufCommEnd(DTP_RXTX_COMM_RET_LINK_LOST); // �������� ��������� - ���������� ��������
			}
			break;
		default:
			DTP_Delay(1000);
		}
	}
}

/*
 * ������� ��������� ������ - ���������� ������� �� ��������� ������ ������
 * ���������� �������� �� ������ ���, ��� ���������� ����� ������ ����� ��������� �����
 */
void DTP_RxCpltCallback(void)
{
	switch(conSincStat)
	{
	case CON_SINC_STAT_SEARCH:
		reciverBufLen = 16;
		break;
	case CON_SINC_STAT_CONNECTED:
		if((rxtxBufFlags & TXRX_FLAG_RX_BUF_NOT_EMPTY) == 0)
		{
			rxframeAckCode = (rxframeAckCode & ~COM_ACK_CODE_MASC) | COM_ACK_CODE_BUSY;	// �������, ��� ����� �����
			rxtxBufFlags |= TXRX_FLAG_RX_BUF_NOT_EMPTY; // ��������� ���� ������ ����� ������
		}
		break;
	}
	if((commState & DTP_PORT_STATE_BUSY_TX) == DTP_PORT_STATE_BUSY_TX)
		commState &= ~(DTP_PORT_STATE_BUSY_RX & ~DTP_PORT_STATE_READY);
	else
		commState = DTP_PORT_STATE_READY;
}

/*
 * ������� ��������� ������ - ���������� ������� �� ��������� �������� ������
 * ���������� �������� �� ������ ���, ��� ���������� �������� ������ ����� ��������� �����
 */
void DTP_TxCpltCallback(void)
{
	if((commState & DTP_PORT_STATE_BUSY_RX) == DTP_PORT_STATE_BUSY_RX)
		commState &= ~(DTP_PORT_STATE_BUSY_TX & ~DTP_PORT_STATE_READY);
	else
		commState = DTP_PORT_STATE_READY;
}

/*
 * ������� ��������� ������ - ���������� ������� �� ������������� ������ � �������� ������
 * ���������� �������� �� ������ ���, ��� ��������� ������ ������ ������� � ���������� ����� �����
 */
void DTP_ErrorCallback(void)
{
	commState = DTP_PORT_STATE_ERROR;
}

/*
 * �������� dataLen ���� �� ������ ������ � ��������� �����
 * �������� dataReadMode ��������� �� ����� ��������� ������ (���� ����� 0, ��  �������
 * �� ������ �������� �� ��� ���, ���� �� ��������� �������� ������� (��������� ������ ��� ������)
 * � ���� �� ����, �� ������ �������� �����, � �� ��������� ������ ������� ������� ��������� ������
 * ���������� ����, ���� ������ ����������� �������, � �� ���� � ���� ������
 */
int DTP_ReadDataBuf(uint8_t* pBufData, uint32_t dataLen, uint8_t dataReadMode)
{
	if(DTP_InitFlag != 0) return -1;
	if(ReadSmBuf != NULL) return 1; // �����
	if((pBufData == NULL) || (dataLen == 0))  return -1; // ������
	if(((commState & DTP_PORT_STATE_ERROR) == DTP_PORT_STATE_ERROR) || ((commState & DTP_PORT_STATE_TIMEOUT) == DTP_PORT_STATE_TIMEOUT)) return -1; // ������

	ReadSmBufLen = dataLen;
	ReadSmBufPos = 0;
	ReadSmBuf = pBufData;
	if(dataReadMode == 0) // ������ ��������� ������ ������ � ����� ��� ��������� ���������
	{
		while(ReadSmBuf != NULL)
		{
			if(((commState & DTP_PORT_STATE_ERROR) == DTP_PORT_STATE_ERROR) || ((commState & DTP_PORT_STATE_TIMEOUT) == DTP_PORT_STATE_TIMEOUT)) return -1; // ������
			DTP_Delay(5);
		}
	}
	if(((commState & DTP_PORT_STATE_ERROR) == DTP_PORT_STATE_ERROR) || ((commState & DTP_PORT_STATE_TIMEOUT) == DTP_PORT_STATE_TIMEOUT)) return -1; // ������
	return 0;
}

/*
 * �������� dataLen ���� �� ������ ������ � ��������� �����
 * �������� dataWriteMode ��������� �� ����� �������� ������ (���� ����� 0, ��  �������
 * �� ������ �������� �� ��� ���, ���� �� ��������� �������� ������� (��������� ������ ��� ������)
 * � ���� �� ����, �� ������ �������� �����, � �� ��������� ������ ������� ������� ��������� ������
 * ���������� ����, ���� ������ ����������� �������, � �� ���� � ���� ������
 */
int DTP_WriteDataBuf(uint8_t* pBufData, uint32_t dataLen, uint8_t dataWriteMode)
{
	if(DTP_InitFlag != 0) return -1; // ������
	if(WriteSmBuf != NULL) return 1; // �����
	if((pBufData == NULL) || (dataLen == 0))  return -1; // ������
	if(((commState & DTP_PORT_STATE_ERROR) == DTP_PORT_STATE_ERROR) || ((commState & DTP_PORT_STATE_TIMEOUT) == DTP_PORT_STATE_TIMEOUT)) return -1; // ������

	WriteSmBufLen = dataLen;
	WriteSmBufPos = 0;
	WriteSmBuf = pBufData;
	if(dataWriteMode == 0) // ������ ��������� �������� ������ � ����� ��� ��������� ���������
	{
		while(WriteSmBuf != NULL)
		{
			if(((commState & DTP_PORT_STATE_ERROR) == DTP_PORT_STATE_ERROR) || ((commState & DTP_PORT_STATE_TIMEOUT) == DTP_PORT_STATE_TIMEOUT)) return -1; // ������
			DTP_Delay(5);
		}
	}
	if(((commState & DTP_PORT_STATE_ERROR) == DTP_PORT_STATE_ERROR) || ((commState & DTP_PORT_STATE_TIMEOUT) == DTP_PORT_STATE_TIMEOUT)) return -1; // ������
	return 0;
}


///////////////////////// ���������� ���������� //////////////////////
inline void DTP_SearchAdnCon(void)
{
	int tmp;
	uint32_t crc32, crc32ret;

	memcpy(transmitBuf, startSq, 5);
	tmp = rand();
	memcpy(&transmitBuf[5], &tmp, 4);
	transmitBuf[9] = 0x00;
	transmitBuf[10] = (uint8_t)(rxtxBufMaxLen & 0xff);
	transmitBuf[11] = (uint8_t)((rxtxBufMaxLen >> 8) & 0xff);
	crc32 = CRC32CalcFunc((uint32_t*)transmitBuf, 3);
	memcpy(&transmitBuf[12], &crc32, 4);
	reciverBufLen = 0;
	commState = DTP_PORT_STATE_BUSY_TX_RX;
	if(DTP_TransmitBuf(transmitBuf, 16)) return;
	if(DTP_ReceiveBuf(reciverBuf, 16)) return;

	DTP_Delay(500);

	if(reciverBufLen == 16)
	{
		reciverBufLen = 0;
		commState |= DTP_PORT_STATE_BUSY_RX;
		if(DTP_ReceiveBuf(&reciverBuf[16], 16)) return;	// ��� ������ ���������� ������
		memcpy(&crc32, &reciverBuf[12], 4);
		crc32ret = CRC32CalcFunc((uint32_t*)reciverBuf, 3);
		if(crc32ret == crc32)
		{
			if(memcmp(reciverBuf, startSq, 5) == 0)
			{
				if(memcmp(&transmitBuf[5], &tmp, 4) != 0)	// �������� ����� �� ���� ������� ���� �� ����� ������, ��� � ����������� (������ �� ��������� ����� �����)
				{
					transmitBufLen = (uint16_t)((((uint16_t)reciverBuf[11] << 8) & 0xff00) | ((uint16_t)reciverBuf[10] & 0x00ff));
					if(transmitBufLen <= rxtxBufMaxLen) // ����� ������� ����� �� �����������
					{
						reciverBufLen = transmitBufLen;	// ���� ���� �������� ������ ����� ������
					}
					else
					{
						transmitBufLen = reciverBufLen = rxtxBufMaxLen; // ���� ���� ������ ������ ����� ��������
					}
					// ������� ��������� ����� ��� ��������
					memcpy(transmitBuf, startSq, 5);
					memcpy(&transmitBuf[5], &reciverBuf[5], 4); // �������� ���������� ������������� ��������� ��������� ������������ ������
					transmitBuf[9] = 0x00;
					transmitBuf[10] = (uint8_t)(transmitBufLen & 0xff);			// ��������� �������������� ������ ����� �������� �� ����� ����������
					transmitBuf[11] = (uint8_t)((transmitBufLen >> 8) & 0xff);	// -------- || ---------
					crc32 = CRC32CalcFunc((uint32_t*)transmitBuf, 3);
					memcpy(&transmitBuf[12], &crc32, 4);
					commState |= DTP_PORT_STATE_BUSY_TX;
					DTP_Delay(10);
					if(DTP_TransmitBuf(transmitBuf, 16)) return;	// ��������� ��������� �����

					if(reciverBufLen != 16) DTP_Delay(1000); // ���� ��������� ����� �� ������ - ��� 1���
					if(reciverBufLen != 16) return; // ���� ������ ������� ����� ��� � �� ������ ������� �� �������������

					memcpy(&crc32, &reciverBuf[16+12], 4);
					crc32ret = CRC32CalcFunc((uint32_t*)&reciverBuf[16], 3);
					if(crc32ret == crc32)
					{
						if(memcmp(&reciverBuf[16], startSq, 5) == 0)
							if(memcmp(&reciverBuf[16+5], &tmp, 4) == 0) // ���� � ��������� ������ ������������ ����� ���������� ������������� ������
							{
								// �������� �������� ����� ����� (� �������� ��������� ��� ������ ���� ����� ����� ������������)
								if(reciverBufLen == ((uint16_t)((((uint16_t)reciverBuf[16+11] << 8) & 0xff00) | ((uint16_t)reciverBuf[16+10] & 0x00ff))))
								{	// ���� ��� ������ �� � �������
									commState = DTP_PORT_STATE_READY;
									conSincStat = CON_SINC_STAT_CONNECTED;
									if(DTP_RxTxBufCommEnd != NULL) DTP_RxTxBufCommEnd(DTP_RXTX_COMM_RET_LINK_INSTALLED); // �������� ��������� - ���������� �����������
								}
							}
					}
				}
			}
		}
	}
}

uint32_t DTP_CalcCRC32(uint32_t pBuffer[], uint32_t BufferLength)
{
	// todo ���������� ����������� ���������� CRC32 �������
	return BufferLength;
}

void DTP_DelayInternal(uint32_t period)
{
	uint32_t ticks;

	ticks = DTP_GetSysTicks() + period;
	while(ticks < DTP_GetSysTicks());
}

