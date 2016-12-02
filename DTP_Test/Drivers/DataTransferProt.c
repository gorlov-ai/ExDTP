/*
 * DataTransferProt.c
 *
 *  Created on: 26 нояб. 2016 г.
 *      Author: Артем
 */

#include "DataTransferProt.h"
#include <string.h>
#include <stdlib.h>

#define CON_SINC_STAT_SEARCH		0
#define CON_SINC_STAT_CONNECTED		1
#define CON_SINC_STAT_RESYNC		3

#define COM_ACK_CODE_MASC			0x00FF
#define COM_ACK_CODE_NON			0x0000
#define COM_ACK_CODE_ACCEPTED		0x0001	// выставляется при приеме пакета (в любом случае если кадр принят)
#define COM_ACK_CODE_INVALID		0x0002	// выставляется если принятый кадр поврежден
#define COM_ACK_CODE_BUSY			0x0004	// выставляется если буфер не готов принять данные
#define COM_ACK_CODE_FR_LOST		0x0008	// выставляется если онаружена потеря буфера
#define COM_ACK_CODE_RESYNC			0x0010	// выставляется если запущен режим ресинхронизации


#define RX_SHFR_BUF_NOT_EMPTY		0x01	// буфер приема коротких кадров не пуст

#define TXRX_FLAG_RX_BUF_NOT_EMPTY	0x00000001	// приемных буфер кадров не пустой
#define TXRX_FLAG_TX_BUF_BLOKED		0x00010000	// если буфер для передачи кадров данных заблокирован до момента получения подтверждения о приеме
#define TXRX_FLAG_FRAME_RESYNC_F	0x10001000	// при первом заходе на ресинхронизацию

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

// локальные объявления переменных
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
uint16_t rxframeID, rxframeAckCode; // rxframeAckCode - формируется из состояния приемного буфера этой стороны
uint16_t txframeID, txframeAckCode;	// txframeAckCode - принимается из пришедшего кадра (указывает на состояние приемного буфера с другой стороны)
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

// локальный объявления функций
inline void DTP_SearchAdnCon(void);
uint32_t DTP_CalcCRC32(uint32_t pBuffer[], uint32_t BufferLength);
void DTP_DelayInternal(uint32_t period);

//////////// определения глобальных функций ////////////////
int DTP_Init(DTP_HandlerTDef* initHandler)
{
	if(DTP_InitFlag != 0) return 1;

	if(initHandler->rxtxBufLen_W < (32 / 4)) return -1;	// если размер буфера меньше 32 байт
	if((initHandler->pReciverBuf == NULL) || (initHandler->pTransmitBuf == NULL)) return -1;
	if(initHandler->GetSysTicks == NULL) return -1;
	if((initHandler->ReceiveBuf == NULL) || (initHandler->TransmitBuf == NULL) || (initHandler->CommAbort == NULL)) return -1;

	reciverBuf = (uint8_t*)initHandler->pReciverBuf;
	transmitBuf = (uint8_t*)initHandler->pTransmitBuf;
	rxtxBufMaxLen = (initHandler->rxtxBufLen_W * 4);

	if(initHandler->CRC32CalcFunc != NULL)	// назначаю вычислитель CRC32
		CRC32CalcFunc = initHandler->CRC32CalcFunc;	// внешний
	else
		CRC32CalcFunc = DTP_CalcCRC32;				// внутренний

	if(initHandler->Delay != NULL)	// назначаю функцию задержки
		DTP_Delay = initHandler->Delay;
	else
		DTP_Delay = DTP_DelayInternal;

	if(initHandler->RxTxBufCommEnd != NULL)	// назначаю функцию обработчик завершения приема/передачи или коннекта/дисконнекта линий
		DTP_RxTxBufCommEnd = initHandler->RxTxBufCommEnd;
	else
		DTP_RxTxBufCommEnd = NULL;

	DTP_GetSysTicks = initHandler->GetSysTicks;	// назначаю функцию запроса счетчика тиков в мсек
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
 * Эту функцию необходимо периодически вызывать для работы системы обмена данными
 */
void DTP_RxTxProc(void)
{
	if(DTP_InitFlag != 0)
	{
		switch(conSincStat)
		{
		case CON_SINC_STAT_SEARCH:		// режим поиска оборудования
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
				txframeID = 1;	// именно так
				rxframeIDPrev = 0;	// именно так
			}
			break;


		case CON_SINC_STAT_CONNECTED:	// режим обмена данными
			///////////////// проверка ошибок обмена ////////////////////
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

			///////////////// обработка приема кадров ////////////////////
			if(((commState & DTP_PORT_STATE_BUSY_RX) != DTP_PORT_STATE_BUSY_RX) && ((rxframeAckCode & COM_ACK_CODE_BUSY) == 0))
			{	// если прием завершен и буфер приема данных готов к обмену - назначаю новый прием
				commState |= DTP_PORT_STATE_BUSY_RX;
				rxtxBufFlags &= ~TXRX_FLAG_RX_BUF_NOT_EMPTY;
				while(DTP_ReceiveBuf(reciverBuf, reciverBufLen + 4) != 0); // запускаю прием данных кадра данных + CRC32
				tOutTicks = DTP_GetSysTicks() + 5000;
				rxframeLen = 0;
			}
			else if((commState & DTP_PORT_STATE_BUSY_RX) != DTP_PORT_STATE_BUSY_RX) // если буфер приема данных занят а прием уже закончен
			{	// принимаю кадры коротыши
				if((reciverShFrBufFlag & RX_SHFR_BUF_NOT_EMPTY) == 0)
				{
					commState |= DTP_PORT_STATE_BUSY_RX;
					while(DTP_ReceiveBuf(reciverShFrBuf, 16) != 0); // запускаю прием данных кадра данных + CRC32
					tOutTicks = DTP_GetSysTicks() + 5000;
					reciverShFrBufFlag |= RX_SHFR_BUF_NOT_EMPTY;	// выставляю флаг перехода к разбору
				}
				else // обработка коротыша
				{
					memcpy(&crc32, &reciverShFrBuf[12], 4);
					crc32ret = CRC32CalcFunc((uint32_t*)reciverShFrBuf, 12 / 4);
					if((crc32ret == crc32) && (memcmp(reciverShFrBuf, commSq, 2) == 0))
					{
						memcpy(&rxframeFlags, &reciverShFrBuf[2], 4); 	// получаю статус-флаговый регистр	(Bit 31..16 - txframeAckCode, Bit 15..0 - ExData)
						memcpy(&rxframeID, &reciverShFrBuf[6], 2);		// получаю идентификатор кадра данных
						memcpy(&rxframeLen, &reciverShFrBuf[8], 4);		// получаю длину пакета данных
						if(rxframeLen != 0)	// опа! приходил пакет с данными, а ожидался коротыш
						{
							rxframeAckCode = (rxframeAckCode & ~COM_ACK_CODE_MASC) | (COM_ACK_CODE_ACCEPTED | COM_ACK_CODE_INVALID); // не подтверждаю данные для повторной отправки
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
					reciverShFrBufFlag &= ~RX_SHFR_BUF_NOT_EMPTY; // снимаю флаг перехода к разбору
					rxframeLen = 0;
				}
			}
			if((rxtxBufFlags & TXRX_FLAG_RX_BUF_NOT_EMPTY) != 0) // если прием завершен - разбираю принятый кадр
			{	// обработчик принятого кадра данных
				if(rxframeLen == 0) // если приемный буфер еще не был разобран
				{
					memcpy(&crc32, &reciverBuf[reciverBufLen], 4);
					crc32ret = CRC32CalcFunc((uint32_t*)reciverBuf, reciverBufLen / 4);
					if((crc32ret == crc32) && (memcmp(reciverBuf, startSq, 2) == 0))
					{	// пришли корректные данные
						memcpy(&rxframeFlags, &reciverBuf[2], 4); 	// получаю статус-флаговый регистр	(Bit 31..16 - txframeAckCode, Bit 15..0 - ExData)
						memcpy(&rxframeID, &reciverBuf[6], 2);		// получаю идентификатор кадра данных
						memcpy(&rxframeLen, &reciverBuf[8], 4);		// получаю длину пакета данных
							// пропускаю CRC32_1 для микрокадров
						txframeAckCode = (uint16_t)((rxframeFlags >> 16) & 0x00FF); // состояние приемного буфера с другой стороны
						rxframeAckCode = (rxframeAckCode & ~COM_ACK_CODE_MASC) | (COM_ACK_CODE_ACCEPTED | COM_ACK_CODE_BUSY);	// подтверждаю данные и говорю, что буфер занят
						if(rxframeID != (rxframeIDPrev + 1))
						{	// если кадр данных былпропущен начинаю прием с начала
							rxframeAckCode = (rxframeAckCode & ~COM_ACK_CODE_MASC) | (COM_ACK_CODE_ACCEPTED | COM_ACK_CODE_FR_LOST); // не подтверждаю данные и сообщаю о пропуске кадра данных
							rxframeIDPrev = 0;	// сбрасываю ID счетчик кадров, а противоположная сторона txframeID = 1
							if(ReadSmBuf != NULL) // если был прием данных в считывающий буфер
							{
								ReadSmBufPos = 0;	// начинаю все с начала
							}
							rxframeLen = 0;
							rxtxBufFlags &= ~TXRX_FLAG_RX_BUF_NOT_EMPTY; // снимаю флаг приема кадра данных
						}
						else
						{
							rxframeIDPrev = rxframeID;
						}
						rxFailFrameCntr = 0;	// сброс счетчика фейлов приема
					}
					else
					{	// пришли поврежденные данные (неверная контрольная сумма)
						rxframeAckCode = (rxframeAckCode & ~COM_ACK_CODE_MASC) | (COM_ACK_CODE_ACCEPTED | COM_ACK_CODE_INVALID); // не подтверждаю данные
						rxFailFrameCntr++;
						if(rxFailFrameCntr > RX_FAIL_FRAME_COUNTER_MAX)
						{
							conSincStat = CON_SINC_STAT_RESYNC;
							rxtxBufFlags |= TXRX_FLAG_FRAME_RESYNC_F;
						}
					}
				}
				if(rxframeLen != 0)	// если приемный буфер уже был разобран или в нем оставались данные - просто скачиваю его
				{
					if(ReadSmBuf != NULL)
					{	//если буфер чтения данных с порта еще не полон
						if((ReadSmBufLen - ReadSmBufPos) > rxframeLen)	// если в читаюещем буфере места бльше, чем данных в приемном буфере
						{
							memcpy(&ReadSmBuf[ReadSmBufPos], &reciverBuf[12], rxframeLen);
							ReadSmBufPos += rxframeLen;
							rxframeLen = 0;
							rxtxBufFlags &= ~TXRX_FLAG_RX_BUF_NOT_EMPTY; // снимаю флаг приема кадра данных
							rxframeAckCode = (rxframeAckCode & ~COM_ACK_CODE_MASC) | COM_ACK_CODE_ACCEPTED;	// подтверждаю данные и говорю, что буфер свободен
						}
						else		// если в читаюещем буфере места меньше или столько же, сколько данных в приемном буфере
						{
							memcpy(&ReadSmBuf[ReadSmBufPos], &reciverBuf[12], ReadSmBufLen - ReadSmBufPos);
							ReadSmBuf = NULL;
							if(DTP_RxTxBufCommEnd != NULL) DTP_RxTxBufCommEnd(DTP_RXTX_COMM_RET_READ_END); // отправка сообщения об успешном приеме
							rxframeLen -= ReadSmBufLen - ReadSmBufPos;
							if(rxframeLen != 0) // если в приемном буфере еще остались данные
							{
								memcpy(&reciverBuf[12], &reciverBuf[12+(ReadSmBufLen-ReadSmBufPos)], rxframeLen);
								ReadSmBufPos = ReadSmBufLen;
								if(DTP_RxTxBufCommEnd != NULL) DTP_RxTxBufCommEnd(DTP_RXTX_COMM_RET_RXBUF_NOT_EMPTY); // отправка сообщения, что приемный буфер не пуст
							}
							else
							{
								rxframeAckCode = (rxframeAckCode & ~COM_ACK_CODE_MASC) | COM_ACK_CODE_ACCEPTED;	// подтверждаю данные и говорю, что буфер свободен
								rxtxBufFlags &= ~TXRX_FLAG_RX_BUF_NOT_EMPTY; // снимаю флаг приема кадра данных
							}
						}
					}
				}
			}

			if((DTP_GetSysTicks() > tOutTicks)) // если не было обмена данными более 5сек - считаю что связь разорвана
			{
				DTP_CommAbort();
				conSincStat = CON_SINC_STAT_SEARCH;
				commState = DTP_PORT_STATE_TIMEOUT;
				if(DTP_RxTxBufCommEnd != NULL) DTP_RxTxBufCommEnd(DTP_RXTX_COMM_RET_LINK_LOST); // отправка сообщения - соединение потеряно
				break;
			}
			if(conSincStat != CON_SINC_STAT_CONNECTED) break; // если связь первана

			///////////////// обработка передачи кадров ////////////////////
			if(((txframeAckCode & COM_ACK_CODE_ACCEPTED) != 0) && ((txframeAckCode & COM_ACK_CODE_RESYNC) == 0)) // если противоположная сторона ответила о приеме пакета и нет ресинхронизации
			{
				rxtxBufFlags &= ~TXRX_FLAG_TX_BUF_BLOKED;
			}
			else if((txframeAckCode & COM_ACK_CODE_RESYNC) != 0)	// при наличии ресинхронизации с противоположной стороны
			{
				rxtxBufFlags |= TXRX_FLAG_TX_BUF_BLOKED;
			}
			if(((commState & DTP_PORT_STATE_BUSY_TX) != DTP_PORT_STATE_BUSY_TX) && ((rxtxBufFlags & TXRX_FLAG_TX_BUF_BLOKED) == 0))
			{	// если передача завершена и буфер приема данных с другой стороны готов к обмену - назначаю новую передачу
				memcpy(transmitBuf, startSq, 2);				// вписываю преамбулу
				memcpy(&transmitBuf[2], &rxframeIDPrev, 2); 	// наполняю статус-флаговый регистр	(Bit 15..0 - ExData)
				memcpy(&transmitBuf[4], &rxframeAckCode, 2); 	// наполняю статус-флаговый регистр	(Bit 15..0  - txframeAckCode)
				if((txframeAckCode & COM_ACK_CODE_FR_LOST) != 0)	// если было сообщение об пропуске кадра данных
				{	// запускаю передачу всего отправляемого буфера данных заново
					txframeID = 1;	// = 1 т.к. приемник считает, что предыдущий = 0
					if(WriteSmBuf != NULL)
					{
						WriteSmBufPos = 0;
					}
				}
				else if((txframeAckCode & COM_ACK_CODE_INVALID) == 0)
				{	// проверка на отсутствие некорректной передачи предыдущего буфера (повторяю отправку того же буфера если с ошибкой)
					txframeID++;
//					txframeAckCode &= ~COM_ACK_CODE_MASC;
				}
				memcpy(&transmitBuf[6], &txframeID, 2);			// добавляю идентификатор кадра данных
				if((WriteSmBuf != NULL) && ((txframeAckCode & COM_ACK_CODE_INVALID) == 0))
				{	// если внешний буфер данных еще не передан полностью и предыдущая отправка кадра успешная (наполняю новыми данными)
					if(WriteSmBufPos < WriteSmBufLen)
					{	// если есть еще данные для отправки
						memset(&transmitBuf[12], 0, (rxtxBufMaxLen - 12));
						if((WriteSmBufLen - WriteSmBufPos) < (transmitBufLen - (12+4)))
							txframeLen = WriteSmBufLen - WriteSmBufPos;
						else
							txframeLen = transmitBufLen - (12+4);
						memcpy(&transmitBuf[12], &WriteSmBuf[WriteSmBufPos], txframeLen);
						WriteSmBufPos += txframeLen;
					}
					else if((txframeAckCode & COM_ACK_CODE_ACCEPTED) != 0)
					{	// если в отправляемом буфере больше не осталось данных и данные подтверждены с другой стороны
						WriteSmBuf = NULL;
						if(DTP_RxTxBufCommEnd != NULL) DTP_RxTxBufCommEnd(DTP_RXTX_COMM_RET_WRITE_END); // отправка сообщения об успешной передаче
					}
				}
				memcpy(&transmitBuf[8], &txframeLen, 4);		// добавляю длину пакета данных
				crc32 = CRC32CalcFunc((uint32_t*)transmitBuf, transmitBufLen / 4);
				memcpy(&transmitBuf[transmitBufLen], &crc32, 4);
				// запуск передачи
				rxtxBufFlags |= TXRX_FLAG_TX_BUF_BLOKED;
				commState |= DTP_PORT_STATE_BUSY_TX;
				while(DTP_TransmitBuf(transmitBuf, transmitBufLen + 4) != 0); // запускаю передачу данных кадра данных + CRC32
				txPeriodTicks = DTP_GetSysTicks() + TX_SHORT_FRAME_PERIOD;
			}
			else if((commState & DTP_PORT_STATE_BUSY_TX) != DTP_PORT_STATE_BUSY_TX)	// отправщик коротких пакетов
			{
				if(txPeriodTicks < DTP_GetSysTicks())	// если период между короткими посылками истек - отправляю следующий короткий кадр
				{
					memcpy(transmitBuf, startSq, 2);				// вписываю преамбулу
					memcpy(&transmitBuf[2], &rxframeIDPrev, 2); 	// наполняю статус-флаговый регистр	(Bit 31..16 - txframeAckCode, Bit 15..0 - ExData)
					memcpy(&transmitBuf[4], &rxframeAckCode, 2); 	// наполняю статус-флаговый регистр	(Bit 31..16 - txframeAckCode, Bit 15..0 - ExData)
					memcpy(&transmitBuf[6], &txframeID, 2);			// добавляю идентификатор кадра данных
					txframeLen = 0;
					memcpy(&transmitBuf[8], &txframeLen, 4);		// добавляю длину пакета данных
					crc32 = CRC32CalcFunc((uint32_t*)transmitBuf, 12 / 4);
					memcpy(&transmitBuf[12], &crc32, 4);
					// запуск передачи
					commState |= DTP_PORT_STATE_BUSY_TX;
					while(DTP_TransmitBuf(transmitBuf, 12 + 4) != 0); // запускаю передачу короткого кадра данных + CRC32
					txPeriodTicks = DTP_GetSysTicks() + TX_SHORT_FRAME_PERIOD;
				}
			}
			break;


		case CON_SINC_STAT_RESYNC:		// режим ресинхронизации
			if(rxtxBufFlags & TXRX_FLAG_FRAME_RESYNC_F)
			{	// подготавливаю параметры и переменные
				rxTrueResyncFrCntr = 0;
				rxFailFrameCntr = 0;
				reciverShFrBufFlag &= ~RX_SHFR_BUF_NOT_EMPTY;
				if((commState & DTP_PORT_STATE_BUSY_TX_RX) == 0)	// ожидаю завершения приема и передачи кадров и снимаю флаг первого входа в ресинхронизацию
					rxtxBufFlags &= ~TXRX_FLAG_FRAME_RESYNC_F;
			}
			else
			{
				if((commState & DTP_PORT_STATE_BUSY_RX) != DTP_PORT_STATE_BUSY_RX)
				{
					if((reciverShFrBufFlag & RX_SHFR_BUF_NOT_EMPTY) == 0)	// прием коротких кадров для синхронизации
					{
						commState |= DTP_PORT_STATE_BUSY_RX;
						while(DTP_ReceiveBuf(reciverShFrBuf, 16) != 0); // запускаю прием данных кадра данных + CRC32
						tOutTicks = DTP_GetSysTicks() + 5000;
						reciverShFrBufFlag |= RX_SHFR_BUF_NOT_EMPTY;	// выставляю флаг перехода к разбору
					}
					else // обработка коротыша
					{
						memcpy(&crc32, &reciverShFrBuf[12], 4);
						crc32ret = CRC32CalcFunc((uint32_t*)reciverShFrBuf, 12 / 4);
						if((crc32ret == crc32) && (memcmp(reciverShFrBuf, commSq, 2) == 0))
						{
							memcpy(&rxframeFlags, &reciverShFrBuf[2], 4); 	// получаю статус-флаговый регистр	(Bit 31..16 - txframeAckCode, Bit 15..0 - ExData)
							memcpy(&rxframeID, &reciverShFrBuf[6], 2);		// получаю идентификатор кадра данных
							memcpy(&rxframeLen, &reciverShFrBuf[8], 4);		// получаю длину пакета данных
							rxframeAckCode |= COM_ACK_CODE_RESYNC; // сообщаю о режиме ресинхронизации
							rxFailFrameCntr = 0;
							rxTrueResyncFrCntr++;
							if(rxTrueResyncFrCntr > RX_TRUE_RESYNC_COUNTER_MAX)
							{
								conSincStat = CON_SINC_STAT_CONNECTED;
								commState = DTP_PORT_STATE_READY;
								rxframeAckCode &= ~COM_ACK_CODE_RESYNC; // сообщаю о выходе из режима ресинхронизации
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
								if(DTP_RxTxBufCommEnd != NULL) DTP_RxTxBufCommEnd(DTP_RXTX_COMM_RET_LINK_LOST); // отправка сообщения - соединение потеряно
								break;
							}
						}
						reciverShFrBufFlag &= ~RX_SHFR_BUF_NOT_EMPTY; // снимаю флаг перехода к разбору
						rxframeLen = 0;
					}
				}

				// отправка коротких кадров для информирования
				if(txPeriodTicks < DTP_GetSysTicks())	// если период между короткими посылками истек - отправляю следующий короткий кадр
				{
					memcpy(transmitBuf, startSq, 2);				// вписываю преамбулу
					memcpy(&transmitBuf[2], &rxframeIDPrev, 2); 	// наполняю статус-флаговый регистр	(Bit 31..16 - txframeAckCode, Bit 15..0 - ExData)
					memcpy(&transmitBuf[4], &rxframeAckCode, 2); 	// наполняю статус-флаговый регистр	(Bit 31..16 - txframeAckCode, Bit 15..0 - ExData)
					memcpy(&transmitBuf[6], &txframeID, 2);			// добавляю идентификатор кадра данных
					txframeLen = 0;
					memcpy(&transmitBuf[8], &txframeLen, 4);		// добавляю длину пакета данных
					crc32 = CRC32CalcFunc((uint32_t*)transmitBuf, 12 / 4);
					memcpy(&transmitBuf[12], &crc32, 4);
					// запуск передачи
					commState |= DTP_PORT_STATE_BUSY_TX;
					while(DTP_TransmitBuf(transmitBuf, 12 + 4) != 0); // запускаю передачу короткого кадра данных + CRC32
					txPeriodTicks = DTP_GetSysTicks() + TX_SHORT_FRAME_PERIOD;
				}
			}

			if((DTP_GetSysTicks() > tOutTicks)) // если не было обмена данными более 5сек - считаю что связь разорвана
			{
				DTP_CommAbort();
				conSincStat = CON_SINC_STAT_SEARCH;
				commState = DTP_PORT_STATE_TIMEOUT;
				if(DTP_RxTxBufCommEnd != NULL) DTP_RxTxBufCommEnd(DTP_RXTX_COMM_RET_LINK_LOST); // отправка сообщения - соединение потеряно
			}
			break;
		default:
			DTP_Delay(1000);
		}
	}
}

/*
 * Функция обратного вызова - обработчик событий об окончании приема буфера
 * необходимо вызывать ее всякий раз, как закончится прием данных через указанную среду
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
			rxframeAckCode = (rxframeAckCode & ~COM_ACK_CODE_MASC) | COM_ACK_CODE_BUSY;	// сообщаю, что буфер занят
			rxtxBufFlags |= TXRX_FLAG_RX_BUF_NOT_EMPTY; // выставляю флаг приеме кадра данных
		}
		break;
	}
	if((commState & DTP_PORT_STATE_BUSY_TX) == DTP_PORT_STATE_BUSY_TX)
		commState &= ~(DTP_PORT_STATE_BUSY_RX & ~DTP_PORT_STATE_READY);
	else
		commState = DTP_PORT_STATE_READY;
}

/*
 * Функция обратного вызова - обработчик событий об окончании передачи буфера
 * необходимо вызывать ее всякий раз, как закончится передача данных через указанную среду
 */
void DTP_TxCpltCallback(void)
{
	if((commState & DTP_PORT_STATE_BUSY_RX) == DTP_PORT_STATE_BUSY_RX)
		commState &= ~(DTP_PORT_STATE_BUSY_TX & ~DTP_PORT_STATE_READY);
	else
		commState = DTP_PORT_STATE_READY;
}

/*
 * Функция обратного вызова - обработчик событий об возникновении ошибки в процессе обмена
 * необходимо вызывать ее всякий раз, как возникнет ошибка обмена данными в физической среде порта
 */
void DTP_ErrorCallback(void)
{
	commState = DTP_PORT_STATE_ERROR;
}

/*
 * Получает dataLen байт из потока данных в указанный буфер
 * параметр dataReadMode указывает на режим получения данных (если равен 0, то  функция
 * не вернет значения до тех пор, пока не достигнет кончного события (окончания буфера или ошибки)
 * а если не нуль, то вернет значение сразу, а по окончанию обмена вызовет функцию обратного вызова
 * возвращает нуль, если чтение закончилось успешно, и не нуль в ином случае
 */
int DTP_ReadDataBuf(uint8_t* pBufData, uint32_t dataLen, uint8_t dataReadMode)
{
	if(DTP_InitFlag != 0) return -1;
	if(ReadSmBuf != NULL) return 1; // занят
	if((pBufData == NULL) || (dataLen == 0))  return -1; // ошибка
	if(((commState & DTP_PORT_STATE_ERROR) == DTP_PORT_STATE_ERROR) || ((commState & DTP_PORT_STATE_TIMEOUT) == DTP_PORT_STATE_TIMEOUT)) return -1; // ошибка

	ReadSmBufLen = dataLen;
	ReadSmBufPos = 0;
	ReadSmBuf = pBufData;
	if(dataReadMode == 0) // ожидаю окончания приема буфера и после это возвращаю результат
	{
		while(ReadSmBuf != NULL)
		{
			if(((commState & DTP_PORT_STATE_ERROR) == DTP_PORT_STATE_ERROR) || ((commState & DTP_PORT_STATE_TIMEOUT) == DTP_PORT_STATE_TIMEOUT)) return -1; // ошибка
			DTP_Delay(5);
		}
	}
	if(((commState & DTP_PORT_STATE_ERROR) == DTP_PORT_STATE_ERROR) || ((commState & DTP_PORT_STATE_TIMEOUT) == DTP_PORT_STATE_TIMEOUT)) return -1; // ошибка
	return 0;
}

/*
 * Получает dataLen байт из потока данных в указанный буфер
 * параметр dataWriteMode указывает на режим отправки данных (если равен 0, то  функция
 * не вернет значения до тех пор, пока не достигнет кончного события (окончания буфера или ошибки)
 * а если не нуль, то вернет значение сразу, а по окончанию обмена вызовет функцию обратного вызова
 * возвращает нуль, если чтение закончилось успешно, и не нуль в ином случае
 */
int DTP_WriteDataBuf(uint8_t* pBufData, uint32_t dataLen, uint8_t dataWriteMode)
{
	if(DTP_InitFlag != 0) return -1; // ошибка
	if(WriteSmBuf != NULL) return 1; // занят
	if((pBufData == NULL) || (dataLen == 0))  return -1; // ошибка
	if(((commState & DTP_PORT_STATE_ERROR) == DTP_PORT_STATE_ERROR) || ((commState & DTP_PORT_STATE_TIMEOUT) == DTP_PORT_STATE_TIMEOUT)) return -1; // ошибка

	WriteSmBufLen = dataLen;
	WriteSmBufPos = 0;
	WriteSmBuf = pBufData;
	if(dataWriteMode == 0) // ожидаю окончания отправки буфера и после это возвращаю результат
	{
		while(WriteSmBuf != NULL)
		{
			if(((commState & DTP_PORT_STATE_ERROR) == DTP_PORT_STATE_ERROR) || ((commState & DTP_PORT_STATE_TIMEOUT) == DTP_PORT_STATE_TIMEOUT)) return -1; // ошибка
			DTP_Delay(5);
		}
	}
	if(((commState & DTP_PORT_STATE_ERROR) == DTP_PORT_STATE_ERROR) || ((commState & DTP_PORT_STATE_TIMEOUT) == DTP_PORT_STATE_TIMEOUT)) return -1; // ошибка
	return 0;
}


///////////////////////// внутренний функционал //////////////////////
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
		if(DTP_ReceiveBuf(&reciverBuf[16], 16)) return;	// для приема повторного ответа
		memcpy(&crc32, &reciverBuf[12], 4);
		crc32ret = CRC32CalcFunc((uint32_t*)reciverBuf, 3);
		if(crc32ret == crc32)
		{
			if(memcmp(reciverBuf, startSq, 5) == 0)
			{
				if(memcmp(&transmitBuf[5], &tmp, 4) != 0)	// проверяю чтобы не было принято того же кадра поиска, что и отправлялся (защита от замыкания линий связи)
				{
					transmitBufLen = (uint16_t)((((uint16_t)reciverBuf[11] << 8) & 0xff00) | ((uint16_t)reciverBuf[10] & 0x00ff));
					if(transmitBufLen <= rxtxBufMaxLen) // выбор размера кадра по наименьшему
					{
						reciverBufLen = transmitBufLen;	// если кадр передачи меньше кадра приема
					}
					else
					{
						transmitBufLen = reciverBufLen = rxtxBufMaxLen; // если кадр приема меньше кадра передачи
					}
					// собираю повторный ответ для отправки
					memcpy(transmitBuf, startSq, 5);
					memcpy(&transmitBuf[5], &reciverBuf[5], 4); // вклеиваю уникальный идентификатор пересылки принятого контрольного пакета
					transmitBuf[9] = 0x00;
					transmitBuf[10] = (uint8_t)(transmitBufLen & 0xff);			// отправляю результирующий размер кадра передачи от этого устройства
					transmitBuf[11] = (uint8_t)((transmitBufLen >> 8) & 0xff);	// -------- || ---------
					crc32 = CRC32CalcFunc((uint32_t*)transmitBuf, 3);
					memcpy(&transmitBuf[12], &crc32, 4);
					commState |= DTP_PORT_STATE_BUSY_TX;
					DTP_Delay(10);
					if(DTP_TransmitBuf(transmitBuf, 16)) return;	// отправляю повторный ответ

					if(reciverBufLen != 16) DTP_Delay(1000); // если повторный ответ не пришел - жду 1сек
					if(reciverBufLen != 16) return; // если спустя секунду ответ так и не пришел выпадаю из синхронизации

					memcpy(&crc32, &reciverBuf[16+12], 4);
					crc32ret = CRC32CalcFunc((uint32_t*)&reciverBuf[16], 3);
					if(crc32ret == crc32)
					{
						if(memcmp(&reciverBuf[16], startSq, 5) == 0)
							if(memcmp(&reciverBuf[16+5], &tmp, 4) == 0) // если в повторном ответе отправленный ранее уникальный идентификатор совпал
							{
								// проверяю ответную длину кадра (в принятом сообщении она должна быть равна ранее отправленной)
								if(reciverBufLen == ((uint16_t)((((uint16_t)reciverBuf[16+11] << 8) & 0xff00) | ((uint16_t)reciverBuf[16+10] & 0x00ff))))
								{	// если все готово то и отлично
									commState = DTP_PORT_STATE_READY;
									conSincStat = CON_SINC_STAT_CONNECTED;
									if(DTP_RxTxBufCommEnd != NULL) DTP_RxTxBufCommEnd(DTP_RXTX_COMM_RET_LINK_INSTALLED); // отправка сообщения - соединение установлено
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
	// todo необходимо реализовать встроенный CRC32 счетчик
	return BufferLength;
}

void DTP_DelayInternal(uint32_t period)
{
	uint32_t ticks;

	ticks = DTP_GetSysTicks() + period;
	while(ticks < DTP_GetSysTicks());
}

