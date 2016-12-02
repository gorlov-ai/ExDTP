/*
 * DataTransferProt.h
 *
 *  Created on: 1 дек. 2016 г.
 *      Author: Артем
 */

#ifndef DRIVERS_DATATRANSFERPROT_H_
#define DRIVERS_DATATRANSFERPROT_H_

#include <stdint.h>

#define RX_FAIL_FRAME_COUNTER_MAX	3	// максимальное количесво неудачных приемо кадров, поселе которых наступает ресинхронизация
#define RX_TRUE_RESYNC_COUNTER_MAX	5	// количество успешно принятых кадров ресинхронизации, после которых возобновляется соединение
#define RX_FAIL_RESYNC_COUNTER_MAX	5	// количество принятых подряд с ошибкой кадров ресинхронизации, после которых разрывается соединение

#define TX_SHORT_FRAME_PERIOD	250		// период времени между отправками коротких информационных кадров

#define DTP_RXTX_COMM_RET_LINK_INSTALLED	0
#define DTP_RXTX_COMM_RET_READ_END			1
#define DTP_RXTX_COMM_RET_RXBUF_NOT_EMPTY	2
#define DTP_RXTX_COMM_RET_WRITE_END			3
#define DTP_RXTX_COMM_RET_LINK_LOST			4
#define DTP_RXTX_COMM_RET_LINK_LOST_12121			44

typedef struct
{
	uint32_t* pReciverBuf;		// указатель на буфер приема кадров данных
	uint32_t* pTransmitBuf;		// указатель на буфер передачи кадров данных
	uint32_t rxtxBufLen_W;		// длины предыдущих буферов
	uint32_t (*GetSysTicks)(void);	// указатель на функцию получения системного времени в мсек	(обязательная функция)
	uint8_t (*ReceiveBuf)(uint8_t *pRxData, uint16_t Size);	// указатель на функцию запуска протоколом обмена приема в буфер кадра данных (обязательная функция)
	uint8_t (*TransmitBuf)(uint8_t *pRxData, uint16_t Size);// указатель на функцию запуска протоколом обмена передачи буфера кадра данных (обязательная функция)
	void (*CommAbort)(void);	// указатель на функцию запуска протоколом обмена прекращения обмена данными (обязательная функция)
	uint32_t (*CRC32CalcFunc)(uint32_t pBuffer[], uint32_t BufferLength);	// указатель на функцию вычисления CRC32 (не обязательная функция - может быть NULL)
	void (*Delay)(uint32_t period);		// указатель на функцию вызова системной задержки в мсек (не обязательная функция - может быть NULL)
	void (*RxTxBufCommEnd)(uint8_t);	// указатель на функцию вызываемую протоколом при различных событиях обмена (не обязательная функция - может быть NULL)
}DTP_HandlerTDef;


int DTP_Init(DTP_HandlerTDef* initHandler);
void DTP_RxTxProc(void);
void DTP_RxCpltCallback(void);
void DTP_TxCpltCallback(void);
void DTP_ErrorCallback(void);
int DTP_ReadDataBuf(uint8_t* pBufData, uint32_t dataLen, uint8_t dataReadMode);
int DTP_WriteDataBuf(uint8_t* pBufData, uint32_t dataLen, uint8_t dataWriteMode);



#endif /* DRIVERS_DATATRANSFERPROT_H_ */
