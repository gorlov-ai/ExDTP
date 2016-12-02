/*
 * DataTransferProt.h
 *
 *  Created on: 1 ���. 2016 �.
 *      Author: �����
 */

#ifndef DRIVERS_DATATRANSFERPROT_H_
#define DRIVERS_DATATRANSFERPROT_H_

#include <stdint.h>

#define RX_FAIL_FRAME_COUNTER_MAX	3	// ������������ ��������� ��������� ������ ������, ������ ������� ��������� ���������������
#define RX_TRUE_RESYNC_COUNTER_MAX	5	// ���������� ������� �������� ������ ���������������, ����� ������� �������������� ����������
#define RX_FAIL_RESYNC_COUNTER_MAX	5	// ���������� �������� ������ � ������� ������ ���������������, ����� ������� ����������� ����������

#define TX_SHORT_FRAME_PERIOD	250		// ������ ������� ����� ���������� �������� �������������� ������

#define DTP_RXTX_COMM_RET_LINK_INSTALLED	0
#define DTP_RXTX_COMM_RET_READ_END			1
#define DTP_RXTX_COMM_RET_RXBUF_NOT_EMPTY	2
#define DTP_RXTX_COMM_RET_WRITE_END			3
#define DTP_RXTX_COMM_RET_LINK_LOST			4
#define DTP_RXTX_COMM_RET_LINK_LOST_12121			44

typedef struct
{
	uint32_t* pReciverBuf;		// ��������� �� ����� ������ ������ ������
	uint32_t* pTransmitBuf;		// ��������� �� ����� �������� ������ ������
	uint32_t rxtxBufLen_W;		// ����� ���������� �������
	uint32_t (*GetSysTicks)(void);	// ��������� �� ������� ��������� ���������� ������� � ����	(������������ �������)
	uint8_t (*ReceiveBuf)(uint8_t *pRxData, uint16_t Size);	// ��������� �� ������� ������� ���������� ������ ������ � ����� ����� ������ (������������ �������)
	uint8_t (*TransmitBuf)(uint8_t *pRxData, uint16_t Size);// ��������� �� ������� ������� ���������� ������ �������� ������ ����� ������ (������������ �������)
	void (*CommAbort)(void);	// ��������� �� ������� ������� ���������� ������ ����������� ������ ������� (������������ �������)
	uint32_t (*CRC32CalcFunc)(uint32_t pBuffer[], uint32_t BufferLength);	// ��������� �� ������� ���������� CRC32 (�� ������������ ������� - ����� ���� NULL)
	void (*Delay)(uint32_t period);		// ��������� �� ������� ������ ��������� �������� � ���� (�� ������������ ������� - ����� ���� NULL)
	void (*RxTxBufCommEnd)(uint8_t);	// ��������� �� ������� ���������� ���������� ��� ��������� �������� ������ (�� ������������ ������� - ����� ���� NULL)
}DTP_HandlerTDef;


int DTP_Init(DTP_HandlerTDef* initHandler);
void DTP_RxTxProc(void);
void DTP_RxCpltCallback(void);
void DTP_TxCpltCallback(void);
void DTP_ErrorCallback(void);
int DTP_ReadDataBuf(uint8_t* pBufData, uint32_t dataLen, uint8_t dataReadMode);
int DTP_WriteDataBuf(uint8_t* pBufData, uint32_t dataLen, uint8_t dataWriteMode);



#endif /* DRIVERS_DATATRANSFERPROT_H_ */
