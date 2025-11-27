/******************************************************************************
 * File:    Modbus.h
 * Brief:   Unified Modbus header for Common / Slave / Master modules
 * Author:  Alejandro Mera (original), adapted by Masoud hamadi
 * Date:    Nov 2025
 ******************************************************************************/

#ifndef THIRD_PARTY_MODBUS_INC_MODBUS_H_
#define THIRD_PARTY_MODBUS_INC_MODBUS_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "BitUtils.h"
#include "ModbusConfig.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* ------------------------- تنظیمات حافظه EEPROM ------------------------- */
#define AT24_BASE_ADDR      ((uint16_t)0x0000)   /* آدرس شروع کلی */
#define DB_COUNT            (3)                  /* تعداد دیتابیس‌ها */
#define COILS_PER_DB        (1024)               /* تعداد کویل در هر دیتابیس */
#define REGS_PER_DB         (512)                /* تعداد رجیستر در هر دیتابیس */

/* هر کویل یک بیت است → هر دیتابیس = COILS_PER_DB/8 بایت */
#define COILS_BASE_ADDR     (AT24_BASE_ADDR)
#define TOTAL_COILS_BYTES   (DB_COUNT * (COILS_PER_DB/8))

/* رجیسترها بعد از کویل‌ها ذخیره می‌شوند → هر رجیستر ۲ بایت */
#define REGS_BASE_ADDR      (COILS_BASE_ADDR + TOTAL_COILS_BYTES)
#define TOTAL_REGS_BYTES    (DB_COUNT * REGS_PER_DB * 2)

/* ------------------------- انواع داده ------------------------- */
typedef enum { USART_HW = 1, USART_HW_DMA = 4 } mb_hardware_t;
typedef enum { MB_SLAVE = 3, MB_MASTER = 4 } mb_masterslave_t;

typedef enum MB_FC {
    MB_FC_READ_COILS               = 1,
    MB_FC_READ_DISCRETE_INPUT      = 2,
    MB_FC_READ_REGISTERS           = 3,
    MB_FC_READ_INPUT_REGISTER      = 4,
    MB_FC_WRITE_COIL               = 5,
    MB_FC_WRITE_REGISTER           = 6,
    MB_FC_WRITE_MULTIPLE_COILS     = 15,
    MB_FC_WRITE_MULTIPLE_REGISTERS = 16
} mb_functioncode_t;

#ifndef MAX_BUFFER
#define MAX_BUFFER 256
#endif

typedef struct {
    uint8_t uxBuffer[MAX_BUFFER];
    uint8_t u8start;
    uint8_t u8end;
    uint8_t u8available;
    bool    overflow;
} modbusRingBuffer_t;

typedef enum MESSAGE {
    ID = 0,
    FUNC,
    ADD_HI,
    ADD_LO,
    NB_HI,
    NB_LO,
    BYTE_CNT
} mb_message_t;

typedef enum COM_STATES { COM_IDLE = 0, COM_WAITING = 1 } mb_com_state_t;
typedef enum ERR_LIST {
    ERR_NOT_MASTER    = -1,
    ERR_POLLING       = -2,
    ERR_BUFF_OVERFLOW = -3,
    ERR_BAD_CRC       = -4,
    ERR_EXCEPTION     = -5,
    ERR_BAD_SIZE      = -6,
    ERR_BAD_ADDRESS   = -7,
    ERR_TIME_OUT      = -8,
    ERR_BAD_SLAVE_ID  = -9,
    ERR_BAD_TCP_ID    = -10,
    ERR_OK_QUERY      = -11
} mb_errot_t;

/* ------------------------- کدهای Exception پروتکل Modbus ------------------------- */
enum {
    EXC_FUNC_CODE   = 1,  /* Function code not supported */
    EXC_ADDR_RANGE  = 2,  /* Address out of range */
    EXC_REGS_QUANT  = 3,  /* Invalid quantity of registers/coils */
    EXC_EXECUTE     = 4   /* Execution error */
};

typedef union {
    uint8_t  u8[4];
    uint16_t u16[2];
    uint32_t u32;
} bytesFields;

enum {
    DB_COILS = 1,
    DB_INPUT_COILS = 2,
    DB_HOLDING_REGISTER = 3,
    DB_INPUT_REGISTERS = 4
};

typedef struct {
    uint8_t u8id;
    mb_functioncode_t u8fct;
    uint16_t u16RegAdd;
    uint16_t u16CoilsNo;
    uint16_t *u16reg;
    uint32_t *u32CurrentTask;
} modbus_t;

typedef struct {
    mb_masterslave_t uModbusType;
    UART_HandleTypeDef *port;
    uint8_t u8id;
    GPIO_TypeDef* EN_Port;
    uint16_t EN_Pin;
    mb_errot_t i8lastError;
    uint8_t u8Buffer[MAX_BUFFER];
    uint8_t u8BufferSize;
    uint8_t u8lastRec;
    uint16_t *u16regsHR;
    uint16_t *u16regsRO;
    uint16_t *u16regsCoils;
    uint16_t *u16regsCoilsRO;
    uint16_t u16InCnt, u16OutCnt, u16errCnt;
    uint16_t u16timeOut;
    uint16_t u16regHR_size;
    uint16_t u16regRO_size;
    uint16_t u16regCoils_size;
    uint16_t u16regCoilsRO_size;
    uint8_t dataRX;
    int8_t i8state;
    osMessageQueueId_t QueueTelegramHandle;
    osThreadId_t myTaskModbusAHandle;
    xTimerHandle xTimerT35;
    xTimerHandle xTimerTimeout;
    osSemaphoreId_t ModBusSphrHandle;
    modbusRingBuffer_t xBufferRX;
    mb_hardware_t xTypeHW;
} modbusHandler_t;

enum { RESPONSE_SIZE = 6, EXCEPTION_SIZE = 3, CHECKSUM_SIZE = 2 };

/* Globals shared across modules */
extern modbusHandler_t *mHandlers[MAX_M_HANDLERS];
extern uint8_t numberHandlers;

/* ------------------------- پروتوتایپ توابع عمومی ------------------------- */
/* مدیریت عمومی */
void ModbusInit(modbusHandler_t * modH);
void ModbusStart(modbusHandler_t * modH);
void setTimeOut(uint16_t u16timeOut);
uint16_t getTimeOut(void);
bool getTimeOutState(void);


/* Tasks (public) */
void StartTaskModbusSlave(void *argument);

/* CRC (public) */
uint16_t calcCRC(uint8_t *Buffer, uint8_t u8length);

/* Ring buffer (public) */
void    RingAdd(modbusRingBuffer_t *xRingBuffer, uint8_t u8Val);
uint8_t RingGetAllBytes(modbusRingBuffer_t *xRingBuffer, uint8_t *buffer);
uint8_t RingGetNBytes(modbusRingBuffer_t *xRingBuffer, uint8_t *buffer, uint8_t uNumber);
uint8_t RingCountBytes(modbusRingBuffer_t *xRingBuffer);
void    RingClear(modbusRingBuffer_t *xRingBuffer);



#endif /* THIRD_PARTY_MODBUS_INC_MODBUS_H_ */
