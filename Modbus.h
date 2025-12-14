#ifndef MODBUS_H
#define MODBUS_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

/* ------------------------- Configuration ------------------------- */
#define MAX_BUFFER       128
#define MAX_M_HANDLERS   2

#define DB_COUNT     (3)
#define COILS_PER_DB (1024)   /* bits per DB */
#define REGS_PER_DB  (512)    /* 16-bit regs per DB */

/* ------------------------- PLC-style addressing ------------------------- */
/* Default: raw = db * 1000 + offset (fits within 16-bit Modbus address) */
#ifndef MB_ADDR_DB_DIVISOR
#define MB_ADDR_DB_DIVISOR 1000U
#endif

#define MB_ADDR_DECODE_DB(raw)     ((uint8_t)((raw) / MB_ADDR_DB_DIVISOR))
#define MB_ADDR_DECODE_OFFSET(raw) ((uint16_t)((raw) % MB_ADDR_DB_DIVISOR))

/* Fixed timing values (ms) for Modbus RTU */
#define MODBUS_T35_MS     (5U)     /* 3.5 character times */
#define MODBUS_TIMEOUT_MS (100U)   /* frame timeout */

/* ------------------------- Hardware type ------------------------- */
typedef enum { USART_HW = 1, USART_HW_DMA = 4 } mb_hardware_t;

/* ------------------------- Handler ------------------------- */
typedef struct {
    UART_HandleTypeDef *port;
    uint8_t  u8id;

    GPIO_TypeDef* EN_Port;
    uint16_t EN_Pin;

    int8_t i8lastError;

    uint8_t u8Buffer[MAX_BUFFER];
    uint8_t u8BufferSize;

    uint16_t u16InCnt, u16OutCnt, u16errCnt;

    uint8_t dataRX;
    int8_t i8state;

    TaskHandle_t myTaskModbusAHandle;

    mb_hardware_t xTypeHW;
    uint16_t u16timeOut;   /* fixed timeout from define */
} modbusHandler_t;

/* ------------------------- Static RAM DBs ------------------------- */
extern uint8_t  gCoils[DB_COUNT][COILS_PER_DB / 8];
extern uint16_t gRegs[DB_COUNT][REGS_PER_DB];

/* ------------------------- Shared handlers ------------------------- */
extern modbusHandler_t *mHandlers[MAX_M_HANDLERS];
extern uint8_t numberHandlers;

/* ------------------------- Core Modbus API ------------------------- */
void ModbusInit(modbusHandler_t *modH,
                UART_HandleTypeDef *huart,
                uint8_t slave_id,
                GPIO_TypeDef *en_port,
                uint16_t en_pin,
                mb_hardware_t hw_type);

void ModbusStart(modbusHandler_t *modH);
void StartTaskModbusSlave(void *argument);

uint16_t calcCRC(uint8_t *Buffer, uint8_t u8length);

/* ------------------------- Interface functions ------------------------- */
bool     mb_read_coil(modbusHandler_t *modH, uint8_t db, int16_t address);
void     mb_write_coil(modbusHandler_t *modH, uint8_t db, int16_t address, bool state);
uint16_t mb_read_register(modbusHandler_t *modH, uint8_t db, int16_t address);
void     mb_write_register(modbusHandler_t *modH, uint8_t db, int16_t address, const uint16_t *data, uint16_t len);

#endif /* MODBUS_H */
