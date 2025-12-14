#ifndef MODBUS_H
#define MODBUS_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

#define MAX_BUFFER       128
#define DB_COUNT     (3)
#define COILS_PER_DB (1024)
#define REGS_PER_DB  (512)

/* PLC-style addressing: raw = db*1000 + offset */
#define MB_ADDR_DB_DIVISOR 1000U
#define MB_ADDR_DECODE_DB(raw)     ((uint8_t)((raw) / MB_ADDR_DB_DIVISOR))
#define MB_ADDR_DECODE_OFFSET(raw) ((uint16_t)((raw) % MB_ADDR_DB_DIVISOR))

#define MODBUS_T35_MS     (5U)
#define MODBUS_TIMEOUT_MS (100U)

typedef enum { USART_HW_DMA = 4 } mb_hardware_t;

typedef struct {
    uint8_t  uxBuffer[MAX_BUFFER];
    uint16_t u8available;
    bool     overflow;
} mb_rx_dma_buffer_t;

typedef struct {
    UART_HandleTypeDef *port;
    uint8_t  u8id;
    GPIO_TypeDef* EN_Port;
    uint16_t EN_Pin;

    uint8_t u8Buffer[MAX_BUFFER];
    uint8_t u8BufferSize;

    uint16_t u16InCnt, u16OutCnt, u16errCnt;

    TaskHandle_t myTaskModbusAHandle;
    mb_hardware_t xTypeHW;
    uint16_t u16timeOut;

    mb_rx_dma_buffer_t xBufferRX;
} modbusHandler_t;

/* Global single handler */
extern modbusHandler_t gModbusH;

/* Static DBs */
extern uint8_t  gCoils[DB_COUNT][COILS_PER_DB / 8];
extern uint16_t gRegs[DB_COUNT][REGS_PER_DB];

void ModbusInit(UART_HandleTypeDef *huart,
                uint8_t slave_id,
                GPIO_TypeDef *en_port,
                uint16_t en_pin);

void ModbusStart(void);
void StartTaskModbusSlave(void *argument);

uint16_t calcCRC(uint8_t *Buffer, uint8_t u8length);

bool     mb_read_coil(uint8_t db, int16_t address);
void     mb_write_coil(uint8_t db, int16_t address, bool state);
uint16_t mb_read_register(uint8_t db, int16_t address);
void     mb_write_register(uint8_t db, int16_t address, const uint16_t *data, uint16_t len);

#endif
