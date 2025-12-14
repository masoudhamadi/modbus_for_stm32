/*
 * Modbus.c — unified Modbus RTU slave for STM32F405 + FreeRTOS + HAL
 * - Static RAM DBs for Coils/Registers
 * - Shared handlers
 * - Init/Start
 * - Full processing of FC1, FC3, FC5, FC6, FC15, FC16 with bounds/limits
 * - Frame validation (slave address + CRC)
 * - RS-485 DE control
 * - ISR-driven RX and task processing
 */

#include "Modbus.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "cmsis_os.h"

/* ------------------------- Modbus limits ------------------------- */
#ifndef MODBUS_MAX_READ_REGS
#define MODBUS_MAX_READ_REGS    125U
#endif
#ifndef MODBUS_MAX_READ_COILS
#define MODBUS_MAX_READ_COILS   2000U
#endif
#ifndef MODBUS_MAX_WRITE_REGS
#define MODBUS_MAX_WRITE_REGS   123U
#endif
#ifndef MODBUS_MAX_WRITE_COILS
#define MODBUS_MAX_WRITE_COILS  1968U
#endif

/* ------------------------- Static RAM DBs ------------------------- */
uint8_t  gCoils[DB_COUNT][COILS_PER_DB / 8] = {0};
uint16_t gRegs[DB_COUNT][REGS_PER_DB]       = {0};

/* ------------------------- Shared handlers ------------------------- */
modbusHandler_t *mHandlers[MAX_M_HANDLERS] = {0};
uint8_t numberHandlers = 0;

/* ------------------------- Helpers ------------------------- */
static uint16_t word(uint8_t H, uint8_t L) { return ((uint16_t)H << 8) | (uint16_t)L; }

/* CRC-16 Modbus */
uint16_t calcCRC(uint8_t *buf, uint8_t len) {
    uint16_t crc = 0xFFFFU;
    for (uint8_t i = 0U; i < len; i++) {
        crc ^= (uint16_t)buf[i];
        for (uint8_t j = 0U; j < 8U; j++) {
            if (crc & 0x0001U) crc = (uint16_t)((crc >> 1) ^ 0xA001U);
            else crc >>= 1;
        }
    }
    return crc;
}

/* ------------------------- Forward declarations ------------------------- */
static void handle_frame(modbusHandler_t *modH);
static void transmit_response(modbusHandler_t *modH);
static void send_exception(modbusHandler_t *modH, uint8_t func, uint8_t ex_code);

/* ------------------------- Init & start ------------------------- */
void ModbusInit(modbusHandler_t *modH,
                UART_HandleTypeDef *huart,
                uint8_t slave_id,
                GPIO_TypeDef *en_port,
                uint16_t en_pin,
                mb_hardware_t hw_type)
{
    if (modH == NULL) return;

    modH->port    = huart;
    modH->u8id    = slave_id;
    modH->EN_Port = en_port;
    modH->EN_Pin  = en_pin;
    modH->xTypeHW = hw_type;

    if (numberHandlers < MAX_M_HANDLERS) {
        mHandlers[numberHandlers++] = modH;
    } else {
        return;
    }

    modH->u16timeOut  = MODBUS_TIMEOUT_MS;
    modH->i8lastError = (int8_t)MODBUS_T35_MS;

    modH->u8BufferSize = 0U;
    modH->u16InCnt = 0U;
    modH->u16OutCnt = 0U;
    modH->u16errCnt = 0U;

    modH->dataRX = 0U;
    modH->i8state = 0;
}

void ModbusStart(modbusHandler_t *modH) {
    if (modH == NULL || modH->port == NULL) return;

    if (modH->EN_Port != NULL) {
        HAL_GPIO_WritePin(modH->EN_Port, modH->EN_Pin, GPIO_PIN_RESET);
    }

    (void)HAL_UART_Receive_IT(modH->port, &modH->dataRX, 1U);
    modH->u8BufferSize = 0U;

#if ENABLE_USART_DMA == 1
    if (modH->xTypeHW == USART_HW_DMA) {
        if (HAL_UARTEx_ReceiveToIdle_DMA(modH->port, modH->xBufferRX.uxBuffer, MAX_BUFFER) == HAL_OK) {
            if (modH->port->hdmarx != NULL) {
                __HAL_DMA_DISABLE_IT(modH->port->hdmarx, DMA_IT_HT);
            }
        }
    }
#endif
}

/* ------------------------- Function Codes ------------------------- */

/* FC1: Read Coils */
static void process_FC1(modbusHandler_t *modH) {
    uint16_t rawAddr   = word(modH->u8Buffer[2], modH->u8Buffer[3]);
    uint8_t  db        = MB_ADDR_DECODE_DB(rawAddr);
    uint16_t startCoil = MB_ADDR_DECODE_OFFSET(rawAddr);
    uint16_t coilCount = word(modH->u8Buffer[4], modH->u8Buffer[5]);

    if (coilCount == 0U || coilCount > MODBUS_MAX_READ_COILS) {
        send_exception(modH, 1U, 0x03U);
        return;
    }
    if ((db >= DB_COUNT) || ((uint32_t)startCoil + coilCount > COILS_PER_DB)) {
        send_exception(modH, 1U, 0x02U);
        return;
    }

    uint8_t byteCount = (uint8_t)((coilCount + 7U) / 8U);
    if ((uint16_t)3U + (uint16_t)byteCount + 2U > MAX_BUFFER) {
        send_exception(modH, 1U, 0x03U);
        return;
    }

    modH->u8Buffer[0] = modH->u8id;
    modH->u8Buffer[1] = 1U;
    modH->u8Buffer[2] = byteCount;
    modH->u8BufferSize = 3U;
    memset(&modH->u8Buffer[3], 0, byteCount);

    for (uint16_t i = 0U; i < coilCount; i++) {
        uint16_t idx = startCoil + i;
        uint16_t byteIdx = idx / 8U;
        uint8_t bitPos = (uint8_t)(idx % 8U);
        uint8_t val = (gCoils[db][byteIdx] >> bitPos) & 0x01U;
        if (val) {
            modH->u8Buffer[3U + (i / 8U)] |= (uint8_t)(1U << (i % 8U));
        }
    }

    modH->u8BufferSize = (uint8_t)(3U + byteCount);
}

/* FC3: Read Holding Registers */
static void process_FC3(modbusHandler_t *modH) {
    uint16_t rawAddr  = word(modH->u8Buffer[2], modH->u8Buffer[3]);
    uint8_t  db       = MB_ADDR_DECODE_DB(rawAddr);
    uint16_t startReg = MB_ADDR_DECODE_OFFSET(rawAddr);
    uint16_t regCount = word(modH->u8Buffer[4], modH->u8Buffer[5]);

    if (regCount == 0U || regCount > MODBUS_MAX_READ_REGS) {
        send_exception(modH, 3U, 0x03U);
        return;
    }
    if ((db >= DB_COUNT) || ((uint32_t)startReg + regCount > REGS_PER_DB)) {
        send_exception(modH, 3U, 0x02U);
        return;
    }

    uint8_t byteCount = (uint8_t)(regCount * 2U);
    if ((uint16_t)3U + (uint16_t)byteCount + 2U > MAX_BUFFER) {
        send_exception(modH, 3U, 0x03U);
        return;
    }

    modH->u8Buffer[0] = modH->u8id;
    modH->u8Buffer[1] = 3U;
    modH->u8Buffer[2] = byteCount;
    modH->u8BufferSize = 3U;

    for (uint16_t i = 0U; i < regCount; i++) {
        uint16_t val = gRegs[db][startReg + i];
        modH->u8Buffer[modH->u8BufferSize++] = (uint8_t)(val >> 8);
        modH->u8Buffer[modH->u8BufferSize++] = (uint8_t)(val & 0xFF);
    }
}

/* FC5: Write Single Coil */
static void process_FC5(modbusHandler_t *modH) {
    uint16_t rawAddr  = word(modH->u8Buffer[2], modH->u8Buffer[3]);
    uint8_t  db       = MB_ADDR_DECODE_DB(rawAddr);
    uint16_t coilAddr = MB_ADDR_DECODE_OFFSET(rawAddr);
    uint16_t value    = word(modH->u8Buffer[4], modH->u8Buffer[5]);
    bool state = (value == 0xFF00U);

    if ((db >= DB_COUNT) || (coilAddr >= COILS_PER_DB)) {
        send_exception(modH, 5U, 0x02U);
        return;
    }

    mb_write_coil(modH, db, (int16_t)coilAddr, state);

    modH->u8Buffer[0] = modH->u8id;
    modH->u8Buffer[1] = 5U;
    modH->u8Buffer[2] = (uint8_t)(rawAddr >> 8);
    modH->u8Buffer[3] = (uint8_t)(rawAddr & 0xFF);
    modH->u8Buffer[4] = (uint8_t)(value >> 8);
    modH->u8Buffer[5] = (uint8_t)(value & 0xFF);
    modH->u8BufferSize = 6U;
}

/* FC6: Write Single Register */
static void process_FC6(modbusHandler_t *modH) {
    uint16_t rawAddr = word(modH->u8Buffer[2], modH->u8Buffer[3]);
    uint8_t  db      = MB_ADDR_DECODE_DB(rawAddr);
    uint16_t regAddr = MB_ADDR_DECODE_OFFSET(rawAddr);
    uint16_t value   = word(modH->u8Buffer[4], modH->u8Buffer[5]);

    if ((db >= DB_COUNT) || (regAddr >= REGS_PER_DB)) {
        send_exception(modH, 6U, 0x02U);
        return;
    }

    mb_write_register(modH, db, (int16_t)regAddr, &value, 1U);

    modH->u8Buffer[0] = modH->u8id;
    modH->u8Buffer[1] = 6U;
    modH->u8Buffer[2] = (uint8_t)(rawAddr >> 8);
    modH->u8Buffer[3] = (uint8_t)(rawAddr & 0xFF);
    modH->u8Buffer[4] = (uint8_t)(value >> 8);
    modH->u8Buffer[5] = (uint8_t)(value & 0xFF);
    modH->u8BufferSize = 6U;
}

/* FC15: Write Multiple Coils */
static void process_FC15(modbusHandler_t *modH) {
    uint16_t rawAddr   = word(modH->u8Buffer[2], modH->u8Buffer[3]);
    uint8_t  db        = MB_ADDR_DECODE_DB(rawAddr);
    uint16_t startCoil = MB_ADDR_DECODE_OFFSET(rawAddr);
    uint16_t coilCount = word(modH->u8Buffer[4], modH->u8Buffer[5]);
    uint8_t byteCount  = modH->u8Buffer[6];

    if (coilCount == 0U || coilCount > MODBUS_MAX_WRITE_COILS) {
        send_exception(modH, 15U, 0x03U);
        return;
    }
    if ((db >= DB_COUNT) || ((uint32_t)startCoil + coilCount > COILS_PER_DB)) {
        send_exception(modH, 15U, 0x02U);
        return;
    }
    if (byteCount != (uint8_t)((coilCount + 7U) / 8U)) {
        send_exception(modH, 15U, 0x03U);
        return;
    }

    for (uint16_t i = 0U; i < coilCount; i++) {
        uint8_t byte = modH->u8Buffer[7U + (i / 8U)];
        bool state = ((byte >> (i % 8U)) & 0x01U) != 0U;
        mb_write_coil(modH, db, (int16_t)(startCoil + i), state);
    }

    modH->u8Buffer[0] = modH->u8id;
    modH->u8Buffer[1] = 15U;
    modH->u8Buffer[2] = (uint8_t)(rawAddr >> 8);
    modH->u8Buffer[3] = (uint8_t)(rawAddr & 0xFF);
    modH->u8Buffer[4] = (uint8_t)(coilCount >> 8);
    modH->u8Buffer[5] = (uint8_t)(coilCount & 0xFF);
    modH->u8BufferSize = 6U;
}

/* FC16: Write Multiple Registers */
static void process_FC16(modbusHandler_t *modH) {
    uint16_t rawAddr  = word(modH->u8Buffer[2], modH->u8Buffer[3]);
    uint8_t  db       = MB_ADDR_DECODE_DB(rawAddr);
    uint16_t startReg = MB_ADDR_DECODE_OFFSET(rawAddr);
    uint16_t regCount = word(modH->u8Buffer[4], modH->u8Buffer[5]);
    uint8_t byteCount = modH->u8Buffer[6];

    if (regCount == 0U || regCount > MODBUS_MAX_WRITE_REGS) {
        send_exception(modH, 16U, 0x03U);
        return;
    }
    if ((db >= DB_COUNT) || ((uint32_t)startReg + regCount > REGS_PER_DB)) {
        send_exception(modH, 16U, 0x02U);
        return;
    }
    if (byteCount != (uint8_t)(regCount * 2U)) {
        send_exception(modH, 16U, 0x03U);
        return;
    }

    for (uint16_t i = 0U; i < regCount; i++) {
        uint16_t val = word(modH->u8Buffer[7U + i*2U], modH->u8Buffer[8U + i*2U]);
        mb_write_register(modH, db, (int16_t)(startReg + i), &val, 1U);
    }

    modH->u8Buffer[0] = modH->u8id;
    modH->u8Buffer[1] = 16U;
    modH->u8Buffer[2] = (uint8_t)(rawAddr >> 8);
    modH->u8Buffer[3] = (uint8_t)(rawAddr & 0xFF);
    modH->u8Buffer[4] = (uint8_t)(regCount >> 8);
    modH->u8Buffer[5] = (uint8_t)(regCount & 0xFF);
    modH->u8BufferSize = 6U;
}

/* ------------------------- Frame handler ------------------------- */
static void handle_frame(modbusHandler_t *modH)
{
    if (modH == NULL) return;

    if (modH->u8BufferSize < 4U) {
        modH->u16errCnt++;
        modH->u8BufferSize = 0U;
        return;
    }

    uint16_t recv_crc = (uint16_t)((modH->u8Buffer[modH->u8BufferSize - 2] << 8) |
                                   modH->u8Buffer[modH->u8BufferSize - 1]);
    uint16_t calc = calcCRC(modH->u8Buffer, (uint8_t)(modH->u8BufferSize - 2U));
    if (calc != recv_crc) {
        modH->u16errCnt++;
        modH->u8BufferSize = 0U;
        return;
    }

    uint8_t addr = modH->u8Buffer[0];
    if ((addr != modH->u8id) && (addr != 0x00U)) {
        modH->u8BufferSize = 0U;
        return;
    }

    uint8_t func = modH->u8Buffer[1];
    switch (func) {
        case 1U:
            if (modH->u8BufferSize < 8U) { send_exception(modH, func, 0x03U); break; }
            process_FC1(modH);
            break;
        case 3U:
            if (modH->u8BufferSize < 8U) { send_exception(modH, func, 0x03U); break; }
            process_FC3(modH);
            break;
        case 5U:
            if (modH->u8BufferSize < 8U) { send_exception(modH, func, 0x03U); break; }
            process_FC5(modH);
            break;
        case 6U:
            if (modH->u8BufferSize < 8U) { send_exception(modH, func, 0x03U); break; }
            process_FC6(modH);
            break;
        case 15U:
            if (modH->u8BufferSize < 9U) { send_exception(modH, func, 0x03U); break; }
            process_FC15(modH);
            break;
        case 16U:
            if (modH->u8BufferSize < 9U) { send_exception(modH, func, 0x03U); break; }
            process_FC16(modH);
            break;
        default:
            send_exception(modH, func, 0x01U);
            break;
    }

    if (addr != 0x00U && modH->u8BufferSize > 0U) {
        transmit_response(modH);
    } else {
        modH->u8BufferSize = 0U;
    }
}

/* ------------------------- Transmit response ------------------------- */
static void transmit_response(modbusHandler_t *modH)
{
    if (modH == NULL) return;

    if ((uint16_t)modH->u8BufferSize + 2U > MAX_BUFFER) {
        modH->u16errCnt++;
        modH->u8BufferSize = 0U;
        return;
    }

    uint16_t crc = calcCRC(modH->u8Buffer, (uint8_t)modH->u8BufferSize);
    modH->u8Buffer[modH->u8BufferSize++] = (uint8_t)(crc >> 8);
    modH->u8Buffer[modH->u8BufferSize++] = (uint8_t)(crc & 0xFF);

    if (modH->EN_Port != NULL) {
        HAL_GPIO_WritePin(modH->EN_Port, modH->EN_Pin, GPIO_PIN_SET);
    }

    if (HAL_UART_Transmit_IT(modH->port, modH->u8Buffer, modH->u8BufferSize) != HAL_OK) {
        if (modH->EN_Port != NULL) {
            HAL_GPIO_WritePin(modH->EN_Port, modH->EN_Pin, GPIO_PIN_RESET);
        }
        modH->u16errCnt++;
        modH->u8BufferSize = 0U;
    } else {
        modH->u16OutCnt++;
    }
}

/* ------------------------- Exceptions ------------------------- */
static void send_exception(modbusHandler_t *modH, uint8_t func, uint8_t ex_code)
{
    if (modH == NULL) return;

    modH->u8Buffer[0] = modH->u8id;
    modH->u8Buffer[1] = (uint8_t)(func | 0x80U);
    modH->u8Buffer[2] = ex_code;
    modH->u8BufferSize = 3U;

    transmit_response(modH);
}

/* ------------------------- Slave Task ------------------------- */
void StartTaskModbusSlave(void *argument)
{
    modbusHandler_t *modH = (modbusHandler_t*)argument;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        handle_frame(modH);
        modH->u8BufferSize = 0U;
    }
}

/* ------------------------- Interface functions ------------------------- */
bool mb_read_coil(modbusHandler_t *modH, uint8_t db, int16_t address) {
    (void)modH;
    if ((db >= DB_COUNT) || (address < 0) || (address >= (int16_t)COILS_PER_DB)) return false;

    uint16_t byteIdx = (uint16_t)address / 8U;
    uint8_t  bitPos  = (uint8_t)address % 8U;

    return ((gCoils[db][byteIdx] >> bitPos) & 0x01U) != 0U;
}

void mb_write_coil(modbusHandler_t *modH, uint8_t db, int16_t address, bool state) {
    (void)modH;
    if ((db >= DB_COUNT) || (address < 0) || (address >= (int16_t)COILS_PER_DB)) return;

    uint16_t byteIdx = (uint16_t)address / 8U;
    uint8_t  bitPos  = (uint8_t)address % 8U;

    taskENTER_CRITICAL();
    if (state) gCoils[db][byteIdx] |=  (uint8_t)(1U << bitPos);
    else       gCoils[db][byteIdx] &= (uint8_t)~(1U << bitPos);
    taskEXIT_CRITICAL();
}

uint16_t mb_read_register(modbusHandler_t *modH, uint8_t db, int16_t address) {
    (void)modH;
    if ((db >= DB_COUNT) || (address < 0) || (address >= (int16_t)REGS_PER_DB)) return 0U;

    uint16_t val;
    taskENTER_CRITICAL();
    val = gRegs[db][address];
    taskEXIT_CRITICAL();
    return val;
}

void mb_write_register(modbusHandler_t *modH, uint8_t db, int16_t address,
                       const uint16_t *data, uint16_t len) {
    (void)modH;
    if ((db >= DB_COUNT) || (address < 0) || (data == NULL)) return;

    taskENTER_CRITICAL();
    for (uint16_t i = 0U; (i < len) && ((address + i) < (int16_t)REGS_PER_DB); i++) {
        gRegs[db][address + i] = data[i];
    }
    taskEXIT_CRITICAL();
}

/* ------------------------- HAL Callbacks ------------------------- */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    for (int i = 0; i < numberHandlers; i++)
    {
        modbusHandler_t *mh = mHandlers[i];
        if ((mh != NULL) && (mh->port == huart))
        {
            if (mh->EN_Port != NULL) {
                HAL_GPIO_WritePin(mh->EN_Port, mh->EN_Pin, GPIO_PIN_RESET);
            }

            if (mh->myTaskModbusAHandle != NULL)
            {
                vTaskNotifyGiveFromISR(mh->myTaskModbusAHandle, &xHigherPriorityTaskWoken);
            }

            (void)HAL_UART_Receive_IT(huart, &mh->dataRX, 1U);
            break;
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    for (int i = 0; i < numberHandlers; i++)
    {
        modbusHandler_t *mh = mHandlers[i];
        if ((mh != NULL) && (mh->port == huart))
        {
            if (mh->u8BufferSize < MAX_BUFFER) {
                mh->u8Buffer[mh->u8BufferSize++] = mh->dataRX;
            } else {
                mh->u16errCnt++;
                mh->u8BufferSize = 0U;
            }

            (void)HAL_UART_Receive_IT(huart, &mh->dataRX, 1U);

            if (mh->xTimerT35 != NULL)
            {
                xTimerResetFromISR(mh->xTimerT35, &xHigherPriorityTaskWoken);
            }
            break;
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (int i = 0; i < numberHandlers; i++)
    {
        modbusHandler_t *mh = mHandlers[i];
        if ((mh != NULL) && (mh->port == huart))
        {
#if ENABLE_USART_DMA == 1
            if (mh->xTypeHW == USART_HW_DMA)
            {
                HAL_UART_DMAStop(mh->port);

                int attempts = 0;
                const int max_attempts = 3;
                while ((HAL_UARTEx_ReceiveToIdle_DMA(mh->port, mh->xBufferRX.uxBuffer, MAX_BUFFER) != HAL_OK) &&
                       (++attempts < max_attempts))
                {
                    HAL_UART_DMAStop(mh->port);
                }

                if (attempts >= max_attempts)
                {
                    mh->u16errCnt++;
                }
                else
                {
                    if (mh->port->hdmarx != NULL)
                    {
                        __HAL_DMA_DISABLE_IT(mh->port->hdmarx, DMA_IT_HT);
                    }
                }
            }
#endif
            break;
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    for (int i = 0; i < numberHandlers; i++)
    {
        modbusHandler_t *mh = mHandlers[i];
        if ((mh != NULL) && (mh->port == huart))
        {
#if ENABLE_USART_DMA == 1
            if (mh->xTypeHW == USART_HW_DMA)
            {
                if (Size > 0U)
                {
                    mh->xBufferRX.u8available = Size;
                    mh->xBufferRX.overflow = false;

                    int attempts = 0;
                    const int max_attempts = 3;
                    while ((HAL_UARTEx_ReceiveToIdle_DMA(mh->port, mh->xBufferRX.uxBuffer, MAX_BUFFER) != HAL_OK) &&
                           (++attempts < max_attempts))
                    {
                        HAL_UART_DMAStop(mh->port);
                    }

                    if (attempts >= max_attempts)
                    {
                        mh->u16errCnt++;
                        break;
                    }

                    if (mh->port->hdmarx != NULL)
                    {
                        __HAL_DMA_DISABLE_IT(mh->port->hdmarx, DMA_IT_HT);
                    }

                    if (mh->myTaskModbusAHandle != NULL)
                    {
                        xTaskNotifyFromISR(mh->myTaskModbusAHandle,
                                           (uint32_t)Size,
                                           eSetValueWithOverwrite,
                                           &xHigherPriorityTaskWoken);
                    }
                }
            }
#endif
            break;
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
