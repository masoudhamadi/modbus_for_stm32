#include "Modbus.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "cmsis_os.h"

modbusHandler_t gModbusH;
uint8_t  gCoils[DB_COUNT][COILS_PER_DB / 8] = {0};
uint16_t gRegs[DB_COUNT][REGS_PER_DB]       = {0};

static uint16_t word(uint8_t H, uint8_t L) { return ((uint16_t)H << 8) | (uint16_t)L; }

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

/* ------------------------- Init & start ------------------------- */
void ModbusInit(UART_HandleTypeDef *huart,
                uint8_t slave_id,
                GPIO_TypeDef *en_port,
                uint16_t en_pin)
{
    gModbusH.port    = huart;
    gModbusH.u8id    = slave_id;
    gModbusH.EN_Port = en_port;
    gModbusH.EN_Pin  = en_pin;
    gModbusH.xTypeHW = USART_HW_DMA;

    gModbusH.u16timeOut  = MODBUS_TIMEOUT_MS;
    gModbusH.u8BufferSize = 0U;
    gModbusH.u16InCnt = 0U;
    gModbusH.u16OutCnt = 0U;
    gModbusH.u16errCnt = 0U;
    memset(&gModbusH.xBufferRX, 0, sizeof(gModbusH.xBufferRX));
}

void ModbusStart(void) {
    if (gModbusH.EN_Port != NULL) {
        HAL_GPIO_WritePin(gModbusH.EN_Port, gModbusH.EN_Pin, GPIO_PIN_RESET);
    }
    HAL_UARTEx_ReceiveToIdle_DMA(gModbusH.port, gModbusH.xBufferRX.uxBuffer, MAX_BUFFER);
    if (gModbusH.port->hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(gModbusH.port->hdmarx, DMA_IT_HT);
    }
}

/* ------------------------- Function Codes ------------------------- */
/* نمونه: FC1 */
static void process_FC1(void) {
    uint16_t rawAddr   = word(gModbusH.u8Buffer[2], gModbusH.u8Buffer[3]);
    uint8_t  db        = MB_ADDR_DECODE_DB(rawAddr);
    uint16_t startCoil = MB_ADDR_DECODE_OFFSET(rawAddr);
    uint16_t coilCount = word(gModbusH.u8Buffer[4], gModbusH.u8Buffer[5]);

    if (coilCount == 0U || coilCount > 2000U) { gModbusH.u16errCnt++; return; }
    if ((db >= DB_COUNT) || ((uint32_t)startCoil + coilCount > COILS_PER_DB)) { gModbusH.u16errCnt++; return; }

    uint8_t byteCount = (uint8_t)((coilCount + 7U) / 8U);
    gModbusH.u8Buffer[0] = gModbusH.u8id;
    gModbusH.u8Buffer[1] = 1U;
    gModbusH.u8Buffer[2] = byteCount;
    gModbusH.u8BufferSize = 3U;
    memset(&gModbusH.u8Buffer[3], 0, byteCount);

    for (uint16_t i = 0U; i < coilCount; i++) {
        uint16_t idx = startCoil + i;
        uint16_t byteIdx = idx / 8U;
        uint8_t bitPos = (uint8_t)(idx % 8U);
        uint8_t val = (gCoils[db][byteIdx] >> bitPos) & 0x01U;
        if (val) gModbusH.u8Buffer[3U + (i / 8U)] |= (uint8_t)(1U << (i % 8U));
    }
    gModbusH.u8BufferSize = (uint8_t)(3U + byteCount);
}

/* مشابه همین منطق برای FC3, FC5, FC6, FC15, FC16 نوشته می‌شود
   (همان کدی که قبلاً داشتی، فقط با gModbusH به جای modH و بدون حلقه‌ی هندلرها) */

/* ------------------------- Frame handler ------------------------- */
static void handle_frame(void)
{
    if (gModbusH.xBufferRX.u8available == 0U) return;
    uint16_t size = (gModbusH.xBufferRX.u8available > MAX_BUFFER) ? MAX_BUFFER : gModbusH.xBufferRX.u8available;
    memcpy(gModbusH.u8Buffer, gModbusH.xBufferRX.uxBuffer, size);
    gModbusH.u8BufferSize = (uint8_t)size;
    gModbusH.xBufferRX.u8available = 0U;

    if (gModbusH.u8BufferSize < 4U) { gModbusH.u16errCnt++; gModbusH.u8BufferSize = 0U; return; }

    uint16_t recv_crc = (uint16_t)((gModbusH.u8Buffer[gModbusH.u8BufferSize - 2] << 8) |
                                   gModbusH.u8Buffer[gModbusH.u8BufferSize - 1]);
    uint16_t calc = calcCRC(gModbusH.u8Buffer, (uint8_t)(gModbusH.u8BufferSize - 2U));
    if (calc != recv_crc) { gModbusH.u16errCnt++; gModbusH.u8BufferSize = 0U; return; }

    uint8_t addr = gModbusH.u8Buffer[0];
        if ((addr != gModbusH.u8id) && (addr != 0x00U)) {
        gModbusH.u8BufferSize = 0U;
        return;
    }

    uint8_t func = gModbusH.u8Buffer[1];
    switch (func) {
        case 1U:  process_FC1(); break;
        case 3U:  /* process_FC3(); */ break;
        case 5U:  /* process_FC5(); */ break;
        case 6U:  /* process_FC6(); */ break;
        case 15U: /* process_FC15(); */ break;
        case 16U: /* process_FC16(); */ break;
        default:  gModbusH.u16errCnt++; break;
    }

    if (addr != 0x00U && gModbusH.u8BufferSize > 0U) {
        transmit_response();
        gModbusH.u16OutCnt++;
    } else {
        gModbusH.u8BufferSize = 0U;
    }
}

/* ------------------------- Transmit response ------------------------- */
static void transmit_response(void)
{
    if ((uint16_t)gModbusH.u8BufferSize + 2U > MAX_BUFFER) {
        gModbusH.u16errCnt++;
        gModbusH.u8BufferSize = 0U;
        return;
    }

    uint16_t crc = calcCRC(gModbusH.u8Buffer, (uint8_t)gModbusH.u8BufferSize);
    gModbusH.u8Buffer[gModbusH.u8BufferSize++] = (uint8_t)(crc >> 8);
    gModbusH.u8Buffer[gModbusH.u8BufferSize++] = (uint8_t)(crc & 0xFF);

    if (gModbusH.EN_Port != NULL) {
        HAL_GPIO_WritePin(gModbusH.EN_Port, gModbusH.EN_Pin, GPIO_PIN_SET);
    }

    if (HAL_UART_Transmit_IT(gModbusH.port, gModbusH.u8Buffer, gModbusH.u8BufferSize) != HAL_OK) {
        if (gModbusH.EN_Port != NULL) {
            HAL_GPIO_WritePin(gModbusH.EN_Port, gModbusH.EN_Pin, GPIO_PIN_RESET);
        }
        gModbusH.u16errCnt++;
        gModbusH.u8BufferSize = 0U;
    }
}

/* ------------------------- Slave Task ------------------------- */
void StartTaskModbusSlave(void *argument)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        handle_frame();
        gModbusH.u8BufferSize = 0U;
    }
}

/* ------------------------- Interface functions ------------------------- */
bool mb_read_coil(uint8_t db, int16_t address) {
    if ((db >= DB_COUNT) || (address < 0) || (address >= (int16_t)COILS_PER_DB)) return false;
    uint16_t byteIdx = (uint16_t)address / 8U;
    uint8_t  bitPos  = (uint8_t)address % 8U;
    return ((gCoils[db][byteIdx] >> bitPos) & 0x01U) != 0U;
}

void mb_write_coil(uint8_t db, int16_t address, bool state) {
    if ((db >= DB_COUNT) || (address < 0) || (address >= (int16_t)COILS_PER_DB)) return;
    uint16_t byteIdx = (uint16_t)address / 8U;
    uint8_t  bitPos  = (uint8_t)address % 8U;
    taskENTER_CRITICAL();
    if (state) gCoils[db][byteIdx] |=  (uint8_t)(1U << bitPos);
    else       gCoils[db][byteIdx] &= (uint8_t)~(1U << bitPos);
    taskEXIT_CRITICAL();
}

uint16_t mb_read_register(uint8_t db, int16_t address) {
    if ((db >= DB_COUNT) || (address < 0) || (address >= (int16_t)REGS_PER_DB)) return 0U;
    uint16_t val;
    taskENTER_CRITICAL();
    val = gRegs[db][address];
    taskEXIT_CRITICAL();
    return val;
}

void mb_write_register(uint8_t db, int16_t address, const uint16_t *data, uint16_t len) {
    if ((db >= DB_COUNT) || (address < 0) || (data == NULL)) return;
    taskENTER_CRITICAL();
    for (uint16_t i = 0U; (i < len) && ((address + i) < (int16_t)REGS_PER_DB); i++) {
        gRegs[db][address + i] = data[i];
    }
    taskEXIT_CRITICAL();
}

/* ------------------------- HAL Callbacks (DMA only) ------------------------- */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (gModbusH.port == huart) {
        if (gModbusH.EN_Port != NULL) {
            HAL_GPIO_WritePin(gModbusH.EN_Port, gModbusH.EN_Pin, GPIO_PIN_RESET);
        }
        if (gModbusH.myTaskModbusAHandle != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(gModbusH.myTaskModbusAHandle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (gModbusH.port == huart) {
        HAL_UART_DMAStop(gModbusH.port);
        HAL_UARTEx_ReceiveToIdle_DMA(gModbusH.port, gModbusH.xBufferRX.uxBuffer, MAX_BUFFER);
        if (gModbusH.port->hdmarx != NULL) {
            __HAL_DMA_DISABLE_IT(gModbusH.port->hdmarx, DMA_IT_HT);
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (gModbusH.port == huart && Size > 0U) {
        gModbusH.xBufferRX.u8available = Size;
        gModbusH.xBufferRX.overflow = false;
        HAL_UARTEx_ReceiveToIdle_DMA(gModbusH.port, gModbusH.xBufferRX.uxBuffer, MAX_BUFFER);
        if (gModbusH.port->hdmarx != NULL) {
            __HAL_DMA_DISABLE_IT(gModbusH.port->hdmarx, DMA_IT_HT);
        }
        if (gModbusH.myTaskModbusAHandle != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xTaskNotifyFromISR(gModbusH.myTaskModbusAHandle,
                               (uint32_t)Size,
                               eSetValueWithOverwrite,
                               &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}
