/*
 * Modbus.c
 *  Modbus RTU Master and Slave library for STM32 CUBE with FreeRTOS
 *  Created on: May 5, 2020
 *      Author: Alejandro Mera
 *      Adapted from https://github.com/smarmengol/Modbus-Master-Slave-for-Arduino
 */

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "queue.h"
#include "main.h"
#include "Modbus.h"
#include "timers.h"
#include "semphr.h"

/* =======================================================================
 * 🟦 COMMON SECTION
 * Globals, configuration attributes, ring buffer, timers, validation,
 * CRC, word, exceptions, RX/TX buffer handling, init/start.
 * Static functions are grouped at the top of this section.
 * ======================================================================= */

/* --- Common static prototypes (local to common section) --- */
static void     sendTxBuffer(modbusHandler_t *modH);
static int16_t  getRxBuffer(modbusHandler_t *modH);
static uint8_t  validateAnswer(modbusHandler_t *modH);
static void     buildException(uint8_t u8exception, modbusHandler_t *modH);
static int8_t   validateRequest(modbusHandler_t *modH);
static uint16_t word(uint8_t H, uint8_t L);
static void     vTimerCallbackT35(TimerHandle_t *pxTimer);
static void     vTimerCallbackTimeout(TimerHandle_t *pxTimer);
modbusHandler_t *mHandlers[MAX_M_HANDLERS];
uint8_t numberHandlers = 0;

/// Queue Modbus telegrams for master
const osMessageQueueAttr_t QueueTelegram_attributes = { .name = "QueueModbusTelegram" };

const osThreadAttr_t myTaskModbusA_attributes = {
    .name = "TaskModbusSlave",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 128 * 20
};


/// Semaphore to access the Modbus data
const osSemaphoreAttr_t ModBusSphr_attributes = { .name = "ModBusSphr" };

const unsigned char fctsupported[] = {
    MB_FC_READ_COILS,
    MB_FC_READ_DISCRETE_INPUT,
    MB_FC_READ_REGISTERS,
    MB_FC_READ_INPUT_REGISTER,
    MB_FC_WRITE_COIL,
    MB_FC_WRITE_REGISTER,
    MB_FC_WRITE_MULTIPLE_COILS,
    MB_FC_WRITE_MULTIPLE_REGISTERS
};

/* --- Ring buffer functions --- */
// This function must be called only after disabling USART RX interrupt or inside of the RX interrupt
void RingAdd(modbusRingBuffer_t *xRingBuffer, uint8_t u8Val)
{
    xRingBuffer->uxBuffer[xRingBuffer->u8end] = u8Val;
    xRingBuffer->u8end = (xRingBuffer->u8end + 1) % MAX_BUFFER;
    if (xRingBuffer->u8available == MAX_BUFFER)
    {
        xRingBuffer->overflow = true;
        xRingBuffer->u8start = (xRingBuffer->u8start + 1) % MAX_BUFFER;
    }
    else
    {
        xRingBuffer->overflow = false;
        xRingBuffer->u8available++;
    }
}

// This function must be called only after disabling USART RX interrupt
uint8_t RingGetAllBytes(modbusRingBuffer_t *xRingBuffer, uint8_t *buffer)
{
    return RingGetNBytes(xRingBuffer, buffer, xRingBuffer->u8available);
}

// This function must be called only after disabling USART RX interrupt
uint8_t RingGetNBytes(modbusRingBuffer_t *xRingBuffer, uint8_t *buffer, uint8_t uNumber)
{
    uint8_t uCounter;
    if (xRingBuffer->u8available == 0 || uNumber == 0) return 0;
    if (uNumber > MAX_BUFFER) return 0;

    for (uCounter = 0; uCounter < uNumber && uCounter < xRingBuffer->u8available; uCounter++)
    {
        buffer[uCounter] = xRingBuffer->uxBuffer[xRingBuffer->u8start];
        xRingBuffer->u8start = (xRingBuffer->u8start + 1) % MAX_BUFFER;
    }
    xRingBuffer->u8available = xRingBuffer->u8available - uCounter;
    xRingBuffer->overflow = false;
    RingClear(xRingBuffer);

    return uCounter;
}

uint8_t RingCountBytes(modbusRingBuffer_t *xRingBuffer)
{
    return xRingBuffer->u8available;
}

void RingClear(modbusRingBuffer_t *xRingBuffer)
{
    xRingBuffer->u8start = 0;
    xRingBuffer->u8end = 0;
    xRingBuffer->u8available = 0;
    xRingBuffer->overflow = false;
}

/* --- Initialization and start --- */
void ModbusInit(modbusHandler_t *modH)
{
    if (numberHandlers < MAX_M_HANDLERS)
    {
        // Initialize the ring buffer
        RingClear(&modH->xBufferRX);

        if (modH->uModbusType == MB_SLAVE)
        {
            // Create Modbus task slave
            modH->myTaskModbusAHandle = osThreadNew(StartTaskModbusSlave, modH, &myTaskModbusA_attributes);
        }

        else
        {
            while (1) { /* Error Modbus type not supported */ }
        }

        if (modH->myTaskModbusAHandle == NULL)
        {
            while (1) { /* Error creating Modbus task */ }
        }

        modH->xTimerT35 = xTimerCreate(
            "TimerT35",                    // name
            T35,                           // period in ticks
            pdFALSE,                       // one-shot
            (void *)modH->xTimerT35,       // id
            (TimerCallbackFunction_t)vTimerCallbackT35
        );
        if (modH->xTimerT35 == NULL)
        {
            while (1) { /* Error creating the timer */ }
        }

        modH->ModBusSphrHandle = osSemaphoreNew(1, 1, &ModBusSphr_attributes);
        if (modH->ModBusSphrHandle == NULL)
        {
            while (1) { /* Error creating the semaphore */ }
        }

        mHandlers[numberHandlers] = modH;
        numberHandlers++;
    }
    else
    {
        while (1) { /* error no more Modbus handlers supported */ }
    }
}

void ModbusStart(modbusHandler_t *modH)
{
    if (modH->xTypeHW != USART_HW && modH->xTypeHW != USART_HW_DMA)
    {
        while (1) { /* ERROR select the type of hardware */ }
    }

    if (modH->xTypeHW == USART_HW_DMA && ENABLE_USART_DMA == 0)
    {
        while (1) { /* ERROR To use USART_HW_DMA enable it in ModbusConfig.h */ }
    }

    if (modH->xTypeHW == USART_HW || modH->xTypeHW == USART_HW_DMA)
    {
        if (modH->EN_Port != NULL)
        {
            // return RS485 transceiver to transmit mode
            HAL_GPIO_WritePin(modH->EN_Port, modH->EN_Pin, GPIO_PIN_RESET);
        }

        if (modH->uModbusType == MB_SLAVE && modH->u16regsHR == NULL)
        {
            while (1) { /* ERROR define the DATA pointer shared through Modbus */ }
        }

        // check that port is initialized
        while (HAL_UART_GetState(modH->port) != HAL_UART_STATE_READY)
        {
        }

#if ENABLE_USART_DMA == 1
        if (modH->xTypeHW == USART_HW_DMA)
        {
            if (HAL_UARTEx_ReceiveToIdle_DMA(modH->port, modH->xBufferRX.uxBuffer, MAX_BUFFER) != HAL_OK)
            {
                while (1) { /* error in initialization code */ }
            }
            __HAL_DMA_DISABLE_IT(modH->port->hdmarx, DMA_IT_HT); // we don't need half-transfer interrupt
        }
        else
        {
            // Receive data from serial port for Modbus using interrupt
            if (HAL_UART_Receive_IT(modH->port, &modH->dataRX, 1) != HAL_OK)
            {
                while (1) { /* error in initialization code */ }
            }
        }
#else
        // Receive data from serial port for Modbus using interrupt
        if (HAL_UART_Receive_IT(modH->port, &modH->dataRX, 1) != HAL_OK)
        {
            while (1) { /* error in initialization code */ }
        }
#endif

        if (modH->u8id != 0 && modH->uModbusType == MB_MASTER)
        {
            while (1) { /* error Master ID must be zero */ }
        }

        if (modH->u8id == 0 && modH->uModbusType == MB_SLAVE)
        {
            while (1) { /* error Slave ID must be non-zero */ }
        }
    }

    modH->u8lastRec = modH->u8BufferSize = 0;
    modH->u16InCnt = modH->u16OutCnt = modH->u16errCnt = 0;
}

/* --- Timer callbacks --- */
static void vTimerCallbackT35(TimerHandle_t *pxTimer)
{
    // Notify that a stream has just arrived
    for (int i = 0; i < numberHandlers; i++)
    {
        if ((TimerHandle_t *)mHandlers[i]->xTimerT35 == pxTimer)
        {
            if (mHandlers[i]->uModbusType == MB_MASTER)
            {
                xTimerStop(mHandlers[i]->xTimerTimeout, 0);
            }
            xTaskNotify(mHandlers[i]->myTaskModbusAHandle, 0, eSetValueWithOverwrite);
        }
    }
}

 static void vTimerCallbackTimeout(TimerHandle_t *pxTimer)
{
    for (int i = 0; i < numberHandlers; i++)
    {
        if ((TimerHandle_t *)mHandlers[i]->xTimerTimeout == pxTimer)
        {
            xTaskNotify(mHandlers[i]->myTaskModbusAHandle, ERR_TIME_OUT, eSetValueWithOverwrite);
        }
    }
}

/* --- RX buffer --- */
static int16_t getRxBuffer(modbusHandler_t *modH)
{
    int16_t i16result;

    if (modH->xTypeHW == USART_HW)
    {
        HAL_UART_AbortReceive_IT(modH->port); // disable interrupts to avoid race conditions on serial port
    }

    if (modH->xBufferRX.overflow)
    {
        RingClear(&modH->xBufferRX); // clean up the overflowed buffer
        i16result = ERR_BUFF_OVERFLOW;
    }
    else
    {
        modH->u8BufferSize = RingGetAllBytes(&modH->xBufferRX, modH->u8Buffer);
        modH->u16InCnt++;
        i16result = modH->u8BufferSize;
    }

    if (modH->xTypeHW == USART_HW)
    {
        HAL_UART_Receive_IT(modH->port, &modH->dataRX, 1);
    }

    return i16result;
}

/* --- Master answer validation --- */
static uint8_t validateAnswer(modbusHandler_t *modH)
{
    // check message crc vs calculated crc
    uint16_t u16MsgCRC = ((modH->u8Buffer[modH->u8BufferSize - 2] << 8)
                         | modH->u8Buffer[modH->u8BufferSize - 1]); // combine CRC

    if (calcCRC(modH->u8Buffer, modH->u8BufferSize - 2) != u16MsgCRC)
    {
        modH->u16errCnt++;
        return ERR_BAD_CRC;
    }

    // check exception
    if ((modH->u8Buffer[FUNC] & 0x80) != 0)
    {
        modH->u16errCnt++;
        return ERR_EXCEPTION;
    }

    // check fct code
    bool isSupported = false;
    for (uint8_t i = 0; i < sizeof(fctsupported); i++)
    {
        if (fctsupported[i] == modH->u8Buffer[FUNC])
        {
            isSupported = true;
            break;
        }
    }
    if (!isSupported)
    {
        modH->u16errCnt++;
        return EXC_FUNC_CODE;
    }

    return 0; // OK
}

/* --- Slave request validation --- */
static int8_t validateRequest(modbusHandler_t *modH)
{
    // check message crc vs calculated crc
    uint16_t u16MsgCRC = ((modH->u8Buffer[modH->u8BufferSize - 2] << 8)
                         | modH->u8Buffer[modH->u8BufferSize - 1]);

    if (calcCRC(modH->u8Buffer, modH->u8BufferSize - 2) != u16MsgCRC)
    {
        modH->u16errCnt++;
        return ERR_BAD_CRC; // negative value in original ERR_LIST
    }

    // check fct code
    bool isSupported = false;
    for (uint8_t i = 0; i < sizeof(fctsupported); i++)
    {
        if (fctsupported[i] == modH->u8Buffer[FUNC])
        {
            isSupported = true;
            break;
        }
    }
    if (!isSupported)
    {
        modH->u16errCnt++;
        return EXC_FUNC_CODE; // positive (exception code)
    }

    // check start address & nb range
    uint16_t u16AdRegs = 0;
    uint16_t u16NRegs = 0;

    switch (modH->u8Buffer[FUNC])
    {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUT:
        case MB_FC_WRITE_MULTIPLE_COILS:
            u16AdRegs = word(modH->u8Buffer[ADD_HI], modH->u8Buffer[ADD_LO]) / 16;
            u16NRegs  = word(modH->u8Buffer[NB_HI], modH->u8Buffer[NB_LO]) / 16;
            if (word(modH->u8Buffer[NB_HI], modH->u8Buffer[NB_LO]) % 16) u16NRegs++;
            if ((u16AdRegs + u16NRegs) > modH->u16regCoils_size) return EXC_ADDR_RANGE;

            u16NRegs = word(modH->u8Buffer[NB_HI], modH->u8Buffer[NB_LO]) / 8;
            if (word(modH->u8Buffer[NB_HI], modH->u8Buffer[NB_LO]) % 8) u16NRegs++;
            u16NRegs = u16NRegs + 5; // header + CRC
            if (u16NRegs > 256) return EXC_REGS_QUANT;
            break;

        case MB_FC_WRITE_COIL:
            u16AdRegs = word(modH->u8Buffer[ADD_HI], modH->u8Buffer[ADD_LO]) / 16;
            if (word(modH->u8Buffer[ADD_HI], modH->u8Buffer[ADD_LO]) % 16) u16AdRegs++;
            if (u16AdRegs > modH->u16regCoils_size) return EXC_ADDR_RANGE;
            break;

        case MB_FC_WRITE_REGISTER:
            u16AdRegs = word(modH->u8Buffer[ADD_HI], modH->u8Buffer[ADD_LO]);
            if (u16AdRegs > modH->u16regHR_size) return EXC_ADDR_RANGE;
            break;

        case MB_FC_READ_REGISTERS:
        case MB_FC_READ_INPUT_REGISTER:
        case MB_FC_WRITE_MULTIPLE_REGISTERS:
            u16AdRegs = word(modH->u8Buffer[ADD_HI], modH->u8Buffer[ADD_LO]);
            u16NRegs  = word(modH->u8Buffer[NB_HI], modH->u8Buffer[NB_LO]);
            if ((u16AdRegs + u16NRegs) > modH->u16regHR_size) return EXC_ADDR_RANGE;

            u16NRegs = u16NRegs * 2 + 5; // header + CRC
            if (u16NRegs > 256) return EXC_REGS_QUANT;
            break;
    }

    return 0; // OK
}

/* --- Helpers: word and CRC --- */
static uint16_t word(uint8_t H, uint8_t L)
{
    bytesFields W;
    W.u8[0] = L;
    W.u8[1] = H;
    return W.u16[0];
}

uint16_t calcCRC(uint8_t *Buffer, uint8_t u8length)
{
    unsigned int temp, temp2, flag;
    temp = 0xFFFF;
    for (unsigned char i = 0; i < u8length; i++)
    {
        temp = temp ^ Buffer[i];
        for (unsigned char j = 1; j <= 8; j++)
        {
            flag = temp & 0x0001;
            temp >>= 1;
            if (flag) temp ^= 0xA001;
        }
    }
    // Reverse byte order.
    temp2 = temp >> 8;
    temp = (temp << 8) | temp2;
    temp &= 0xFFFF;
    // the returned value is already swapped (crcLo first, crcHi last)
    return temp;
}

/* --- Build exception --- */
static void buildException(uint8_t u8exception, modbusHandler_t *modH)
{
    uint8_t u8func = modH->u8Buffer[FUNC];  // original FUNC code

    modH->u8Buffer[ID]   = modH->u8id;
    modH->u8Buffer[FUNC] = u8func + 0x80;
    modH->u8Buffer[2]    = u8exception;
    modH->u8BufferSize   = EXCEPTION_SIZE;
}

/* --- Transmit buffer --- */
static void sendTxBuffer(modbusHandler_t *modH)
{
    // append CRC to message
#if ENABLE_TCP == 1
    if (modH->xTypeHW != TCP_HW)
    {
#endif
        uint16_t u16crc = calcCRC(modH->u8Buffer, modH->u8BufferSize);
        modH->u8Buffer[modH->u8BufferSize] = (u16crc >> 8);
        modH->u8BufferSize++;
        modH->u8Buffer[modH->u8BufferSize] = (u16crc & 0x00ff);
        modH->u8BufferSize++;
#if ENABLE_TCP == 1
    }
#endif

#if ENABLE_USB_CDC == 1 || ENABLE_TCP == 1
    if (modH->xTypeHW == USART_HW || modH->xTypeHW == USART_HW_DMA)
    {
#endif
        if (modH->EN_Port != NULL)
        {
            // enable transmitter, disable receiver to avoid echo on RS485 transceivers
            HAL_HalfDuplex_EnableTransmitter(modH->port);
            HAL_GPIO_WritePin(modH->EN_Port, modH->EN_Pin, GPIO_PIN_SET);
        }

#if ENABLE_USART_DMA == 1
        if (modH->xTypeHW == USART_HW)
        {
#endif
            // transfer buffer to serial line IT
            HAL_UART_Transmit_IT(modH->port, modH->u8Buffer, modH->u8BufferSize);
#if ENABLE_USART_DMA == 1
        }
        else
        {
            // transfer buffer to serial line DMA
            HAL_UART_Transmit_DMA(modH->port, modH->u8Buffer, modH->u8BufferSize);
        }
#endif

        ulTaskNotifyTake(pdTRUE, 250); // wait notification from TXE interrupt

#if defined(STM32H7) || defined(STM32F3) || defined(STM32L4) || defined(STM32L082xx) || defined(STM32F7) || defined(STM32WB) || defined(STM32G070xx) || defined(STM32F0) || defined(STM32G431xx) || defined(STM32H5)
        while ((modH->port->Instance->ISR & USART_ISR_TC) == 0)
#else
        // F429, F103, L152 ...
        while ((modH->port->Instance->SR & USART_SR_TC) == 0)
#endif
        {
            // block the task until the last byte is sent from USART shift register
        }

        if (modH->EN_Port != NULL)
        {
            // return RS485 transceiver to receive mode
            HAL_GPIO_WritePin(modH->EN_Port, modH->EN_Pin, GPIO_PIN_RESET);
            // enable receiver, disable transmitter
            HAL_HalfDuplex_EnableReceiver(modH->port);
        }

        // set timeout for master query
        if (modH->uModbusType == MB_MASTER)
        {
            xTimerReset(modH->xTimerTimeout, 0);
        }
#if ENABLE_USB_CDC == 1 || ENABLE_TCP == 1
    }

#if ENABLE_USB_CDC == 1
    else if (modH->xTypeHW == USB_CDC_HW)
    {
        CDC_Transmit_FS(modH->u8Buffer, modH->u8BufferSize);
        if (modH->uModbusType == MB_MASTER)
        {
            xTimerReset(modH->xTimerTimeout, 0);
        }
    }
#endif

#if ENABLE_TCP == 1
    else if (modH->xTypeHW == TCP_HW)
    {
        struct netvector  xNetVectors[2];
        uint8_t u8MBAPheader[6];
        size_t uBytesWritten;

        u8MBAPheader[0] = highByte(modH->u16TransactionID);
        u8MBAPheader[1] = lowByte(modH->u16TransactionID);
        u8MBAPheader[2] = 0; // protocol ID
        u8MBAPheader[3] = 0; // protocol ID
        u8MBAPheader[4] = 0; // highbyte data length always 0
        u8MBAPheader[5] = modH->u8BufferSize; // data length

        xNetVectors[0].len = 6;
        xNetVectors[0].ptr = (void *)u8MBAPheader;

        xNetVectors[1].len = modH->u8BufferSize;
        xNetVectors[1].ptr = (void *)modH->u8Buffer;

        netconn_set_sendtimeout(modH->newconns[modH->newconnIndex].conn, modH->u16timeOut);
        err_enum_t err;

        err = netconn_write_vectors_partly(modH->newconns[modH->newconnIndex].conn, xNetVectors, 2, NETCONN_COPY, &uBytesWritten);
        if (err != ERR_OK)
        {
            // ModbusCloseConn(modH->newconns[modH->newconnIndex].conn);
            ModbusCloseConnNull(modH);
        }

        if (modH->uModbusType == MB_MASTER)
        {
            xTimerReset(modH->xTimerTimeout, 0);
        }
    }
#endif
#endif

    modH->u8BufferSize = 0;
    modH->u16OutCnt++;
}

/* =======================================================================
 * 🟩 SLAVE SECTION
 * Slave task and static FC processing functions. Static functions are placed
 * at the top of the section.
 * ======================================================================= */

/* --- Slave static prototypes (local to slave section) --- */
static int8_t process_FC1(modbusHandler_t *modH, uint8_t database);   // read coils (bit-packed)
static int8_t process_FC3(modbusHandler_t *modH, uint8_t database);   // read registers
static int8_t process_FC5(modbusHandler_t *modH, uint8_t database);   // write single coil
static int8_t process_FC6(modbusHandler_t *modH, uint8_t database);   // write single register
static int8_t process_FC15(modbusHandler_t *modH, uint8_t database);  // write multiple coils
static int8_t process_FC16(modbusHandler_t *modH, uint8_t database);  // write multiple registers

void StartTaskModbusSlave(void *argument)
{
    modbusHandler_t *modH = (modbusHandler_t *)argument;

    for (;;)
    {
#if __stack_show
        if (osMutexAcquire(printf_mutex, osWaitForever) == osOK)
        {
            printf("modbus Stack left: %lu words\n", uxTaskGetStackHighWaterMark(NULL));
            osMutexRelease(printf_mutex);
        }
#endif

        modH->i8lastError = 0;

        if (modH->xTypeHW == USART_HW || modH->xTypeHW == USART_HW_DMA)
        {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY); /* Block until a Modbus Frame arrives */

            if (getRxBuffer(modH) == ERR_BUFF_OVERFLOW)
            {
                modH->i8lastError = ERR_BUFF_OVERFLOW;
                modH->u16errCnt++;
                continue;
            }
        }

        if (modH->u8BufferSize < 7)
        {
            modH->i8lastError = ERR_BAD_SIZE;
            modH->u16errCnt++;
            continue;
        }

        // Check Slave ID
        if (modH->u8Buffer[ID] != modH->u8id)
        {
            continue;
        }

        // Validate message
        int8_t u8exception = validateRequest(modH);
        if (u8exception != 0)
        {
            if (u8exception > 0)   // Modbus exception (positive)
            {
                buildException(u8exception, modH);
                sendTxBuffer(modH);
            }
            modH->i8lastError = u8exception;
            continue;
        }

        modH->i8lastError = 0;
        xSemaphoreTake(modH->ModBusSphrHandle, portMAX_DELAY);

        // Process message
        switch (modH->u8Buffer[FUNC])
        {
            case MB_FC_READ_COILS:
                modH->i8state = process_FC1(modH, DB_COILS);
                break;

            case MB_FC_READ_DISCRETE_INPUT:
                modH->i8state = process_FC1(modH, DB_INPUT_COILS);
                break;

            case MB_FC_READ_REGISTERS:
                modH->i8state = process_FC3(modH, DB_HOLDING_REGISTER);
                break;

            case MB_FC_READ_INPUT_REGISTER:
                modH->i8state = process_FC3(modH, DB_INPUT_REGISTERS);
                break;

            case MB_FC_WRITE_COIL:
                modH->i8state = process_FC5(modH, DB_COILS);
                break;

            case MB_FC_WRITE_REGISTER:
                modH->i8state = process_FC6(modH, DB_HOLDING_REGISTER);
                break;

            case MB_FC_WRITE_MULTIPLE_COILS:
                modH->i8state = process_FC15(modH, DB_COILS);
                break;

            case MB_FC_WRITE_MULTIPLE_REGISTERS:
                modH->i8state = process_FC16(modH, DB_HOLDING_REGISTER);
                break;

            default:
                break;
        }

        xSemaphoreGive(modH->ModBusSphrHandle);
        continue;
    }
}

/* ------------------------- FC1: read coils (bit-packed) ------------------------- */
static int8_t process_FC1(modbusHandler_t *modH, uint8_t database)
{
    uint16_t startCoilX = word(modH->u8Buffer[ADD_HI], modH->u8Buffer[ADD_LO]);
    uint16_t coilCount  = word(modH->u8Buffer[NB_HI],  modH->u8Buffer[NB_LO]);

    uint8_t byteCount = (coilCount + 7) / 8;
    modH->u8Buffer[ADD_HI] = byteCount;
    modH->u8BufferSize     = ADD_LO;
    memset(&modH->u8Buffer[modH->u8BufferSize], 0, byteCount);

    // Compute start byte address in EEPROM
    uint16_t startByteAddr = COILS_BASE_ADDR + (database * (COILS_PER_DB / 8)) + (startCoilX / 8);
    uint16_t bitOffset     = startCoilX % 8;
    uint16_t totalBytes    = (coilCount + bitOffset + 7) / 8;

    uint8_t *coilBytes = (uint8_t *)malloc(totalBytes);
    if (!coilBytes) return -1;
    if (!at24_read(startByteAddr, coilBytes, totalBytes, 100)) { free(coilBytes); return -1; }

    uint8_t bitIndex = 0;
    for (uint16_t i = 0; i < coilCount; i++)
    {
        uint16_t byteIndex = (bitOffset + i) / 8;
        uint8_t  bitPos    = (bitOffset + i) % 8;
        uint8_t  coilVal   = bitRead(coilBytes[byteIndex], bitPos);

        bitWrite(modH->u8Buffer[modH->u8BufferSize], bitIndex, coilVal);

        bitIndex++;
        if (bitIndex > 7) { bitIndex = 0; modH->u8BufferSize++; }
    }
    if (coilCount % 8 != 0) modH->u8BufferSize++;

    free(coilBytes);
    uint8_t copyBufferSize = modH->u8BufferSize + 2;
    sendTxBuffer(modH);
    return copyBufferSize;
}

/* ------------------------- FC3: read registers ------------------------- */
static int8_t process_FC3(modbusHandler_t *modH, uint8_t database)
{
    uint16_t startRegX = word(modH->u8Buffer[ADD_HI], modH->u8Buffer[ADD_LO]);
    uint16_t regsCount = word(modH->u8Buffer[NB_HI],  modH->u8Buffer[NB_LO]);
    uint8_t  byteCount = regsCount * 2;

    modH->u8Buffer[2]  = byteCount;
    modH->u8BufferSize = 3;

    uint16_t eepromAddr = REGS_BASE_ADDR + (database * REGS_PER_DB * 2) + (startRegX * 2);

    uint8_t *regsBytes = (uint8_t *)malloc(byteCount);
    if (!regsBytes) return -1;
    if (!at24_read(eepromAddr, regsBytes, byteCount, 100)) { free(regsBytes); return -1; }

    for (uint16_t i = 0; i < regsCount; i++)
    {
        modH->u8Buffer[modH->u8BufferSize++] = regsBytes[i*2];
        modH->u8Buffer[modH->u8BufferSize++] = regsBytes[i*2+1];
    }

    free(regsBytes);
    uint8_t copyBufferSize = modH->u8BufferSize + 2;
    sendTxBuffer(modH);
    return copyBufferSize;
}

/* ------------------------- FC5: write single coil ------------------------- */
static int8_t process_FC5(modbusHandler_t *modH, uint8_t database)
{
    uint16_t coilX = word(modH->u8Buffer[ADD_HI], modH->u8Buffer[ADD_LO]);
    bool coilOn    = (modH->u8Buffer[NB_HI] == 0xFF);

    uint16_t byteAddr = COILS_BASE_ADDR + (database * (COILS_PER_DB / 8)) + (coilX / 8);
    uint8_t  bitPos   = coilX % 8;

    uint8_t byteVal;
    if (!at24_read(byteAddr, &byteVal, 1, 100)) return -1;
    bitWrite(byteVal, bitPos, coilOn);
    if (!at24_write(byteAddr, &byteVal, 1, 100)) return -1;

    modH->u8BufferSize = 6;
    uint8_t copyBufferSize = modH->u8BufferSize + 2;
    sendTxBuffer(modH);
    return copyBufferSize;
}

/* ------------------------- FC6: write single register ------------------------- */
static int8_t process_FC6(modbusHandler_t *modH, uint8_t database)
{
    uint16_t regX   = word(modH->u8Buffer[ADD_HI], modH->u8Buffer[ADD_LO]);
    uint16_t regVal = word(modH->u8Buffer[NB_HI],  modH->u8Buffer[NB_LO]);

    uint16_t eepromAddr = REGS_BASE_ADDR + (database * REGS_PER_DB * 2) + (regX * 2);

    uint8_t bytes[2] = { highByte(regVal), lowByte(regVal) };
    if (!at24_write(eepromAddr, bytes, 2, 100)) return -1;

    modH->u8BufferSize = RESPONSE_SIZE;
    uint8_t copyBufferSize = modH->u8BufferSize + 2;
    sendTxBuffer(modH);
    return copyBufferSize;
}

/* ------------------------- FC15: write multiple coils ------------------------- */
static int8_t process_FC15(modbusHandler_t *modH, uint8_t database)
{
    // Extract coil range
    uint16_t startCoilX = word(modH->u8Buffer[ADD_HI], modH->u8Buffer[ADD_LO]);
    uint16_t coilCount  = word(modH->u8Buffer[NB_HI],  modH->u8Buffer[NB_LO]);

    // Index of coil data in Modbus frame (bit-packed)
    uint8_t frameByteIndex = 7;
    uint8_t bitIndex       = 0;

    for (uint16_t i = 0; i < coilCount; i++)
    {
        uint8_t inByte = modH->u8Buffer[frameByteIndex];
        uint8_t bitVal = bitRead(inByte, bitIndex);

        // Compute EEPROM address
        uint16_t byteAddr = COILS_BASE_ADDR + (database * (COILS_PER_DB / 8)) + ((startCoilX + i) / 8);
        uint8_t  bitPos   = (startCoilX + i) % 8;

        // Read current byte
        uint8_t byteVal;
        if (!at24_read(byteAddr, &byteVal, 1, 100)) return -1;

        // Modify bit
        bitWrite(byteVal, bitPos, bitVal);

        // Write byte back
        if (!at24_write(byteAddr, &byteVal, 1, 100)) return -1;

        // Move to next bit in frame
        bitIndex++;
        if (bitIndex > 7) { bitIndex = 0; frameByteIndex++; }
    }

    // Response (copy input header up to byte 6)
    modH->u8BufferSize = 6;
    uint8_t copyBufferSize = modH->u8BufferSize + 2;
    sendTxBuffer(modH);
    return copyBufferSize;
}

/* ------------------------- FC16: write multiple registers ------------------------- */
static int8_t process_FC16(modbusHandler_t *modH, uint8_t database)
{
    // Extract register range
    uint16_t startRegX = word(modH->u8Buffer[ADD_HI], modH->u8Buffer[ADD_LO]);
    uint16_t regsCount = word(modH->u8Buffer[NB_HI],  modH->u8Buffer[NB_LO]);

    // Build response header
    modH->u8Buffer[NB_HI]  = 0;
    modH->u8Buffer[NB_LO]  = (uint8_t)regsCount;
    modH->u8BufferSize     = RESPONSE_SIZE;

    // Extract register data from input frame
    for (uint16_t i = 0; i < regsCount; i++)
    {
        uint8_t msb = modH->u8Buffer[(BYTE_CNT + 1) + i * 2];
        uint8_t lsb = modH->u8Buffer[(BYTE_CNT + 2) + i * 2];

        uint16_t eepromAddr = REGS_BASE_ADDR + (database * REGS_PER_DB * 2) + ((startRegX + i) * 2);
        uint8_t bytes[2] = { msb, lsb };

        if (!at24_write(eepromAddr, bytes, 2, 100)) return -1;
    }

    uint8_t copyBufferSize = modH->u8BufferSize + 2;
    sendTxBuffer(modH);
    return copyBufferSize;
}

