#include "main.h"

void modbus_tick_and_check_timeout(UART_DATA *uart)
{
    // If inter-frame timeout exceeded and there are received bytes: frame ready
    if ((uart1.rxtimer++ > uart1.delay) && (uart1.rxcnt > 1))
    {
        uart1.rxgap = 1;
    }
    else
    {
        uart1.rxgap = 0;
    }
}

bool is_modbus_frame_ready(UART_DATA *uart)
{
    return uart->rxgap == 1;
}

void serialEnableTransfer()
{
    // tx enable
    GPIOA->BSRR = GPIO_BSRR_BS_12;

    // disable interruption on receive
    USART1->CR1 &= ~USART_CR1_RXNEIE;

    // enable interruption on transfer complete
    USART1->CR1 |= USART_CR1_TCIE;

    // led on
    GPIOC->BSRR = GPIO_BSRR_BR_14;
}
void serialDisableTransfer()
{
    // tx disable
    GPIOA->BSRR = GPIO_BSRR_BR_12;

    // enable interruption on receive
    USART1->CR1 |= USART_CR1_RXNEIE;

    // disable interruption on transfer complete
    USART1->CR1 &= ~USART_CR1_TCIE;

    // led off
    GPIOC->BSRR = GPIO_BSRR_BS_14;
}

void TIM6_IRQHandler(void)
{
    TIM6->SR &= ~TIM_SR_UIF; // clear interruption flag
    modbus_tick_and_check_timeout(&uart1);
}

void USART1_IRQHandler(void)
{
    //Receive Data register not empty interrupt
    if (USART1->ISR & USART_ISR_ORE)
    {
        USART1->ISR &= ~USART_ISR_ORE;
    }

    if (USART1->ISR & USART_ISR_RXNE)
    {
        USART1->ISR &= ~USART_ISR_RXNE;

        if ((uart1.rxtimer > uart1.charDelay) && (uart1.rxcnt > 1))
        {
            // exceeded max inter-char delay: clear buffer
            uart1.rxcnt = 0;
        }

        uart1.rxtimer = 0;

        if (uart1.rxcnt > (MODBUS_BUFFER_SIZE - 2))
        {
            uart1.rxcnt = 0;
        }
        uart1.buffer[uart1.rxcnt++] = USART1->RDR;
    }

    //Transmission complete interrupt
    if (USART1->ISR & USART_ISR_TC)
    {
        USART1->ISR &= ~USART_ISR_TC;

        if (uart1.txcnt < uart1.txlen)
        {
            USART1->TDR = uart1.buffer[uart1.txcnt++];
        }
        else
        {
            // transfer completed
            uart1.txlen = 0;
            serialDisableTransfer();
        }
    }
}

void modbus_start_response(UART_DATA *uart)
{
    if ((uart->txlen > 0) & (uart->txcnt == 0))
    {
        serialEnableTransfer();
        USART1->TDR = uart->buffer[uart->txcnt++];
    }
}

void modbus_process_message(UART_DATA *uart)
{
    uint16_t crcValue;

    // Check frame size and slave address
    if ((uart->buffer[0] != 0) & (uart->rxcnt > 5) &
        ((uart->buffer[0] == MODBUS_SLAVE_ADDRESS) | (uart->buffer[0] == 255)))
    {
        // Check CRC
        crcValue = Crc16(uart->buffer, uart->rxcnt - 2);
        if ((uart->buffer[uart->rxcnt - 2] == (crcValue & 0x00FF)) & (uart->buffer[uart->rxcnt - 1] == (crcValue >> 8)))
        {
            switch (uart->buffer[1])
            {
                case 3: // read registers
                    modbus_handler_fc_03_04(uart);
                    break;
                case 6: // write one register
                    modbus_handler_fc_06(uart);
                    break;
                case 16: // write multiple registers
                    modbus_handler_fc_16(uart);
                    break;
                default: //illegal operation
                    modbus_handle_exception(uart, 0x01);
            }

            // adding CRC16 to reply
            crcValue = Crc16(uart->buffer, uart->txlen - 2);
            uart->buffer[uart->txlen - 2] = crcValue;
            uart->buffer[uart->txlen - 1] = crcValue >> 8;
            // Reset transfer counter
            uart->txcnt = 0;
        }
    }

    uart->rxgap = 0;
    uart->rxcnt = 0;
    uart->rxtimer = 0;
}

/**
 * To read a word (16 bits) from a register you send the following:
 * 
 * Byte 0 Modbus Device Address (1 to 247)
 * Byte 1 Function Code / Message Type (0x03)
 * Byte 2 Register Number (high byte)
 * Byte 3 Register Number (low byte)
 * Byte 4 Data Length (high byte, specified in Words) (1)
 * Byte 5 Data Length (low byte)
 * Byte 6 CRC of bytes 1 to 6 (high byte)
 * Byte 7 CRC of bytes 1 to 6 (low byte)
 * 
 * The response is sent as follows:
 * Byte 0 Modbus Device Address
 * Byte 1 Function Code / Message Type (0x03)
 * Byte 2 Number of bytes returned (specified in bytes, 1 register x 2 bytes = 2)
 * Byte 3 Data0
 * Byte 4 Data1
 * Byte 5 CRC of bytes 1 to 5 (high byte)
 * Byte 6 CRC of bytes 1 to 5 (low byte)
*/
void modbus_handler_fc_03_04(UART_DATA *uart)
{
    unsigned int registersCount = 0, currentDataByte = 0;

    uint16_t startRegisterAddress = (uart->buffer[2] << 8) + uart->buffer[3];
    uint16_t numberOfRegisters = (uart->buffer[4] << 8) + uart->buffer[5];

    // Response data start byte
    currentDataByte = 3;

    if ((((startRegisterAddress + numberOfRegisters) < OBJ_SZ) & (numberOfRegisters < MODBUS_WORDS_MAX + 1)))
    {
        registersCount = 1;
        uart->buffer[currentDataByte] = AXIS_CALIBRATION_FINISHED == ax1.calibration_state ? 1 : 0;
        uart->buffer[currentDataByte + 1] = AXIS_CALIBRATION_FINISHED == ax2.calibration_state ? 1 : 0;

        currentDataByte += 2;

        //uart->buffer[0] = MODBUS_SLAVE_ADDRESS; // address, the same as in received message
        //uart->buffer[1] = 3; // FC, the same as in received message
        uart->buffer[2] = registersCount * 2; //byte count
        uart->txlen = registersCount * 2 + 5; //responce length // 5 = (addr=1) + (func=1) + (count of data bytes=1) + (crc=2)
    }
    else
    {
        //exception illegal data adress 0x02
        modbus_handle_exception(uart, 0x02);
    }
}

//*******************************************************
//Writing
//*******************************************************

/**
 * To write a word (16 bits) to a register you send the following:
 * Byte 0 Modbus Device Address (1 to 247)
 * Byte 1 Function Code / Message Type (0x06)
 * Byte 2 Register Number (high byte)
 * Byte 3 Register Number (low byte)
 * Byte 4 Data0
 * Byte 5 Data1
 * Byte 6 CRC of bytes 1 to 6 (high byte)
 * Byte 7 CRC of bytes 1 to 6 (low byte)
 * 
 * The response is a copy of what was sent.
 */
void modbus_handler_fc_06(UART_DATA *uart)
{
    uint16_t startRegisterAddress = (uart->buffer[2] << 8) + uart->buffer[3];
    uint16_t registerData = (uart->buffer[4] << 8) + uart->buffer[5];

    if (startRegisterAddress < OBJ_SZ)
    {
        uart->txlen = uart->rxcnt; //responce length
        //buf.reg16 = (uart->buffer[4] << 8) + uart->buffer[5];

        if (startRegisterAddress & 1)
        {
            struct AxisTask *task = malloc(sizeof(struct AxisTask));
            task->type = AXIS_TASK_TYPE_CALIBRATION;
            queue_push(ax1.queue, &task->n);
        }
        if (startRegisterAddress & 2)
        {
            struct AxisTask *task = malloc(sizeof(struct AxisTask));
            task->type = AXIS_TASK_TYPE_CALIBRATION;
            queue_push(ax2.queue, &task->n);
        }

        if (startRegisterAddress & 4)
        {
            struct AxisTask *task = malloc(sizeof(struct AxisTask));
            task->type = AXIS_TASK_TYPE_WAIT;
            task->wait_ms = registerData;
            queue_push(ax1.queue, &task->n);
        }
        if (startRegisterAddress & 8)
        {
            struct AxisTask *task = malloc(sizeof(struct AxisTask));
            task->type = AXIS_TASK_TYPE_WAIT;
            task->wait_ms = registerData;
            queue_push(ax2.queue, &task->n);
        }
    }
    else
    {
        //illegal data
        modbus_handle_exception(uart, 0x02);
    }

    //uart[0] = MODBUS_SLAVE_ADDRESS; // adress - stays a same as in recived
    //uart[1] = 6; //query type - - stay a same as in recived
    //uart->buffer[2]  - byte count a same as in rx query
    //2-3  - adress   , 4-5 - value
}

/**
 * Modbus function code 16
 * 
 * Request:
 * Byte 0 Modbus Device Address (1 to 247)
 * Byte 1 Function Code / Message Type (0x10)
 * Byte 2 Start Register Number (high byte)
 * Byte 3 Start Register Number (low byte)
 * Byte 4 Number of registers to write(high byte)
 * Byte 5 Number of registers to write (low byte)
 * Byte 6 Number of data bytes that follow(3 registers x 2 bytes each = 6)
 * Byte 7 Data0
 * Byte 8 Data1
 * Byte 9 Data2​
 * Byte 10 Data​3
 * Byte 11 Data4
 * Byte 12 Data5
 * Byte 13 CRC of bytes 1 to 6 (high byte)
 * Byte 14 CRC of bytes 1 to 6 (low byte)
 * 
 * Response:
 * Byte 0 Modbus Device Address
 * Byte 1 Function Code / Message Type (0x10)
 * Byte 2 Start Register Number (high byte)
 * Byte 3 Start Register Number (low byte)
 * Byte 4 The number of registers written (high byte)
 * Byte 5 The number of registers written (low byte)
 * Byte 6 CRC of bytes 1 to 6 (high byte)
 * Byte 7 CRC of bytes 1 to 6 (low byte)
 */
void modbus_handler_fc_16(UART_DATA *uart)
{
    uint16_t startRegisterAddress = (uart->buffer[2] << 8) + uart->buffer[3];
    // uint16_t numberOfRegisters = (uart->buffer[4] << 8) + uart->buffer[5];
    // uint8_t numberOfDataBytes = uart->buffer[6];

    if (startRegisterAddress < OBJ_SZ)
    {
        struct AxisTask *task = malloc(sizeof(struct AxisTask));
        task->type = AXIS_TASK_TYPE_MOVE;
        task->degree = convertDegreesFromModbus((uart->buffer[7] << 8) + uart->buffer[8], true);
        task->speed = convertDegreesFromModbus((uart->buffer[9] << 8) + uart->buffer[10], false);

        if (startRegisterAddress & 1)
        {
            task->relative = startRegisterAddress & 4;
            queue_push(ax1.queue, &task->n);

            if (startRegisterAddress & 2)
            {
                struct AxisTask *task = malloc(sizeof(struct AxisTask));
                task->type = AXIS_TASK_TYPE_MOVE;
                task->degree = convertDegreesFromModbus((uart->buffer[11] << 8) + uart->buffer[12], true);
                task->speed = convertDegreesFromModbus((uart->buffer[13] << 8) + uart->buffer[14], false);
                task->relative = startRegisterAddress & 8;
                queue_push(ax2.queue, &task->n);
            }
        }
        else if (startRegisterAddress & 2)
        {
            task->relative = startRegisterAddress & 8;
            queue_push(ax2.queue, &task->n);
        }
        else
        {
            if (startRegisterAddress & 4)
            {
                struct AxisTask *task = malloc(sizeof(struct AxisTask));
                task->type = AXIS_TASK_TYPE_WAIT;
                task->wait_ms = (uart->buffer[7] << 8) + uart->buffer[8];
                queue_push(ax1.queue, &task->n);
            }
            if (startRegisterAddress & 8)
            {
                struct AxisTask *task = malloc(sizeof(struct AxisTask));
                task->type = AXIS_TASK_TYPE_WAIT;
                task->wait_ms = (uart->buffer[9] << 8) + uart->buffer[10];
                queue_push(ax2.queue, &task->n);
            }
        }
    }
    else
    {
        //illegal data
        modbus_handle_exception(uart, 0x02);
    }

    // Reply
    // 0   — MODBUS_SLAVE_ADDRESS
    // 1   — FC, the same (6)
    // 2-3 — adress, the same
    // 4-5 — The number of registers written, the same
    uart->txlen = 8; // 6 (payload) + 2 crc16
}

//modbus exception - illegal data=01, adress=02 etc
void modbus_handle_exception(UART_DATA *uart, unsigned char error_type)
{
    //illegal operation
    uart->buffer[2] = error_type; //exception
    uart->txlen = 5;              //responce length
}

float convertDegreesFromModbus(int d, bool withSign)
{
    if (withSign)
    {
        uint32_t limit = (1 << (16 - 1)) - 1;
        if (d > limit)
        {
            d = d - (1 << 16);
        }
    }
    return ((float)d) / 10;
}

//*********************************************************************
//CRC16 for Modbus Calculation
//*********************************************************************
unsigned int Crc16(unsigned char *ptrByte, int byte_cnt)
{
    unsigned int w = 0;
    char shift_cnt;

    if (ptrByte)
    {
        w = 0xffffU;
        for (; byte_cnt > 0; byte_cnt--)
        {
            w = (unsigned int)((w / 256U) * 256U + ((w % 256U) ^ (*ptrByte++)));
            for (shift_cnt = 0; shift_cnt < 8; shift_cnt++)
            {
                if ((w & 0x1) == 1)
                    w = (unsigned int)((w >> 1) ^ 0xa001U);
                else
                    w >>= 1;
            }
        }
    }
    return w;
}
