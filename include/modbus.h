#ifndef MODBUS_H
#define MODBUS_H

#include <stm32f0xx.h>

#define MODBUS_SLAVE_ADDRESS 11

#define OBJ_SZ 123 // Some kind of inner-protection
#define MODBUS_BUFFER_SIZE 256
#define MODBUS_WORDS_MAX (MODBUS_BUFFER_SIZE - 5) / 2 // Max register count for response

typedef struct
{
    unsigned char buffer[MODBUS_BUFFER_SIZE]; // read / write buffer
    unsigned int rxtimer;                     // timer for frame/char delay handling
    unsigned char rxcnt;                      // count of received bytes
    unsigned char txcnt;                      // count of transferred bytes
    unsigned char txlen;                      // num bytes to transfer
    unsigned char rxgap;                      // inter-frame timeout exceeded: frame ready
    unsigned char delay;                      // max inter-frame delay (timer periods)
    unsigned char charDelay;                  // max inter-char delay (timer periods)
} UART_DATA;

UART_DATA uart1;

void modbus_start_response(UART_DATA *uart);
void modbus_process_message(UART_DATA *uart);

void modbus_tick_and_check_timeout(UART_DATA *uart);
bool is_modbus_frame_ready(UART_DATA *uart);

void modbus_handler_fc_03_04(UART_DATA *uart);
void modbus_handler_fc_06(UART_DATA *uart);
void modbus_handler_fc_16(UART_DATA *uart);
void modbus_handle_exception(UART_DATA *MODBUS, unsigned char error_type);

unsigned int Crc16(unsigned char *ptrByte, int byte_cnt);

float convertDegreesFromModbus(int d, bool withSign);

void serialEnableTransfer(void);
void serialDisableTransfer(void);

#endif // MODBUS_H