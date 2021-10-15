/**
 *  Header file for AVR UART functionality.
 */

#ifndef _AVR_UART_H_
#define _AVR_UART_H_

/***** INCLUDES ***************************************************************/
#include <stdint.h>


/***** MACROS *****************************************************************/
#ifndef F_CPU
# warning "F_CPU not defined for \"uart.h\""
# define F_CPU 4096000UL
#endif


/***** FUNCTION PROTOTYPES ****************************************************/
/*** GENERAL ***/
void uart_init(void);
void uart_set_baudrate(uint32_t baudrate);
/*** BLOCKING ***/
/* write */
void uart_putc(char c);
void uart_puts(char* s);
void uart_write_blocking(uint8_t* data, uint16_t len);
/* read */
uint8_t uart_getc(void);
void uart_gets(uint8_t* s, uint16_t len);
/* helper */
void uart_print_int(uint32_t value);
void uart_print_hex(uint8_t value);
void uart_print_binary(uint8_t value);

#endif // _AVR_UART_H_
