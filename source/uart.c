/**
 *  Source file for AVR UART functionality.
 */

/***** INCLUDES ***************************************************************/
/* STD */
#include <stddef.h>
/* AVR */
#include <avr/io.h>
#include <avr/interrupt.h>
/* OWN */
#include "uart.h"


/***** MACROS *****************************************************************/
/*** UART CONFIG ***/
/* Macro to get the BAUD register value */
#define UBRR_VAL(bd)        ((F_CPU+bd*8)/(bd*16)-1)


/***** FUNCTIONS **************************************************************/
/*
 * Initialization of the UART
 */
void uart_init(void) {
    /* Set baudrate per default to 9600 */
    uart_set_baudrate(9600);
    
    /* Set the initial register values */
    UCSRA = 0x00;
    /* Set the framing (8N1) */
    UCSRC = _BV(URSEL) | _BV(UCSZ1) | _BV(UCSZ0);
    /* Enable RX and TX */
    UCSRB = _BV(RXEN) | _BV(TXEN);
}


/*
 * Set the UART baudrate
 */
void uart_set_baudrate(uint32_t baudrate) {
    /* Set the baudrate */
    UBRRH = UBRR_VAL(baudrate)>>8;
    UBRRL = UBRR_VAL(baudrate) & 0xFF;
}


/**************************************/
/***** BLOCKING ***********************/
/**************************************/

/*
 * Write a character via UART
 */
void uart_putc(char c) {
    /* Wait for transmit buffer to be empty */
    while(!(UCSRA & _BV(UDRE)));
    /* Write byte to the output buffer */
    UDR = c;
}


/*
 * Write a string via UART
 */
void uart_puts(char* s) {
    /* Transmit the string character by character */
    while(*s) {
        uart_putc(*s++);
    }
}

/*
 * Write data to the UART in blocking mode
 */
void uart_write_blocking(uint8_t* data, uint16_t len) {
    uint16_t i;
    /* Put the specified number of bytes in the TX buffer */
    for(i=0; i<len; i++) {
        /* Write the byte via UART */
        uart_putc(data[i]);
    }
}


/*
 * Read a character from UART
 */
uint8_t uart_getc(void) {
    /* Wait until reception is finished */
    while(!(UCSRA & _BV(RXC)));
    /* Return the received byte */
    return UDR;
}


/*
 * Read a string from UART
 */
void uart_gets(uint8_t* s, uint16_t len) {
    uint16_t index = 0;
    uint8_t temp;

    /* Read until string is finished or len is reached */
    do {
        /* Read next character */
        temp = uart_getc();
        /* Store character in string buffer */
        s[index++] = temp;
    } while((index < len) && (temp != '\n'));

    /* Make sure the string is null terminated */
    s[index] = '\0';
}

/* Print an integer number using putc only */
void uart_print_int(uint32_t value) {
    /* First, find the highest power of 10 */
    uint8_t digits = 0;
    uint32_t pow10 = 1;
    uint32_t tmp = value;
    while(tmp >= 10) {
        digits++;
        pow10 *= 10;
        tmp /= 10;
    }
    /* Second, print the digits of the number (from high to low) */
    while(pow10) {
        tmp = value / pow10;
        uart_putc(tmp+'0');
        value -= tmp * pow10;
        pow10 /= 10;
        /* Group digits ('.' before 3 digits) */
        if((digits%3 == 0) && (digits>0)) {
            uart_putc('.');
        }
        digits--;
    }
}

/* Print a byte in hexadecimal representation using putc only */
void uart_print_hex(uint8_t value) {
    /* Print preamble */
    uart_puts("0x");
    /* First, check the upper nibble */
    uint8_t tmp = (value&0xF0)>>4;
    if(tmp>9) {
        uart_putc('A'+(tmp-10));
    } else {
        uart_putc('0'+tmp);
    }
    /* Second, check the lower nibble */
    tmp = (value&0x0F);
    if(tmp>9) {
        uart_putc('A'+(tmp-10));
    } else {
        uart_putc('0'+tmp);
    }
}

/* Print a byte in binary representation using putc only */
void uart_print_binary(uint8_t value) {
    uint8_t i;
    for(i=0; i<8; i++) {
        if(value & (1 << (7-i))) {
            uart_putc('1');
        } else {
            uart_putc('0');
        }
    }
}
