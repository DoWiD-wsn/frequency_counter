/**********************************************************************
 *  AVR-BASED FREQUENCY COUNTER                                       *
 *                                                                    *
 *  Author: Dominik Widhalm                                           *
 *  Date:   2020-11-29                                                *
 *                                                                    *
 *  Brief:                                                            *
 *      Frequency counter based on an AVR ATmega8. The MCU is clocked *
 *      by an 4.096MHz crystal whose oscillation also driven a        *
 *      prescaler circuit based on 74HC4040 to generate a 128ms       *
 *      enable signal for the frequency measurement using a 74HC00.   *
 *      During on enable-period the oscillations of the input channel *
 *      are counted by another 74HC4040 where the 8th digit drives    *
 *      the timer/counter module 1 (16 bit) of the ATmega8, creating  *
 *      a cascaded 24-bit counter (16 bit intern + 8 bit extern).     *
 *      Additionally, the switching of the enable signal triggers     *
 *      an external interrupt of the ATmega8 telling the AVR when a   *
 *      sample period has finished. After that, the AVR simply takes  *
 *      the 24 bit and calculates the corresponding frequency.        *
 *      The result can be read via I2C (AVR acts as a slave).         *
 *      Additionally, some configurations like the input channel can  *
 *      also be set via I2C commands.                                 *
 *                                                                    *
 *  I2C Register:                                                     *
 *      0x00 - CONFIG (R/W):                                          *
 *      |RST|RDY|SMP1|SMP0|RES1|RES0|CHSEL1|CHSEL0|                   *
 *      0x01 - FREQ-LSB (R):                                          *
 *      |LSB7|LSB6|LSB5|LSB4|LSB3|LSB2|LSB1|LSB0|                     *
 *      0x02 - FREQ-MSB (R):                                          *
 *      |MSB7|MSB6|MSB5|MSB4|MSB3|MSB2|MSB1|MSB0|                     *
 *      0x03 - FREQ-XMSB (R):                                         *
 *      |XMSB7|XMSB6|XMSB5|XMSB4|XMSB3|XMSB2|XMSB1|XMSB0|             *
 *                                                                    *
 *      Description:                                                  *
 *      -RST- Reset                                                   *
 *        0   nothing                                                 *
 *        1   request reset                                           *
 *      -RDY- Result Ready                                            *
 *        0   no measurement result                                   *
 *        1   measurement result  available                           *
 *      -SMP1-SMP0- Sampling                                          *
 *        0    0    no sampling (1)                                   *
 *        0    1    3 samples                                         *
 *        1    0    5 samples                                         *
 *        1    1    10 samples                                        *
 *      -RES1-RES0- Resolution                                        *
 *        0    0    Hz        (24 bit -> LSB + MSB + XMSB)            *
 *        0    1    kHz       (16 bit -> LSB + MSB)                   *
 *        1    0    MHz       (8 bit  -> LSB)                         *
 *        1    1    reserved                                          *
 *      -CHSEL1-CHSEL0- Channel Selection                             *
 *         0      0     CH0                                           *
 *         0      1     CH1                                           *
 *         1      0     CH2                                           *
 *         1      1     CH3                                           *
 *                                                                    *
 *  Resources:                                                        *
 *  -) Frequency counter:                                             *
 *     www.herbert-dingfelder.de/?page_id=304                         *
 *  -) I2C Slave:                                                     *
 *     rn-wissen.de/wiki/index.php/TWI_Slave_mit_avr-gcc              *
 *     pc.gameactive.org/i2cslavetutorial/I2C_SlaveTutorial.html      *
 *                                                                    *
 *  Note:                                                             *
 *  -) Writing config works ... reading data not yet!                 *
 *                                                                    *
 **********************************************************************/

/*** Enable Debugging (UART@9600) ***/
#define DEBUG_UART_ENABLE           (1)


/***** INCLUDES *******************************************************/
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>

/* Invoke UART library if debugging is enabled */
#if DEBUG_UART_ENABLE
# include "uart.h"
#endif


/***** DEFINES ********************************************************/
/*** I2C ***/
/* Slave Address */
#define I2C_SLAVE_ADDRESS           (0x24)
/* Register Addresses */
#define I2C_REG_CONFIG              (0x00)
#define I2C_REG_MSB                 (0x01)
#define I2C_REG_LSB                 (0x02)
#define I2C_REG_XLSB                (0x03)
#define I2C_REG_UNDEF               (0xFF)
/* Config Register Bits */
#define I2C_CONFIG_RST              (7)
#define I2C_CONFIG_RST_MASK         (0x80)
#define I2C_CONFIG_RDY              (6)
#define I2C_CONFIG_RDY_MASK         (0x40)
#define I2C_CONFIG_SMP              (4)
#define I2C_CONFIG_SMP_MASK         (0x30)
#define I2C_CONFIG_RES              (2)
#define I2C_CONFIG_RES_MASK         (0x0C)
#define I2C_CONFIG_CHSEL            (0)
#define I2C_CONFIG_CHSEL_MASK       (0x03)

/*** I2C slave responses ***/
/* ACK      -> TWEN, TWIE, TWINT, TWEA */
#define TWCR_ACK                    (0xC5)
/* NACK     -> TWEN, TWIE, TWINT */
#define TWCR_NACK                   (0x85)
/* RESET    -> TWEN, TWIE, TWINT, TWEA, TWSTO */
#define TWCR_RESET                  (0xD5)

/*** Control GPIO ***/
/* Prescaler-Counter Reset */
#define GPIO_PRESCALER_RST_DDR      (DDRC)
#define GPIO_PRESCALER_RST_PORT     (PORTC)
#define GPIO_PRESCALER_RST_PIN      (PINC)
#define GPIO_PRESCALER_RST          _BV(PC1)
/* Frequency-Counter Reset */
#define GPIO_CNT_RST_DDR            (DDRC)
#define GPIO_CNT_RST_PORT           (PORTC)
#define GPIO_CNT_RST_PIN            (PINC)
#define GPIO_CNT_RST                _BV(PC0)
/* MUX Select */
#define GPIO_MUX_DDR                (DDRC)
#define GPIO_MUX_PORT               (PORTC)
#define GPIO_MUX_PIN                (PINC)
#define GPIO_MUX_S0                 _BV(PC3)
#define GPIO_MUX_S1                 _BV(PC2)

/*** Gate-enable Clock ***/
#define GPIO_GATE_EN_DDR            (DDRD)
#define GPIO_GATE_EN_PORT           (PORTD)
#define GPIO_GATE_EN_PIN            (PIND)
#define GPIO_GATE_EN                _BV(PD2)

/*** Frequency Counter Value ***/
/* Q7 (T1) */
#define GPIO_CNT_Q7_DDR             (DDRD)
#define GPIO_CNT_Q7_PORT            (PORTD)
#define GPIO_CNT_Q7_PIN             (PIND)
#define GPIO_CNT_Q7                 _BV(PD5)
/* Q6 */
#define GPIO_CNT_Q6_DDR             (DDRD)
#define GPIO_CNT_Q6_PORT            (PORTD)
#define GPIO_CNT_Q6_PIN             (PIND)
#define GPIO_CNT_Q6                 _BV(PD4)
/* Q5 */
#define GPIO_CNT_Q5_DDR             (DDRD)
#define GPIO_CNT_Q5_PORT            (PORTD)
#define GPIO_CNT_Q5_PIN             (PIND)
#define GPIO_CNT_Q5                 _BV(PD3)
/* Q4 */
#define GPIO_CNT_Q4_DDR             (DDRD)
#define GPIO_CNT_Q4_PORT            (PORTD)
#define GPIO_CNT_Q4_PIN             (PIND)
#define GPIO_CNT_Q4                 _BV(PD6)
/* Q3 */
#define GPIO_CNT_Q3_DDR             (DDRD)
#define GPIO_CNT_Q3_PORT            (PORTD)
#define GPIO_CNT_Q3_PIN             (PIND)
#define GPIO_CNT_Q3                 _BV(PD7)
/* Q2 */
#define GPIO_CNT_Q2_DDR             (DDRB)
#define GPIO_CNT_Q2_PORT            (PORTB)
#define GPIO_CNT_Q2_PIN             (PINB)
#define GPIO_CNT_Q2                 _BV(PB2)
/* Q1 */
#define GPIO_CNT_Q1_DDR             (DDRB)
#define GPIO_CNT_Q1_PORT            (PORTB)
#define GPIO_CNT_Q1_PIN             (PINB)
#define GPIO_CNT_Q1                 _BV(PB1)
/* Q0 */
#define GPIO_CNT_Q0_DDR             (DDRB)
#define GPIO_CNT_Q0_PORT            (PORTB)
#define GPIO_CNT_Q0_PIN             (PINB)
#define GPIO_CNT_Q0                 _BV(PB0)


/***** GLOBAL VARIABLES ***********************************************/
/* Data Array */
uint8_t i2c_data[4] = {0,0,0,0};
/* I2C pointer positions */
uint8_t i2c_data_pos = I2C_REG_UNDEF;
/* Pointer to data register */
uint8_t *config = &i2c_data[0];
uint8_t *lsb    = &i2c_data[1];
uint8_t *msb    = &i2c_data[2];
uint8_t *xmsb   = &i2c_data[3];


/***** LOCAL FUNCTION PROTOTYPES **************************************/
/* Init functions */
void gpio_init(void);
void timer1_init(void);
void extint_init(void);
void counter_init(void);
void i2c_slave_init(uint8_t address);

/* MUX functions */
void mux_select(uint8_t ch);

/* Helper functions */
uint8_t cnt_read_lsb(void);
void cnt_reset_lsb(void);


/***** INTERRUPT SERVICE ROUTINES (ISR) *****/
/* External Interrupt (INT0 falling edge) */
ISR(INT0_vect) {
    uint32_t cnt=0;
    /* Read the lower (external) 8 bit */
    cnt = (uint32_t)cnt_read_lsb();
    /* Read the higher (internal) 16 bit */
    cnt |= (uint32_t)(TCNT1L)<<8;
    cnt |= (uint32_t)(TCNT1H)<<16;
    /* Reset the external counter */
    cnt_reset_lsb();
    /* Reset the internal counter */
    TCNT1H = 0;
    TCNT1L = 0;
    /* Calculate resulting frequency (disable IRS meanwhile) */
    uint32_t frequency = (cnt*1000)>>7;
    uint8_t resolution = (*config & I2C_CONFIG_RES_MASK) >> I2C_CONFIG_RES;
    cli();
    switch(resolution) {
        /*** Hz ***/
        case 0:
            /* Copy result */
            *lsb  = (frequency & 0x000000FF);
            *msb  = (frequency & 0x0000FF00)>>8;
            *xmsb = (frequency & 0x00FF0000)>>16;
            /* Result is available */
            *config |= _BV(I2C_CONFIG_RDY);
            break;
        /*** kHz ***/
        case 1:
            /* Scale result */
            frequency = (uint32_t)(round((double)frequency/1000.0));
            /* Copy result */
            *lsb  = (frequency & 0x000000FF);
            *msb  = (frequency & 0x0000FF00)>>8;
            *xmsb = 0x00;
            /* Result is available */
            *config |= _BV(I2C_CONFIG_RDY);
            break;
        /*** MHz ***/
        case 2:
            /* Scale result */
            frequency = (uint32_t)(round((double)frequency/1000000.0));
            /* Copy result */
            *lsb  = (frequency & 0x000000FF);
            *msb  = 0x00;
            *xmsb = 0x00;
            /* Result is available */
            *config |= _BV(I2C_CONFIG_RDY);
            break;
        /*** Reserved ***/
        default:
            /* Clear result bytes */
            *lsb  = 0x00;
            *msb  = 0x00;
            *xmsb = 0x00;
            /* No result is available */
            *config &= ~_BV(I2C_CONFIG_RDY);
    };
    sei();
}

/* I2C interrupt service (see AVR311 for more detail) */
ISR(TWI_vect) {
    /* Temporary variable for received data */
    uint8_t data = 0;
    
    /*** DBG ***/ uart_puts("\nSTATUS ");uart_print_hex(TW_STATUS);uart_puts("\n");
    
    /* Slave's reaction depends on status register value */
    switch(TW_STATUS) {
        /*** Slave Receiver ***/
        
        /* 0x60 - Own SLA+W has been received; ACK has been returned */
        case TW_SR_SLA_ACK:
        /* 0x68 - Arbitration lost in SLA+R/W as mater; own SLA+W has been received; ACK has been returned */
        case TW_SR_ARB_LOST_SLA_ACK:
            /*** DBG ***/ uart_puts("TW_SR_SLA_ACK\n");
            /* Set positions to "undef" */
            i2c_data_pos = I2C_REG_UNDEF;
            /* Data byte will be received and ACK will be returned */
            TWCR = TWCR_ACK;
            break;
        
        /* 0x70 - General call address has been received; ACK has been returned */
        //case TW_SR_GCALL_ACK:
        /* 0x78 - Arbitration lost in SLA+R/W as Master; General call address has been received; ACK has been returned */
        //case TW_SR_ARB_LOST_GCALL_ACK:
        /* 0x90 - Previously addressed with general call; data has been received; ACK has been returned */
        //case TW_SR_GCALL_DATA_ACK:
        /* 0x98 - Previously addressed with general call; data has been received; NOT ACK has been returned */
        //case TW_SR_GCALL_DATA_NACK:
        
        /* 0x80 - Previously addressed with own SLA+W; data has been received; ACK has been returned */
        case TW_SR_DATA_ACK:
            /* Read the received byte */
            data = TWDR;
            /* Check if the read position is "undef" (first access) */
            if(i2c_data_pos == I2C_REG_UNDEF) {
                /*** DBG ***/ uart_puts("TW_SR_DATA_ACK-1 ");uart_print_hex(TWDR);uart_puts("\n");
                /* Set the data array position to the requested position */
                i2c_data_pos = data;
                /* Respond with ACK to wait for next data byte (value) */
                TWCR = TWCR_ACK;
            /* Write position has already been set - ready to receive data */
            } else {
                /* Check if previously requested register address is valid (only config can be written) */
                if(i2c_data_pos == I2C_REG_CONFIG) {
                    /*** DBG ***/ uart_puts("TW_SR_DATA_ACK-3 ");uart_print_hex(TWDR);uart_puts("\n");
                    /* Check which bits have changed */
                    uint8_t delta = *config ^ data;
                    /* Check if a reset was requested */
                    if(delta & I2C_CONFIG_RST_MASK) {
                        /* Reset the counter */
                        counter_init();
                        /* Reset the RDY flag */
                        *config &= ~_BV(I2C_CONFIG_RDY);
                        /* Clear the result bytes */
                        *lsb = 0x00;
                        *msb = 0x00;
                        *xmsb = 0x00;
                    }
                    /* Check if the sampling has changed */
                    if(delta & I2C_CONFIG_SMP_MASK) {
                        /* Reset the RDY flag */
                        *config &= ~_BV(I2C_CONFIG_RDY);
                        /* Clear the result bytes */
                        *lsb = 0x00;
                        *msb = 0x00;
                        *xmsb = 0x00;
                        /* Copy the new resolution selection */
                        *config = (*config & ~I2C_CONFIG_SMP_MASK) | (data & I2C_CONFIG_SMP_MASK);
                    }
                    /* Check if the resolution has changed */
                    if(delta & I2C_CONFIG_RES_MASK) {
                        /* Reset the RDY flag */
                        *config &= ~_BV(I2C_CONFIG_RDY);
                        /* Clear the result bytes */
                        *lsb = 0x00;
                        *msb = 0x00;
                        *xmsb = 0x00;
                        /* Copy the new resolution selection */
                        *config = (*config & ~I2C_CONFIG_RES_MASK) | (data & I2C_CONFIG_RES_MASK);
                    }
                    /* Check if the channel has changed */
                    if(delta & I2C_CONFIG_CHSEL_MASK) {
                        /* Set the new channel */
                        mux_select((delta & I2C_CONFIG_CHSEL_MASK)>>I2C_CONFIG_CHSEL);
                        /* Reset the RDY flag */
                        *config &= ~_BV(I2C_CONFIG_RDY);
                        /* Clear the result bytes */
                        *lsb = 0x00;
                        *msb = 0x00;
                        *xmsb = 0x00;
                        /* Copy the new channel selection */
                        *config = (*config & ~I2C_CONFIG_CHSEL_MASK) | (data & I2C_CONFIG_CHSEL_MASK);
                    }
                    /* Operation finished */
                    i2c_data_pos = I2C_REG_UNDEF;
                    /* Respond with NACK */
                    TWCR = TWCR_NACK;
                } else {
                    /*** DBG ***/ uart_puts("TW_SR_DATA_ACK-2 ");uart_print_hex(TWDR);uart_puts("\n");
                    
                    /* Invalid request */
                    i2c_data_pos = I2C_REG_UNDEF;
                    /* Respond with NACK */
                    TWCR = TWCR_NACK;
                }
            }
            break;
        
        /* 0x88 - Previously addressed with own SLA+W; data has been received; NOT ACK has been returned */
        case TW_SR_DATA_NACK:
        /* 0xA0 - A STOP condition or repeated START condition has been received while still addressed as slave */
        case TW_SR_STOP:
            /*** DBG ***/ uart_puts("TW_SR_STOP\n");
            /* Respont with ACK */
            TWCR = TWCR_ACK;
            break;
        
        
        /*** Slave Transmitter ***/
        
        /* 0xA8 - Own SLA+R has been received; ACK has been returned */
        case TW_ST_SLA_ACK:
        /* 0xB0 - Arbitration lost in SLA+R/W as Master; own SLA+R has been received; ACK has been returned */
        case TW_ST_ARB_LOST_SLA_ACK:
        /* 0xB8 - Data byte in TWDR has been transmitted; ACK has been received */
        case TW_ST_DATA_ACK:
            /*** DBG ***/ uart_puts("TW_ST_SLA_ACK");uart_print_hex(TWDR);uart_puts("\n");
            /* Check if data position is valid */
            if(i2c_data_pos != I2C_REG_UNDEF) {
                /*** DBG ***/ uart_puts("TW_ST_SLA_ACK-1\n");
                /* Check if read pointer is set to CONFIG (no consecutive read) */
                if(i2c_data_pos == I2C_REG_CONFIG) {
                    /* Copy the config register content */
                    TWDR = *config;
                    /* Operation finished */
                    i2c_data_pos = I2C_REG_UNDEF;
                    /* Respond with NACK */
                    TWCR = TWCR_NACK;
                } else {
                    /* Copy addressed result byte */
                    TWDR = i2c_data[i2c_data_pos];
                    /* Check if there is still data to read */
                    if(i2c_data_pos < I2C_REG_XLSB) {
                        /* Increment read position */
                        i2c_data_pos++;
                        /* Send ACK */
                        TWCR = TWCR_ACK;
                    } else {
                        /* No more data left */
                        i2c_data_pos = I2C_REG_UNDEF;
                        /* Respond with NACK */
                        TWCR = TWCR_NACK;
                    }
                }
            } else {
                /*** DBG ***/ uart_puts("TW_ST_SLA_ACK-2\n");
                /* Respond with NACK */
                TWCR = TWCR_NACK;
            }
            break;
        
        /* 0xC0 - Data byte in TWDR has been transmitted; NOT ACK has been received */
        case TW_ST_DATA_NACK:
        /* 0xC8 - Last data byte in TWDR has been transmitted; ACK has been received */
        case TW_ST_LAST_DATA:
            /* Respont with ACK */
            TWCR = TWCR_ACK;
            break;
        
        
        /*** Misc ***/
        
        /* 0xF8 - No relevant state information available */
        case TW_NO_INFO:
        /* 0x00 - Bus error due to an illegal START or STOP condition */
        case TW_BUS_ERROR:
        /* In case something else happened */
        default:
            /* Respond with RESET */
            TWCR = TWCR_RESET;
            break;
    }
}



/***** FUNCTIONS ******************************************************/

/*** INIT FUNCTIONS ***/

/* Initialize the GPIOs (input/outputs) */
void gpio_init(void) {
    /*** INPUTS ***/
    /* Frequency Counter Value */
    GPIO_CNT_Q7_DDR     &= ~GPIO_CNT_Q7;
    GPIO_CNT_Q7_PORT    &= ~GPIO_CNT_Q7;
    GPIO_CNT_Q6_DDR     &= ~GPIO_CNT_Q6;
    GPIO_CNT_Q6_PORT    &= ~GPIO_CNT_Q6;
    GPIO_CNT_Q5_DDR     &= ~GPIO_CNT_Q5;
    GPIO_CNT_Q5_PORT    &= ~GPIO_CNT_Q5;
    GPIO_CNT_Q4_DDR     &= ~GPIO_CNT_Q4;
    GPIO_CNT_Q4_PORT    &= ~GPIO_CNT_Q4;
    GPIO_CNT_Q3_DDR     &= ~GPIO_CNT_Q3;
    GPIO_CNT_Q3_PORT    &= ~GPIO_CNT_Q3;
    GPIO_CNT_Q2_DDR     &= ~GPIO_CNT_Q2;
    GPIO_CNT_Q2_PORT    &= ~GPIO_CNT_Q2;
    GPIO_CNT_Q1_DDR     &= ~GPIO_CNT_Q1;
    GPIO_CNT_Q1_PORT    &= ~GPIO_CNT_Q1;
    GPIO_CNT_Q0_DDR     &= ~GPIO_CNT_Q0;
    GPIO_CNT_Q0_PORT    &= ~GPIO_CNT_Q0;
    /* Gate enable clock */
    GPIO_GATE_EN_DDR    &= ~GPIO_GATE_EN;
    GPIO_GATE_EN_PORT   &= ~GPIO_GATE_EN;
    
    /*** OUTPUTS ***/
    /* Prescaler-Counter Reset (initially low) */
    GPIO_PRESCALER_RST_DDR  |= GPIO_PRESCALER_RST;
    GPIO_PRESCALER_RST_PORT &= ~GPIO_PRESCALER_RST;
    /* Frequency-Counter Reset (initially low) */
    GPIO_CNT_RST_DDR        |= GPIO_CNT_RST;
    GPIO_CNT_RST_PORT       &= ~GPIO_CNT_RST;
    /* MUX Select (initially CH0) */
    GPIO_MUX_DDR            |= GPIO_MUX_S0 | GPIO_MUX_S1;
    GPIO_MUX_PORT           &= ~GPIO_MUX_S0 & ~GPIO_MUX_S1;
}

/* Initialize the timer/counter 1 module to cnt with external clk */
void timer1_init(void) {
    /* Enable tcnt1 with clk input T1 on falling edges */
    TCCR1A = 0x00;
    TCCR1B = _BV(CS12) | _BV(CS11);
}

/* Initialize the external interrupt source */
void extint_init(void) {
    /* Enable INT0 on falling edge */
    MCUCR |= _BV(ISC01);
    /* Enable interrupt for INT0 */
    GICR |= _BV(INT0);
}

/* Initialize the I2C as slave */
void i2c_slave_init(uint8_t address) {
    /* Set the slave address */
    TWAR = (address << 1);
    /* Enable the I2C interface with interrupts and acknowledgement */
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
    /* Reset pointer positions */
    i2c_data_pos = I2C_REG_UNDEF;
}



/*** MUX FUNCTIONS */

/* Select the given MUX channel */
void mux_select(uint8_t ch) {
    /* Check if the value is valid */
    if(ch>3) {
        return;
    }
    /* Select the chosen channel */
    switch(ch) {
        case 0:
            GPIO_MUX_PORT &= ~GPIO_MUX_S0 & ~GPIO_MUX_S1;
            break;
        case 1:
            GPIO_MUX_PORT = (GPIO_MUX_PORT | GPIO_MUX_S0) & ~GPIO_MUX_S1;
            break;
        case 2:
            GPIO_MUX_PORT = (GPIO_MUX_PORT | GPIO_MUX_S1) & ~GPIO_MUX_S0;
            break;
        case 3:
            GPIO_MUX_PORT |= GPIO_MUX_S0 | GPIO_MUX_S1;
            break;
    }
}

/* Reset all external counter */
void counter_init(void) {
    /* Set the reset lines to "1" */
    GPIO_CNT_RST_PORT |= GPIO_CNT_RST;
    GPIO_PRESCALER_RST_PORT |= GPIO_PRESCALER_RST;
    /* Wait for 10ms to ensure proper reseting */
    _delay_ms(10);
    /* Set the reset lines back to "0" */
    GPIO_CNT_RST_PORT &= ~GPIO_CNT_RST;
    GPIO_PRESCALER_RST_PORT &= ~GPIO_PRESCALER_RST;
}


/*** HELPER FUNCTIONS */

/* Read the LSB from the external counter */
uint8_t cnt_read_lsb(void) {
    uint8_t lsb = 0;
    /* Check if Q0 is set */
    if(GPIO_CNT_Q0_PIN & GPIO_CNT_Q0) {
        lsb |= _BV(0);
    }
    /* Check if Q1 is set */
    if(GPIO_CNT_Q1_PIN & GPIO_CNT_Q1) {
        lsb |= _BV(1);
    }
    /* Check if Q2 is set */
    if(GPIO_CNT_Q2_PIN & GPIO_CNT_Q2) {
        lsb |= _BV(2);
    }
    /* Check if Q3 is set */
    if(GPIO_CNT_Q3_PIN & GPIO_CNT_Q3) {
        lsb |= _BV(3);
    }
    /* Check if Q4 is set */
    if(GPIO_CNT_Q4_PIN & GPIO_CNT_Q4) {
        lsb |= _BV(4);
    }
    /* Check if Q5 is set */
    if(GPIO_CNT_Q5_PIN & GPIO_CNT_Q5) {
        lsb |= _BV(5);
    }
    /* Check if Q6 is set */
    if(GPIO_CNT_Q6_PIN & GPIO_CNT_Q6) {
        lsb |= _BV(6);
    }
    /* Check if Q7 is set */
    if(GPIO_CNT_Q7_PIN & GPIO_CNT_Q7) {
        lsb |= _BV(7);
    }
    /* Return the LSB */
    return lsb;
}

/* Reset the LSB counter */
void cnt_reset_lsb(void) {
    /* Set the reset line to "1" */
    GPIO_CNT_RST_PORT |= GPIO_CNT_RST;
    /* Wait for 10ms to ensure proper reseting */
    _delay_ms(10);
    /* Set the reset line back to "0" */
    GPIO_CNT_RST_PORT &= ~GPIO_CNT_RST;
}


/***** MAIN ROUTINE ***************************************************/
int main(void) {
    /*** Variables ***/
    
    /*** Initialize the hardware ***/
    /* GPIO */
    gpio_init();
    /* Timer/counter 1 */
    timer1_init();
    /* External interrupt */
    extint_init();
    /* I2C slave */
    i2c_slave_init(I2C_SLAVE_ADDRESS);
#if DEBUG_UART_ENABLE
    /* UART */
    uart_init();
#endif
    
    /* Enable interrupts */
    sei();
    
    /* Main Routine */
#if DEBUG_UART_ENABLE
    while (1) {
        if(*config & _BV(I2C_CONFIG_RDY)) {
            /*** Write the latest frequency measurement ***/
            /* First print the selected channel */
            uint8_t ch = (*config & I2C_CONFIG_CHSEL_MASK) >> I2C_CONFIG_CHSEL;
            uart_puts("CH");
            uart_putc(ch+'0');
            /* Second print the frequency */
            uint8_t resolution = (*config & I2C_CONFIG_RES_MASK) >> I2C_CONFIG_RES;
            uint32_t frequency = ((uint32_t)(*xmsb)<<16) | ((uint32_t)(*msb)<<8) | (uint32_t)(*lsb);
            uart_puts(" - Fmeas = ");
            uart_print_int(frequency);
            switch(resolution) {
                case 0:
                    uart_puts(" Hz\n");
                    break;
                case 1:
                    uart_puts(" kHz\n");
                    break;
                case 2:
                    uart_puts(" MHz\n");
                    break;
                }
        } else {
            uart_puts("No measurement available yet\n");
        }
        /* Wait for 1s */
        _delay_ms(1000);
    }
#else
    while(1);
#endif

    return(0);
}
