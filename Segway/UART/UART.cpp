#include "UART.h"

const unsigned int UART::MYAVR32_UART_OFFSET_CR = 0x00;
const unsigned int UART::MYAVR32_UART_OFFSET_MR = 0x04;
const unsigned int UART::MYAVR32_UART_OFFSET_IDR = 0x0C;
const unsigned int UART::MYAVR32_UART_OFFSET_IMR = 0x10;
const unsigned int UART::MYAVR32_UART_OFFSET_CSR = 0x14;
const unsigned int UART::MYAVR32_UART_OFFSET_RHR = 0x18;
const unsigned int UART::MYAVR32_UART_OFFSET_THR = 0x1C;
const unsigned int UART::MYAVR32_UART_OFFSET_BRGR = 0x20;
const unsigned int UART::MYAVR32_UART_OFFSET_RTOR = 0x24;
const unsigned int UART::MYAVR32_UART_OFFSET_TTGR = 0x28;

const char* UART::MYAVR32_GPIO_ADDRESS = (char*) 0xFFFF1000;
const unsigned int UART::MYAVR32_GPIO_SIZE_PORT = 0x0100;
const unsigned int UART::MYAVR32_GPIO_OFFSET_GPER = 0x00;
const unsigned int UART::MYAVR32_GPIO_OFFSET_ODER = 0x40;
const unsigned int UART::MYAVR32_GPIO_OFFSET_PUER = 0x70;
const unsigned int UART::MYAVR32_GPIO_OFFSET_PMR0 = 0x10;
const unsigned int UART::MYAVR32_GPIO_OFFSET_PMR1 = 0x20;


UART::UART() {
    thisUARTConfig = 0;
}


UART::~UART() {
    cleanUp();
}

// Initializes the USART registers.
// Returns zero if specified baud rate cannot be achieved.
bool UART::init(Configuration::s_UARTConfig* thisUARTConfig_) {
    thisUARTConfig = thisUARTConfig_;
    // Reset previous configuration
    cleanUp();
    // Initialization code taken from http://asf.atmel.com/docs/latest/avr32.drivers.intc.example.evk1101/html/group__group__avr32__drivers__usart.html#gaf8b81384234af455b491c44447d896f1
    // Calculate Baudrate Prescaler
    unsigned int over = (Configuration::PBACLK >= 16 * thisUARTConfig->baudRate) ? 16 : 8;
    unsigned int cd_fp = ((1 << 3) * Configuration::PBACLK + (over * thisUARTConfig->baudRate) / 2) / (over * thisUARTConfig->baudRate);
    unsigned int cd = cd_fp >> 3;
    unsigned int fp = cd_fp & ((1 << 3) - 1);
    if (cd < 1 || cd > (1 << 16) - 1)
        return false;
    //thisUARTConfig->usart->MR.sync = 0; // Async mode
    CLEAR_BIT(* (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_MR), 8);
    //thisUARTConfig->usart->MR.usclks = 0; // use CLK_USART (peripheral clock)
    CLEAR_BIT(* (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_MR), 5);
    CLEAR_BIT(* (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_MR), 4);
    //thisUARTConfig->usart->MR.over = ( over == 8 ); // oversampling: 1 -> 8 times, 0 -> 16 times
    if (over == 8)
        SET_BIT(* (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_MR), 19);
    else
        CLEAR_BIT(* (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_MR), 19);
    //thisUARTConfig->usart->BRGR.cd = cd; // Baudrate generator config
    CLEAR_BITS(* (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_BRGR), 0xFFFF);
    SET_BITS(* (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_BRGR), cd & 0xFFFF);
    //thisUARTConfig->usart->BRGR.fp = fp; // Baudrate generator config
    CLEAR_BITS(* (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_BRGR), 0x07 << 16);
    SET_BITS(* (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_BRGR), (fp & 0x07) << 16);
    if (thisUARTConfig->charlength == 9) {
        // Character length set to 9 bits. MODE9 dominates CHRL.
        //thisUARTConfig->usart->MR.mode9 = 1;
        SET_BIT(* (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_MR), 17);
    } else {
        // CHRL gives the character length (- 5) when MODE9 = 0.
        //thisUARTConfig->usart->MR.mode9 = 0;
        CLEAR_BIT(* (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_MR), 17);
        //thisUARTConfig->usart->MR.chrl = thisUARTConfig->charlength - 5;
        CLEAR_BITS(* (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_MR), 0x03 << 6);
        SET_BITS(* (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_MR), ((thisUARTConfig->charlength - 5) & 0x03) << 6);
    }
    //thisUARTConfig->usart->MR.par = thisUARTConfig->paritytype;
    CLEAR_BITS(* (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_MR), 0x07 << 9);
    SET_BITS(* (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_MR), (thisUARTConfig->paritytype & 0x07) << 9);
    //thisUARTConfig->usart->MR.chmode = thisUARTConfig->channelmode;
    CLEAR_BITS(* (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_MR), 0x03 << 14);
    SET_BITS(* (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_MR), (thisUARTConfig->channelmode & 0x03) << 14);
    // Insert 1 (stopbits=0), 1.5 (stopbits=1) or 2 (stopbits=2) stop bits.
    //thisUARTConfig->usart->MR.nbstop = thisUARTConfig->stopbits;
    CLEAR_BITS(* (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_MR), 0x03 << 12);
    SET_BITS(* (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_MR), (thisUARTConfig->stopbits & 0x03) << 12);
    // Set normal mode.
    //thisUARTConfig->usart->MR.mode = 0;
    CLEAR_BITS(* (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_MR), 0x0F);
    // Setup complete; enable communication.
    // Enable input and output.
    //thisUARTConfig->usart->CR.rxen = 1;
    SET_BIT(* (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_CR), 4);
    //thisUARTConfig->usart->CR.txen = 1;
    SET_BIT(* (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_CR), 6);
    return true;
}


bool UART::enableInPinSelector(bool enabled) {
    if (thisUARTConfig == 0)
        return false;
    if (enabled) {
        // Pin is controlled by function, not by GPIO
        //CLEAR_BIT( AVR32_GPIO.port[thisUARTConfig->RX_GPIO_port].gper, thisUARTConfig->RX_GPIO_pin );
        CLEAR_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + thisUARTConfig->RX_GPIO_port * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_GPER), thisUARTConfig->RX_GPIO_pin);
        //CLEAR_BIT( AVR32_GPIO.port[thisUARTConfig->TX_GPIO_port].gper, thisUARTConfig->TX_GPIO_pin );
        CLEAR_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + thisUARTConfig->TX_GPIO_port * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_GPER), thisUARTConfig->TX_GPIO_pin);
        // TX Pin is driven, RX Pin is not driven
        //CLEAR_BIT( AVR32_GPIO.port[thisUARTConfig->RX_GPIO_port].oder, thisUARTConfig->RX_GPIO_pin );
        CLEAR_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + thisUARTConfig->RX_GPIO_port * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_ODER), thisUARTConfig->RX_GPIO_pin);
        //SET_BIT( AVR32_GPIO.port[thisUARTConfig->TX_GPIO_port].oder, thisUARTConfig->TX_GPIO_pin );
        SET_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + thisUARTConfig->TX_GPIO_port * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_ODER), thisUARTConfig->TX_GPIO_pin);
        // Select function
        if (BIT_IS_SET(thisUARTConfig->RX_GPIO_multiplexRegisterValue, 1)) {
            //SET_BIT( AVR32_GPIO.port[thisUARTConfig->RX_GPIO_port].pmr1, thisUARTConfig->RX_GPIO_pin );
            SET_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + thisUARTConfig->RX_GPIO_port * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_PMR1), thisUARTConfig->RX_GPIO_pin);
        } else {
            //CLEAR_BIT( AVR32_GPIO.port[thisUARTConfig->RX_GPIO_port].pmr1, thisUARTConfig->RX_GPIO_pin );
            CLEAR_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + thisUARTConfig->RX_GPIO_port * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_PMR1), thisUARTConfig->RX_GPIO_pin);
        }
        // Select function
        if (BIT_IS_SET(thisUARTConfig->RX_GPIO_multiplexRegisterValue, 0)) {
            //SET_BIT( AVR32_GPIO.port[thisUARTConfig->RX_GPIO_port].pmr0, thisUARTConfig->RX_GPIO_pin );
            SET_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + thisUARTConfig->RX_GPIO_port * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_PMR0), thisUARTConfig->RX_GPIO_pin);
        } else {
            //CLEAR_BIT( AVR32_GPIO.port[thisUARTConfig->RX_GPIO_port].pmr0, thisUARTConfig->RX_GPIO_pin );
            CLEAR_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + thisUARTConfig->RX_GPIO_port * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_PMR0), thisUARTConfig->RX_GPIO_pin);
        }
        // Select function
        if (BIT_IS_SET(thisUARTConfig->TX_GPIO_multiplexRegisterValue, 1)) {
            //SET_BIT( AVR32_GPIO.port[thisUARTConfig->TX_GPIO_port].pmr1, thisUARTConfig->TX_GPIO_pin );
            SET_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + thisUARTConfig->TX_GPIO_port * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_PMR1), thisUARTConfig->TX_GPIO_pin);
        } else {
            //CLEAR_BIT( AVR32_GPIO.port[thisUARTConfig->TX_GPIO_port].pmr1, thisUARTConfig->TX_GPIO_pin );
            CLEAR_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + thisUARTConfig->TX_GPIO_port * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_PMR1), thisUARTConfig->TX_GPIO_pin);
        }
        // Select function
        if (BIT_IS_SET(thisUARTConfig->TX_GPIO_multiplexRegisterValue, 0)) {
            //SET_BIT( AVR32_GPIO.port[thisUARTConfig->TX_GPIO_port].pmr0, thisUARTConfig->TX_GPIO_pin );
            SET_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + thisUARTConfig->TX_GPIO_port * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_PMR0), thisUARTConfig->TX_GPIO_pin);
        } else {
            //CLEAR_BIT( AVR32_GPIO.port[thisUARTConfig->TX_GPIO_port].pmr0, thisUARTConfig->TX_GPIO_pin );
            CLEAR_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + thisUARTConfig->TX_GPIO_port * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_PMR0), thisUARTConfig->TX_GPIO_pin);
        }
    } else {
        // Handle Pin Control to GPIO
        //SET_BIT( AVR32_GPIO.port[thisUARTConfig->RX_GPIO_port].gper, thisUARTConfig->RX_GPIO_pin );
        SET_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + thisUARTConfig->RX_GPIO_port * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_GPER), thisUARTConfig->RX_GPIO_pin);
        //SET_BIT( AVR32_GPIO.port[thisUARTConfig->TX_GPIO_port].gper, thisUARTConfig->TX_GPIO_pin );
        SET_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + thisUARTConfig->TX_GPIO_port * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_GPER), thisUARTConfig->TX_GPIO_pin);
        // Pin is not driven anymore
        //CLEAR_BIT( AVR32_GPIO.port[thisUARTConfig->RX_GPIO_port].oder, thisUARTConfig->RX_GPIO_pin );
        CLEAR_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + thisUARTConfig->RX_GPIO_port * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_ODER), thisUARTConfig->RX_GPIO_pin);
        //CLEAR_BIT( AVR32_GPIO.port[thisUARTConfig->TX_GPIO_port].oder, thisUARTConfig->TX_GPIO_pin );
        CLEAR_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + thisUARTConfig->TX_GPIO_port * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_ODER), thisUARTConfig->TX_GPIO_pin);
        // Reset Mutiplex Register Values
        //CLEAR_BIT( AVR32_GPIO.port[thisUARTConfig->RX_GPIO_port].pmr1, thisUARTConfig->RX_GPIO_pin );
        CLEAR_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + thisUARTConfig->RX_GPIO_port * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_PMR1), thisUARTConfig->RX_GPIO_pin);
        //CLEAR_BIT( AVR32_GPIO.port[thisUARTConfig->RX_GPIO_port].pmr0, thisUARTConfig->RX_GPIO_pin );
        CLEAR_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + thisUARTConfig->RX_GPIO_port * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_PMR0), thisUARTConfig->RX_GPIO_pin);
        //CLEAR_BIT( AVR32_GPIO.port[thisUARTConfig->TX_GPIO_port].pmr1, thisUARTConfig->TX_GPIO_pin );
        CLEAR_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + thisUARTConfig->TX_GPIO_port * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_PMR1), thisUARTConfig->TX_GPIO_pin);
        //CLEAR_BIT( AVR32_GPIO.port[thisUARTConfig->TX_GPIO_port].pmr0, thisUARTConfig->TX_GPIO_pin );
        CLEAR_BIT(* (volatile unsigned int*)(MYAVR32_GPIO_ADDRESS + thisUARTConfig->TX_GPIO_port * MYAVR32_GPIO_SIZE_PORT + MYAVR32_GPIO_OFFSET_PMR0), thisUARTConfig->TX_GPIO_pin);
    }
    return true;
}


// Returns true if new data was received. Fetch data using getData().
bool UART::isDataAvailable() {
    //return thisUARTConfig->usart->CSR.rxrdy;
    return BIT_IS_SET(* (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_CSR), 0);
}


// Returns the data
unsigned long UART::getData() {
    //return thisUARTConfig->usart->RHR.rxchr;
    return * (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_RHR) & 0x1FF;
}


// Returns true if sendChar can be used.
bool UART::isSendBufferReady() {
    //return thisUARTConfig->usart->CSR.txrdy;
    return BIT_IS_SET(* (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_CSR), 1);
}


// Sends data. Blocks until data can be sent.
void UART::sendChar(unsigned long data) {
    while (!isSendBufferReady());
    //thisUARTConfig->usart->THR.txchr = data;
    CLEAR_BITS(* (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_THR), 0x1FF);
    SET_BITS(* (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_THR), data & 0x1FF);
}


// Sends text. Blocks until data can be sent.
void UART::sendString(const char* text) {
    while (*text != 0) {
        sendChar(*text);
        text++;
    }
}


// Converts number to ascii text and sends it. Blocks until data can be sent.
void UART::sendNumber(long number) {
    if (number < 0) {
        sendChar('-');
        sendNumber((unsigned long)((-1) * number));
    } else
        sendNumber((unsigned long) number);
}


// Converts number to ascii text and sends it. Blocks until data can be sent.
void UART::sendNumber(unsigned long number) {
    if (number == 0) {
        sendChar('0');
        return;
    }
    char buffer[10]; // max value of long is 4.294.967.295 -> 10 digits
    unsigned long tmpNumber = number;
    unsigned char i = 0;
    do {
        buffer[i] = '0' + tmpNumber % 10; // store least significant digit as ascii char
        i++;
        tmpNumber /= 10; // remove least significant digit
    } while (tmpNumber > 0);
    // send buffer ( buffer contains number as ascii, but reversed: number: 123 --> buffer: 321 )
    do {
        i--;
        sendChar(buffer[i]);
    } while (i != 0);
}


// Cleans up the USART registers.
void UART::cleanUp() {
    if (thisUARTConfig == 0)
        return;
    //thisUARTConfig->usart->cr = AVR32_USART_CR_RSTRX_MASK   |
    //                          AVR32_USART_CR_RSTTX_MASK   |
    //                          AVR32_USART_CR_RSTSTA_MASK  |
    //                          AVR32_USART_CR_RSTIT_MASK   |
    //                          AVR32_USART_CR_RSTNACK_MASK |
    //                          AVR32_USART_CR_DTRDIS_MASK  |
    //                          AVR32_USART_CR_RTSDIS_MASK;
    SET_BITS(* (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_CR), (0x04 | 0x08 | 0x0100 | 0x2000 | 0x4000 | 0x20000 | 0x80000));
    //thisUARTConfig->usart->mr = 0;
    * (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_MR) = 0;
    //thisUARTConfig->usart->idr = thisUARTConfig->usart->imr;
    * (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_IDR) = * (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_IMR);
    //thisUARTConfig->usart->csr; // read status register
    * (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_CSR);
    //thisUARTConfig->usart->brgr = 0;
    * (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_BRGR) = 0;
    //thisUARTConfig->usart->rtor = 0;
    * (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_RTOR) = 0;
    //thisUARTConfig->usart->ttgr = 0;
    * (volatile unsigned int*)(thisUARTConfig->usart_address + MYAVR32_UART_OFFSET_TTGR) = 0;
}