#ifndef UART_H_
#define UART_H_

#include "../Configuration/Configuration.h"
#include "../bitmacros.h"

class UART {
  private:
    Configuration::s_UARTConfig* thisUARTConfig;
    bool isSendBufferReady();
    void cleanUp();
    static const unsigned int MYAVR32_UART_OFFSET_CR;
    static const unsigned int MYAVR32_UART_OFFSET_MR;
    static const unsigned int MYAVR32_UART_OFFSET_IDR;
    static const unsigned int MYAVR32_UART_OFFSET_IMR;
    static const unsigned int MYAVR32_UART_OFFSET_CSR;
    static const unsigned int MYAVR32_UART_OFFSET_RHR;
    static const unsigned int MYAVR32_UART_OFFSET_THR;
    static const unsigned int MYAVR32_UART_OFFSET_BRGR;
    static const unsigned int MYAVR32_UART_OFFSET_RTOR;
    static const unsigned int MYAVR32_UART_OFFSET_TTGR;
    static const char* MYAVR32_GPIO_ADDRESS;
    static const unsigned int MYAVR32_GPIO_SIZE_PORT;
    static const unsigned int MYAVR32_GPIO_OFFSET_GPER;
    static const unsigned int MYAVR32_GPIO_OFFSET_ODER;
    static const unsigned int MYAVR32_GPIO_OFFSET_PUER;
    static const unsigned int MYAVR32_GPIO_OFFSET_PMR0;
    static const unsigned int MYAVR32_GPIO_OFFSET_PMR1;

  public:
    UART();
    ~UART();
    bool init(Configuration::s_UARTConfig* thisUARTConfig_);
    bool enableInPinSelector(bool enabled);
    bool isDataAvailable();
    unsigned long getData();
    void sendChar(unsigned long data);
    void sendString(const char* text);
    void sendNumber(long number);
    void sendNumber(unsigned long number);
};

#endif /* UART_H_ */