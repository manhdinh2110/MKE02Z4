#include "OVEN_UART.h"


uint8_t txbuff[]   = "Uart polling example\r\nBoard will send back received characters\r\n";


#define DEMO_UART            UART1
#define DEMO_UART_CLK_FREQ   CLOCK_GetFreq(kCLOCK_BusClk)
#define DEMO_UART_IRQn       UART1_IRQn
#define DEMO_UART_IRQHandler UART1_IRQHandler


#define TIMEOUT_LIMIT 50000
#define BUFFER_SIZE   64
#define MAX_WAIT_TIME 25  //


void Config_UART(void)
{
	uart_config_t config;

	uart_transfer_t xfer;
	uart_transfer_t sendXfer;
	uart_transfer_t receiveXfer;

    UART_GetDefaultConfig(&config);
    config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
    config.enableTx     = true;
    config.enableRx     = true;

    UART_Init(DEMO_UART, &config, DEMO_UART_CLK_FREQ);
    UART_WriteBlocking(DEMO_UART, txbuff, sizeof(txbuff) - 1);

	//UART_WriteBlocking(DEMO_UART,g_tipString,sizeof(g_tipString));

    /* Enable RX interrupt. */
  //  UART_EnableInterrupts(DEMO_UART, kUART_RxDataRegFullInterruptEnable | kUART_RxOverrunInterruptEnable);
   // EnableIRQ(DEMO_UART_IRQn);

}
void UART_WriteMultipleBytes(uint8_t *data, uint32_t length) {
    for (uint32_t i = 0; i < length; i++) {
        UART_WriteByte(DEMO_UART, data[i]);
    }
}

uint32_t UART_ReadMultipleBytes(UART_Type *base, uint8_t *buffer, uint32_t maxLength) {
    uint32_t bytesRead = 0;
    uint32_t waitTime = MAX_WAIT_TIME;

    memset(buffer, 0, maxLength);  // Delete buffer before recived data

    while (bytesRead < maxLength - 1) {
        while (!(kUART_RxDataRegFullFlag & UART_GetStatusFlags(base))) {
            if (--waitTime == 0) {
                buffer[bytesRead] = '\0';
                return bytesRead;
            }
        }

        buffer[bytesRead] = UART_ReadByte(base);

        if (buffer[bytesRead] == '\n' || buffer[bytesRead] == '\r') {
            break;
        }

        bytesRead++;
        waitTime = MAX_WAIT_TIME;
    }

    buffer[bytesRead] = '\0';  // It is important for finishing funtion read
    return bytesRead;
}
