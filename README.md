
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "NRF24.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

extern SPI_HandleTypeDef hspi1;
RF24_t radio;
void RF24_transmitter_init(void)
{
	RF24_init(&radio, &hspi1, NRF_CE_GPIO_Port, NRF_CE_Pin, NRF_CS_GPIO_Port, NRF_CS_Pin); // CE: PB0, CSN: PB1

	if (!RF24_begin(&radio)) {
		char msg[] = "RF24 init failed!\r\n";
		HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
		while (1);
	}

	RF24_setChannel(&radio, 76);
	RF24_setDataRate(&radio, RF24_1MBPS);
	RF24_setPALevel(&radio, RF24_PA_MAX, true);
	RF24_setRetries(&radio, 5, 15);
	RF24_enableDynamicPayloads(&radio);
	RF24_setAutoAck(&radio, true);

	uint8_t address[] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	RF24_openWritingPipe(&radio, address);

	RF24_flush_tx(&radio);
	RF24_write_register_single(&radio, NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
}

void RF24_transmitter_loop(void) {
	static uint32_t counter = 0;
	char msg[32];
	snprintf(msg, sizeof(msg), "Count: %lu", counter++);

	uint8_t fifo_status = RF24_read_register_single(&radio, FIFO_STATUS);
	if (fifo_status & _BV(FIFO_FULL)) {
		RF24_flush_tx(&radio);
		char warn[] = "TX FIFO cleared!\r\n";
		//HAL_UART_Transmit(&huart1, (uint8_t *)warn, strlen(warn), HAL_MAX_DELAY);
	}

	bool tx_ok, tx_fail, rx_ready;
	RF24_write(&radio, msg, strlen(msg), false);
	RF24_whatHappened(&radio, &tx_ok, &tx_fail, &rx_ready);

	if (tx_ok) {
		char status[] = "Sent OK!\r\n";
		//HAL_UART_Transmit(&huart1, (uint8_t *)status, strlen(status), HAL_MAX_DELAY);
	} else if (tx_fail) {
		char status[] = "Send failed (MAX_RT)!\r\n";
		//HAL_UART_Transmit(&huart1, (uint8_t *)status, strlen(status), HAL_MAX_DELAY);
		RF24_flush_tx(&radio); // Clear TX FIFO
		RF24_write_register_single(&radio, NRF_STATUS, _BV(MAX_RT)); // Clear MAX_RT
	} else {
		char status[] = "Send pending...\r\n";
		//HAL_UART_Transmit(&huart1, (uint8_t *)status, strlen(status), HAL_MAX_DELAY);
	}

	RF24_txStandBy(&radio, 500, false); // Wait up to 500ms
}

int main(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_USART1_UART_Init();
 
	RF24_transmitter_init();
 
	while (1)
	{
		RF24_transmitter_loop();
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_Delay(1000); // Send every 100ms
	}
}
