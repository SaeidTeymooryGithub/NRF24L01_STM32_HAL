
#ifndef NRF24_H_
#define NRF24_H_

#include "stm32f1xx_hal.h"  // Adjust for your STM32 series, e.g., stm32f1xx_hal.h
#include "nRF24L01.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

// Enums
typedef enum {
    RF24_PA_MIN = 0,
    RF24_PA_LOW,
    RF24_PA_HIGH,
    RF24_PA_MAX,
    RF24_PA_ERROR
} rf24_pa_dbm_e;

typedef enum {
    RF24_1MBPS = 0,
    RF24_2MBPS,
    RF24_250KBPS
} rf24_datarate_e;

typedef enum {
    RF24_CRC_DISABLED = 0,
    RF24_CRC_8,
    RF24_CRC_16
} rf24_crclength_e;

typedef enum {
    RF24_FIFO_OCCUPIED,
    RF24_FIFO_EMPTY,
    RF24_FIFO_FULL,
    RF24_FIFO_INVALID,
} rf24_fifo_state_e;

typedef enum {
    RF24_IRQ_NONE = 0,
    RF24_TX_DF = (1 << MASK_MAX_RT),
    RF24_TX_DS = (1 << TX_DS),
    RF24_RX_DR = (1 << RX_DR),
    RF24_IRQ_ALL = ((1 << MASK_MAX_RT) | (1 << TX_DS) | (1 << RX_DR)),
} rf24_irq_flags_e;

// Struct for RF24 context
typedef struct {
    SPI_HandleTypeDef *hspi;        // SPI handle
    GPIO_TypeDef *ce_port;          // CE GPIO port
    uint16_t ce_pin;                // CE GPIO pin
    GPIO_TypeDef *csn_port;         // CSN GPIO port
    uint16_t csn_pin;               // CSN GPIO pin
    uint8_t status;                 // Status byte
    uint8_t payload_size;           // Fixed payload size
    uint8_t pipe0_reading_address[5]; // Pipe 0 read address
    uint8_t pipe0_writing_address[5]; // Pipe 0 write address
    uint8_t config_reg;             // NRF_CONFIG register cache
    bool _is_p_variant;             // P-variant flag
    bool _is_p0_rx;                 // Pipe 0 RX flag
    bool ack_payloads_enabled;      // ACK payloads enabled
    uint8_t addr_width;             // Address width (3-5 bytes)
    bool dynamic_payloads_enabled;  // Dynamic payloads enabled
    uint32_t txDelay;               // TX delay based on data rate
    uint8_t csDelay;                // CSN settle delay (in us, default 10)
} RF24_t;

// Function prototypes
void RF24_init(RF24_t *rf24, SPI_HandleTypeDef *hspi, GPIO_TypeDef *ce_port, uint16_t ce_pin, GPIO_TypeDef *csn_port, uint16_t csn_pin);
bool RF24_begin(RF24_t *rf24);
void RF24_setAddressWidth(RF24_t *rf24, uint8_t a_width);
bool RF24_isChipConnected(RF24_t *rf24);
void RF24_startListening(RF24_t *rf24);
void RF24_stopListening(RF24_t *rf24);
bool RF24_available(RF24_t *rf24);
bool RF24_available_pipe(RF24_t *rf24, uint8_t *pipe_num);
bool RF24_read(RF24_t *rf24, void *buf, uint8_t len);
bool RF24_write(RF24_t *rf24, const void *buf, uint8_t len, const bool multicast);
void RF24_reUseTX(RF24_t *rf24);
uint8_t RF24_flush_tx(RF24_t *rf24);
uint8_t RF24_flush_rx(RF24_t *rf24);
bool RF24_txStandBy(RF24_t *rf24, uint32_t timeout, bool startTx);
void RF24_writeAckPayload(RF24_t *rf24, uint8_t pipe, const void *buf, uint8_t len);
bool RF24_isAckPayloadAvailable(RF24_t *rf24);
void RF24_whatHappened(RF24_t *rf24, bool *tx_ok, bool *tx_fail, bool *rx_ready);
void RF24_openWritingPipe(RF24_t *rf24, const uint8_t *address);
void RF24_openReadingPipe(RF24_t *rf24, uint8_t number, const uint8_t *address);
void RF24_closeReadingPipe(RF24_t *rf24, uint8_t pipe);
void RF24_setChannel(RF24_t *rf24, uint8_t channel);
uint8_t RF24_getChannel(RF24_t *rf24);
void RF24_setRetries(RF24_t *rf24, uint8_t delay, uint8_t count);
uint8_t RF24_getRetries(RF24_t *rf24);
void RF24_setPALevel(RF24_t *rf24, uint8_t level, bool lnaEnable);
uint8_t RF24_getPALevel(RF24_t *rf24);
bool RF24_setDataRate(RF24_t *rf24, rf24_datarate_e speed);
rf24_datarate_e RF24_getDataRate(RF24_t *rf24);
void RF24_setCRCLength(RF24_t *rf24, rf24_crclength_e length);
rf24_crclength_e RF24_getCRCLength(RF24_t *rf24);
void RF24_disableCRC(RF24_t *rf24);
void RF24_enableDynamicPayloads(RF24_t *rf24);
void RF24_disableDynamicPayloads(RF24_t *rf24);
void RF24_enableAckPayload(RF24_t *rf24);
void RF24_disableAckPayload(RF24_t *rf24);
void RF24_enableDynamicAck(RF24_t *rf24);
bool RF24_isPVariant(RF24_t *rf24);
void RF24_setAutoAck(RF24_t *rf24, bool enable);
void RF24_setAutoAck_pipe(RF24_t *rf24, uint8_t pipe, bool enable);
bool RF24_testCarrier(RF24_t *rf24);
bool RF24_testRPD(RF24_t *rf24);
uint8_t RF24_getARC(RF24_t *rf24);
void RF24_powerDown(RF24_t *rf24);
void RF24_powerUp(RF24_t *rf24);
uint8_t RF24_getPayloadSize(RF24_t *rf24);
void RF24_setPayloadSize(RF24_t *rf24, uint8_t size);
uint8_t RF24_getDynamicPayloadSize(RF24_t *rf24);
void RF24_startConstCarrier(RF24_t *rf24, rf24_pa_dbm_e level, uint8_t channel);
void RF24_stopConstCarrier(RF24_t *rf24);
void RF24_toggleAllPipes(RF24_t *rf24, bool isEnabled);
void RF24_setRadiation(RF24_t *rf24, uint8_t level, rf24_datarate_e speed, bool lnaEnable);
void RF24_maskIRQ(RF24_t *rf24, bool tx_ok, bool tx_fail, bool rx_ready);

// Helper function prototypes
void RF24_csn(RF24_t *rf24, bool mode);
void RF24_ce(RF24_t *rf24, bool level);
void RF24_beginTransaction(RF24_t *rf24);
void RF24_endTransaction(RF24_t *rf24);
void RF24_read_register(RF24_t *rf24, uint8_t reg, uint8_t *buf, uint8_t len);
uint8_t RF24_read_register_single(RF24_t *rf24, uint8_t reg);
void RF24_write_register(RF24_t *rf24, uint8_t reg, const uint8_t *buf, uint8_t len);
void RF24_write_register_single(RF24_t *rf24, uint8_t reg, uint8_t value);
void RF24_write_payload(RF24_t *rf24, const void *buf, uint8_t data_len, const uint8_t writeType);
void RF24_read_payload(RF24_t *rf24, void *buf, uint8_t len);
uint8_t RF24_flush_rx(RF24_t *rf24);
uint8_t RF24_flush_tx(RF24_t *rf24);
uint8_t RF24_get_status(RF24_t *rf24);
void RF24_toggle_features(RF24_t *rf24);
uint8_t RF24_getDynamicPayloadSize(RF24_t *rf24);
uint8_t RF24_data_rate_reg_value(rf24_datarate_e speed);
uint8_t RF24_pa_level_reg_value(uint8_t level, bool lnaEnable);

#endif /* NRF24_H_ */
