
#include "NRF24.h"

// Helper functions
void RF24_csn(RF24_t *rf24, bool mode) {
	HAL_GPIO_WritePin(rf24->csn_port, rf24->csn_pin, mode ? GPIO_PIN_SET : GPIO_PIN_RESET);
	DELAY_US(rf24->csDelay);
}

void RF24_ce(RF24_t *rf24, bool level) {
	HAL_GPIO_WritePin(rf24->ce_port, rf24->ce_pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void RF24_beginTransaction(RF24_t *rf24) {
	RF24_csn(rf24, false);
}

void RF24_endTransaction(RF24_t *rf24) {
	RF24_csn(rf24, true);
}

void RF24_read_register(RF24_t *rf24, uint8_t reg, uint8_t *buf, uint8_t len) {
	uint8_t tx_buf[33] = {0};
	uint8_t rx_buf[33] = {0};
	tx_buf[0] = R_REGISTER | (reg & REGISTER_MASK);
	RF24_beginTransaction(rf24);
	HAL_SPI_TransmitReceive(rf24->hspi, tx_buf, rx_buf, len + 1, HAL_MAX_DELAY);
	rf24->status = rx_buf[0];
	memcpy(buf, &rx_buf[1], len);
	RF24_endTransaction(rf24);
}

uint8_t RF24_read_register_single(RF24_t *rf24, uint8_t reg) {
	uint8_t result;
	RF24_read_register(rf24, reg, &result, 1);
	return result;
}

void RF24_write_register(RF24_t *rf24, uint8_t reg, const uint8_t *buf, uint8_t len) {
	uint8_t tx_buf[33] = {0};
	uint8_t rx_buf[33] = {0};
	tx_buf[0] = W_REGISTER | (reg & REGISTER_MASK);
	memcpy(&tx_buf[1], buf, len);
	RF24_beginTransaction(rf24);
	HAL_SPI_TransmitReceive(rf24->hspi, tx_buf, rx_buf, len + 1, HAL_MAX_DELAY);
	rf24->status = rx_buf[0];
	RF24_endTransaction(rf24);
}

void RF24_write_register_single(RF24_t *rf24, uint8_t reg, uint8_t value) {
	RF24_write_register(rf24, reg, &value, 1);
}

void RF24_write_payload(RF24_t *rf24, const void *buf, uint8_t data_len, const uint8_t writeType) {
	const uint8_t *current = (const uint8_t *)buf;
	uint8_t blank_len = rf24->dynamic_payloads_enabled ? 0 : rf24->payload_size - data_len;
	data_len = rf24_min(data_len, 32);
	blank_len = rf24_min(blank_len, 32 - data_len);

	uint8_t tx_buf[33] = {0};
	uint8_t rx_buf[33] = {0};
	tx_buf[0] = writeType;
	memcpy(&tx_buf[1], current, data_len);

	RF24_beginTransaction(rf24);
	HAL_SPI_TransmitReceive(rf24->hspi, tx_buf, rx_buf, data_len + blank_len + 1, HAL_MAX_DELAY);
	rf24->status = rx_buf[0];
	RF24_endTransaction(rf24);
}

void RF24_read_payload(RF24_t *rf24, void *buf, uint8_t len) {
	uint8_t tx_buf[33] = {0};
	uint8_t rx_buf[33] = {0};
	tx_buf[0] = R_RX_PAYLOAD;
	for (uint8_t i = 1; i <= len; i++) {
		tx_buf[i] = RF24_NOP;
	}
	RF24_beginTransaction(rf24);
	HAL_SPI_TransmitReceive(rf24->hspi, tx_buf, rx_buf, len + 1, HAL_MAX_DELAY);
	rf24->status = rx_buf[0];
	memcpy(buf, &rx_buf[1], len);
	RF24_endTransaction(rf24);
}

uint8_t RF24_flush_rx(RF24_t *rf24) {
	uint8_t cmd = FLUSH_RX;
	uint8_t rx;
	RF24_beginTransaction(rf24);
	HAL_SPI_TransmitReceive(rf24->hspi, &cmd, &rx, 1, HAL_MAX_DELAY);
	rf24->status = rx;
	RF24_endTransaction(rf24);
	return rf24->status;
}

uint8_t RF24_flush_tx(RF24_t *rf24) {
	uint8_t cmd = FLUSH_TX;
	uint8_t rx;
	RF24_beginTransaction(rf24);
	HAL_SPI_TransmitReceive(rf24->hspi, &cmd, &rx, 1, HAL_MAX_DELAY);
	rf24->status = rx;
	RF24_endTransaction(rf24);
	return rf24->status;
}

uint8_t RF24_get_status(RF24_t *rf24) {
	uint8_t cmd = RF24_NOP;
	uint8_t rx;
	RF24_beginTransaction(rf24);
	HAL_SPI_TransmitReceive(rf24->hspi, &cmd, &rx, 1, HAL_MAX_DELAY);
	rf24->status = rx;
	RF24_endTransaction(rf24);
	return rf24->status;
}

void RF24_toggle_features(RF24_t *rf24) {
	uint8_t cmd[2] = {ACTIVATE, 0x73};
	uint8_t rx[2];
	RF24_beginTransaction(rf24);
	HAL_SPI_TransmitReceive(rf24->hspi, cmd, rx, 2, HAL_MAX_DELAY);
	rf24->status = rx[0];
	RF24_endTransaction(rf24);
}

uint8_t RF24_getDynamicPayloadSize(RF24_t *rf24) {
	uint8_t result = 0;
	uint8_t tx_buf[2] = {R_RX_PL_WID, RF24_NOP};
	uint8_t rx_buf[2];
	RF24_beginTransaction(rf24);
	HAL_SPI_TransmitReceive(rf24->hspi, tx_buf, rx_buf, 2, HAL_MAX_DELAY);
	rf24->status = rx_buf[0];
	result = rx_buf[1];
	RF24_endTransaction(rf24);
	if (result > 32) {
		RF24_flush_rx(rf24);
		HAL_Delay(2);
		return 0;
	}
	return result;
}

uint8_t RF24_data_rate_reg_value(rf24_datarate_e speed) {
	if (speed == RF24_250KBPS) {
		return _BV(RF_DR_LOW);
	} else if (speed == RF24_2MBPS) {
		return _BV(RF_DR_HIGH);
	}
	return 0;
}

uint8_t RF24_pa_level_reg_value(uint8_t level, bool lnaEnable) {
	level = (level > RF24_PA_MAX) ? RF24_PA_MAX : level;
	return ((level << 1) + lnaEnable);
}

// Public functions
void RF24_init(RF24_t *rf24, SPI_HandleTypeDef *hspi, GPIO_TypeDef *ce_port, uint16_t ce_pin, GPIO_TypeDef *csn_port, uint16_t csn_pin) {
	rf24->hspi = hspi;
	rf24->ce_port = ce_port;
	rf24->ce_pin = ce_pin;
	rf24->csn_port = csn_port;
	rf24->csn_pin = csn_pin;
	rf24->payload_size = 32;
	rf24->addr_width = 5;
	rf24->dynamic_payloads_enabled = false;
	rf24->ack_payloads_enabled = false;
	rf24->csDelay = 10;
	memset(rf24->pipe0_reading_address, 0, 5);
	memset(rf24->pipe0_writing_address, 0, 5);
	rf24->_is_p_variant = false;
	rf24->_is_p0_rx = false;
	rf24->txDelay = 85;
}

bool RF24_begin(RF24_t *rf24) {
	RF24_ce(rf24, false);
	RF24_csn(rf24, true);

	HAL_Delay(5);

	uint8_t setup = RF24_read_register_single(rf24, RF_SETUP);
	if (setup != 0) {
		RF24_write_register_single(rf24, RF_SETUP, 0);
		setup = RF24_read_register_single(rf24, RF_SETUP);
		if (setup != 0) return false;
	}

	RF24_toggle_features(rf24);
	rf24->_is_p_variant = (RF24_read_register_single(rf24, FEATURE) == 0);
	RF24_toggle_features(rf24);
	if (RF24_read_register_single(rf24, FEATURE) != 0) rf24->_is_p_variant = true;

	rf24->config_reg = 0;
	rf24->dynamic_payloads_enabled = false;
	rf24->ack_payloads_enabled = false;
	rf24->addr_width = 5;

	RF24_write_register_single(rf24, NRF_CONFIG, (rf24->config_reg & (1 << MASK_RX_DR | 1 << MASK_TX_DS | 1 << MASK_MAX_RT)) | (1 << EN_CRC) | (0 << CRCO));
	RF24_write_register_single(rf24, EN_AA, 0x3F);
	RF24_write_register_single(rf24, EN_RXADDR, 0x03);
	RF24_write_register_single(rf24, SETUP_AW, 0x03);
	RF24_write_register_single(rf24, SETUP_RETR, 0x03);
	RF24_write_register_single(rf24, RF_CH, 0x4C);
	RF24_write_register_single(rf24, RF_SETUP, _BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH) | 0);
	RF24_write_register_single(rf24, NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
	RF24_write_register_single(rf24, DYNPD, 0);
	RF24_write_register_single(rf24, FEATURE, 0);

	RF24_setChannel(rf24, 76);
	RF24_setCRCLength(rf24, RF24_CRC_16);
	RF24_setPALevel(rf24, RF24_PA_MAX, true);
	RF24_setDataRate(rf24, RF24_1MBPS);
	RF24_setRetries(rf24, 5, 15);
	RF24_setAutoAck(rf24, true);
	RF24_enableDynamicPayloads(rf24);

	RF24_flush_rx(rf24);
	RF24_flush_tx(rf24);

	RF24_powerUp(rf24);
	rf24->config_reg &= ~(1 << PRIM_RX);
	RF24_write_register_single(rf24, NRF_CONFIG, rf24->config_reg);

	return true;
}

void RF24_powerUp(RF24_t *rf24) {
	uint8_t cfg = rf24->config_reg;
	if (!(cfg & _BV(PWR_UP))) {
		rf24->config_reg |= _BV(PWR_UP);
		RF24_write_register_single(rf24, NRF_CONFIG, rf24->config_reg);
		DELAY_US(RF24_POWERUP_DELAY);
	}
}

void RF24_powerDown(RF24_t *rf24) {
	rf24->config_reg &= ~_BV(PWR_UP);
	RF24_write_register_single(rf24, NRF_CONFIG, rf24->config_reg);
}

bool RF24_available(RF24_t *rf24) {
	return RF24_available_pipe(rf24, NULL);
}

bool RF24_available_pipe(RF24_t *rf24, uint8_t *pipe_num) {
	RF24_get_status(rf24);
	uint8_t pipe = (rf24->status >> RX_P_NO) & 0x07;
	if (pipe > 5) return false;
	if (pipe_num) *pipe_num = pipe;
	return true;
}

bool RF24_read(RF24_t *rf24, void *buf, uint8_t len) {
	RF24_read_payload(rf24, buf, len);
	RF24_write_register_single(rf24, NRF_STATUS, _BV(RX_DR));
	if (len > rf24->payload_size) return false;
	return true;
}

bool RF24_write(RF24_t *rf24, const void *buf, uint8_t len, const bool multicast) {
	RF24_stopListening(rf24);
	RF24_write_payload(rf24, buf, len, multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD);
	RF24_ce(rf24, true);
	DELAY_US(rf24->txDelay);
	RF24_ce(rf24, false);
	return true;
}

void RF24_startListening(RF24_t *rf24) {
	rf24->config_reg |= _BV(PRIM_RX);
	RF24_write_register_single(rf24, NRF_CONFIG, rf24->config_reg);
	RF24_write_register_single(rf24, NRF_STATUS, _BV(RX_DR));
	RF24_ce(rf24, true);
	if (rf24->pipe0_reading_address[0] > 0) {
		RF24_write_register(rf24, RX_ADDR_P0, rf24->pipe0_reading_address, rf24->addr_width);
	}
	rf24->_is_p0_rx = true;
}

void RF24_stopListening(RF24_t *rf24) {
	RF24_ce(rf24, false);
	rf24->config_reg &= ~_BV(PRIM_RX);
	RF24_write_register_single(rf24, NRF_CONFIG, rf24->config_reg);
	if (rf24->ack_payloads_enabled) {
		RF24_flush_tx(rf24);
	}
	rf24->_is_p0_rx = false;
}

void RF24_openWritingPipe(RF24_t *rf24, const uint8_t *address) {
	RF24_write_register(rf24, RX_ADDR_P0, address, rf24->addr_width);
	RF24_write_register(rf24, TX_ADDR, address, rf24->addr_width);
	memcpy(rf24->pipe0_writing_address, address, rf24->addr_width);
}

void RF24_openReadingPipe(RF24_t *rf24, uint8_t number, const uint8_t *address) {
	if (number == 0) {
		memcpy(rf24->pipe0_reading_address, address, rf24->addr_width);
		rf24->_is_p0_rx = true;
	}
	if (number <= 5) {
		if (number < 2) {
			RF24_write_register(rf24, RX_ADDR_P0 + number, address, rf24->addr_width);
		} else {
			RF24_write_register_single(rf24, RX_ADDR_P0 + number, address[0]);
		}
		RF24_write_register_single(rf24, RX_PW_P0 + number, rf24->payload_size);
		RF24_write_register_single(rf24, EN_RXADDR, RF24_read_register_single(rf24, EN_RXADDR) | _BV(number));
	}
}

void RF24_setAddressWidth(RF24_t *rf24, uint8_t a_width) {
	a_width = a_width < 3 ? 3 : (a_width > 5 ? 5 : a_width);
	RF24_write_register_single(rf24, SETUP_AW, a_width - 2);
	rf24->addr_width = a_width;
}

void RF24_enableDynamicPayloads(RF24_t *rf24) {
	RF24_write_register_single(rf24, FEATURE, RF24_read_register_single(rf24, FEATURE) | _BV(EN_DPL));
	RF24_write_register_single(rf24, DYNPD, _BV(DPL_P5) | _BV(DPL_P4) | _BV(DPL_P3) | _BV(DPL_P2) | _BV(DPL_P1) | _BV(DPL_P0));
	rf24->dynamic_payloads_enabled = true;
}

void RF24_disableDynamicPayloads(RF24_t *rf24) {
	RF24_write_register_single(rf24, FEATURE, 0);
	RF24_write_register_single(rf24, DYNPD, 0);
	rf24->dynamic_payloads_enabled = false;
	rf24->ack_payloads_enabled = false;
}

void RF24_enableAckPayload(RF24_t *rf24) {
	if (!rf24->ack_payloads_enabled) {
		RF24_write_register_single(rf24, FEATURE, RF24_read_register_single(rf24, FEATURE) | _BV(EN_ACK_PAY) | _BV(EN_DPL));
		RF24_write_register_single(rf24, DYNPD, RF24_read_register_single(rf24, DYNPD) | _BV(DPL_P1) | _BV(DPL_P0));
		rf24->dynamic_payloads_enabled = true;
		rf24->ack_payloads_enabled = true;
	}
}

void RF24_setRetries(RF24_t *rf24, uint8_t delay, uint8_t count) {
	RF24_write_register_single(rf24, SETUP_RETR, (rf24_min(15, delay) << ARD) | rf24_min(15, count));
}

void RF24_setPALevel(RF24_t *rf24, uint8_t level, bool lnaEnable) {
	uint8_t setup = RF24_read_register_single(rf24, RF_SETUP) & 0xF8;
	setup |= RF24_pa_level_reg_value(level, lnaEnable);
	RF24_write_register_single(rf24, RF_SETUP, setup);
}

bool RF24_setDataRate(RF24_t *rf24, rf24_datarate_e speed) {
	bool result = false;
	uint8_t setup = RF24_read_register_single(rf24, RF_SETUP);
	setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));
	setup |= RF24_data_rate_reg_value(speed);
	RF24_write_register_single(rf24, RF_SETUP, setup);
	if (RF24_read_register_single(rf24, RF_SETUP) == setup) {
		result = true;
		rf24->txDelay = (speed == RF24_250KBPS) ? 505 : (speed == RF24_2MBPS) ? 240 : 85;
	}
	return result;
}

void RF24_setCRCLength(RF24_t *rf24, rf24_crclength_e length) {
	rf24->config_reg &= ~(_BV(CRCO) | _BV(EN_CRC));
	if (length == RF24_CRC_8) {
		rf24->config_reg |= _BV(EN_CRC);
	} else if (length == RF24_CRC_16) {
		rf24->config_reg |= _BV(EN_CRC) | _BV(CRCO);
	}
	RF24_write_register_single(rf24, NRF_CONFIG, rf24->config_reg);
}

void RF24_setChannel(RF24_t *rf24, uint8_t channel) {
	RF24_write_register_single(rf24, RF_CH, rf24_min(channel, 125));
}

void RF24_setAutoAck(RF24_t *rf24, bool enable) {
	if (enable) {
		RF24_write_register_single(rf24, EN_AA, 0x3F);
	} else {
		RF24_write_register_single(rf24, EN_AA, 0);
		if (rf24->ack_payloads_enabled) {
			RF24_disableAckPayload(rf24);
		}
	}
}

void RF24_disableAckPayload(RF24_t *rf24) {
	if (rf24->ack_payloads_enabled) {
		RF24_write_register_single(rf24, FEATURE, RF24_read_register_single(rf24, FEATURE) & ~_BV(EN_ACK_PAY));
		rf24->ack_payloads_enabled = false;
	}
}

void RF24_enableDynamicAck(RF24_t *rf24) {
	RF24_write_register_single(rf24, FEATURE, RF24_read_register_single(rf24, FEATURE) | _BV(EN_DYN_ACK));
}

bool RF24_isPVariant(RF24_t *rf24) {
	return rf24->_is_p_variant;
}

bool RF24_testCarrier(RF24_t *rf24) {
	return (RF24_read_register_single(rf24, CD) & 1);
}

bool RF24_testRPD(RF24_t *rf24) {
	return (RF24_read_register_single(rf24, RPD) & 1);
}

uint8_t RF24_getARC(RF24_t *rf24) {
	return RF24_read_register_single(rf24, OBSERVE_TX) & 0x0F;
}

uint8_t RF24_getPayloadSize(RF24_t *rf24) {
	return rf24->payload_size;
}

void RF24_setPayloadSize(RF24_t *rf24, uint8_t size) {
	rf24->payload_size = rf24_min(size, 32);
}

uint8_t _RF24_getDynamicPayloadSize(RF24_t *rf24) {
	return RF24_getDynamicPayloadSize(rf24);
}

void RF24_startConstCarrier(RF24_t *rf24, rf24_pa_dbm_e level, uint8_t channel) {
	RF24_stopListening(rf24);
	RF24_write_register_single(rf24, RF_SETUP, RF24_read_register_single(rf24, RF_SETUP) | _BV(CONT_WAVE) | _BV(PLL_LOCK));
	if (rf24->_is_p_variant) {
		RF24_setAutoAck(rf24, false);
		RF24_setRetries(rf24, 0, 0);
		uint8_t dummy_buf[32];
		for (uint8_t i = 0; i < 32; ++i) dummy_buf[i] = 0xFF;
		RF24_write_register(rf24, TX_ADDR, dummy_buf, 5);
		RF24_flush_tx(rf24);
		RF24_write_payload(rf24, dummy_buf, 32, W_TX_PAYLOAD);
		RF24_disableCRC(rf24);
	}
	RF24_setPALevel(rf24, level, true);
	RF24_setChannel(rf24, channel);
	RF24_ce(rf24, true);
	if (rf24->_is_p_variant) {
		HAL_Delay(1);
		RF24_reUseTX(rf24);
	}
}

void RF24_stopConstCarrier(RF24_t *rf24) {
	RF24_powerDown(rf24);
	RF24_write_register_single(rf24, RF_SETUP, RF24_read_register_single(rf24, RF_SETUP) & ~_BV(CONT_WAVE) & ~_BV(PLL_LOCK));
	RF24_ce(rf24, false);
	RF24_flush_tx(rf24);
	if (rf24->_is_p_variant) {
		RF24_write_register(rf24, TX_ADDR, rf24->pipe0_writing_address, rf24->addr_width);
	}
}

void RF24_toggleAllPipes(RF24_t *rf24, bool isEnabled) {
	RF24_write_register_single(rf24, EN_RXADDR, isEnabled ? 0x3F : 0);
}

void RF24_setRadiation(RF24_t *rf24, uint8_t level, rf24_datarate_e speed, bool lnaEnable) {
	uint8_t setup = RF24_data_rate_reg_value(speed);
	setup |= RF24_pa_level_reg_value(level, lnaEnable);
	RF24_write_register_single(rf24, RF_SETUP, setup);
}

void RF24_maskIRQ(RF24_t *rf24, bool tx_ok, bool tx_fail, bool rx_ready) {
	uint8_t config = RF24_read_register_single(rf24, NRF_CONFIG);
	config &= ~(1 << MASK_RX_DR | 1 << MASK_TX_DS | 1 << MASK_MAX_RT);
	config |= (rx_ready ? 0 : (1 << MASK_RX_DR)) | (tx_ok ? 0 : (1 << MASK_TX_DS)) | (tx_fail ? 0 : (1 << MASK_MAX_RT));
	RF24_write_register_single(rf24, NRF_CONFIG, config);
	rf24->config_reg = config;
}

void RF24_reUseTX(RF24_t *rf24) {
	uint8_t cmd = REUSE_TX_PL;
	uint8_t rx;
	RF24_beginTransaction(rf24);
	HAL_SPI_TransmitReceive(rf24->hspi, &cmd, &rx, 1, HAL_MAX_DELAY);
	rf24->status = rx;
	RF24_endTransaction(rf24);
	RF24_ce(rf24, true);
	HAL_Delay(1);
	RF24_ce(rf24, false);
}

bool RF24_txStandBy(RF24_t *rf24, uint32_t timeout, bool startTx) {
	if (startTx) {
		RF24_ce(rf24, true);
	}
	uint32_t start = HAL_GetTick();
	while (!(RF24_get_status(rf24) & (_BV(TX_DS) | _BV(MAX_RT)))) {
		if (HAL_GetTick() - start > timeout) {
			RF24_ce(rf24, false);
			return false;
		}
	}
	RF24_ce(rf24, false);
	RF24_write_register_single(rf24, NRF_STATUS, _BV(TX_DS) | _BV(MAX_RT));
	return true;
}

void RF24_writeAckPayload(RF24_t *rf24, uint8_t pipe, const void *buf, uint8_t len) {
	if (rf24->ack_payloads_enabled) {
		RF24_write_payload(rf24, buf, rf24_min(len, 32), W_ACK_PAYLOAD | (pipe & 0x07));
	}
}

bool RF24_isAckPayloadAvailable(RF24_t *rf24) {
	return RF24_available(rf24);
}

void RF24_whatHappened(RF24_t *rf24, bool *tx_ok, bool *tx_fail, bool *rx_ready) {
	uint8_t status = RF24_get_status(rf24);
	*tx_ok = status & _BV(TX_DS);
	*tx_fail = status & _BV(MAX_RT);
	*rx_ready = status & _BV(RX_DR);
	RF24_write_register_single(rf24, NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
}

void RF24_closeReadingPipe(RF24_t *rf24, uint8_t pipe) {
	RF24_write_register_single(rf24, EN_RXADDR, RF24_read_register_single(rf24, EN_RXADDR) & ~_BV(pipe));
}

uint8_t RF24_getChannel(RF24_t *rf24) {
	return RF24_read_register_single(rf24, RF_CH);
}

uint8_t RF24_getRetries(RF24_t *rf24) {
	return RF24_read_register_single(rf24, SETUP_RETR);
}

uint8_t RF24_getPALevel(RF24_t *rf24) {
	return (RF24_read_register_single(rf24, RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH))) >> 1;
}

rf24_datarate_e RF24_getDataRate(RF24_t *rf24) {
	uint8_t dr = RF24_read_register_single(rf24, RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));
	if (dr == _BV(RF_DR_LOW)) return RF24_250KBPS;
	else if (dr == _BV(RF_DR_HIGH)) return RF24_2MBPS;
	return RF24_1MBPS;
}

rf24_crclength_e RF24_getCRCLength(RF24_t *rf24) {
	rf24_crclength_e result = RF24_CRC_DISABLED;
	uint8_t AA = RF24_read_register_single(rf24, EN_AA);
	rf24->config_reg = RF24_read_register_single(rf24, NRF_CONFIG);
	if (rf24->config_reg & _BV(EN_CRC) || AA) {
		result = (rf24->config_reg & _BV(CRCO)) ? RF24_CRC_16 : RF24_CRC_8;
	}
	return result;
}

void RF24_disableCRC(RF24_t *rf24) {
	rf24->config_reg &= ~_BV(EN_CRC);
	RF24_write_register_single(rf24, NRF_CONFIG, rf24->config_reg);
}

void RF24_setAutoAck_pipe(RF24_t *rf24, uint8_t pipe, bool enable) {
	if (pipe < 6) {
		uint8_t en_aa = RF24_read_register_single(rf24, EN_AA);
		if (enable) {
			en_aa |= _BV(pipe);
		} else {
			en_aa &= ~_BV(pipe);
			if (rf24->ack_payloads_enabled && !pipe) {
				RF24_disableAckPayload(rf24);
			}
		}
		RF24_write_register_single(rf24, EN_AA, en_aa);
	}
}

bool RF24_isChipConnected(RF24_t *rf24) {
	uint8_t setup = RF24_read_register_single(rf24, RF_SETUP);
	RF24_write_register_single(rf24, RF_SETUP, 0xFF);
	uint8_t new_setup = RF24_read_register_single(rf24, RF_SETUP);
	RF24_write_register_single(rf24, RF_SETUP, setup);
	return new_setup == 0xFF;
}
