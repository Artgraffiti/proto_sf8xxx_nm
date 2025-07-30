// file: sf8xxx_nm.c

#include "sf8xxx_nm.h"

#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "sf8xxx_nm_defs.h"

const static char *TAG = "SF8XXX_NM DRIVER";

#define FLOAT_TO_UINT16(val, multiplier) ((uint16_t)((val) * (multiplier)))
#define UINT16_TO_FLOAT(val, divider) ((float)(val) / (divider))

namespace sf8xxx_nm {

sf8xxx_nm_err_t sf8xxx_nm_init(void) {
    uart_config_t uart_config = {
        .baud_rate = SF8XXX_NM_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(SF8XXX_NM_UART_PORT_NUM, SF8XXX_NM_RX_BUF_SIZE, SF8XXX_NM_TX_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(SF8XXX_NM_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(SF8XXX_NM_UART_PORT_NUM, SF8XXX_NM_UART_TXD, SF8XXX_NM_UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART initialized on port %d with baud rate %d", SF8XXX_NM_UART_PORT_NUM, SF8XXX_NM_UART_BAUDRATE);
    return SF8XXX_NM_OK;
}

void sf8xxx_nm_deinit(void) {
    uart_driver_delete(SF8XXX_NM_UART_PORT_NUM);
    ESP_LOGI(TAG, "UART deinitialized on port %d", SF8XXX_NM_UART_PORT_NUM);
}

int sf8xxx_nm_send_command(const char *command) {
    size_t len = strlen(command);
    int tx_bytes = uart_write_bytes(SF8XXX_NM_UART_PORT_NUM, command, len);
    ESP_LOGD(TAG, "Sent %d bytes: %s", tx_bytes, command);
    return tx_bytes;
}

int sf8xxx_nm_recv_response(char *buffer, int buffer_len) {
    int idx = 0;
    TickType_t timeout_ticks = pdMS_TO_TICKS(SF8XXX_NM_RESPONSE_TIMEOUT_MS);

    while (idx < buffer_len - 1) {
        uint8_t ch;
        int len = uart_read_bytes(SF8XXX_NM_UART_PORT_NUM, &ch, 1, timeout_ticks);
        if (len > 0) {
            buffer[idx++] = (char)ch;
            if (ch == '\r') {
                break; // End of command
            }
        } else {
            ESP_LOGW(TAG, "Timeout or no data received.");
            break;
        }
    }
    buffer[idx] = '\0';
    ESP_LOGD(TAG, "Received %d bytes: %s", idx, buffer);
    return idx;
}

sf8xxx_nm_err_t sf8xxx_nm_set_param(sf8xxx_nm_param_t param_num, uint16_t value) {
    char command[SF8XXX_NM_TX_BUF_SIZE];
    snprintf(command, SF8XXX_NM_TX_BUF_SIZE, "P%04X %04X\r", param_num, value);
    if (sf8xxx_nm_send_command(command) < 0) {
        return SF8XXX_NM_E_UART_TX;
    }

    esp_rom_delay_us(SF8XXX_NM_DELAY_BTW_CMDS * 1000);

    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_param(sf8xxx_nm_param_t param_num, uint16_t *value) {
    char command[SF8XXX_NM_TX_BUF_SIZE];
    char response[SF8XXX_NM_RX_BUF_SIZE];
    int bytes_read;

    if (value == NULL)
        return SF8XXX_NM_E_NULL_PTR;

    snprintf(command, SF8XXX_NM_TX_BUF_SIZE, "J%04X\r", param_num);
    if (sf8xxx_nm_send_command(command) < 0) {
        return SF8XXX_NM_E_UART_TX;
    }

    bytes_read = sf8xxx_nm_recv_response(response, SF8XXX_NM_RX_BUF_SIZE);
    if (bytes_read <= 0) {
        return SF8XXX_NM_E_UART_RX;
    }

    uint16_t received_param_num;
    unsigned int received_value_u;

    if (sscanf(response, "K%4hX %4X\r", &received_param_num, &received_value_u) != 2) {
        return SF8XXX_NM_E_PARSE;
    }
    if (received_param_num != param_num) {
        return SF8XXX_NM_E_PARSE;
    }
    *value = (uint16_t)received_value_u;

    esp_rom_delay_us(SF8XXX_NM_DELAY_BTW_CMDS * 1000);

    return SF8XXX_NM_OK;
}

// Datasheet p.20 table "Available parameters and its description"
// Frequency (0.1 Hz)
sf8xxx_nm_err_t sf8xxx_nm_get_freq(float *freq_val) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_FREQUENCY_VALUE, &value);
    if (err == SF8XXX_NM_OK) {
        *freq_val = UINT16_TO_FLOAT(value, 10.0f);
    }
    return err;
}
sf8xxx_nm_err_t sf8xxx_nm_get_freq_min(float *freq_min) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_FREQUENCY_MIN, &value);
    if (err == SF8XXX_NM_OK) {
        *freq_min = UINT16_TO_FLOAT(value, 10.0f);
    }
    return err;
}
sf8xxx_nm_err_t sf8xxx_nm_get_freq_max(float *freq_max) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_FREQUENCY_MAX, &value);
    if (err == SF8XXX_NM_OK) {
        *freq_max = UINT16_TO_FLOAT(value, 10.0f);
    }
    return err;
}
sf8xxx_nm_err_t sf8xxx_nm_set_freq(float freq_val) {
    uint16_t value = FLOAT_TO_UINT16(freq_val, 10.0f);
    return sf8xxx_nm_set_param(SF8XXX_NM_PARAM_FREQUENCY_VALUE, value);
}

// Duration (0.1 ms)
sf8xxx_nm_err_t sf8xxx_nm_get_dur(float *dur_val) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_DURATION_VALUE, &value);
    if (err == SF8XXX_NM_OK) {
        *dur_val = UINT16_TO_FLOAT(value, 10.0f);
    }
    return err;
}
sf8xxx_nm_err_t sf8xxx_nm_get_dur_min(float *dur_min) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_DURATION_MIN, &value);
    if (err == SF8XXX_NM_OK) {
        *dur_min = UINT16_TO_FLOAT(value, 10.0f);
    }
    return err;
}
sf8xxx_nm_err_t sf8xxx_nm_get_dur_max(float *dur_max) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_DURATION_MAX, &value);
    if (err == SF8XXX_NM_OK) {
        *dur_max = UINT16_TO_FLOAT(value, 10.0f);
    }
    return err;
}
sf8xxx_nm_err_t sf8xxx_nm_set_dur(float dur_val) {
    uint16_t value = FLOAT_TO_UINT16(dur_val, 10.0f);
    return sf8xxx_nm_set_param(SF8XXX_NM_PARAM_DURATION_VALUE, value);
}

// Current (0.1 mA)
sf8xxx_nm_err_t sf8xxx_nm_get_cur(float *cur_val) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_CURRENT_VALUE, &value);
    if (err == SF8XXX_NM_OK) {
        *cur_val = UINT16_TO_FLOAT(value, 10.0f);
    }
    return err;
}
sf8xxx_nm_err_t sf8xxx_nm_get_cur_min(float *cur_min) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_CURRENT_MIN, &value);
    if (err == SF8XXX_NM_OK) {
        *cur_min = UINT16_TO_FLOAT(value, 10.0f);
    }
    return err;
}
sf8xxx_nm_err_t sf8xxx_nm_get_cur_max(float *cur_max) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_CURRENT_MAX, &value);
    if (err == SF8XXX_NM_OK) {
        *cur_max = UINT16_TO_FLOAT(value, 10.0f);
    }
    return err;
}
sf8xxx_nm_err_t sf8xxx_nm_get_cur_max_lim(float *cur_max_lim) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_CURRENT_MAX_LIMIT, &value);
    if (err == SF8XXX_NM_OK) {
        *cur_max_lim = UINT16_TO_FLOAT(value, 10.0f);
    }
    return err;
}
sf8xxx_nm_err_t sf8xxx_nm_get_meas_cur(float *meas_cur_val) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_CURRENT_MEASURED_VALUE, &value);
    if (err == SF8XXX_NM_OK) {
        *meas_cur_val = UINT16_TO_FLOAT(value, 10.0f);
    }
    return err;
}
sf8xxx_nm_err_t sf8xxx_nm_get_cur_prot_thr(float *thr_val) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_CURRENT_PROTECTION_THRESHOLD, &value);
    if (err == SF8XXX_NM_OK) {
        *thr_val = UINT16_TO_FLOAT(value, 10.0f);
    }
    return err;
}
sf8xxx_nm_err_t sf8xxx_nm_set_cur(float cur_val) {
    uint16_t value = FLOAT_TO_UINT16(cur_val, 10.0f);
    return sf8xxx_nm_set_param(SF8XXX_NM_PARAM_CURRENT_VALUE, value);
}
sf8xxx_nm_err_t sf8xxx_nm_set_cur_max(float cur_max) {
    uint16_t value = FLOAT_TO_UINT16(cur_max, 10.0f);
    return sf8xxx_nm_set_param(SF8XXX_NM_PARAM_CURRENT_MAX, value);
}

// Current set calibration (0.01%)
sf8xxx_nm_err_t sf8xxx_nm_get_cal_cur(float *cal_val) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_CURRENT_SET_CALIBRATION_VALUE, &value);
    if (err == SF8XXX_NM_OK) {
        *cal_val = UINT16_TO_FLOAT(value, 100.0f);
    }
    return err;
}
sf8xxx_nm_err_t sf8xxx_nm_set_cal_cur(float cal_val) {
    uint16_t value = FLOAT_TO_UINT16(cal_val, 100.0f);
    return sf8xxx_nm_set_param(SF8XXX_NM_PARAM_CURRENT_SET_CALIBRATION_VALUE, value);
}

// Voltage (0.1 V)
sf8xxx_nm_err_t sf8xxx_nm_get_meas_volt(float *meas_volt_val) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_VOLTAGE_MEASURED_VALUE, &value);
    if (err == SF8XXX_NM_OK) {
        *meas_volt_val = UINT16_TO_FLOAT(value, 10.0f);
    }
    return err;
}

// State of the driver
sf8xxx_nm_err_t sf8xxx_nm_get_state(sf8xxx_nm_state_info_t *drv_state) {
    uint16_t raw_flags;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_DRIVER_STATE, &raw_flags);
    if (err != SF8XXX_NM_OK) {
        return err;
    }

    drv_state->is_powered_on = (raw_flags & SF8XXX_NM_STATE_POWERED_ON) != 0;
    drv_state->is_started = (raw_flags & SF8XXX_NM_STATE_STARTED) != 0;
    drv_state->current_set_internal = (raw_flags & SF8XXX_NM_STATE_CURRENT_SET_INTERNAL) != 0;
    drv_state->enable_internal = (raw_flags & SF8XXX_NM_STATE_ENABLE_INTERNAL) != 0;
    drv_state->ext_ntc_interlock_denied = (raw_flags & SF8XXX_NM_STATE_EXT_NTC_INTERLOCK_DENIED) != 0;
    drv_state->interlock_denied = (raw_flags & SF8XXX_NM_STATE_INTERLOCK_DENIED) != 0;

    return SF8XXX_NM_OK;
}
sf8xxx_nm_err_t sf8xxx_nm_set_state(sf8xxx_nm_state_w_flags_t flag) {
    return sf8xxx_nm_set_param(SF8XXX_NM_PARAM_DRIVER_STATE, (uint16_t)flag);
}

// Serial number
sf8xxx_nm_err_t sf8xxx_nm_get_serial_number(uint16_t *serial_number) {
    return sf8xxx_nm_get_param(SF8XXX_NM_PARAM_SERIAL_NUMBER, serial_number);
}
sf8xxx_nm_err_t sf8xxx_nm_get_device_id(uint16_t *device_id) {
    return sf8xxx_nm_get_param(SF8XXX_NM_PARAM_DEVICE_ID, device_id);
}

// Lock status (bit mask)
sf8xxx_nm_err_t sf8xxx_nm_get_lock_status(sf8xxx_nm_lock_status_info_t *lock_status) {
    uint16_t raw_flags;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_LOCK_STATUS, &raw_flags);
    if (err != SF8XXX_NM_OK) {
        return err;
    }

    lock_status->interlock = (raw_flags & SF8XXX_NM_LOCK_STATUS_INTERLOCK) != 0;
    lock_status->ld_over_current = (raw_flags & SF8XXX_NM_LOCK_STATUS_LD_OVER_CURRENT) != 0;
    lock_status->ld_overheat = (raw_flags & SF8XXX_NM_LOCK_STATUS_LD_OVERHEAT) != 0;
    lock_status->ext_ntc_interlock = (raw_flags & SF8XXX_NM_LOCK_STATUS_EXT_NTC_INTERLOCK) != 0;
    lock_status->tec_error = (raw_flags & SF8XXX_NM_LOCK_STATUS_TEC_ERROR) != 0;
    lock_status->tec_self_heat = (raw_flags & SF8XXX_NM_LOCK_STATUS_TEC_SELF_HEAT) != 0;

    return SF8XXX_NM_OK;
}

// Save parameters
sf8xxx_nm_err_t sf8xxx_nm_save_params(void) {
    return sf8xxx_nm_set_param(SF8XXX_NM_PARAM_SAVE_PARAMETERS, 1);
}

// Reset parameters
sf8xxx_nm_err_t sf8xxx_nm_reset_params(void) {
    return sf8xxx_nm_set_param(SF8XXX_NM_PARAM_RESET_PARAMETERS, 1);
}

// External NTC sensor temperature (0.1°C)
sf8xxx_nm_err_t sf8xxx_nm_get_ext_ntc_temp_lower_lim(float *temp_lim) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_EXTERNAL_NTC_SENSOR_TEMP_LOWER_LIMIT, &value);
    if (err == SF8XXX_NM_OK) {
        *temp_lim = UINT16_TO_FLOAT(value, 10.0f);
    }
    return err;
}
sf8xxx_nm_err_t sf8xxx_nm_get_ext_ntc_temp_upper_lim(float *temp_lim) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_EXTERNAL_NTC_SENSOR_TEMP_UPPER_LIMIT, &value);
    if (err == SF8XXX_NM_OK) {
        *temp_lim = UINT16_TO_FLOAT(value, 10.0f);
    }
    return err;
}
sf8xxx_nm_err_t sf8xxx_nm_get_ext_ntc_temp_meas(float *meas_val) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_EXTERNAL_NTC_SENSOR_TEMP_MEASURED_VALUE, &value);
    if (err == SF8XXX_NM_OK) {
        *meas_val = UINT16_TO_FLOAT(value, 10.0f);
    }
    return err;
}
sf8xxx_nm_err_t sf8xxx_nm_get_ext_ntc_b25_100(float *b25_100_val) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_B25_100_EXTERNAL_NTC, &value);
    if (err == SF8XXX_NM_OK) {
        *b25_100_val = UINT16_TO_FLOAT(value, 1.0f);
    }
    return err;
}
sf8xxx_nm_err_t sf8xxx_nm_set_ext_ntc_temp_lower_lim(float temp_lim) {
    uint16_t value = FLOAT_TO_UINT16(temp_lim, 10.0f);
    return sf8xxx_nm_set_param(SF8XXX_NM_PARAM_EXTERNAL_NTC_SENSOR_TEMP_LOWER_LIMIT, value);
}
sf8xxx_nm_err_t sf8xxx_nm_set_ext_ntc_temp_upper_lim(float temp_lim) {
    uint16_t value = FLOAT_TO_UINT16(temp_lim, 10.0f);
    return sf8xxx_nm_set_param(SF8XXX_NM_PARAM_EXTERNAL_NTC_SENSOR_TEMP_UPPER_LIMIT, value);
}
sf8xxx_nm_err_t sf8xxx_nm_set_ext_ntc_b25_100(float b25_100_val) {
    uint16_t value = FLOAT_TO_UINT16(b25_100_val, 1.0f);
    return sf8xxx_nm_set_param(SF8XXX_NM_PARAM_B25_100_EXTERNAL_NTC, value);
}

// TEC temperature (0.01°C)
sf8xxx_nm_err_t sf8xxx_nm_get_tec_temp(float *temp_val) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_TEC_TEMPERATURE_VALUE, &value);
    if (err == SF8XXX_NM_OK) {
        *temp_val = UINT16_TO_FLOAT(value, 100.0f);
    }
    return err;
}
sf8xxx_nm_err_t sf8xxx_nm_get_tec_temp_max(float *temp_max) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_TEC_TEMPERATURE_MAX, &value);
    if (err == SF8XXX_NM_OK) {
        *temp_max = UINT16_TO_FLOAT(value, 100.0f);
    }
    return err;
}
sf8xxx_nm_err_t sf8xxx_nm_get_tec_temp_min(float *temp_min) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_TEC_TEMPERATURE_MIN, &value);
    if (err == SF8XXX_NM_OK) {
        *temp_min = UINT16_TO_FLOAT(value, 100.0f);
    }
    return err;
}
sf8xxx_nm_err_t sf8xxx_nm_get_tec_temp_max_lim(float *temp_lim) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_TEC_TEMPERATURE_MAX_LIMIT, &value);
    if (err == SF8XXX_NM_OK) {
        *temp_lim = UINT16_TO_FLOAT(value, 100.0f);
    }
    return err;
}
sf8xxx_nm_err_t sf8xxx_nm_get_tec_temp_min_lim(float *temp_lim) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_TEC_TEMPERATURE_MIN_LIMIT, &value);
    if (err == SF8XXX_NM_OK) {
        *temp_lim = UINT16_TO_FLOAT(value, 100.0f);
    }
    return err;
}
sf8xxx_nm_err_t sf8xxx_nm_get_tec_temp_meas(float *meas_val) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_TEC_TEMPERATURE_MEASURED_VALUE, &value);
    if (err == SF8XXX_NM_OK) {
        *meas_val = UINT16_TO_FLOAT(value, 100.0f);
    }
    return err;
}
sf8xxx_nm_err_t sf8xxx_nm_set_tec_temp(float temp_val) {
    uint16_t value = FLOAT_TO_UINT16(temp_val, 100.0f);
    return sf8xxx_nm_set_param(SF8XXX_NM_PARAM_TEC_TEMPERATURE_VALUE, value);
}
sf8xxx_nm_err_t sf8xxx_nm_set_tec_temp_max(float temp_max) {
    uint16_t value = FLOAT_TO_UINT16(temp_max, 100.0f);
    return sf8xxx_nm_set_param(SF8XXX_NM_PARAM_TEC_TEMPERATURE_MAX, value);
}
sf8xxx_nm_err_t sf8xxx_nm_set_tec_temp_min(float temp_min) {
    uint16_t value = FLOAT_TO_UINT16(temp_min, 100.0f);
    return sf8xxx_nm_set_param(SF8XXX_NM_PARAM_TEC_TEMPERATURE_MIN, value);
}

// TEC current (0.1 A)
sf8xxx_nm_err_t sf8xxx_nm_get_tec_meas_cur(float *meas_val) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_TEC_CURRENT_MEASURED_VALUE, &value);
    if (err == SF8XXX_NM_OK) {
        *meas_val = UINT16_TO_FLOAT(value, 10.0f);
    }
    return err;
}
sf8xxx_nm_err_t sf8xxx_nm_get_tec_cur_lim(float *cur_lim) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_TEC_CURRENT_LIMIT, &value);
    if (err == SF8XXX_NM_OK) {
        *cur_lim = UINT16_TO_FLOAT(value, 10.0f);
    }
    return err;
}
sf8xxx_nm_err_t sf8xxx_nm_set_tec_cur_lim(float cur_lim) {
    uint16_t value = FLOAT_TO_UINT16(cur_lim, 10.0f);
    return sf8xxx_nm_set_param(SF8XXX_NM_PARAM_TEC_CURRENT_LIMIT, value);
}

// TEC voltage (0.1 V)
sf8xxx_nm_err_t sf8xxx_nm_get_tec_meas_volt(float *meas_val) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_TEC_VOLTAGE_MEASURED_VALUE, &value);
    if (err == SF8XXX_NM_OK) {
        *meas_val = UINT16_TO_FLOAT(value, 10.0f);
    }
    return err;
}

// State of the TEC
sf8xxx_nm_err_t sf8xxx_nm_get_tec_state(sf8xxx_nm_tec_state_info_t *tec_state) {
    uint16_t raw_flags;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_TEC_STATE, &raw_flags);
    if (err != SF8XXX_NM_OK) {
        return err;
    }

    tec_state->is_started = (raw_flags & SF8XXX_NM_TEC_STATE_STARTED) != 0;
    tec_state->temp_set_internal = (raw_flags & SF8XXX_NM_TEC_STATE_TEMP_SET_INTERNAL) != 0;
    tec_state->enable_internal = (raw_flags & SF8XXX_NM_TEC_STATE_ENABLE_INTERNAL) != 0;

    return SF8XXX_NM_OK;
}
sf8xxx_nm_err_t sf8xxx_nm_set_tec_state(sf8xxx_nm_tec_state_w_flags_t flag) {
    return sf8xxx_nm_set_param(SF8XXX_NM_PARAM_TEC_STATE, (uint16_t)flag);
}

// Current set calibration (0.01%)
sf8xxx_nm_err_t sf8xxx_nm_get_tec_cal_cur(float *cal_val) {
    uint16_t value;
    sf8xxx_nm_err_t err = sf8xxx_nm_get_param(SF8XXX_NM_PARAM_TEC_CURRENT_SET_CALIBRATION_VALUE, &value);
    if (err == SF8XXX_NM_OK) {
        *cal_val = UINT16_TO_FLOAT(value, 100.0f);
    }
    return err;
}
sf8xxx_nm_err_t sf8xxx_nm_set_tec_cal_cur(float cal_val) {
    uint16_t value = FLOAT_TO_UINT16(cal_val, 100.0f);
    return sf8xxx_nm_set_param(SF8XXX_NM_PARAM_TEC_CURRENT_SET_CALIBRATION_VALUE, value);
}

// Internal LD NTC sensor
sf8xxx_nm_err_t sf8xxx_nm_get_int_ld_ntc_sensor_b25_100(uint16_t *b25_100_val) {
    return sf8xxx_nm_get_param(SF8XXX_NM_PARAM_INTERNAL_LD_NTC_SENSOR_B25_100, b25_100_val);
}
sf8xxx_nm_err_t sf8xxx_nm_set_int_ld_ntc_sensor_b25_100(uint16_t b25_100_val) {
    return sf8xxx_nm_set_param(SF8XXX_NM_PARAM_INTERNAL_LD_NTC_SENSOR_B25_100, b25_100_val);
}

// P coefficient
sf8xxx_nm_err_t sf8xxx_nm_get_p_coef(uint16_t *p_coeff) {
    return sf8xxx_nm_get_param(SF8XXX_NM_PARAM_P_COEFFICIENT_VALUE, p_coeff);
}
sf8xxx_nm_err_t sf8xxx_nm_set_p_coef(uint16_t p_coeff) {
    return sf8xxx_nm_set_param(SF8XXX_NM_PARAM_P_COEFFICIENT_VALUE, p_coeff);
}

// I coefficient
sf8xxx_nm_err_t sf8xxx_nm_get_i_coef(uint16_t *i_coeff) {
    return sf8xxx_nm_get_param(SF8XXX_NM_PARAM_I_COEFFICIENT_VALUE, i_coeff);
}
sf8xxx_nm_err_t sf8xxx_nm_set_i_coef(uint16_t i_coeff) {
    return sf8xxx_nm_set_param(SF8XXX_NM_PARAM_I_COEFFICIENT_VALUE, i_coeff);
}

// D coefficient
sf8xxx_nm_err_t sf8xxx_nm_get_d_coef(uint16_t *d_coeff) {
    return sf8xxx_nm_get_param(SF8XXX_NM_PARAM_D_COEFFICIENT_VALUE, d_coeff);
}
sf8xxx_nm_err_t sf8xxx_nm_set_d_coef(uint16_t d_coeff) {
    return sf8xxx_nm_set_param(SF8XXX_NM_PARAM_D_COEFFICIENT_VALUE, d_coeff);
}

};