#include <cstdio>
#include <cstring>
#include <memory>
#include <vector>
#include <inttypes.h>  // Добавляем заголовок для макросов PRI
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "sf8xxx_nm_stub.h"
#include "proto/sf8xxx_nm.pb.h"

static const char *TAG = "SF8XXX_CTRL";
constexpr int TCP_PORT = 3333;
constexpr int MAX_CLIENTS = 1;
constexpr int MAX_MESSAGE_SIZE = 4096; // Максимальный размер сообщения

// Wi-Fi конфигурация
#define WIFI_SSID "Homenow"
#define WIFI_PASS "31121974"

// Преобразование кодов ошибок (улучшенная версия из эмулятора)
sf8xxx_nm::ErrorCode convert_error(sf8xxx_nm::sf8xxx_nm_err_t err) {
    switch (err) {
        case sf8xxx_nm::SF8XXX_NM_OK: return sf8xxx_nm::ERROR_UNSPECIFIED;
        case sf8xxx_nm::SF8XXX_NM_E_UART_TX: 
        case sf8xxx_nm::SF8XXX_NM_E_UART_RX: 
        case sf8xxx_nm::SF8XXX_NM_E_PARSE: return sf8xxx_nm::ERROR_SERIAL_PROTOCOL;
        case sf8xxx_nm::SF8XXX_NM_E_PARAMETER_OUT_OF_RANGE: return sf8xxx_nm::ERROR_PARAMETER_OUT_OF_RANGE;
        case sf8xxx_nm::SF8XXX_NM_E_INTERLOCK_DENIED: return sf8xxx_nm::ERROR_INTERLOCK_INPUT;
        case sf8xxx_nm::SF8XXX_NM_E_OVER_CURRENT: return sf8xxx_nm::ERROR_LD_OVERCURRENT;
        case sf8xxx_nm::SF8XXX_NM_E_EXTERNAL_NTC_INTERLOCK: return sf8xxx_nm::ERROR_EXTERNAL_NTC_INTERLOCK;
        case sf8xxx_nm::SF8XXX_NM_E_NULL_PTR: return sf8xxx_nm::ERROR_INTERNAL_PROBLEM;
        default: return sf8xxx_nm::ERROR_UNSPECIFIED;
    }
}

// Обработка ConfigureCommand
void handle_configure(const sf8xxx_nm::ConfigureCommand& config, sf8xxx_nm::DriverResponse& response) {
    sf8xxx_nm::sf8xxx_nm_err_t err = sf8xxx_nm::SF8XXX_NM_OK;
    
    // LDD Settings
    if (config.has_ldd_settings()) {
        const auto& ldd = config.ldd_settings();
        if (ldd.has_frequency_hz()) {
            err = sf8xxx_nm::sf8xxx_nm_set_freq(ldd.frequency_hz());
            if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                ESP_LOGE(TAG, "Failed to set frequency: %d", err);
                response.set_error_code(convert_error(err)); 
                return; 
            }
        }
        if (ldd.has_duration_ms()) {
            err = sf8xxx_nm::sf8xxx_nm_set_dur(ldd.duration_ms());
            if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                ESP_LOGE(TAG, "Failed to set duration: %d", err);
                response.set_error_code(convert_error(err)); 
                return; 
            }
        }
        if (ldd.has_current_ma()) {
            err = sf8xxx_nm::sf8xxx_nm_set_cur(ldd.current_ma());
            if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                ESP_LOGE(TAG, "Failed to set current: %d", err);
                response.set_error_code(convert_error(err)); 
                return; 
            }
        }
        if (ldd.has_current_max_ma()) {
            err = sf8xxx_nm::sf8xxx_nm_set_cur_max(ldd.current_max_ma());
            if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                ESP_LOGE(TAG, "Failed to set current max: %d", err);
                response.set_error_code(convert_error(err)); 
                return; 
            }
        }
        if (ldd.has_current_calibration_percent()) {
            err = sf8xxx_nm::sf8xxx_nm_set_cal_cur(ldd.current_calibration_percent());
            if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                ESP_LOGE(TAG, "Failed to set current calibration: %d", err);
                response.set_error_code(convert_error(err)); 
                return; 
            }
        }
    }
    
    // TEC Settings
    if (config.has_tec_settings()) {
        const auto& tec = config.tec_settings();
        if (tec.has_temperature_celsius()) {
            err = sf8xxx_nm::sf8xxx_nm_set_tec_temp(tec.temperature_celsius());
            if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                ESP_LOGE(TAG, "Failed to set TEC temperature: %d", err);
                response.set_error_code(convert_error(err)); 
                return; 
            }
        }
        if (tec.has_temp_max_celsius()) {
            err = sf8xxx_nm::sf8xxx_nm_set_tec_temp_max(tec.temp_max_celsius());
            if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                ESP_LOGE(TAG, "Failed to set TEC temp max: %d", err);
                response.set_error_code(convert_error(err)); 
                return; 
            }
        }
        if (tec.has_temp_min_celsius()) {
            err = sf8xxx_nm::sf8xxx_nm_set_tec_temp_min(tec.temp_min_celsius());
            if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                ESP_LOGE(TAG, "Failed to set TEC temp min: %d", err);
                response.set_error_code(convert_error(err)); 
                return; 
            }
        }
        if (tec.has_current_limit_a()) {
            err = sf8xxx_nm::sf8xxx_nm_set_tec_cur_lim(tec.current_limit_a());
            if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                ESP_LOGE(TAG, "Failed to set TEC current limit: %d", err);
                response.set_error_code(convert_error(err)); 
                return; 
            }
        }
        if (tec.has_calibration_percent()) {
            err = sf8xxx_nm::sf8xxx_nm_set_tec_cal_cur(tec.calibration_percent());
            if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                ESP_LOGE(TAG, "Failed to set TEC calibration: %d", err);
                response.set_error_code(convert_error(err)); 
                return; 
            }
        }
        if (tec.has_internal_ntc_b_coefficient()) {
            err = sf8xxx_nm::sf8xxx_nm_set_int_ld_ntc_sensor_b25_100(static_cast<uint16_t>(tec.internal_ntc_b_coefficient()));
            if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                ESP_LOGE(TAG, "Failed to set internal NTC B coefficient: %d", err);
                response.set_error_code(convert_error(err)); 
                return; 
            }
        }
    }
    
    // PID Settings
    if (config.has_pid_settings()) {
        const auto& pid = config.pid_settings();
        if (pid.has_p_coefficient()) {
            err = sf8xxx_nm::sf8xxx_nm_set_p_coef(static_cast<uint16_t>(pid.p_coefficient()));
            if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                ESP_LOGE(TAG, "Failed to set P coefficient: %d", err);
                response.set_error_code(convert_error(err)); 
                return; 
            }
        }
        if (pid.has_i_coefficient()) {
            err = sf8xxx_nm::sf8xxx_nm_set_i_coef(static_cast<uint16_t>(pid.i_coefficient()));
            if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                ESP_LOGE(TAG, "Failed to set I coefficient: %d", err);
                response.set_error_code(convert_error(err)); 
                return; 
            }
        }
        if (pid.has_d_coefficient()) {
            err = sf8xxx_nm::sf8xxx_nm_set_d_coef(static_cast<uint16_t>(pid.d_coefficient()));
            if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                ESP_LOGE(TAG, "Failed to set D coefficient: %d", err);
                response.set_error_code(convert_error(err)); 
                return; 
            }
        }
    }
    
    // NTC Interlock Settings
    if (config.has_ntc_interlock_settings()) {
        const auto& ntc = config.ntc_interlock_settings();
        if (ntc.has_lower_limit_celsius()) {
            err = sf8xxx_nm::sf8xxx_nm_set_ext_ntc_temp_lower_lim(ntc.lower_limit_celsius());
            if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                ESP_LOGE(TAG, "Failed to set NTC lower limit: %d", err);
                response.set_error_code(convert_error(err)); 
                return; 
            }
        }
        if (ntc.has_upper_limit_celsius()) {
            err = sf8xxx_nm::sf8xxx_nm_set_ext_ntc_temp_upper_lim(ntc.upper_limit_celsius());
            if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                ESP_LOGE(TAG, "Failed to set NTC upper limit: %d", err);
                response.set_error_code(convert_error(err)); 
                return; 
            }
        }
        if (ntc.has_b_coefficient()) {
            err = sf8xxx_nm::sf8xxx_nm_set_ext_ntc_b25_100(ntc.b_coefficient());
            if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                ESP_LOGE(TAG, "Failed to set NTC B coefficient: %d", err);
                response.set_error_code(convert_error(err)); 
                return; 
            }
        }
    }
    
    // Driver State Settings
    if (config.has_driver_state_settings()) {
        const auto& state = config.driver_state_settings();
        if (state.has_current_set_source()) {
            sf8xxx_nm::sf8xxx_nm_state_w_flags_t flag = sf8xxx_nm::SF8XXX_NM_STATE_WRITE_EXTERNAL_CURRENT_SET;
            if (state.current_set_source() == sf8xxx_nm::DriverStateSettings::INTERNAL) {
                flag = sf8xxx_nm::SF8XXX_NM_STATE_WRITE_INTERNAL_CURRENT_SET;
            }
            err = sf8xxx_nm::sf8xxx_nm_set_state(flag);
            if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                ESP_LOGE(TAG, "Failed to set current set source: %d", err);
                response.set_error_code(convert_error(err)); 
                return; 
            }
        }
        if (state.has_enable_source()) {
            sf8xxx_nm::sf8xxx_nm_state_w_flags_t flag = sf8xxx_nm::SF8XXX_NM_STATE_WRITE_EXTERNAL_ENABLE;
            if (state.enable_source() == sf8xxx_nm::DriverStateSettings::INTERNAL) {
                flag = sf8xxx_nm::SF8XXX_NM_STATE_WRITE_INTERNAL_ENABLE;
            }
            err = sf8xxx_nm::sf8xxx_nm_set_state(flag);
            if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                ESP_LOGE(TAG, "Failed to set enable source: %d", err);
                response.set_error_code(convert_error(err)); 
                return; 
            }
        }
        if (state.has_interlock_policy()) {
            sf8xxx_nm::sf8xxx_nm_state_w_flags_t flag = sf8xxx_nm::SF8XXX_NM_STATE_WRITE_ALLOW_INTERLOCK;
            if (state.interlock_policy() == sf8xxx_nm::DriverStateSettings::DENY) {
                flag = sf8xxx_nm::SF8XXX_NM_STATE_WRITE_DENY_INTERLOCK;
            }
            err = sf8xxx_nm::sf8xxx_nm_set_state(flag);
            if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                ESP_LOGE(TAG, "Failed to set interlock policy: %d", err);
                response.set_error_code(convert_error(err)); 
                return; 
            }
        }
        if (state.has_ntc_interlock_policy()) {
            sf8xxx_nm::sf8xxx_nm_state_w_flags_t flag = sf8xxx_nm::SF8XXX_NM_STATE_WRITE_ALLOW_EXT_NTC_INTERLOCK;
            if (state.ntc_interlock_policy() == sf8xxx_nm::DriverStateSettings::DENY_NTC) {
                flag = sf8xxx_nm::SF8XXX_NM_STATE_WRITE_DENY_EXT_NTC_INTERLOCK;
            }
            err = sf8xxx_nm::sf8xxx_nm_set_state(flag);
            if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                ESP_LOGE(TAG, "Failed to set NTC interlock policy: %d", err);
                response.set_error_code(convert_error(err)); 
                return; 
            }
        }
    }
    
    // TEC State Settings
    if (config.has_tec_state_settings()) {
        const auto& tec_state = config.tec_state_settings();
        if (tec_state.has_temp_set_source()) {
            sf8xxx_nm::sf8xxx_nm_tec_state_w_flags_t flag = sf8xxx_nm::SF8XXX_NM_TEC_STATE_WRITE_EXTERNAL_TEMP_SET;
            if (tec_state.temp_set_source() == sf8xxx_nm::TecStateSettings::INTERNAL) {
                flag = sf8xxx_nm::SF8XXX_NM_TEC_STATE_WRITE_INTERNAL_TEMP_SET;
            }
            err = sf8xxx_nm::sf8xxx_nm_set_tec_state(flag);
            if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                ESP_LOGE(TAG, "Failed to set TEC temp set source: %d", err);
                response.set_error_code(convert_error(err)); 
                return; 
            }
        }
        if (tec_state.has_enable_source()) {
            sf8xxx_nm::sf8xxx_nm_tec_state_w_flags_t flag = sf8xxx_nm::SF8XXX_NM_TEC_STATE_WRITE_EXTERNAL_ENABLE;
            if (tec_state.enable_source() == sf8xxx_nm::TecStateSettings::INTERNAL) {
                flag = sf8xxx_nm::SF8XXX_NM_TEC_STATE_WRITE_INTERNAL_ENABLE;
            }
            err = sf8xxx_nm::sf8xxx_nm_set_tec_state(flag);
            if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                ESP_LOGE(TAG, "Failed to set TEC enable source: %d", err);
                response.set_error_code(convert_error(err)); 
                return; 
            }
        }
    }
}

// Обработка RequestCommand
void handle_request(const sf8xxx_nm::RequestCommand& request, sf8xxx_nm::DriverResponse& response) {
    sf8xxx_nm::sf8xxx_nm_err_t err = sf8xxx_nm::SF8XXX_NM_OK;
    
    for (const auto& req_type : request.requests()) {
        switch (req_type) {
            // LDD Parameters
            case sf8xxx_nm::REQ_FREQUENCY_HZ: {
                float freq;
                err = sf8xxx_nm::sf8xxx_nm_get_freq(&freq);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get frequency: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* ldd_status = response.mutable_ldd_status();
                ldd_status->set_frequency_hz(freq);
                break;
            }
            case sf8xxx_nm::REQ_FREQUENCY_MIN_HZ: {
                float freq_min;
                err = sf8xxx_nm::sf8xxx_nm_get_freq_min(&freq_min);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get frequency min: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* ldd_status = response.mutable_ldd_status();
                ldd_status->set_frequency_min_hz(freq_min);
                break;
            }
            case sf8xxx_nm::REQ_FREQUENCY_MAX_HZ: {
                float freq_max;
                err = sf8xxx_nm::sf8xxx_nm_get_freq_max(&freq_max);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get frequency max: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* ldd_status = response.mutable_ldd_status();
                ldd_status->set_frequency_max_hz(freq_max);
                break;
            }
            case sf8xxx_nm::REQ_DURATION_MS: {
                float duration;
                err = sf8xxx_nm::sf8xxx_nm_get_dur(&duration);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get duration: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* ldd_status = response.mutable_ldd_status();
                ldd_status->set_duration_ms(duration);
                break;
            }
            case sf8xxx_nm::REQ_DURATION_MIN_MS: {
                float duration_min;
                err = sf8xxx_nm::sf8xxx_nm_get_dur_min(&duration_min);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get duration min: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* ldd_status = response.mutable_ldd_status();
                ldd_status->set_duration_min_ms(duration_min);
                break;
            }
            case sf8xxx_nm::REQ_DURATION_MAX_MS: {
                float duration_max;
                err = sf8xxx_nm::sf8xxx_nm_get_dur_max(&duration_max);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get duration max: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* ldd_status = response.mutable_ldd_status();
                ldd_status->set_duration_max_ms(duration_max);
                break;
            }
            case sf8xxx_nm::REQ_CURRENT_MA: {
                float current;
                err = sf8xxx_nm::sf8xxx_nm_get_cur(&current);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get current: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* ldd_status = response.mutable_ldd_status();
                ldd_status->set_current_ma(current);
                break;
            }
            case sf8xxx_nm::REQ_CURRENT_MIN_MA: {
                float current_min;
                err = sf8xxx_nm::sf8xxx_nm_get_cur_min(&current_min);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get current min: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* ldd_status = response.mutable_ldd_status();
                ldd_status->set_current_min_ma(current_min);
                break;
            }
            case sf8xxx_nm::REQ_CURRENT_MAX_MA: {
                float current_max;
                err = sf8xxx_nm::sf8xxx_nm_get_cur_max(&current_max);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get current max: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* ldd_status = response.mutable_ldd_status();
                ldd_status->set_current_max_ma(current_max);
                break;
            }
            case sf8xxx_nm::REQ_CURRENT_MAX_LIMIT_MA: {
                float current_max_limit;
                err = sf8xxx_nm::sf8xxx_nm_get_cur_max_lim(&current_max_limit);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get current max limit: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* ldd_status = response.mutable_ldd_status();
                ldd_status->set_current_max_limit_ma(current_max_limit);
                break;
            }
            case sf8xxx_nm::REQ_MEASURED_CURRENT_MA: {
                float measured_current;
                err = sf8xxx_nm::sf8xxx_nm_get_meas_cur(&measured_current);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get measured current: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* ldd_status = response.mutable_ldd_status();
                ldd_status->set_measured_current_ma(measured_current);
                break;
            }
            case sf8xxx_nm::REQ_CURRENT_PROTECTION_THRESHOLD_MA: {
                float threshold;
                err = sf8xxx_nm::sf8xxx_nm_get_cur_prot_thr(&threshold);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get current protection threshold: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* ldd_status = response.mutable_ldd_status();
                ldd_status->set_current_protection_threshold_ma(threshold);
                break;
            }
            case sf8xxx_nm::REQ_CURRENT_CALIBRATION_PERCENT: {
                float calibration;
                err = sf8xxx_nm::sf8xxx_nm_get_cal_cur(&calibration);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get current calibration: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* ldd_status = response.mutable_ldd_status();
                ldd_status->set_current_calibration_percent(calibration);
                break;
            }
            case sf8xxx_nm::REQ_MEASURED_VOLTAGE_V: {
                float voltage;
                err = sf8xxx_nm::sf8xxx_nm_get_meas_volt(&voltage);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get measured voltage: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* ldd_status = response.mutable_ldd_status();
                ldd_status->set_measured_voltage_v(voltage);
                break;
            }
            
            // TEC Parameters
            case sf8xxx_nm::REQ_TEC_TEMPERATURE_CELSIUS: {
                float temp;
                err = sf8xxx_nm::sf8xxx_nm_get_tec_temp(&temp);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get TEC temperature: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* tec_status = response.mutable_tec_status();
                tec_status->set_temperature_celsius(temp);
                break;
            }
            case sf8xxx_nm::REQ_TEC_TEMPERATURE_MAX_CELSIUS: {
                float temp_max;
                err = sf8xxx_nm::sf8xxx_nm_get_tec_temp_max(&temp_max);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get TEC temp max: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* tec_status = response.mutable_tec_status();
                tec_status->set_temp_max_celsius(temp_max);
                break;
            }
            case sf8xxx_nm::REQ_TEC_TEMPERATURE_MIN_CELSIUS: {
                float temp_min;
                err = sf8xxx_nm::sf8xxx_nm_get_tec_temp_min(&temp_min);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get TEC temp min: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* tec_status = response.mutable_tec_status();
                tec_status->set_temp_min_celsius(temp_min);
                break;
            }
            case sf8xxx_nm::REQ_TEC_TEMPERATURE_MAX_LIMIT_CELSIUS: {
                float temp_max_limit;
                err = sf8xxx_nm::sf8xxx_nm_get_tec_temp_max_lim(&temp_max_limit);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get TEC temp max limit: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* tec_status = response.mutable_tec_status();
                tec_status->set_temp_max_limit_celsius(temp_max_limit);
                break;
            }
            case sf8xxx_nm::REQ_TEC_TEMPERATURE_MIN_LIMIT_CELSIUS: {
                float temp_min_limit;
                err = sf8xxx_nm::sf8xxx_nm_get_tec_temp_min_lim(&temp_min_limit);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get TEC temp min limit: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* tec_status = response.mutable_tec_status();
                tec_status->set_temp_min_limit_celsius(temp_min_limit);
                break;
            }
            case sf8xxx_nm::REQ_TEC_MEASURED_TEMPERATURE_CELSIUS: {
                float measured_temp;
                err = sf8xxx_nm::sf8xxx_nm_get_tec_temp_meas(&measured_temp);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get TEC measured temp: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* tec_status = response.mutable_tec_status();
                tec_status->set_measured_temperature_celsius(measured_temp);
                break;
            }
            case sf8xxx_nm::REQ_TEC_MEASURED_CURRENT_A: {
                float current;
                err = sf8xxx_nm::sf8xxx_nm_get_tec_meas_cur(&current);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get TEC measured current: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* tec_status = response.mutable_tec_status();
                tec_status->set_measured_current_a(current);
                break;
            }
            case sf8xxx_nm::REQ_TEC_CURRENT_LIMIT_A: {
                float current_limit;
                err = sf8xxx_nm::sf8xxx_nm_get_tec_cur_lim(&current_limit);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get TEC current limit: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* tec_status = response.mutable_tec_status();
                tec_status->set_current_limit_a(current_limit);
                break;
            }
            case sf8xxx_nm::REQ_TEC_MEASURED_VOLTAGE_V: {
                float voltage;
                err = sf8xxx_nm::sf8xxx_nm_get_tec_meas_volt(&voltage);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get TEC measured voltage: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* tec_status = response.mutable_tec_status();
                tec_status->set_measured_voltage_v(voltage);
                break;
            }
            case sf8xxx_nm::REQ_TEC_CALIBRATION_PERCENT: {
                float calibration;
                err = sf8xxx_nm::sf8xxx_nm_get_tec_cal_cur(&calibration);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get TEC calibration: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* tec_status = response.mutable_tec_status();
                tec_status->set_calibration_percent(calibration);
                break;
            }
            
            // PID Coefficients
            case sf8xxx_nm::REQ_PID_COEFFICIENTS: {
                uint16_t p, i, d;
                err = sf8xxx_nm::sf8xxx_nm_get_p_coef(&p);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get P coefficient: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_i_coef(&i);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get I coefficient: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_d_coef(&d);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get D coefficient: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* tec_status = response.mutable_tec_status();
                tec_status->set_p_coefficient(p);
                tec_status->set_i_coefficient(i);
                tec_status->set_d_coefficient(d);
                break;
            }
            
            // External NTC
            case sf8xxx_nm::REQ_EXTERNAL_NTC_LOWER_LIMIT_CELSIUS: {
                float limit;
                err = sf8xxx_nm::sf8xxx_nm_get_ext_ntc_temp_lower_lim(&limit);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get NTC lower limit: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* tec_status = response.mutable_tec_status();
                tec_status->set_external_ntc_lower_limit_celsius(limit);
                break;
            }
            case sf8xxx_nm::REQ_EXTERNAL_NTC_UPPER_LIMIT_CELSIUS: {
                float limit;
                err = sf8xxx_nm::sf8xxx_nm_get_ext_ntc_temp_upper_lim(&limit);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get NTC upper limit: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* tec_status = response.mutable_tec_status();
                tec_status->set_external_ntc_upper_limit_celsius(limit);
                break;
            }
            case sf8xxx_nm::REQ_EXTERNAL_NTC_MEASURED_TEMP_CELSIUS: {
                float temp;
                err = sf8xxx_nm::sf8xxx_nm_get_ext_ntc_temp_meas(&temp);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get NTC measured temp: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* tec_status = response.mutable_tec_status();
                tec_status->set_external_ntc_measured_temp_celsius(temp);
                break;
            }
            
            // Status & Info
            case sf8xxx_nm::REQ_DRIVER_STATE: {
                sf8xxx_nm::sf8xxx_nm_state_info_t state_info;
                err = sf8xxx_nm::sf8xxx_nm_get_state(&state_info);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get driver state: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* driver_state = response.mutable_driver_state();
                driver_state->set_is_powered_on(state_info.is_powered_on);
                driver_state->set_is_started(state_info.is_started);
                driver_state->set_current_set_is_internal(state_info.current_set_internal);
                driver_state->set_enable_is_internal(state_info.enable_internal);
                driver_state->set_ext_ntc_interlock_is_denied(state_info.ext_ntc_interlock_denied);
                driver_state->set_interlock_is_denied(state_info.interlock_denied);
                break;
            }
            case sf8xxx_nm::REQ_TEC_STATE: {
                sf8xxx_nm::sf8xxx_nm_tec_state_info_t tec_state_info;
                err = sf8xxx_nm::sf8xxx_nm_get_tec_state(&tec_state_info);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get TEC state: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* tec_state = response.mutable_tec_state();
                tec_state->set_is_started(tec_state_info.is_started);
                tec_state->set_temp_set_is_internal(tec_state_info.temp_set_internal);
                tec_state->set_enable_is_internal(tec_state_info.enable_internal);
                break;
            }
            case sf8xxx_nm::REQ_LOCK_STATUS: {
                sf8xxx_nm::sf8xxx_nm_lock_status_info_t lock_info;
                err = sf8xxx_nm::sf8xxx_nm_get_lock_status(&lock_info);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get lock status: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* lock_status = response.mutable_lock_status();
                lock_status->set_interlock_active(lock_info.interlock);
                lock_status->set_ld_over_current(lock_info.ld_over_current);
                lock_status->set_ld_overheat(lock_info.ld_overheat);
                lock_status->set_ext_ntc_interlock_active(lock_info.ext_ntc_interlock);
                lock_status->set_tec_error(lock_info.tec_error);
                lock_status->set_tec_self_heat(lock_info.tec_self_heat);
                break;
            }
            case sf8xxx_nm::REQ_SERIAL_NUMBER: {
                uint16_t serial;
                err = sf8xxx_nm::sf8xxx_nm_get_serial_number(&serial);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get serial number: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                response.set_serial_number(serial);
                break;
            }
            
            // Bulk requests
            case sf8xxx_nm::REQ_LDD_STATUS_ALL: {
                // Запрос всех LDD параметров
                float freq, freq_min, freq_max, duration, duration_min, duration_max;
                float current, current_min, current_max, current_max_limit, measured_current, threshold, calibration;
                float voltage;
                
                err = sf8xxx_nm::sf8xxx_nm_get_freq(&freq);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get frequency: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_freq_min(&freq_min);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get frequency min: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_freq_max(&freq_max);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get frequency max: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_dur(&duration);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get duration: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_dur_min(&duration_min);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get duration min: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_dur_max(&duration_max);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get duration max: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_cur(&current);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get current: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_cur_min(&current_min);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get current min: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_cur_max(&current_max);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get current max: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_cur_max_lim(&current_max_limit);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get current max limit: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_meas_cur(&measured_current);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get measured current: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_cur_prot_thr(&threshold);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get current protection threshold: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_cal_cur(&calibration);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get current calibration: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_meas_volt(&voltage);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get measured voltage: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* ldd_status = response.mutable_ldd_status();
                ldd_status->set_frequency_hz(freq);
                ldd_status->set_frequency_min_hz(freq_min);
                ldd_status->set_frequency_max_hz(freq_max);
                ldd_status->set_duration_ms(duration);
                ldd_status->set_duration_min_ms(duration_min);
                ldd_status->set_duration_max_ms(duration_max);
                ldd_status->set_current_ma(current);
                ldd_status->set_current_min_ma(current_min);
                ldd_status->set_current_max_ma(current_max);
                ldd_status->set_current_max_limit_ma(current_max_limit);
                ldd_status->set_measured_current_ma(measured_current);
                ldd_status->set_current_protection_threshold_ma(threshold);
                ldd_status->set_current_calibration_percent(calibration);
                ldd_status->set_measured_voltage_v(voltage);
                break;
            }
            case sf8xxx_nm::REQ_TEC_STATUS_ALL: {
                // Запрос всех TEC параметров
                float temp, temp_max, temp_min, temp_max_limit, temp_min_limit, measured_temp;
                float current, current_limit, voltage, calibration;
                uint16_t p, i, d;
                float ntc_lower, ntc_upper, ntc_measured;
                
                err = sf8xxx_nm::sf8xxx_nm_get_tec_temp(&temp);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get TEC temperature: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_tec_temp_max(&temp_max);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get TEC temp max: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_tec_temp_min(&temp_min);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get TEC temp min: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_tec_temp_max_lim(&temp_max_limit);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get TEC temp max limit: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_tec_temp_min_lim(&temp_min_limit);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get TEC temp min limit: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_tec_temp_meas(&measured_temp);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get TEC measured temp: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_tec_meas_cur(&current);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get TEC measured current: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_tec_cur_lim(&current_limit);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get TEC current limit: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_tec_meas_volt(&voltage);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get TEC measured voltage: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_tec_cal_cur(&calibration);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get TEC calibration: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_p_coef(&p);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get P coefficient: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_i_coef(&i);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get I coefficient: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_d_coef(&d);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get D coefficient: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_ext_ntc_temp_lower_lim(&ntc_lower);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get NTC lower limit: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_ext_ntc_temp_upper_lim(&ntc_upper);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get NTC upper limit: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_ext_ntc_temp_meas(&ntc_measured);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get NTC measured temp: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* tec_status = response.mutable_tec_status();
                tec_status->set_temperature_celsius(temp);
                tec_status->set_temp_max_celsius(temp_max);
                tec_status->set_temp_min_celsius(temp_min);
                tec_status->set_temp_max_limit_celsius(temp_max_limit);
                tec_status->set_temp_min_limit_celsius(temp_min_limit);
                tec_status->set_measured_temperature_celsius(measured_temp);
                tec_status->set_measured_current_a(current);
                tec_status->set_current_limit_a(current_limit);
                tec_status->set_measured_voltage_v(voltage);
                tec_status->set_calibration_percent(calibration);
                tec_status->set_p_coefficient(p);
                tec_status->set_i_coefficient(i);
                tec_status->set_d_coefficient(d);
                tec_status->set_external_ntc_lower_limit_celsius(ntc_lower);
                tec_status->set_external_ntc_upper_limit_celsius(ntc_upper);
                tec_status->set_external_ntc_measured_temp_celsius(ntc_measured);
                break;
            }
            case sf8xxx_nm::REQ_EXTERNAL_NTC_ALL: {
                // Запрос всех параметров внешнего NTC
                float lower, upper, measured;
                
                err = sf8xxx_nm::sf8xxx_nm_get_ext_ntc_temp_lower_lim(&lower);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get NTC lower limit: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_ext_ntc_temp_upper_lim(&upper);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get NTC upper limit: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                err = sf8xxx_nm::sf8xxx_nm_get_ext_ntc_temp_meas(&measured);
                if (err != sf8xxx_nm::SF8XXX_NM_OK) { 
                    ESP_LOGE(TAG, "Failed to get NTC measured temp: %d", err);
                    response.set_error_code(convert_error(err)); 
                    return; 
                }
                
                auto* tec_status = response.mutable_tec_status();
                tec_status->set_external_ntc_lower_limit_celsius(lower);
                tec_status->set_external_ntc_upper_limit_celsius(upper);
                tec_status->set_external_ntc_measured_temp_celsius(measured);
                break;
            }
            
            default:
                ESP_LOGW(TAG, "Unhandled request type: %d", req_type);
                break;
        }
    }
}

// Обработка ActionCommand
void handle_action(const sf8xxx_nm::ActionCommand& action, sf8xxx_nm::DriverResponse& response) {
    sf8xxx_nm::sf8xxx_nm_err_t err = sf8xxx_nm::SF8XXX_NM_OK;
    
    switch (action.type()) {
        case sf8xxx_nm::ActionType::START_DRIVER:
            ESP_LOGI(TAG, "Action: START_DRIVER");
            err = sf8xxx_nm::sf8xxx_nm_set_state(sf8xxx_nm::SF8XXX_NM_STATE_WRITE_START);
            break;
        case sf8xxx_nm::ActionType::STOP_DRIVER:
            ESP_LOGI(TAG, "Action: STOP_DRIVER");
            err = sf8xxx_nm::sf8xxx_nm_set_state(sf8xxx_nm::SF8XXX_NM_STATE_WRITE_STOP);
            break;
        case sf8xxx_nm::ActionType::START_TEC:
            ESP_LOGI(TAG, "Action: START_TEC");
            err = sf8xxx_nm::sf8xxx_nm_set_tec_state(sf8xxx_nm::SF8XXX_NM_TEC_STATE_WRITE_START);
            break;
        case sf8xxx_nm::ActionType::STOP_TEC:
            ESP_LOGI(TAG, "Action: STOP_TEC");
            err = sf8xxx_nm::sf8xxx_nm_set_tec_state(sf8xxx_nm::SF8XXX_NM_TEC_STATE_WRITE_STOP);
            break;
        case sf8xxx_nm::ActionType::SAVE_PARAMETERS:
            ESP_LOGI(TAG, "Action: SAVE_PARAMETERS");
            err = sf8xxx_nm::sf8xxx_nm_save_params();
            break;
        case sf8xxx_nm::ActionType::RESET_PARAMETERS:
            ESP_LOGI(TAG, "Action: RESET_PARAMETERS");
            err = sf8xxx_nm::sf8xxx_nm_reset_params();
            break;
        default:
            ESP_LOGW(TAG, "Unknown action type: %d", action.type());
            response.set_error_code(sf8xxx_nm::ERROR_UNKNOWN_COMMAND);
            return;
    }
    
    if (err != sf8xxx_nm::SF8XXX_NM_OK) {
        ESP_LOGE(TAG, "Action failed with error: %d", err);
        response.set_error_code(convert_error(err));
    } else {
        ESP_LOGI(TAG, "Action completed successfully");
    }
}

// Обработка клиента
void handle_client(int sock) {
    uint8_t len_buf[4];
    
    while (true) {
        // Читаем длину сообщения
        int r = recv(sock, len_buf, 4, 0);
        if (r <= 0) {
            ESP_LOGI(TAG, "Client disconnected (read length: %d)", r);
            break;
        }
        
        uint32_t msg_len = ntohl(*reinterpret_cast<uint32_t*>(len_buf));
        if (msg_len > MAX_MESSAGE_SIZE) {
            ESP_LOGE(TAG, "Message too large: %" PRIu32 " bytes (max: %d)", msg_len, MAX_MESSAGE_SIZE);
            break;
        }
        
        // Читаем само сообщение
        std::vector<uint8_t> msg_buf(msg_len);
        r = recv(sock, msg_buf.data(), msg_len, 0);
        if (r != static_cast<int>(msg_len)) {
            ESP_LOGE(TAG, "Incomplete message: %d/%" PRIu32 " bytes", r, msg_len);
            break;
        }
        
        // Десериализация команды
        sf8xxx_nm::DriverCommand command;
        if (!command.ParseFromArray(msg_buf.data(), msg_len)) {
            ESP_LOGE(TAG, "Failed to parse command");
            
            // Отправка ошибки парсинга
            sf8xxx_nm::DriverResponse response;
            response.set_error_code(sf8xxx_nm::ERROR_PARSE);
            
            std::string resp_str = response.SerializeAsString();
            uint32_t resp_len = htonl(resp_str.size());
            send(sock, &resp_len, 4, 0);
            send(sock, resp_str.data(), resp_str.size(), 0);
            continue;
        }
        
        // Обработка команды
        sf8xxx_nm::DriverResponse response;
        response.set_error_code(sf8xxx_nm::ERROR_UNSPECIFIED);
        
        switch (command.command_case()) {
            case sf8xxx_nm::DriverCommand::kConfigure:
                ESP_LOGI(TAG, "Processing Configure command");
                handle_configure(command.configure(), response);
                break;
            case sf8xxx_nm::DriverCommand::kRequest:
                ESP_LOGI(TAG, "Processing Request command");
                handle_request(command.request(), response);
                break;
            case sf8xxx_nm::DriverCommand::kAction:
                ESP_LOGI(TAG, "Processing Action command");
                handle_action(command.action(), response);
                break;
            default:
                ESP_LOGW(TAG, "Unknown command type: %d", command.command_case());
                response.set_error_code(sf8xxx_nm::ERROR_UNKNOWN_COMMAND);
                break;
        }
        
        // Сериализация и отправка ответа
        std::string resp_str = response.SerializeAsString();
        uint32_t resp_len = htonl(resp_str.size());
        
        // Проверка успешности отправки
        if (send(sock, &resp_len, 4, 0) < 0) {
            ESP_LOGE(TAG, "Failed to send response length");
            break;
        }
        
        if (send(sock, resp_str.data(), resp_str.size(), 0) < 0) {
            ESP_LOGE(TAG, "Failed to send response data");
            break;
        }
    }
    
    close(sock);
    ESP_LOGI(TAG, "Client disconnected");
}

// TCP сервер
void tcp_server_task(void* pvParameters) {
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket: %d", errno);
        vTaskDelete(nullptr);
        return;
    }
    
    int opt = 1;
    if (setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        ESP_LOGE(TAG, "Failed to set socket options: %d", errno);
        close(listen_sock);
        vTaskDelete(nullptr);
        return;
    }
    
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(TCP_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;
    
    if (bind(listen_sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind socket: %d", errno);
        close(listen_sock);
        vTaskDelete(nullptr);
        return;
    }
    
    if (listen(listen_sock, MAX_CLIENTS) < 0) {
        ESP_LOGE(TAG, "Failed to listen: %d", errno);
        close(listen_sock);
        vTaskDelete(nullptr);
        return;
    }
    
    ESP_LOGI(TAG, "TCP server started on port %d", TCP_PORT);
    
    while (true) {
        sockaddr_in source_addr{};
        socklen_t addr_len = sizeof(source_addr);
        
        int sock = accept(listen_sock, reinterpret_cast<sockaddr*>(&source_addr), &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Failed to accept connection: %d", errno);
            continue;
        }
        
        ESP_LOGI(TAG, "Client connected from %s:%d", 
                 inet_ntoa(source_addr.sin_addr), ntohs(source_addr.sin_port));
        
        // Обработка клиента в текущем потоке (для простоты)
        handle_client(sock);
    }
    
    close(listen_sock);
    vTaskDelete(nullptr);
}

// Обработчик событий Wi-Fi
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                             int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Wi-Fi started, connecting...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Wi-Fi disconnected, trying to reconnect...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Netmask: " IPSTR, IP2STR(&event->ip_info.netmask));
        ESP_LOGI(TAG, "Gateway: " IPSTR, IP2STR(&event->ip_info.gw));
    }
}

// Инициализация Wi-Fi
static void wifi_init_sta(void) {
    // Инициализация NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    
    wifi_config_t wifi_config = {};
    memcpy(wifi_config.sta.ssid, WIFI_SSID, strlen(WIFI_SSID));
    memcpy(wifi_config.sta.password, WIFI_PASS, strlen(WIFI_PASS));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start Wi-Fi: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "Wi-Fi initialization completed");
}

extern "C" void app_main() {
    ESP_LOGI(TAG, "Starting SF8XXX_NM controller");
    
    // Инициализация драйвера
    sf8xxx_nm::sf8xxx_nm_err_t err = sf8xxx_nm::sf8xxx_nm_init();
    if (err != sf8xxx_nm::SF8XXX_NM_OK) {
        ESP_LOGE(TAG, "Failed to initialize driver: %d", err);
        return;
    }
    ESP_LOGI(TAG, "Driver initialized successfully");
    
    // Инициализация Wi-Fi
    wifi_init_sta();
    
    // Запуск TCP сервера
    xTaskCreate(tcp_server_task, "tcp_server", 12288, nullptr, 5, nullptr);
    
    ESP_LOGI(TAG, "System started successfully");
}