#include <cstring>
#include <cmath>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sf8xxx_nm_stub.h"

static const char *TAG = "SF8XXX_NM";

namespace sf8xxx_nm {

namespace {
    // Вспомогательная функция для ограничения значения в диапазоне
    template<typename T>
    T clamp(T value, T min, T max) {
        return (value < min) ? min : (value > max) ? max : value;
    }
}

// ================= Статические состояния для SF8XXX NM =================
struct Sf8xxxNmState {
    bool device_initialized = false;
    bool device_started = false;
    float current_setpoint = 500.0f;      // mA
    float frequency_setpoint = 100.0f;    // Hz
    float duration_setpoint = 10.0f;      // ms
    float tec_target_temp = 25.0f;        // °C
    bool interlock_active = false;
    bool overheat_detected = false;
    bool ext_ntc_interlock_denied = false;
    bool internal_current_set = true;
    bool internal_enable = true;
    
    // Измеренные значения (симуляция)
    float measured_current = 490.0f;
    float measured_voltage = 2.5f;
    float measured_ext_ntc_temp = 25.0f;
    float measured_tec_temp = 24.8f;
    float measured_tec_current = 0.1f;
    float measured_tec_voltage = 3.3f;
    
    // Пределы и настройки (интегрированные из YAML)
    float current_min = 100.0f;
    float current_max = 1500.0f;
    float current_protection_threshold = 1800.0f;
    float frequency_min = 10.0f;
    float frequency_max = 200.0f;
    float duration_min = 1.0f;
    float duration_max = 10.0f;
    float tec_temp_min = 10.0f;
    float tec_temp_max = 40.0f;
    float tec_current_limit = 1.0f;
    
    // Калибровочные значения
    float current_calibration = 100.0f;
    float tec_current_calibration = 100.0f;
    uint16_t int_ld_ntc_b25_100 = 3950;
    float ext_ntc_b25_100 = 3950.0f;
    float ext_ntc_temp_min = -40.0f;
    float ext_ntc_temp_max = 85.0f;
    
    // PID коэффициенты
    uint16_t p_coefficient = 100;
    uint16_t i_coefficient = 50;
    uint16_t d_coefficient = 25;
    
    // Идентификация
    uint16_t serial_number = 0x1234;
    uint16_t device_id = 0x5678;
    
    // Эмуляция ошибок
    bool emulate_buffer_overflow = false;
    bool emulate_unknown_command = false;
    bool emulate_crc_error = false;
    bool emulate_parameter_not_exist = false;
    float error_probability = 0.01f; // 1% вероятность ошибки
    
    // Временные параметры (интегрированные из YAML)
    int response_delay_ms = 10;
    int processing_delay_ms = 5;
    int tec_settling_time_ms = 1000; // 1 секунда для TEC
    
    // Статистика
    struct Statistics {
        uint32_t total_commands = 0;
        uint32_t successful_commands = 0;
        uint32_t failed_commands = 0;
        int64_t total_processing_time_us = 0;
        int64_t max_processing_time_us = 0;
        int64_t min_processing_time_us = INT64_MAX;
        
        void record_command(bool success, int64_t processing_time_us) {
            total_commands++;
            if (success) successful_commands++;
            else failed_commands++;
            
            total_processing_time_us += processing_time_us;
            
            if (processing_time_us > max_processing_time_us)
                max_processing_time_us = processing_time_us;
            
            if (processing_time_us < min_processing_time_us)
                min_processing_time_us = processing_time_us;
        }
        
        void print() const {
            ESP_LOGI(TAG, "\n=== Driver Statistics ===");
            ESP_LOGI(TAG, "Total commands: %lu", total_commands);
            ESP_LOGI(TAG, "Successful: %lu (%.1f%%)", successful_commands, 
                    total_commands ? (100.0 * successful_commands / total_commands) : 0.0);
            ESP_LOGI(TAG, "Failed: %lu (%.1f%%)", failed_commands, 
                    total_commands ? (100.0 * failed_commands / total_commands) : 0.0);
            ESP_LOGI(TAG, "Avg processing time: %lld us", 
                    total_commands ? (total_processing_time_us / total_commands) : 0);
            ESP_LOGI(TAG, "Max processing time: %lld us", max_processing_time_us);
            ESP_LOGI(TAG, "Min processing time: %lld us", 
                    min_processing_time_us == INT64_MAX ? 0 : min_processing_time_us);
            ESP_LOGI(TAG, "========================\n");
        }
    } statistics;
    
    // Состояния для TEC PID регулятора
    float tec_integral_term = 0.0f;
    float tec_prev_error = 0.0f;
    float tec_dt = 0.1f; // Время дискретизации (сек)
} sf8xxx_nm_state;

// Инициализация генератора случайных чисел
static void init_random() {
    static bool initialized = false;
    if (!initialized) {
        // Используем ESP-IDF генератор случайных чисел
        uint32_t seed = esp_timer_get_time() & 0xFFFFFFFF;
        srand(seed);
        initialized = true;
    }
}

// Вспомогательная функция для генерации случайного шума
static float generate_noise(float amplitude) {
    return amplitude * (rand() % 200 - 100) / 100.0f;
}

// Эмуляция задержки с использованием FreeRTOS
static void emulate_delay(int delay_ms) {
    if (delay_ms > 0) {
        ESP_LOGD(TAG, "Emulating delay: %d ms", delay_ms);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

// Проверка эмуляции ошибки
static sf8xxx_nm_err_t check_simulated_error() {
    if (sf8xxx_nm_state.error_probability <= 0.0f) {
        return SF8XXX_NM_OK;
    }
    
    float random_value = static_cast<float>(rand()) / RAND_MAX;
    
    if (random_value < sf8xxx_nm_state.error_probability) {
        // Выбор случайной ошибки
        sf8xxx_nm_err_t errors[] = {
            SF8XXX_NM_E_UART_TX,
            SF8XXX_NM_E_UART_RX,
            SF8XXX_NM_E_PARSE,
            SF8XXX_NM_E_PARAMETER_OUT_OF_RANGE,
            SF8XXX_NM_E_INTERLOCK_DENIED
        };
        
        size_t error_index = rand() % (sizeof(errors) / sizeof(errors[0]));
        sf8xxx_nm_err_t simulated_error = errors[error_index];
        
        ESP_LOGE(TAG, "Simulated error: %d", simulated_error);
        sf8xxx_nm_state.statistics.failed_commands++;
        return simulated_error;
    }
    
    return SF8XXX_NM_OK;
}

// ================= SF8XXX NM Stub Implementations =================
sf8xxx_nm_err_t sf8xxx_nm_init() {
    ESP_LOGI(TAG, "sf8xxx_nm_init called");
    init_random();
    
    // Инициализация состояния драйвера с интегрированными параметрами
    sf8xxx_nm_state = Sf8xxxNmState();
    sf8xxx_nm_state.device_initialized = true;
    
    ESP_LOGI(TAG, "Driver initialized with embedded configuration");
    return SF8XXX_NM_OK;
}

void sf8xxx_nm_deinit() {
    ESP_LOGI(TAG, "sf8xxx_nm_deinit called");
    sf8xxx_nm_state.statistics.print();
    sf8xxx_nm_state.device_initialized = false;
}

int sf8xxx_nm_send_command(const char* command) {
    if (!command) {
        ESP_LOGE(TAG, "sf8xxx_nm_send_command: error - command is nullptr");
        return -1;
    }
    
    sf8xxx_nm_state.statistics.total_commands++;
    ESP_LOGI(TAG, "sf8xxx_nm_send_command [%lu]: %s", 
             sf8xxx_nm_state.statistics.total_commands, command);
    
    // Эмуляция ошибок
    if (sf8xxx_nm_state.emulate_buffer_overflow) {
        ESP_LOGE(TAG, "Error: Internal buffer overflow");
        sf8xxx_nm_state.statistics.failed_commands++;
        return -1;
    }
    
    if (sf8xxx_nm_state.emulate_unknown_command) {
        ESP_LOGE(TAG, "Error: Unknown command");
        sf8xxx_nm_state.statistics.failed_commands++;
        return -1;
    }
    
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    return 0;
}

int sf8xxx_nm_recv_response(char* buffer, int buffer_len) {
    if (!buffer || buffer_len <= 0) {
        ESP_LOGE(TAG, "sf8xxx_nm_recv_response: error - invalid buffer parameters");
        return -1;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_recv_response called");
    
    emulate_delay(sf8xxx_nm_state.response_delay_ms);
    
    strncpy(buffer, "K0000 0000\r", buffer_len);
    buffer[buffer_len - 1] = '\0';
    
    return 0;
}

sf8xxx_nm_err_t sf8xxx_nm_set_param(sf8xxx_nm_param_t param_num, uint16_t value) {
    ESP_LOGI(TAG, "sf8xxx_nm_set_param: param = 0x%02x, value = %u", param_num, value);
    
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_param(sf8xxx_nm_param_t param_num, uint16_t* value) {
    if (!value) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_param: error - value is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_param: param = 0x%02x", param_num);
    
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    *value = 1234;
    return SF8XXX_NM_OK;
}

// ================= Реализация функций параметров =================
sf8xxx_nm_err_t sf8xxx_nm_get_freq(float* freq_val) {
    if (!freq_val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_freq: error - freq_val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_freq called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    *freq_val = sf8xxx_nm_state.frequency_setpoint;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_set_freq(float freq_val) {
    ESP_LOGI(TAG, "sf8xxx_nm_set_freq: freq = %.2f", freq_val);
    
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    // Проверка диапазона
    if (freq_val < sf8xxx_nm_state.frequency_min || freq_val > sf8xxx_nm_state.frequency_max) {
        ESP_LOGE(TAG, "Error: Frequency out of range");
        return SF8XXX_NM_E_PARAMETER_OUT_OF_RANGE;
    }
    
    sf8xxx_nm_state.frequency_setpoint = freq_val;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_freq_min(float* freq_min) {
    if (!freq_min) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_freq_min: error - freq_min is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_freq_min called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    *freq_min = sf8xxx_nm_state.frequency_min;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_freq_max(float* freq_max) {
    if (!freq_max) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_freq_max: error - freq_max is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_freq_max called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    *freq_max = sf8xxx_nm_state.frequency_max;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_dur(float* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_dur: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_dur called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    *val = sf8xxx_nm_state.duration_setpoint;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_set_dur(float val) {
    ESP_LOGI(TAG, "sf8xxx_nm_set_dur: val = %.2f", val);
    
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    // Проверка диапазона
    if (val < sf8xxx_nm_state.duration_min || val > sf8xxx_nm_state.duration_max) {
        ESP_LOGE(TAG, "Error: Duration out of range");
        return SF8XXX_NM_E_PARAMETER_OUT_OF_RANGE;
    }
    
    sf8xxx_nm_state.duration_setpoint = val;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_dur_min(float* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_dur_min: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_dur_min called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    *val = sf8xxx_nm_state.duration_min;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_dur_max(float* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_dur_max: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_dur_max called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    *val = sf8xxx_nm_state.duration_max;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_cur(float* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_cur: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_cur called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    *val = sf8xxx_nm_state.current_setpoint;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_set_cur(float val) {
    ESP_LOGI(TAG, "sf8xxx_nm_set_cur: val = %.2f", val);
    
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    // Проверка диапазона
    if (val < sf8xxx_nm_state.current_min || val > sf8xxx_nm_state.current_max) {
        ESP_LOGE(TAG, "Error: Current out of range");
        return SF8XXX_NM_E_PARAMETER_OUT_OF_RANGE;
    }
    
    // Проверка порога защиты
    if (val > sf8xxx_nm_state.current_protection_threshold) {
        ESP_LOGE(TAG, "Error: Current exceeds protection threshold");
        return SF8XXX_NM_E_OVER_CURRENT;
    }
    
    sf8xxx_nm_state.current_setpoint = val;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_cur_min(float* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_cur_min: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_cur_min called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    *val = sf8xxx_nm_state.current_min;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_cur_max(float* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_cur_max: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_cur_max called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    *val = sf8xxx_nm_state.current_max;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_cur_max_lim(float* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_cur_max_lim: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_cur_max_lim called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    *val = sf8xxx_nm_state.current_protection_threshold;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_set_cur_max(float val) {
    ESP_LOGI(TAG, "sf8xxx_nm_set_cur_max: val = %.2f", val);
    
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    sf8xxx_nm_state.current_max = val;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_meas_cur(float* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_meas_cur: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_meas_cur called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    // Физическая модель измеренного тока
    static float filtered_current = 0.0f;
    const float time_constant = 0.1f;
    
    // Эмуляция задержки установления тока
    filtered_current += (sf8xxx_nm_state.current_setpoint - filtered_current) * time_constant;
    
    // Температурная зависимость
    float temp_coefficient = 0.002f; // 0.2%/°C
    float temp_effect = filtered_current * temp_coefficient * 
                       (sf8xxx_nm_state.measured_ext_ntc_temp - 25.0f);
    
    // Добавление шума и нелинейностей
    float noise = generate_noise(filtered_current * 0.01f);
    float nonlinearity = 0.001f * filtered_current * filtered_current / 1000.0f;
    
    sf8xxx_nm_state.measured_current = filtered_current + temp_effect + noise + nonlinearity;
    *val = sf8xxx_nm_state.measured_current;
    
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_cur_prot_thr(float* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_cur_prot_thr: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_cur_prot_thr called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    *val = sf8xxx_nm_state.current_protection_threshold;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_cal_cur(float* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_cal_cur: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_cal_cur called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    *val = sf8xxx_nm_state.current_calibration;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_set_cal_cur(float val) {
    ESP_LOGI(TAG, "sf8xxx_nm_set_cal_cur: val = %.2f", val);
    
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    sf8xxx_nm_state.current_calibration = val;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_meas_volt(float* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_meas_volt: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_meas_volt called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    // Модель напряжения: V = V0 + I * R + шум
    const float nominal_voltage = 2.5f;
    const float resistance = 0.1f;
    
    // Расчет падения напряжения
    float voltage_drop = sf8xxx_nm_state.measured_current * resistance;
    
    // Добавление нелинейных эффектов
    float nonlinearity = 0.001f * sf8xxx_nm_state.measured_current * 
                        sf8xxx_nm_state.measured_current / 1000.0f;
    
    // Температурная зависимость
    float temp_coefficient = -0.001f;
    float temp_effect = nominal_voltage * temp_coefficient * 
                       (sf8xxx_nm_state.measured_ext_ntc_temp - 25.0f);
    
    // Шум измерения
    float noise = generate_noise(0.02f);
    
    sf8xxx_nm_state.measured_voltage = nominal_voltage + voltage_drop + 
                                      nonlinearity + temp_effect + noise;
    *val = sf8xxx_nm_state.measured_voltage;
    
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_state(sf8xxx_nm_state_info_t* info) {
    if (!info) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_state: error - info is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_state called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    memset(info, 0, sizeof(*info));
    
    info->is_powered_on = 1;  // Всегда включен
    info->is_started = sf8xxx_nm_state.device_started ? 1 : 0;
    info->current_set_internal = sf8xxx_nm_state.internal_current_set ? 1 : 0;
    info->enable_internal = sf8xxx_nm_state.internal_enable ? 1 : 0;
    info->ext_ntc_interlock_denied = sf8xxx_nm_state.ext_ntc_interlock_denied ? 1 : 0;
    info->interlock_denied = sf8xxx_nm_state.interlock_active ? 1 : 0;
    
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_set_state(sf8xxx_nm_state_w_flags_t flag) {
    ESP_LOGI(TAG, "sf8xxx_nm_set_state: flag = 0x%02x", flag);
    
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    // Проверка предусловий перед запуском
    if (flag & SF8XXX_NM_STATE_WRITE_START) {
        ESP_LOGI(TAG, "Attempting to start device");
        
        if (sf8xxx_nm_state.interlock_active) {
            ESP_LOGE(TAG, "Error: Cannot start - interlock active");
            return SF8XXX_NM_E_INTERLOCK_DENIED;
        }
        
        if (sf8xxx_nm_state.ext_ntc_interlock_denied) {
            ESP_LOGE(TAG, "Error: Cannot start - external NTC interlock denied");
            return SF8XXX_NM_E_EXTERNAL_NTC_INTERLOCK;
        }
        
        if (!sf8xxx_nm_state.internal_enable) {
            ESP_LOGE(TAG, "Error: Cannot start - internal enable not set");
            return SF8XXX_NM_E_PARAMETER_OUT_OF_RANGE;
        }
        
        if (sf8xxx_nm_state.current_setpoint > sf8xxx_nm_state.current_protection_threshold) {
            ESP_LOGE(TAG, "Error: Cannot start - current exceeds protection threshold");
            return SF8XXX_NM_E_OVER_CURRENT;
        }
        
        ESP_LOGI(TAG, "Device started successfully");
        sf8xxx_nm_state.device_started = true;
    }
    
    if (flag & SF8XXX_NM_STATE_WRITE_STOP) {
        ESP_LOGI(TAG, "Stopping device");
        sf8xxx_nm_state.device_started = false;
    }
    
    // Обработка источника тока
    if (flag & SF8XXX_NM_STATE_WRITE_INTERNAL_CURRENT_SET) {
        ESP_LOGI(TAG, "Setting internal current source");
        sf8xxx_nm_state.internal_current_set = true;
    }
    if (flag & SF8XXX_NM_STATE_WRITE_EXTERNAL_CURRENT_SET) {
        ESP_LOGI(TAG, "Setting external current source");
        sf8xxx_nm_state.internal_current_set = false;
    }
    
    // Обработка включения
    if (flag & SF8XXX_NM_STATE_WRITE_INTERNAL_ENABLE) {
        ESP_LOGI(TAG, "Setting internal enable");
        sf8xxx_nm_state.internal_enable = true;
    }
    if (flag & SF8XXX_NM_STATE_WRITE_EXTERNAL_ENABLE) {
        ESP_LOGI(TAG, "Setting external enable");
        sf8xxx_nm_state.internal_enable = false;
    }
    
    // Обработка блокировок
    if (flag & SF8XXX_NM_STATE_WRITE_ALLOW_INTERLOCK) {
        ESP_LOGI(TAG, "Allowing interlock");
        sf8xxx_nm_state.interlock_active = false;
    }
    if (flag & SF8XXX_NM_STATE_WRITE_DENY_INTERLOCK) {
        ESP_LOGI(TAG, "Denying interlock");
        sf8xxx_nm_state.interlock_active = true;
    }
    
    // Обработка NTC interlock
    if (flag & SF8XXX_NM_STATE_WRITE_ALLOW_EXT_NTC_INTERLOCK) {
        ESP_LOGI(TAG, "Allowing external NTC interlock");
        sf8xxx_nm_state.ext_ntc_interlock_denied = false;
    }
    if (flag & SF8XXX_NM_STATE_WRITE_DENY_EXT_NTC_INTERLOCK) {
        ESP_LOGI(TAG, "Denying external NTC interlock");
        sf8xxx_nm_state.ext_ntc_interlock_denied = true;
    }
    
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_lock_status(sf8xxx_nm_lock_status_info_t* info) {
    if (!info) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_lock_status: error - info is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_lock_status called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    memset(info, 0, sizeof(*info));
    
    info->interlock = sf8xxx_nm_state.interlock_active ? 1 : 0;
    info->ld_over_current = 0;  // Нет перегрузки по току
    info->ld_overheat = sf8xxx_nm_state.overheat_detected ? 1 : 0;
    info->ext_ntc_interlock = sf8xxx_nm_state.ext_ntc_interlock_denied ? 1 : 0;
    info->tec_error = 0;        // Нет ошибки TEC
    info->tec_self_heat = 0;    // Нет саморазогрева TEC
    
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_serial_number(uint16_t* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_serial_number: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_serial_number called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    *val = sf8xxx_nm_state.serial_number;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_device_id(uint16_t* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_device_id: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_device_id called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    *val = sf8xxx_nm_state.device_id;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_save_params() {
    ESP_LOGI(TAG, "sf8xxx_nm_save_params called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    // В реальной реализации здесь бы сохранялись параметры в NVS
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_reset_params() {
    ESP_LOGI(TAG, "sf8xxx_nm_reset_params called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    // Сброс к значениям по умолчанию
    sf8xxx_nm_state.current_setpoint = 500.0f;
    sf8xxx_nm_state.frequency_setpoint = 100.0f;
    sf8xxx_nm_state.duration_setpoint = 10.0f;
    sf8xxx_nm_state.tec_target_temp = 25.0f;
    sf8xxx_nm_state.device_started = false;
    sf8xxx_nm_state.interlock_active = false;
    sf8xxx_nm_state.overheat_detected = false;
    sf8xxx_nm_state.ext_ntc_interlock_denied = false;
    sf8xxx_nm_state.internal_current_set = true;
    sf8xxx_nm_state.internal_enable = true;
    
    return SF8XXX_NM_OK;
}

// ================= Реализация функций TEC =================
sf8xxx_nm_err_t sf8xxx_nm_get_ext_ntc_temp_lower_lim(float* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_ext_ntc_temp_lower_lim: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_ext_ntc_temp_lower_lim called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    *val = sf8xxx_nm_state.ext_ntc_temp_min;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_ext_ntc_temp_upper_lim(float* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_ext_ntc_temp_upper_lim: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_ext_ntc_temp_upper_lim called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    *val = sf8xxx_nm_state.ext_ntc_temp_max;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_ext_ntc_temp_meas(float* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_ext_ntc_temp_meas: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_ext_ntc_temp_meas called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    // Симуляция измеренной температуры с шумом
    float noise = generate_noise(0.5f);
    sf8xxx_nm_state.measured_ext_ntc_temp = 25.0f + noise;
    *val = sf8xxx_nm_state.measured_ext_ntc_temp;
    
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_ext_ntc_b25_100(float* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_ext_ntc_b25_100: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_ext_ntc_b25_100 called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    *val = sf8xxx_nm_state.ext_ntc_b25_100;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_set_ext_ntc_temp_lower_lim(float val) {
    ESP_LOGI(TAG, "sf8xxx_nm_set_ext_ntc_temp_lower_lim: val = %.2f", val);
    
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    sf8xxx_nm_state.ext_ntc_temp_min = val;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_set_ext_ntc_temp_upper_lim(float val) {
    ESP_LOGI(TAG, "sf8xxx_nm_set_ext_ntc_temp_upper_lim: val = %.2f", val);
    
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    sf8xxx_nm_state.ext_ntc_temp_max = val;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_set_ext_ntc_b25_100(float val) {
    ESP_LOGI(TAG, "sf8xxx_nm_set_ext_ntc_b25_100: val = %.2f", val);
    
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    sf8xxx_nm_state.ext_ntc_b25_100 = val;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_tec_temp(float* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_tec_temp: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_tec_temp called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    *val = sf8xxx_nm_state.tec_target_temp;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_tec_temp_max(float* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_tec_temp_max: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_tec_temp_max called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    *val = sf8xxx_nm_state.tec_temp_max;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_tec_temp_min(float* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_tec_temp_min: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_tec_temp_min called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    *val = sf8xxx_nm_state.tec_temp_min;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_tec_temp_max_lim(float* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_tec_temp_max_lim: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_tec_temp_max_lim called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    *val = sf8xxx_nm_state.tec_temp_max + 10.0f;  // Предел на 10° выше максимума
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_tec_temp_min_lim(float* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_tec_temp_min_lim: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_tec_temp_min_lim called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    *val = sf8xxx_nm_state.tec_temp_min - 10.0f;  // Предел на 10° ниже минимума
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_tec_temp_meas(float* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_tec_temp_meas: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_tec_temp_meas called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    // Модель ПИД-регулятора для TEC
    float error = sf8xxx_nm_state.tec_target_temp - sf8xxx_nm_state.measured_tec_temp;
    
    // ПИД-регулятор
    float p_term = error * (sf8xxx_nm_state.p_coefficient / 100.0f);
    sf8xxx_nm_state.tec_integral_term += error * sf8xxx_nm_state.tec_dt * 
                                         (sf8xxx_nm_state.i_coefficient / 100.0f);
    float d_term = (error - sf8xxx_nm_state.tec_prev_error) / sf8xxx_nm_state.tec_dt * 
                   (sf8xxx_nm_state.d_coefficient / 100.0f);
    
    // Ограничение интегральной составляющей
    sf8xxx_nm_state.tec_integral_term = clamp(sf8xxx_nm_state.tec_integral_term, -10.0f, 10.0f);
    
    // Выход регулятора (ток TEC)
    float control_signal = p_term + sf8xxx_nm_state.tec_integral_term + d_term;
    control_signal = clamp(control_signal, -sf8xxx_nm_state.tec_current_limit, 
                           sf8xxx_nm_state.tec_current_limit);
    
    // Обновление измеренной температуры
    float temp_change = control_signal * 0.5f * sf8xxx_nm_state.tec_dt; // 0.5°C/A * с
    sf8xxx_nm_state.measured_tec_temp += temp_change;
    
    // Добавление шумов и возмущений
    float noise = generate_noise(0.05f);
    float disturbance = 0.01f * sin(sf8xxx_nm_state.statistics.total_commands * 0.1f);
    
    sf8xxx_nm_state.measured_tec_temp += noise + disturbance;
    *val = sf8xxx_nm_state.measured_tec_temp;
    
    sf8xxx_nm_state.tec_prev_error = error;
    
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_set_tec_temp(float val) {
    ESP_LOGI(TAG, "sf8xxx_nm_set_tec_temp: val = %.2f", val);
    
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    // Проверка диапазона
    if (val < sf8xxx_nm_state.tec_temp_min || val > sf8xxx_nm_state.tec_temp_max) {
        ESP_LOGE(TAG, "Error: TEC temperature out of range");
        return SF8XXX_NM_E_PARAMETER_OUT_OF_RANGE;
    }
    
    sf8xxx_nm_state.tec_target_temp = val;
    
    // Эмуляция времени установления температуры TEC
    emulate_delay(sf8xxx_nm_state.tec_settling_time_ms);
    
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_set_tec_temp_max(float val) {
    ESP_LOGI(TAG, "sf8xxx_nm_set_tec_temp_max: val = %.2f", val);
    
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    sf8xxx_nm_state.tec_temp_max = val;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_set_tec_temp_min(float val) {
    ESP_LOGI(TAG, "sf8xxx_nm_set_tec_temp_min: val = %.2f", val);
    
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    sf8xxx_nm_state.tec_temp_min = val;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_tec_meas_cur(float* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_tec_meas_cur: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_tec_meas_cur called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    // Ток TEC зависит от разницы температур
    float temp_diff = sf8xxx_nm_state.tec_target_temp - sf8xxx_nm_state.measured_tec_temp;
    float current = temp_diff * 0.1f;  // 0.1A/°C
    float noise = generate_noise(0.01f);
    sf8xxx_nm_state.measured_tec_current = current + noise;
    *val = sf8xxx_nm_state.measured_tec_current;
    
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_tec_cur_lim(float* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_tec_cur_lim: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_tec_cur_lim called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    *val = sf8xxx_nm_state.tec_current_limit;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_set_tec_cur_lim(float val) {
    ESP_LOGI(TAG, "sf8xxx_nm_set_tec_cur_lim: val = %.2f", val);
    
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    sf8xxx_nm_state.tec_current_limit = val;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_tec_meas_volt(float* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_tec_meas_volt: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_tec_meas_volt called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    // Напряжение TEC с шумом
    float noise = generate_noise(0.1f);
    sf8xxx_nm_state.measured_tec_voltage = 3.3f + noise;
    *val = sf8xxx_nm_state.measured_tec_voltage;
    
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_tec_state(sf8xxx_nm_tec_state_info_t* tec_state) {
    if (!tec_state) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_tec_state: error - tec_state is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_tec_state called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    memset(tec_state, 0, sizeof(*tec_state));
    
    tec_state->is_started = sf8xxx_nm_state.device_started ? 1 : 0;
    tec_state->temp_set_internal = 1;  // Внутреннее управление температурой
    tec_state->enable_internal = 1;     // Внутреннее включение TEC
    
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_set_tec_state(sf8xxx_nm_tec_state_w_flags_t flag) {
    ESP_LOGI(TAG, "sf8xxx_nm_set_tec_state: flag = 0x%02x", flag);
    
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    // Обработка флагов запуска/остановки TEC
    if (flag & SF8XXX_NM_TEC_STATE_WRITE_START) {
        ESP_LOGI(TAG, "Starting TEC");
        // TEC запускается вместе с устройством
    }
    if (flag & SF8XXX_NM_TEC_STATE_WRITE_STOP) {
        ESP_LOGI(TAG, "Stopping TEC");
        // TEC останавливается вместе с устройством
    }
    
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_tec_cal_cur(float* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_tec_cal_cur: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_tec_cal_cur called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    *val = sf8xxx_nm_state.tec_current_calibration;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_set_tec_cal_cur(float val) {
    ESP_LOGI(TAG, "sf8xxx_nm_set_tec_cal_cur: val = %.2f", val);
    
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    sf8xxx_nm_state.tec_current_calibration = val;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_int_ld_ntc_sensor_b25_100(uint16_t* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_int_ld_ntc_sensor_b25_100: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_int_ld_ntc_sensor_b25_100 called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    *val = sf8xxx_nm_state.int_ld_ntc_b25_100;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_set_int_ld_ntc_sensor_b25_100(uint16_t val) {
    ESP_LOGI(TAG, "sf8xxx_nm_set_int_ld_ntc_sensor_b25_100: val = %u", val);
    
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    sf8xxx_nm_state.int_ld_ntc_b25_100 = val;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_p_coef(uint16_t* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_p_coef: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_p_coef called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    *val = sf8xxx_nm_state.p_coefficient;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_set_p_coef(uint16_t val) {
    ESP_LOGI(TAG, "sf8xxx_nm_set_p_coef: val = %u", val);
    
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    sf8xxx_nm_state.p_coefficient = val;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_i_coef(uint16_t* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_i_coef: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_i_coef called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    *val = sf8xxx_nm_state.i_coefficient;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_set_i_coef(uint16_t val) {
    ESP_LOGI(TAG, "sf8xxx_nm_set_i_coef: val = %u", val);
    
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    sf8xxx_nm_state.i_coefficient = val;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_d_coef(uint16_t* val) {
    if (!val) {
        ESP_LOGE(TAG, "sf8xxx_nm_get_d_coef: error - val is nullptr");
        return SF8XXX_NM_E_NULL_PTR;
    }
    
    ESP_LOGI(TAG, "sf8xxx_nm_get_d_coef called");
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    *val = sf8xxx_nm_state.d_coefficient;
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_set_d_coef(uint16_t val) {
    ESP_LOGI(TAG, "sf8xxx_nm_set_d_coef: val = %u", val);
    
    emulate_delay(sf8xxx_nm_state.processing_delay_ms);
    
    sf8xxx_nm_state.d_coefficient = val;
    return SF8XXX_NM_OK;
}

} // namespace sf8xxx_nm