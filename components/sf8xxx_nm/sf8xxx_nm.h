#ifndef SF8XXX_NM_H
#define SF8XXX_NM_H

#include "sf8xxx_nm_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

namespace sf8xxx_nm {

sf8xxx_nm_err_t sf8xxx_nm_init(void);
void sf8xxx_nm_deinit(void);

int sf8xxx_nm_send_command(const char *command);
int sf8xxx_nm_recv_response(char *buffer, int buffer_len);

// Helper function to send P-type (set) commands
sf8xxx_nm_err_t sf8xxx_nm_set_param(sf8xxx_nm_param_t param_num, uint16_t value);
// Helper function to send J-type (get) commands and receive K-type responses
sf8xxx_nm_err_t sf8xxx_nm_get_param(sf8xxx_nm_param_t param_num, uint16_t *value);

// Datasheet p.20 table "Available parameters and its description"
// Frequency (0.1 Hz)
sf8xxx_nm_err_t sf8xxx_nm_get_freq(float *freq_val);
sf8xxx_nm_err_t sf8xxx_nm_get_freq_min(float *freq_min);
sf8xxx_nm_err_t sf8xxx_nm_get_freq_max(float *freq_max);
sf8xxx_nm_err_t sf8xxx_nm_set_freq(float freq_val);

// Duration (0.1 ms)
sf8xxx_nm_err_t sf8xxx_nm_get_dur(float *dur_val);
sf8xxx_nm_err_t sf8xxx_nm_get_dur_min(float *dur_min);
sf8xxx_nm_err_t sf8xxx_nm_get_dur_max(float *dur_max);
sf8xxx_nm_err_t sf8xxx_nm_set_dur(float dur_val);

// Current (0.1 mA)
sf8xxx_nm_err_t sf8xxx_nm_get_cur(float *cur_val);
sf8xxx_nm_err_t sf8xxx_nm_get_cur_min(float *cur_min);
sf8xxx_nm_err_t sf8xxx_nm_get_cur_max(float *cur_max);
sf8xxx_nm_err_t sf8xxx_nm_get_cur_max_lim(float *cur_max_lim);
sf8xxx_nm_err_t sf8xxx_nm_get_meas_cur(float *meas_cur_val);
sf8xxx_nm_err_t sf8xxx_nm_get_cur_prot_thr(float *thr_val);
sf8xxx_nm_err_t sf8xxx_nm_set_cur(float cur_val);
sf8xxx_nm_err_t sf8xxx_nm_set_cur_max(float cur_max);

// Current set calibration (0.01%)
sf8xxx_nm_err_t sf8xxx_nm_get_cal_cur(float *cal_val);
sf8xxx_nm_err_t sf8xxx_nm_set_cal_cur(float cal_val);

// Voltage (0.1 V)
sf8xxx_nm_err_t sf8xxx_nm_get_meas_volt(float *meas_volt_val);

// State of the driver
sf8xxx_nm_err_t sf8xxx_nm_get_state(sf8xxx_nm_state_info_t *drv_state);
sf8xxx_nm_err_t sf8xxx_nm_set_state(sf8xxx_nm_state_w_flags_t flag);

// Serial number
sf8xxx_nm_err_t sf8xxx_nm_get_serial_number(uint16_t *serial_number);
sf8xxx_nm_err_t sf8xxx_nm_get_device_id(uint16_t *device_id);

// Lock status (bit mask)
sf8xxx_nm_err_t sf8xxx_nm_get_lock_status(sf8xxx_nm_lock_status_info_t *lock_status);

// Save parameters
sf8xxx_nm_err_t sf8xxx_nm_save_params(void);

// Reset parameters
sf8xxx_nm_err_t sf8xxx_nm_reset_params(void);

// External NTC sensor temperature (0.1°C)
sf8xxx_nm_err_t sf8xxx_nm_get_ext_ntc_temp_lower_lim(float *temp_lim);
sf8xxx_nm_err_t sf8xxx_nm_get_ext_ntc_temp_upper_lim(float *temp_lim);
sf8xxx_nm_err_t sf8xxx_nm_get_ext_ntc_temp_meas(float *meas_val);
sf8xxx_nm_err_t sf8xxx_nm_get_ext_ntc_b25_100(float *b25_100_val);
sf8xxx_nm_err_t sf8xxx_nm_set_ext_ntc_temp_lower_lim(float temp_lim);
sf8xxx_nm_err_t sf8xxx_nm_set_ext_ntc_temp_upper_lim(float temp_lim);
sf8xxx_nm_err_t sf8xxx_nm_set_ext_ntc_b25_100(float b25_100_val);

// TEC temperature (0.01°C)
sf8xxx_nm_err_t sf8xxx_nm_get_tec_temp(float *temp_val);
sf8xxx_nm_err_t sf8xxx_nm_get_tec_temp_max(float *temp_max);
sf8xxx_nm_err_t sf8xxx_nm_get_tec_temp_min(float *temp_min);
sf8xxx_nm_err_t sf8xxx_nm_get_tec_temp_max_lim(float *temp_lim);
sf8xxx_nm_err_t sf8xxx_nm_get_tec_temp_min_lim(float *temp_lim);
sf8xxx_nm_err_t sf8xxx_nm_get_tec_temp_meas(float *meas_val);
sf8xxx_nm_err_t sf8xxx_nm_set_tec_temp(float temp_val);
sf8xxx_nm_err_t sf8xxx_nm_set_tec_temp_max(float temp_max);
sf8xxx_nm_err_t sf8xxx_nm_set_tec_temp_min(float temp_min);

// TEC current (0.1 A)
sf8xxx_nm_err_t sf8xxx_nm_get_tec_meas_cur(float *meas_val);
sf8xxx_nm_err_t sf8xxx_nm_get_tec_cur_lim(float *cur_lim);
sf8xxx_nm_err_t sf8xxx_nm_set_tec_cur_lim(float cur_lim);

// TEC voltage (0.1 V)
sf8xxx_nm_err_t sf8xxx_nm_get_tec_meas_volt(float *meas_val);

// State of the TEC
sf8xxx_nm_err_t sf8xxx_nm_get_tec_state(sf8xxx_nm_tec_state_info_t *tec_state);
sf8xxx_nm_err_t sf8xxx_nm_set_tec_state(sf8xxx_nm_tec_state_w_flags_t flag);

// Current set calibration (0.01%)
sf8xxx_nm_err_t sf8xxx_nm_get_tec_cal_cur(float *cal_val);
sf8xxx_nm_err_t sf8xxx_nm_set_tec_cal_cur(float cal_val);

// Internal LD NTC sensor
sf8xxx_nm_err_t sf8xxx_nm_get_int_ld_ntc_sensor_b25_100(uint16_t *b25_100_val);
sf8xxx_nm_err_t sf8xxx_nm_set_int_ld_ntc_sensor_b25_100(uint16_t b25_100_val);

// P coefficient
sf8xxx_nm_err_t sf8xxx_nm_get_p_coef(uint16_t *p_coeff);
sf8xxx_nm_err_t sf8xxx_nm_set_p_coef(uint16_t p_coeff);

// I coefficient
sf8xxx_nm_err_t sf8xxx_nm_get_i_coef(uint16_t *i_coeff);
sf8xxx_nm_err_t sf8xxx_nm_set_i_coef(uint16_t i_coeff);

// D coefficient
sf8xxx_nm_err_t sf8xxx_nm_get_d_coef(uint16_t *d_coeff);
sf8xxx_nm_err_t sf8xxx_nm_set_d_coef(uint16_t d_coeff);

};

#ifdef __cplusplus
}
#endif

#endif  // SF8XXX_NM_H