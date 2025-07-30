#include "sf8xxx_nm_stub.h"
#include <stdio.h>

namespace sf8xxx_nm {

// --- STUB IMPLEMENTATIONS ---
sf8xxx_nm_err_t sf8xxx_nm_init(void) {
    printf("[STUB] sf8xxx_nm_init() called\n");
    return SF8XXX_NM_OK;
}

void sf8xxx_nm_deinit(void) {
    printf("[STUB] sf8xxx_nm_deinit() called\n");
}

int sf8xxx_nm_send_command(const char *command) {
    printf("[STUB] sf8xxx_nm_send_command: %s\n", command);
    return 0;
}

int sf8xxx_nm_recv_response(char *buffer, int buffer_len) {
    printf("[STUB] sf8xxx_nm_recv_response(buffer, %d) called\n", buffer_len);
    if (buffer && buffer_len > 0) buffer[0] = '\0';
    return 0;
}

sf8xxx_nm_err_t sf8xxx_nm_set_param(sf8xxx_nm_param_t param_num, uint16_t value) {
    printf("[STUB] sf8xxx_nm_set_param(param_num=0x%04X, value=%u)\n", param_num, value);
    return SF8XXX_NM_OK;
}

sf8xxx_nm_err_t sf8xxx_nm_get_param(sf8xxx_nm_param_t param_num, uint16_t *value) {
    printf("[STUB] sf8xxx_nm_get_param(param_num=0x%04X)\n", param_num);
    if (value) *value = 0;
    return SF8XXX_NM_OK;
}

#define STUB_FLOAT_FN(name, param) \
    sf8xxx_nm_err_t name(float *param) { \
        printf("[STUB] %s(%p)\n", #name, (void*)param); \
        if (param) *param = 0.0f; \
        return SF8XXX_NM_OK; \
    }
#define STUB_FLOAT_SET_FN(name, param) \
    sf8xxx_nm_err_t name(float param) { \
        printf("[STUB] %s(%f)\n", #name, param); \
        return SF8XXX_NM_OK; \
    }
#define STUB_UINT16_FN(name, param) \
    sf8xxx_nm_err_t name(uint16_t *param) { \
        printf("[STUB] %s(%p)\n", #name, (void*)param); \
        if (param) *param = 0; \
        return SF8XXX_NM_OK; \
    }
#define STUB_UINT16_SET_FN(name, param) \
    sf8xxx_nm_err_t name(uint16_t param) { \
        printf("[STUB] %s(%u)\n", #name, param); \
        return SF8XXX_NM_OK; \
    }
#define STUB_STRUCT_FN(name, type, param) \
    sf8xxx_nm_err_t name(type *param) { \
        printf("[STUB] %s(%p)\n", #name, (void*)param); \
        return SF8XXX_NM_OK; \
    }
#define STUB_ENUM_FN(name, type, param) \
    sf8xxx_nm_err_t name(type param) { \
        printf("[STUB] %s(%d)\n", #name, param); \
        return SF8XXX_NM_OK; \
    }

STUB_FLOAT_FN(sf8xxx_nm_get_freq, freq_val)
STUB_FLOAT_FN(sf8xxx_nm_get_freq_min, freq_min)
STUB_FLOAT_FN(sf8xxx_nm_get_freq_max, freq_max)
STUB_FLOAT_SET_FN(sf8xxx_nm_set_freq, freq_val)
STUB_FLOAT_FN(sf8xxx_nm_get_dur, dur_val)
STUB_FLOAT_FN(sf8xxx_nm_get_dur_min, dur_min)
STUB_FLOAT_FN(sf8xxx_nm_get_dur_max, dur_max)
STUB_FLOAT_SET_FN(sf8xxx_nm_set_dur, dur_val)
STUB_FLOAT_FN(sf8xxx_nm_get_cur, cur_val)
STUB_FLOAT_FN(sf8xxx_nm_get_cur_min, cur_min)
STUB_FLOAT_FN(sf8xxx_nm_get_cur_max, cur_max)
STUB_FLOAT_FN(sf8xxx_nm_get_cur_max_lim, cur_max_lim)
STUB_FLOAT_FN(sf8xxx_nm_get_meas_cur, meas_cur_val)
STUB_FLOAT_FN(sf8xxx_nm_get_cur_prot_thr, thr_val)
STUB_FLOAT_SET_FN(sf8xxx_nm_set_cur, cur_val)
STUB_FLOAT_SET_FN(sf8xxx_nm_set_cur_max, cur_max)
STUB_FLOAT_FN(sf8xxx_nm_get_cal_cur, cal_val)
STUB_FLOAT_SET_FN(sf8xxx_nm_set_cal_cur, cal_val)
STUB_FLOAT_FN(sf8xxx_nm_get_meas_volt, meas_volt_val)
STUB_STRUCT_FN(sf8xxx_nm_get_state, sf8xxx_nm_state_info_t, drv_state)
STUB_ENUM_FN(sf8xxx_nm_set_state, sf8xxx_nm_state_w_flags_t, flag)
STUB_UINT16_FN(sf8xxx_nm_get_serial_number, serial_number)
STUB_UINT16_FN(sf8xxx_nm_get_device_id, device_id)
STUB_STRUCT_FN(sf8xxx_nm_get_lock_status, sf8xxx_nm_lock_status_info_t, lock_status)
sf8xxx_nm_err_t sf8xxx_nm_save_params(void) {
    printf("[STUB] sf8xxx_nm_save_params()\n");
    return SF8XXX_NM_OK;
}
sf8xxx_nm_err_t sf8xxx_nm_reset_params(void) {
    printf("[STUB] sf8xxx_nm_reset_params()\n");
    return SF8XXX_NM_OK;
}
STUB_FLOAT_FN(sf8xxx_nm_get_ext_ntc_temp_lower_lim, temp_lim)
STUB_FLOAT_FN(sf8xxx_nm_get_ext_ntc_temp_upper_lim, temp_lim)
STUB_FLOAT_FN(sf8xxx_nm_get_ext_ntc_temp_meas, meas_val)
STUB_FLOAT_FN(sf8xxx_nm_get_ext_ntc_b25_100, b25_100_val)
STUB_FLOAT_SET_FN(sf8xxx_nm_set_ext_ntc_temp_lower_lim, temp_lim)
STUB_FLOAT_SET_FN(sf8xxx_nm_set_ext_ntc_temp_upper_lim, temp_lim)
STUB_FLOAT_SET_FN(sf8xxx_nm_set_ext_ntc_b25_100, b25_100_val)
STUB_FLOAT_FN(sf8xxx_nm_get_tec_temp, temp_val)
STUB_FLOAT_FN(sf8xxx_nm_get_tec_temp_max, temp_max)
STUB_FLOAT_FN(sf8xxx_nm_get_tec_temp_min, temp_min)
STUB_FLOAT_FN(sf8xxx_nm_get_tec_temp_max_lim, temp_lim)
STUB_FLOAT_FN(sf8xxx_nm_get_tec_temp_min_lim, temp_lim)
STUB_FLOAT_FN(sf8xxx_nm_get_tec_temp_meas, meas_val)
STUB_FLOAT_SET_FN(sf8xxx_nm_set_tec_temp, temp_val)
STUB_FLOAT_SET_FN(sf8xxx_nm_set_tec_temp_max, temp_max)
STUB_FLOAT_SET_FN(sf8xxx_nm_set_tec_temp_min, temp_min)
STUB_FLOAT_FN(sf8xxx_nm_get_tec_meas_cur, meas_val)
STUB_FLOAT_FN(sf8xxx_nm_get_tec_cur_lim, cur_lim)
STUB_FLOAT_SET_FN(sf8xxx_nm_set_tec_cur_lim, cur_lim)
STUB_FLOAT_FN(sf8xxx_nm_get_tec_meas_volt, meas_val)
STUB_STRUCT_FN(sf8xxx_nm_get_tec_state, sf8xxx_nm_tec_state_info_t, tec_state)
STUB_ENUM_FN(sf8xxx_nm_set_tec_state, sf8xxx_nm_tec_state_w_flags_t, flag)
STUB_FLOAT_FN(sf8xxx_nm_get_tec_cal_cur, cal_val)
STUB_FLOAT_SET_FN(sf8xxx_nm_set_tec_cal_cur, cal_val)
STUB_UINT16_FN(sf8xxx_nm_get_int_ld_ntc_sensor_b25_100, b25_100_val)
STUB_UINT16_SET_FN(sf8xxx_nm_set_int_ld_ntc_sensor_b25_100, b25_100_val)
STUB_UINT16_FN(sf8xxx_nm_get_p_coef, p_coeff)
STUB_UINT16_SET_FN(sf8xxx_nm_set_p_coef, p_coeff)
STUB_UINT16_FN(sf8xxx_nm_get_i_coef, i_coeff)
STUB_UINT16_SET_FN(sf8xxx_nm_set_i_coef, i_coeff)
STUB_UINT16_FN(sf8xxx_nm_get_d_coef, d_coeff)
STUB_UINT16_SET_FN(sf8xxx_nm_set_d_coef, d_coeff)

};