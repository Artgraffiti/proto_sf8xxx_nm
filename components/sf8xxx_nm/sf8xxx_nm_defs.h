#ifndef SF8XXX_NM_DEFS_H
#define SF8XXX_NM_DEFS_H

#include <stdint.h>
#include <stdbool.h>

#define SF8XXX_NM_UART_PORT_NUM CONFIG_SF8XXX_NM_UART_PORT_NUM
#define SF8XXX_NM_UART_BAUDRATE CONFIG_SF8XXX_NM_UART_BAUDRATE
#define SF8XXX_NM_UART_TXD CONFIG_SF8XXX_NM_UART_TXD
#define SF8XXX_NM_UART_RXD CONFIG_SF8XXX_NM_UART_RXD
#define SF8XXX_NM_RESPONSE_TIMEOUT_MS CONFIG_SF8XXX_NM_RESPONSE_TIMEOUT_MS
#define SF8XXX_NM_DELAY_BTW_CMDS CONFIG_SF8XXX_NM_DELAY_BTW_CMDS
#define SF8XXX_NM_RX_BUF_SIZE CONFIG_SF8XXX_NM_RX_BUF_SIZE
#define SF8XXX_NM_TX_BUF_SIZE CONFIG_SF8XXX_NM_TX_BUF_SIZE

#define SF8XXX_NM_PROTO_E_INTERNAL_BUFFER_OVERFLOW   "E0000\r" // E0000: Digital buffer overflow or command format invalid
#define SF8XXX_NM_PROTO_E_UNKNOWN_COMMAND            "E0001\r" // E0001: Unknown or unrecognized command
#define SF8XXX_NM_PROTO_E_CRC                        "E0002\r" // E0002: CRC failure
#define SF8XXX_NM_PROTO_E_PARAMETER_NOT_EXIST        "K0000 0000\r"  // K0000 0000: Request or set parameter that does not exist. 

namespace sf8xxx_nm {

typedef enum {
    SF8XXX_NM_OK =                              0x00,
    SF8XXX_NM_E_NULL_PTR =                      0x01,
    SF8XXX_NM_E_UART_TX =                       0x02,
    SF8XXX_NM_E_UART_RX =                       0x03,
    SF8XXX_NM_E_PARSE =                         0x04,
    SF8XXX_NM_E_PARAMETER_OUT_OF_RANGE =        0x05,
    SF8XXX_NM_E_INTERLOCK_DENIED =              0x06,
    SF8XXX_NM_E_OVER_CURRENT =                  0x07,
    SF8XXX_NM_E_EXTERNAL_NTC_INTERLOCK =        0x08,
    SF8XXX_NM_E_RESERVED =                      0xff,
} sf8xxx_nm_err_t;

typedef enum {
    SF8XXX_NM_PARAM_FREQUENCY_VALUE =                           0x0100,
    SF8XXX_NM_PARAM_FREQUENCY_MIN =                             0x0101,
    SF8XXX_NM_PARAM_FREQUENCY_MAX =                             0x0102,

    SF8XXX_NM_PARAM_DURATION_VALUE =                            0x0200,
    SF8XXX_NM_PARAM_DURATION_MIN =                              0x0201,
    SF8XXX_NM_PARAM_DURATION_MAX =                              0x0202,

    SF8XXX_NM_PARAM_CURRENT_VALUE =                             0x0300,
    SF8XXX_NM_PARAM_CURRENT_MIN =                               0x0301,
    SF8XXX_NM_PARAM_CURRENT_MAX =                               0x0302,
    SF8XXX_NM_PARAM_CURRENT_MAX_LIMIT =                         0x0306,
    SF8XXX_NM_PARAM_CURRENT_MEASURED_VALUE =                    0x0307,
    SF8XXX_NM_PARAM_CURRENT_PROTECTION_THRESHOLD =              0x0308,

    SF8XXX_NM_PARAM_CURRENT_SET_CALIBRATION_VALUE =             0x030E,

    SF8XXX_NM_PARAM_VOLTAGE_MEASURED_VALUE =                    0x0407,

    SF8XXX_NM_PARAM_DRIVER_STATE =                              0x0700,

    SF8XXX_NM_PARAM_SERIAL_NUMBER =                             0x0701,
    SF8XXX_NM_PARAM_DEVICE_ID =                                 0x0702,

    SF8XXX_NM_PARAM_LOCK_STATUS =                               0x0800,

    SF8XXX_NM_PARAM_SAVE_PARAMETERS =                           0x0900,
    SF8XXX_NM_PARAM_RESET_PARAMETERS =                          0x0901,

    SF8XXX_NM_PARAM_EXTERNAL_NTC_SENSOR_TEMP_LOWER_LIMIT =      0x0A05,
    SF8XXX_NM_PARAM_EXTERNAL_NTC_SENSOR_TEMP_UPPER_LIMIT =      0x0A06,
    SF8XXX_NM_PARAM_EXTERNAL_NTC_SENSOR_TEMP_MEASURED_VALUE =   0x0AE4,
    SF8XXX_NM_PARAM_B25_100_EXTERNAL_NTC =                      0x0B0E,

    SF8XXX_NM_PARAM_TEC_TEMPERATURE_VALUE =                     0x0A10,
    SF8XXX_NM_PARAM_TEC_TEMPERATURE_MAX =                       0x0A11,
    SF8XXX_NM_PARAM_TEC_TEMPERATURE_MIN =                       0x0A12,
    SF8XXX_NM_PARAM_TEC_TEMPERATURE_MAX_LIMIT =                 0x0A13,
    SF8XXX_NM_PARAM_TEC_TEMPERATURE_MIN_LIMIT =                 0x0A14,
    SF8XXX_NM_PARAM_TEC_TEMPERATURE_MEASURED_VALUE =            0x0A15,

    SF8XXX_NM_PARAM_TEC_CURRENT_MEASURED_VALUE =                0x0A16,
    SF8XXX_NM_PARAM_TEC_CURRENT_LIMIT =                         0x0A17,

    SF8XXX_NM_PARAM_TEC_VOLTAGE_MEASURED_VALUE =                0x0A18,

    SF8XXX_NM_PARAM_TEC_STATE =                                 0x0A1A,

    SF8XXX_NM_PARAM_TEC_CURRENT_SET_CALIBRATION_VALUE =         0x0A1E,

    SF8XXX_NM_PARAM_INTERNAL_LD_NTC_SENSOR_B25_100 =            0x0A1F,

    SF8XXX_NM_PARAM_P_COEFFICIENT_VALUE =                       0x0A21,
    SF8XXX_NM_PARAM_I_COEFFICIENT_VALUE =                       0x0A22,
    SF8XXX_NM_PARAM_D_COEFFICIENT_VALUE =                       0x0A23,

    // TODO: SF8XXX_NM_PARAM_EXTENDED_PROTOCOL_INFO =           0x0704, // Used for configuring extended protocol
} sf8xxx_nm_param_t;

typedef enum {
    SF8XXX_NM_STATE_POWERED_ON =               (1 << 0), // 0    bit: 1 – Device is powered on (always = 1)
    SF8XXX_NM_STATE_STARTED =                  (1 << 1), // 1-st bit: 0 – Stopped; 1 – Started
    SF8XXX_NM_STATE_CURRENT_SET_INTERNAL =     (1 << 2), // 2-nd bit: Current set: 0 – External; 1 – Internal
    SF8XXX_NM_STATE_ENABLE_INTERNAL =          (1 << 4), // 4-th bit: Enable: 0 – External; 1 – Internal
    SF8XXX_NM_STATE_EXT_NTC_INTERLOCK_DENIED = (1 << 6), // 6-th bit: External NTC Interlock: 0 – Allowed; 1 – Denied
    SF8XXX_NM_STATE_INTERLOCK_DENIED =         (1 << 7), // 7-th bit: Interlock: 0 – Allowed; 1 – Denied
} sf8xxx_nm_state_r_flags_t;

typedef enum {
    SF8XXX_NM_STATE_WRITE_START =                    0x0008, // Start (Enable)
    SF8XXX_NM_STATE_WRITE_STOP =                     0x0010, // Stop (Disable)
    SF8XXX_NM_STATE_WRITE_INTERNAL_CURRENT_SET =     0x0020, // Internal current set
    SF8XXX_NM_STATE_WRITE_EXTERNAL_CURRENT_SET =     0x0040, // External current set
    SF8XXX_NM_STATE_WRITE_EXTERNAL_ENABLE =          0x0200, // External Enable
    SF8XXX_NM_STATE_WRITE_INTERNAL_ENABLE =          0x0400, // Internal Enable
    SF8XXX_NM_STATE_WRITE_ALLOW_INTERLOCK =          0x1000, // Allow Interlock
    SF8XXX_NM_STATE_WRITE_DENY_INTERLOCK =           0x2000, // Deny Interlock
    SF8XXX_NM_STATE_WRITE_DENY_EXT_NTC_INTERLOCK =   0x4000, // Deny ext. NTC Interlock
    SF8XXX_NM_STATE_WRITE_ALLOW_EXT_NTC_INTERLOCK =  0x8000, // Allow ext. NTC Interlock
} sf8xxx_nm_state_w_flags_t;

typedef enum {
    SF8XXX_NM_LOCK_STATUS_INTERLOCK =           (1 << 1), // 1-st bit: Interlock
    SF8XXX_NM_LOCK_STATUS_LD_OVER_CURRENT =     (1 << 3), // 3-rd bit: LD over current
    SF8XXX_NM_LOCK_STATUS_LD_OVERHEAT =         (1 << 4), // 4-th bit: LD overheat
    SF8XXX_NM_LOCK_STATUS_EXT_NTC_INTERLOCK =   (1 << 5), // 5-th bit: External NTC Interlock
    SF8XXX_NM_LOCK_STATUS_TEC_ERROR =           (1 << 6), // 6-th bit: TEC error
    SF8XXX_NM_LOCK_STATUS_TEC_SELF_HEAT =       (1 << 7), // 7-th bit: TEC self-heat
} sf8xxx_nm_lock_status_r_flags_t;

typedef enum {
    SF8XXX_NM_TEC_STATE_STARTED =               (1 << 1), // 1-st bit: 0 – Stopped; 1 – Started
    SF8XXX_NM_TEC_STATE_TEMP_SET_INTERNAL =     (1 << 2), // 2-nd bit: Temperature set: 0 – External; 1 – Internal
    SF8XXX_NM_TEC_STATE_ENABLE_INTERNAL =       (1 << 4), // 4-th bit: Enable: 0 – External; 1 – Internal
} sf8xxx_nm_tec_state_r_flags_t;

typedef enum {
    SF8XXX_NM_TEC_STATE_WRITE_START =               0x0008, // Start (Enable)
    SF8XXX_NM_TEC_STATE_WRITE_STOP =                0x0010, // Stop (Disable)
    SF8XXX_NM_TEC_STATE_WRITE_INTERNAL_TEMP_SET =   0x0020, // Internal temperature set
    SF8XXX_NM_TEC_STATE_WRITE_EXTERNAL_TEMP_SET =   0x0040, // External temperature set
    SF8XXX_NM_TEC_STATE_WRITE_EXTERNAL_ENABLE =     0x0200, // External Enable
    SF8XXX_NM_TEC_STATE_WRITE_INTERNAL_ENABLE =     0x0400, // Internal Enable
} sf8xxx_nm_tec_state_w_flags_t;

typedef struct {
    unsigned int is_powered_on              : 1; // 0th bit: 1 – Device is powered on (always = 1)
    unsigned int is_started                 : 1; // 1st bit: 0 – Stopped; 1 – Started
    unsigned int current_set_internal       : 1; // 2nd bit: Current set: 0 – External; 1 – Internal
    unsigned int reserved_bit_3             : 1; // 3rd bit: Reserved
    unsigned int enable_internal            : 1; // 4th bit: Enable: 0 – External; 1 – Internal
    unsigned int reserved_bit_5             : 1; // 5th bit: Reserved
    unsigned int ext_ntc_interlock_denied   : 1; // 6th bit: External NTC Interlock: 0 – Allowed; 1 – Denied
    unsigned int interlock_denied           : 1; // 7th bit: Interlock: 0 – Allowed; 1 – Denied
} sf8xxx_nm_state_info_t;

typedef struct {
    unsigned int reserved_bit_0             : 1; // 0th bit: Reserved
    unsigned int is_started                 : 1; // 1st bit: 0 – Stopped; 1 – Started
    unsigned int temp_set_internal          : 1; // 2nd bit: Temperature set: 0 – External; 1 – Internal
    unsigned int reserved_bit_3             : 1; // 3rd bit: Reserved
    unsigned int enable_internal            : 1; // 4th bit: Enable: 0 – External; 1 – Internal
    unsigned int                            : 3; // Remaining 3 bits
} sf8xxx_nm_tec_state_info_t;

typedef struct {
    unsigned int reserved_bit_0             : 1; // 0th bit: Reserved
    unsigned int interlock                  : 1; // 1st bit: Interlock
    unsigned int reserved_bit_2             : 1; // 2nd bit: Reserved
    unsigned int ld_over_current            : 1; // 3rd bit: LD over current
    unsigned int ld_overheat                : 1; // 4th bit: LD overheat
    unsigned int ext_ntc_interlock          : 1; // 5th bit: External NTC Interlock
    unsigned int tec_error                  : 1; // 6th bit: TEC error
    unsigned int tec_self_heat              : 1; // 7th bit: TEC self-heat
} sf8xxx_nm_lock_status_info_t;

} // namespace sf8xxx_nm;

#endif  // SF8XXX_NM_DEFS_H
