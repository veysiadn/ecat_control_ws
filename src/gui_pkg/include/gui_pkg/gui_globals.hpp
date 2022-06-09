#pragma once

#define NUM_OF_SERVO_DRIVES 3
#define ID_CAMERA 0


#define TEST_BIT(NUM,N)    ((NUM &  (1 << N))>>N)  // Check specific bit in the data. 0 or 1.
#define SET_BIT(NUM,N)      (NUM |  (1 << N))  // Set(1) specific bit in the data.
#define RESET_BIT(NUM,N)    (NUM & ~(1 << N))  // Reset(0) specific bit in the data


enum MotorStates{
    kReadyToSwitchOn = 1,
    kSwitchedOn,
    kOperationEnabled,
    kFault,
    kVoltageEnabled,
    kQuickStop,
    kSwitchOnDisabled,
    kWarning,
    kRemote,
    kTargetReached,
    kInternalLimitActivate
};


enum LifeCycleStates{
    kInitialized,
    kUnconfigured,
    kInactive,
    kActive,
    kFinalized,
    kConfiguring = 10,
    kCleaningUp, 
    kShuttingDown,
    kActivating,
    kDeactivating,
    kErrorProcessing
};


struct ReceivedData
{
    int32_t   target_pos ;
    int32_t   target_vel ;
    int16_t   target_tor ;
    int16_t   max_tor ;
    uint16_t  control_word ;
    int32_t   vel_offset ;
    int16_t   tor_offset ;
    uint16_t  error_code ; 
    int32_t  actual_pos ;
    int32_t  actual_vel ;
    int16_t  actual_cur ;
    int16_t  actual_tor ;
    uint16_t status_word ;
    int8_t   op_mode_display ;
    uint8_t  left_limit_switch_val ;
    uint8_t  right_limit_switch_val ;
    int32_t  right_x_axis;
    int32_t  left_x_axis;
    uint8_t  p_emergency_switch_val;
    uint8_t  com_status;
    uint8_t  slave_com_status;
    uint8_t  current_lifecycle_state;
};

typedef enum
{
    kProfilePosition = 1,
    kProfileVelocity = 3,
    kProfileTorque   = 4,
    kHoming = 6,
    kInterpolatedPosition = 7,
    kCSPosition = 8,
    kCSVelocity = 9,
    kCSTorque = 10,
} OpMode ;

struct ControlUIButtonData{
    uint8_t b_init_ecat;
    uint8_t b_reinit_ecat;
    uint8_t b_stop_cyclic_pdo ; 
    uint8_t b_enable_drives;
    uint8_t b_disable_drives;
    uint8_t b_enable_cyclic_pos;
    uint8_t b_enable_cyclic_vel;
    uint8_t b_enable_vel;
    uint8_t b_enable_pos;
    uint8_t b_enter_cyclic_pdo;
    uint8_t b_emergency_mode;
    uint8_t b_send;
    uint32_t l_target_val;
};

#define SM_COMM_RESET              	0x81
#define SM_FULL_RESET              	0x82
#define SM_START                  	0x01
#define SM_GO_READY_TO_SWITCH_ON    0x06
#define SM_GO_SWITCH_ON            	0x07
#define SM_GO_ENABLE               	0X0F
#define SM_GO_SWITCH_ON_DISABLE    	0x00
#define SM_RUN                    	0x1F
#define SM_EXPEDITE               	0x3F       //like run, but dont finish actual position profile
#define SM_QUICKSTOP              	0x02
#define SM_RELATIVE_                0X7F


enum CiA402ErrorType
{
    NO_ERROR = 0,
    GENERIC_ERROR = 0x1000,
    GENERIC_INIT_ERROR = 0x1080,
    GENERIC_INIT_ERROR_1 = 0x1081,
    GENERIC_INIT_ERROR_2 = 0x1082,
    GENERIC_INIT_ERROR_3 = 0x1083,
    GENERIC_INIT_ERROR_4 = 0x1084,
    GENERIC_INIT_ERROR_5 = 0x1085,
    GENERIC_INIT_ERROR_6 = 0x1086,
    GENERIC_INIT_ERROR_7 = 0x1087,
    GENERIC_INIT_ERROR_8 = 0x1088,
    FIRMWARE_INCOMPATIBLITY_ERROR = 0x1090,
    OVER_CURRENT_ERROR = 0x2310,
    POWER_STAGE_PROTECTION_ERROR = 0x2320,
    OVER_VOLTAGE_ERROR = 0x3210,
    UNDER_VOLTAGE_ERROR = 0x3220,
    THERMAL_OVERLOAD_ERROR = 0x4210,
    THERMAL_MOTOR_OVERLOAD_ERRROR = 0x4380,
    LOGIC_SUPPLY_TOO_LOW_ERROR = 0x5113,
    HARDWARE_DEFECT_ERROR = 0x5280,
    HARDWARE_INCOMPATIBLITY_ERROR = 0x5281,
    HARDWARE_ERROR = 0x5480,
    HARDWARE_ERROR_1 = 0x5481,
    HARDWARE_ERROR_2 = 0x5482,
    HARDWARE_ERROR_3 = 0x5483,
    SIGN_OF_LIFE_ERROR = 0x6080,
    EXTENSION_1_WATCHDOG_ERROR = 0x6081,
    INTERNAL_SOFTWARE_ERROR = 0x6180,
    SOFTWARE_PARAMETER_ERROR = 0x6320,
    PERSISTENT_PARAMETER_CORRUPT_ERROR = 0x6380,
    POSITION_SENSOR_ERROR = 0x7320,
    POSITION_SENSOR_BREACH_ERROR = 0x7380,
    POSITION_SENSOR_RESOLUTION_ERROR  = 0x7381,
    POSITION_SENSOR_INDEX_ERROR = 0x7382,
    HALL_SENSOR_ERROR = 0x7388,
    HALL_SENSOR_NOT_FOUND_ERROR =  0x7389,
    HALL_ANGLE_DETECTION_ERROR = 0x738A,
    SSI_SENSOR_ERROR = 0x738C,
    SSI_SENSOR_FRAME_ERROR = 0x738D,
    MISSING_MAIN_SENSOR_ERROR = 0x7390,
    MISSING_COMMUTATION_SENSOR_ERROR = 0x7391, 
    MAIN_SENSOR_DIRECTION_ERROR = 0x7392,
    ETHERCAT_COMMUNCATION_ERROR = 0x8180,
    ETHERCAT_INITIALIZATION_ERROR = 0x8181,
    ETHERCAT_RX_QUEUE_OVERFLOW_ERROR = 0x8182,
    ETHERCAT_COMMUNICATION_ERROR_INTERNAL  = 0x8183,
    ETHERCAT_COMMUNICATION_CYCLE_TIME_ERROR = 0x8184,
    ETHERCAT_PDO_COMMUNICATION_ERROR = 0x8280,
    ETHERCAT_SDO_COMMUNICATION_ERROR = 0x8281,
    FOLLOWING_ERROR = 0x8611,
    NEGATIVE_LIMIT_SWITCH_ERROR = 0x8A80,
    POSITIVE_LIMIT_SWITCH_ERROR = 0x8A81,
    SOFTWARE_POSITION_LIMIT_ERROR = 0x8A82,
    STO_ERROR = 0x8A88,
    SYSTEM_OVERLOADED_ERROR = 0xFF01,
    WATCHDOG_ERROR = 0xFF02,
    SYSTEM_PEAK_OVERLOADED_ERROR = 0XFF0B,
    CONTROLLER_GAIN_ERROR = 0xFF10,
    AUTO_TUNING_INDENTIFICATION_ERROR = 0xFF11,
    AUTO_TUNING_CURRENT_LIMIT_ERROR = 0xFF12,
    AUTO_TUNING_IDENTIFICATION_CURRENT_ERROR = 0xFF13,
    AUTO_TUNING_DATA_SAMPLING_ERROR  = 0xFF14,
    AUTO_TUNING_SAMPLE_MISMATCH_ERROR = 0xFF15,
    AUTO_TUNING_PARAMETER_ERROR = 0xFF16,
    AUTO_TUNING_AMPLITUDE_MISMATCH_ERROR = 0xFF17,
    AUTO_TUNING_TIMEOUT_ERROR = 0xFF19,
    AUTO_TUNING_STAND_STILL_ERROR = 0xFF20,
    AUTO_TUNING_TORQUE_INVALID_ERROR = 0xFF21,
    AUTO_TUNING_MAX_SYSTEM_SPEED_ERROR = 0XFF22,
    AUTO_TUNING_MOTOR_CONNECTION_ERROR = 0xFF23,
    AUTO_TUNING_SENSOR_SIGNAL_ERROR = 0XFF24
};
