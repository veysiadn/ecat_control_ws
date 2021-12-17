#ifndef GUI_GLOBALS_HPP
#define GUI_GLOBALS_HPP

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
    kUnconfigured = 1,
    kInactive,
    kActive
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
};


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
#define SM_GO_READY_TO_SWITCH_ON        0x06
#define SM_GO_SWITCH_ON            	0x07
#define SM_GO_ENABLE               	0X0F
#define SM_GO_SWITCH_ON_DISABLE    	0x00
#define SM_RUN                    	0x1F
#define SM_EXPEDITE               	0x3F       //like run, but dont finish actual position profile
#define SM_QUICKSTOP              	0x02
#define SM_RELATIVE_                    0X7F
#endif // GUI_GLOBALS_HPP
