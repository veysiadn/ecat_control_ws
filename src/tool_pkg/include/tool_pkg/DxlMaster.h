// --------------------------------------------------------------------- //
// CKim - Oct. 14, 2021 : Class encapsulating Dynamixel
// For MX series with protocol version 1.0
// Will need to upgrade for X series with protocol version 2.0
// --------------------------------------------------------------------- //
// CKim - These header files and definitions are for serial communications
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>	// 'file control'
#include <termios.h>
#include <stdio.h>
#include <signal.h>		// CKim - For catching 'ctrl-c' input
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <string.h>

#include "dynamixel_sdk/dynamixel_sdk.h"      // CKim - Uses Dynamixel SDK library that is installed with the ROS

#define MX28 1

// ---------------- Dynamixel Defines --------------------------------- //
// CKim - Control table data address of MX28/ RX24F. May differ between Dynamixel model
// https://emanual.robotis.com/docs/en/dxl/mx/mx-28/#control-table-data-address
// https://emanual.robotis.com/docs/en/dxl/mx/mx-28/#cw-angle-limit

#if MX28                                        
                                                // Description (Byte Size, Access, Init Val)
#define ADDR_MX_MODEL_NUMBER            0       // Model Number (2, R,29)
#define ADDR_MX_FIRMWARE_VER            2       // Firmware Version (1, R, -)
#define ADDR_MX_ID                      3       // Dynamixel ID (1, RW, 1)
#define ADDR_MX_BAUDRATE                4       // Baud Rate (1, RW, 24)
#define ADDR_MX_RETURN_DELAY            5       // Response delay time (1, RW, 250)
#define ADDR_MX_CW_ANGLE_LIM            6       // Clockwise Angle Limit (2, RW, 0). Set this and CCW angle limit to 4095 to enable multiturn
#define ADDR_MX_CCW_ANGLE_LIM           8       // Counter Clockwise Angle Limit (2, RW, 0). Set this and CW angle limit to 4095 to enable multiturn
#define ADDR_MX_TEMP_LIM                11      // Internal temperature limit (1, RW, 80), Don't modify
#define ADDR_MX_MIN_VOLT_LIM            12      // Minimum input voltage limit (1, RW, 60), Don't modify 
#define ADDR_MX_MAX_VOLT_LIM            13      // Maximum input voltage limit (1, RW, 160 ), Don't modify 
#define ADDR_MX_MAX_TORQUE              14      // Maximum torque (2, RW, 1023)
#define ADDR_MX_STATUS_RETURN           16      // Select type of status return (1, RW, 2)
#define ADDR_MX_ALARM_LED               17      // LED for alarm (1, RW, 36)
#define ADDR_MX_SHUTDOWN                18      // Shutdown error info (1, RW, 36)

// // CKim - Only for MX28
// #define ADDR_MX_MULTITURN_OFFSET        20      // Offset for position value (2, RW, 0)
// #define ADDR_MX_RES_DIV                 22      // Position resolution divider (1, RW, 1)

#define ADDR_MX_TORQUE_ENABLE           24      // Torque enable (1, RW, 0)
#define ADDR_MX_GOAL_POSITION           30      // Desired Position (2, RW, -)
#define ADDR_MX_MOVING_SPEED            32      // Moving velocity (2, RW, -)
#define ADDR_MX_PRESENT_POSITION        36      // Current Position (2,R, -)
#define ADDR_MX_PRESENT_SPEED           38      // Current speed (2, R, -)
#define ADDR_MX_MOVING                  46      // Motionstate (1, R, -)

#endif

// CKim - Struct for holding basic Dynamixel info 
typedef struct {
    uint16_t modelNum;
    uint8_t id;
    uint8_t baudrate;
    uint16_t cwAngLim;
    uint16_t ccwAngLim;
    //uint16_t multiturnOffset; // Only for MX28
    //uint16_t resDiv;          // Only for MX28
} DxlInfo;

// CKim - tty means TeleTYpewriter, which used to be the input output 'terminals'
// of the computers system in the past. Now, it refers to the general 'terminals' of the device.
// Change this definition for the correct port. We will be using terminal of the COM port.
#define MODEMDEVICE "/dev/ttyUSB0"		// CKim - Linux. This is for FTDI USB RS485 cable

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define BAUDRATE                        1000000

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque

// Data Byte Length
#define LEN_MX_GOAL_POSITION            2
#define LEN_MX_PRESENT_POSITION         2
#define LEN_MX_MOVING_SPEED             2

#define NUM_DXL                         2                   // Number of Dynamixel

#define DEG2RAD 3.141592/180.0
#define RAD2DEG 180.0/3.141592


class DxlMaster
{

public:
	DxlMaster();
	~DxlMaster();

    // CKim - Initialize Dynamixel .... RS485 port open and close...
    int InitializeDriver(const char* portName, const int baudrate, int protocolVer);
    void Disconnect();

    // CKim - Read Dynamixel Info
    void GetDynamixelInfo(int id);

    // CKim - Set Dynamixel Mode - This is done by setting cw and ccw limit
    // Wheel Mode : Both limit set to 0
    // Joint Mode : Set different cw and ccw limit.
    // Multiturn Mode : Both limit set to 4095, Only for MX series. Use Multiturn offset and res divider to set number of turns
    int SetWheelMode(int id);
    int SetJointMode(int id, int cwlim, int ccwlim);
    // int SetMultiTurnMode(int id);       // CKim - Only for MX

    // CKim - Enable Torque
    int EnableTorque(int id);
    int DisableTorque(int id);

    // CKim - Motor Commands. Position is in counts. Velocity is in...
    int GetJpos(int id, int& Pos);          
    int SetJpos(int id, const int& Pos);
    int GetVel(int id, int& vel);          
    int SetVel(int id, const int& Vel);

    // CKim - Uses SyncRead and Write. GetJposAll(), GetVelAll() only available for protocol V2.0
    int SetJposAll(const int* pList);   
    int SetVelAll(const int* vList);         
    // int GetJposAll(int* pList);          
    // int GetVelAll(int* vList);         
    
    
    // void SetGain(int id, const float* DIP);  // CKim - Only for MX
    // int SetOffset();                         // CKim - Only for MX

private:

    // CKim - Dynamixel Variables. Class for handling serial port, handling packets
    dynamixel::PortHandler*     m_portHandler;
    dynamixel::PacketHandler*   m_packetHandler;
    char m_portName[40];
    int m_baudrate;    

    // CKim - Sync Read(Write) reads from (writes to) same address of each connected dynamixel
    // Bulk Read(Write) can reads from (writes to) different addresses of each connected dynamixel
    // They are fully supported in protocol 2.0. For protocol 1.0, only SyncWrite is available 
    // for all Dynamixel and BulkRead is available for MX and X series.
    dynamixel::GroupSyncWrite*      m_groupSyncWritePos;
    dynamixel::GroupSyncWrite*      m_groupSyncWriteVel;
    dynamixel::GroupSyncRead*       m_groupSyncRead;
    
    dynamixel::GroupBulkRead*       m_groupBulkRead;
    dynamixel::GroupBulkWrite*      m_groupBulkWrite;

    // CKim - Dynamixel Device IDs, baud rates
    int m_Id;

    // CKim - List of all connected dynamixel ids
    int m_devIds[NUM_DXL];
};
