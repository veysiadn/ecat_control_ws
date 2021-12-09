#include "DxlMaster.h"
#include <sys/time.h>
#include <stdio.h>
#define INTERVAL 20*MS

//pthread_mutex_t DxlMaster::mutex1 = PTHREAD_MUTEX_INITIALIZER;

DxlMaster::DxlMaster()
{
    m_groupSyncWritePos = NULL;
    m_groupSyncWriteVel = NULL;
    //m_groupBulkRead = NULL;
    m_groupSyncRead = NULL;
}

DxlMaster::~DxlMaster()
{

}

int DxlMaster::InitializeDriver(const char* portName, const int baudrate, int protocolVer)
{
    // CKim - Set port path, protocol version
    strcpy(m_portName,portName);    
    m_baudrate = baudrate;
    m_portHandler = dynamixel::PortHandler::getPortHandler(m_portName);
    m_packetHandler = dynamixel::PacketHandler::getPacketHandler(protocolVer);

    // CKim - Open port
    if (m_portHandler->openPort())    {
        printf("Succeeded to open the port %s\n",m_portName);
    }
    else    {
        printf("Failed to open the port %s\n",m_portName);
        return 0;
    }

    // CKim - Set port baudrate
    if (m_portHandler->setBaudRate(BAUDRATE))    {
        printf("Succeeded to change the baudrate to %d\n",BAUDRATE);
    }
    else    {
        printf("Failed to change the baudrate to %d\n",BAUDRATE);
        return 0;
    }

    // CKim - Fill up the list of connected Dynamixels......
    // Right now manually adding but should find good way to fill up the list
    // automatically.... json file??
    m_devIds[0] = 1;        // CKim - Tool rotation MX28
    m_devIds[1] = 2;        // CKim - Tool grasping RX24F

    // CKim - Initialize GroupSyncWrite instance. Set up parameters to write at the same time
    m_groupSyncWritePos = new dynamixel::GroupSyncWrite(m_portHandler, m_packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);
    m_groupSyncWriteVel = new dynamixel::GroupSyncWrite(m_portHandler, m_packetHandler, ADDR_MX_MOVING_SPEED, LEN_MX_MOVING_SPEED);
    
    // CKim - Set up GroupSyncRead by specifying the address to read. 
    m_groupSyncRead = new dynamixel::GroupSyncRead(m_portHandler,m_packetHandler,ADDR_MX_PRESENT_POSITION,LEN_MX_PRESENT_POSITION);

    // CKim - Initialize GroupBulkRead for position
    //m_groupBulkRead = new dynamixel::GroupBulkRead(m_portHandler, m_packetHandler);

    return 1;
}

void DxlMaster::Disconnect()
{
    if(m_groupSyncWritePos)    delete m_groupSyncWritePos;
    if(m_groupSyncWriteVel)    delete m_groupSyncWriteVel;
    //if(m_groupBulkRead)     delete m_groupBulkRead;
    if(m_groupSyncRead)     delete m_groupSyncRead;

    // Close port
    m_portHandler->closePort();
    printf("%s","Closing SerialPort\n");
}

void DxlMaster::GetDynamixelInfo(int id)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    DxlInfo info;
    // Model Num
    dxl_comm_result = m_packetHandler->read2ByteTxRx(m_portHandler, id, ADDR_MX_MODEL_NUMBER, &info.modelNum, &dxl_error);
    if ( (dxl_comm_result != COMM_SUCCESS) || (dxl_error != 0) )    {
        printf("Comm result %s, Packet error %s\n", m_packetHandler->getTxRxResult(dxl_comm_result), m_packetHandler->getRxPacketError(dxl_error));
    }
    // m_Id
    dxl_comm_result = m_packetHandler->read1ByteTxRx(m_portHandler, id, ADDR_MX_ID, &info.id, &dxl_error);
    if ( (dxl_comm_result != COMM_SUCCESS) || (dxl_error != 0) )    {
        printf("Comm result %s, Packet error %s\n", m_packetHandler->getTxRxResult(dxl_comm_result), m_packetHandler->getRxPacketError(dxl_error));
    }
    //Baud rate
    dxl_comm_result = m_packetHandler->read1ByteTxRx(m_portHandler, id, ADDR_MX_BAUDRATE, &info.baudrate, &dxl_error);
    if ( (dxl_comm_result != COMM_SUCCESS) || (dxl_error != 0) )    {
        printf("Comm result %s, Packet error %s\n", m_packetHandler->getTxRxResult(dxl_comm_result), m_packetHandler->getRxPacketError(dxl_error));
    }
    // cwAngLim
    dxl_comm_result = m_packetHandler->read2ByteTxRx(m_portHandler, id, ADDR_MX_CW_ANGLE_LIM, &info.cwAngLim, &dxl_error);
    if ( (dxl_comm_result != COMM_SUCCESS) || (dxl_error != 0) )    {
        printf("Comm result %s, Packet error %s\n", m_packetHandler->getTxRxResult(dxl_comm_result), m_packetHandler->getRxPacketError(dxl_error));
    }
    // ccwAngLim
    dxl_comm_result = m_packetHandler->read2ByteTxRx(m_portHandler, id, ADDR_MX_CCW_ANGLE_LIM, &info.ccwAngLim, &dxl_error);
    if ( (dxl_comm_result != COMM_SUCCESS) || (dxl_error != 0) )    {
        printf("Comm result %s, Packet error %s\n", m_packetHandler->getTxRxResult(dxl_comm_result), m_packetHandler->getRxPacketError(dxl_error));
    }
    // // Multiturn offset
    // dxl_comm_result = m_packetHandler->read2ByteTxRx(m_portHandler, id, ADDR_MX_MULTITURN_OFFSET, &info.multiturnOffset, &dxl_error);
    // if ( (dxl_comm_result != COMM_SUCCESS) || (dxl_error != 0) )    {
    //     printf("Comm result %s, Packet error %s\n", m_packetHandler->getTxRxResult(dxl_comm_result), m_packetHandler->getRxPacketError(dxl_error));
    // }
    // // Resolution divider
    // dxl_comm_result = m_packetHandler->read2ByteTxRx(m_portHandler, id, ADDR_MX_RES_DIV, &info.resDiv, &dxl_error);
    // if ( (dxl_comm_result != COMM_SUCCESS) || (dxl_error != 0) )    {
    //     printf("Comm result %s, Packet error %s\n", m_packetHandler->getTxRxResult(dxl_comm_result), m_packetHandler->getRxPacketError(dxl_error));
    // }

    printf("Dynamixel id %d \n", info.id); 
    printf("  Model num %d baudrate %d\n", info.modelNum, info.baudrate); 
    printf("  CW and CCW angle lim %d, %d\n", info.cwAngLim, info.ccwAngLim);
    // printf("  Multiturn offset %d\n",info.multiturnOffset);
    // printf("  Resolution divider %d\n",info.resDiv);
}

int DxlMaster::SetWheelMode(int id)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    // CKim - Write 0 to both cw and ccw limit
    dxl_comm_result = m_packetHandler->write2ByteTxRx(m_portHandler, id, ADDR_MX_CW_ANGLE_LIM, 0, &dxl_error);
    if ( (dxl_comm_result != COMM_SUCCESS) || (dxl_error != 0) )    {
        printf("Comm result %s, Packet error %s\n", m_packetHandler->getTxRxResult(dxl_comm_result), m_packetHandler->getRxPacketError(dxl_error));
        return 0;
    }    

    dxl_comm_result = m_packetHandler->write2ByteTxRx(m_portHandler, id, ADDR_MX_CCW_ANGLE_LIM, 0, &dxl_error);
    if ( (dxl_comm_result != COMM_SUCCESS) || (dxl_error != 0) )    {
        printf("Comm result %s, Packet error %s\n", m_packetHandler->getTxRxResult(dxl_comm_result), m_packetHandler->getRxPacketError(dxl_error));
        return 0;
    }
    return 1;
}

int DxlMaster::SetJointMode(int id, int cwlim, int ccwlim)
{
    if(cwlim > ccwlim)  {
        printf("ccwlim should be larger yhan cwlim\n");
        return 0;
    }
    if((cwlim < 0) || (ccwlim > 4095))  {
        printf("Invalid joint limit\n");
        return 0;
    }

    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    // CKim - Write cw and ccw limit
    dxl_comm_result = m_packetHandler->write2ByteTxRx(m_portHandler, id, ADDR_MX_CW_ANGLE_LIM, cwlim, &dxl_error);
    if ( (dxl_comm_result != COMM_SUCCESS) || (dxl_error != 0) )    {
        printf("Comm result %s, Packet error %s\n", m_packetHandler->getTxRxResult(dxl_comm_result), m_packetHandler->getRxPacketError(dxl_error));
        return 0;
    }    

    dxl_comm_result = m_packetHandler->write2ByteTxRx(m_portHandler, id, ADDR_MX_CCW_ANGLE_LIM, ccwlim, &dxl_error);
    if ( (dxl_comm_result != COMM_SUCCESS) || (dxl_error != 0) )    {
        printf("Comm result %s, Packet error %s\n", m_packetHandler->getTxRxResult(dxl_comm_result), m_packetHandler->getRxPacketError(dxl_error));
        return 0;
    }
    return 1;
}

int DxlMaster::EnableTorque(int id)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    dxl_comm_result = m_packetHandler->write1ByteTxRx(m_portHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if ( (dxl_comm_result != COMM_SUCCESS) || (dxl_error != 0) )    {
        printf("Comm result %s, Packet error %s\n", m_packetHandler->getTxRxResult(dxl_comm_result), m_packetHandler->getRxPacketError(dxl_error));
        return 0;
    }    
    return 1;
}

int DxlMaster::DisableTorque(int id)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    dxl_comm_result = m_packetHandler->write1ByteTxRx(m_portHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if ( (dxl_comm_result != COMM_SUCCESS) || (dxl_error != 0) )    {
        printf("Comm result %s, Packet error %s\n", m_packetHandler->getTxRxResult(dxl_comm_result), m_packetHandler->getRxPacketError(dxl_error));
        return 0;
    }    
    return 1;
}

int DxlMaster::GetJpos(int id, int& Pos)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error
    uint16_t p;
    dxl_comm_result = m_packetHandler->read2ByteTxRx(m_portHandler, id, ADDR_MX_PRESENT_POSITION, &p, &dxl_error);
    if ( (dxl_comm_result != COMM_SUCCESS) || (dxl_error != 0) )    {
        printf("Comm result %s, Packet error %s\n", m_packetHandler->getTxRxResult(dxl_comm_result), m_packetHandler->getRxPacketError(dxl_error));
        return 0;
    }
    Pos = p;
    return 1;
}

int DxlMaster::SetJpos(int id, const int& Pos)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    dxl_comm_result = m_packetHandler->write2ByteTxRx(m_portHandler, id, ADDR_MX_GOAL_POSITION, Pos, &dxl_error);
    if ( (dxl_comm_result != COMM_SUCCESS) || (dxl_error != 0) )    {
        printf("Comm result %s, Packet error %s\n", m_packetHandler->getTxRxResult(dxl_comm_result), m_packetHandler->getRxPacketError(dxl_error));
        return 0;
    }    
    return 1;
}

int DxlMaster::GetVel(int id, int& vel)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error
    uint16_t v;
    dxl_comm_result = m_packetHandler->read2ByteTxRx(m_portHandler, id, ADDR_MX_PRESENT_SPEED, &v, &dxl_error);
    if ( (dxl_comm_result != COMM_SUCCESS) || (dxl_error != 0) )    {
        printf("Comm result %s, Packet error %s\n", m_packetHandler->getTxRxResult(dxl_comm_result), m_packetHandler->getRxPacketError(dxl_error));
        return 0;
    }
    vel = v;
    return 1;
}

int DxlMaster::SetVel(int id, const int& Vel)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    dxl_comm_result = m_packetHandler->write2ByteTxRx(m_portHandler, id, ADDR_MX_MOVING_SPEED, Vel, &dxl_error);
    if ( (dxl_comm_result != COMM_SUCCESS) || (dxl_error != 0) )    {
        printf("Comm result %s, Packet error %s\n", m_packetHandler->getTxRxResult(dxl_comm_result), m_packetHandler->getRxPacketError(dxl_error));
        return 0;
    }    
    return 1;
}

int DxlMaster::SetJposAll(const int *pPos)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    bool dxl_addparam_result = false;               // addParam result
    uint8_t param_goal_position[2];

    // CKim - Fill in data to write
    for(int i=0; i<NUM_DXL; i++)
    {
        // Allocate goal position value into byte array
        param_goal_position[0] = DXL_LOBYTE(pPos[i]);
        param_goal_position[1] = DXL_HIBYTE(pPos[i]);

        // goal position value to the Syncwrite storage
        dxl_addparam_result = m_groupSyncWritePos->addParam(m_devIds[i], param_goal_position);
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", m_devIds[i]);
            return 0;
        }
    }

    // CKim - Synchronized write to multiple dynamixel
	dxl_comm_result = m_groupSyncWritePos->txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        fprintf(stderr, "%s\n", m_packetHandler->getTxRxResult(dxl_comm_result));
        return 0;
    }

    // Clear syncwrite parameter storage
    m_groupSyncWritePos->clearParam();
    return 1;
}

int DxlMaster::SetVelAll(const int *vList)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    bool dxl_addparam_result = false;               // addParam result
    uint8_t param_tgt_vel[2];

    // CKim - Fill in data to write
    for(int i=0; i<NUM_DXL; i++)
    {
        // Allocate goal position value into byte array
        param_tgt_vel[0] = DXL_LOBYTE(vList[i]);
        param_tgt_vel[1] = DXL_HIBYTE(vList[i]);

        // goal position value to the Syncwrite storage
        dxl_addparam_result = m_groupSyncWriteVel->addParam(m_devIds[i], param_tgt_vel);
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", m_devIds[i]);
            return 0;
        }
    }

    // CKim - Synchronized write to multiple dynamixel
	dxl_comm_result = m_groupSyncWriteVel->txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        fprintf(stderr, "%s\n", m_packetHandler->getTxRxResult(dxl_comm_result));
        return 0;
    }

    // Clear syncwrite parameter storage
    m_groupSyncWriteVel->clearParam();
    return 1;
}

// int DxlMaster::SetMultiTurnMode(int id)
// {
//     int dxl_comm_result = COMM_TX_FAIL;             // Communication result
//     uint8_t dxl_error = 0;                          // Dynamixel error
//
//     // CKim - Write 4095 to both cw and ccw limit
//     dxl_comm_result = m_packetHandler->write2ByteTxRx(m_portHandler, id, ADDR_MX_CW_ANGLE_LIM, 4095, &dxl_error);
//     if ( (dxl_comm_result != COMM_SUCCESS) || (dxl_error != 0) )    {
//         printf("Comm result %s, Packet error %s\n", m_packetHandler->getTxRxResult(dxl_comm_result), m_packetHandler->getRxPacketError(dxl_error));
//         return 0;
//     }    
//
//     dxl_comm_result = m_packetHandler->write2ByteTxRx(m_portHandler, id, ADDR_MX_CCW_ANGLE_LIM, 4095, &dxl_error);
//     if ( (dxl_comm_result != COMM_SUCCESS) || (dxl_error != 0) )    {
//         printf("Comm result %s, Packet error %s\n", m_packetHandler->getTxRxResult(dxl_comm_result), m_packetHandler->getRxPacketError(dxl_error));
//         return 0;
//     }
//     return 1;
// }

// int DxlMaster::GetJposAll(int* pPos)
// {
//     bool dxl_addparam_result = false;
//     int dxl_comm_result = COMM_TX_FAIL;     // Communication result
// 	bool dxl_getdata_result = false;        // GetParam result
//
//     // CKim - Add id of dynamixel for sync read
//     for(int i=0; i<NUM_DXL; i++)
//     {
//         // CKim - We will read present position of each actuators
//         dxl_addparam_result = m_groupSyncRead->addParam(m_devIds[i]);
//         if (dxl_addparam_result != true)
//         {
//             fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed\n", m_devIds[i]);
//             return 0;
//         }
//     }
//
// 	// CKim - Request Sync read
// 	dxl_comm_result = m_groupSyncRead->txRxPacket();
// 	if (dxl_comm_result != COMM_SUCCESS)
// 	{
// 		fprintf(stderr,"Sync read error %s\n",m_packetHandler->getTxRxResult(dxl_comm_result));
// 		return 0;
// 	}
//
//     // CKim - Check data
// 	for(int i=0; i<NUM_DXL; i++)
// 	{
// 		dxl_getdata_result = m_groupSyncRead->isAvailable(m_devIds[i], ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
// 		if (dxl_getdata_result != true)
// 		{
// 		  fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", m_devIds[i]);
// 		  return 0;
// 		}
// 		pPos[i] = m_groupSyncRead->getData(m_devIds[i], ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
// 	}
// 	return 1;
// }

    // // CKim - Set up parameters to bulk read
    // for(int i=0; i<NUM_DXL; i++)
    // {
    //     // CKim - We will read present position of each actuators
    //     m_groupSyncRead->
    //     dxl_addparam_result = m_groupBulkRead->addParam(m_devIds[i], ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
    //     if (dxl_addparam_result != true)
    //     {
    //         fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed", m_devIds[i]);
    //         return 0;
    //     }
    // }
    // dynamixel::GroupSyncWrite
	// // CKim - Request bulk read
	// dxl_comm_result = m_groupBulkRead->txRxPacket();
	// if (dxl_comm_result != COMM_SUCCESS)
	// {
	// 	fprintf(stderr,"Bulk read error %s\n",m_packetHandler->getTxRxResult(dxl_comm_result));
	// 	return 0;
	// }

    // // CKim - Check data
	// for(int i=0; i<NUM_DXL; i++)
	// {
	// 	dxl_getdata_result = m_groupBulkRead->isAvailable(m_devIds[i], ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
	// 	if (dxl_getdata_result != true)
	// 	{
	// 	  fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", m_devIds[i]);
	// 	  return 0;
	// 	}
	// 	pPos[i] = m_groupBulkRead->getData(m_devIds[i], ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
	// }
	// return 1;

// void DxlMaster::SetGain(int id, const float* DIP)
// {
//     int dxl_comm_result = COMM_TX_FAIL;             // Communication result
//     uint8_t dxl_error = 0;                          // Dynamixel error
//     uint8_t dipGain[3];

//     // CKim - Conversion formula from Dynamixel manual
//     dipGain[0] = DIP[0]*250.0;
//     dipGain[1] = DIP[1]*2.048;
//     dipGain[2] = DIP[2]*8.0;

//     for(int i=0;i<3; i++)
//     {
//         dxl_comm_result = m_packetHandler->write1ByteTxRx(m_portHandler, m_devIds[id], ADDR_DIP_GAIN+i, dipGain[i],&dxl_error);
//         if (dxl_comm_result != COMM_SUCCESS)
//         {
//             printf("%s\n", m_packetHandler->getTxRxResult(dxl_comm_result));
//         }
//         else if (dxl_error != 0)
//         {
//             printf("%s\n", m_packetHandler->getRxPacketError(dxl_error));
//         }
//         else
//         {
//             printf("DIP[%d] of Dynamixel %d set\n",i,id);
//         }
//     }
// }

// int DxlMaster::SetOffset()
// {
//     int dxl_curr_position[3] = {0,0,0};  // Current position

//     // CKim - Read position
//     bool res = GetJpos(dxl_curr_position);
//     if(!res)    {
//         fprintf(stderr, "Error while reading position");
//         return 0;
//     }

//     for(int i=0; i<3; i++)  {   m_MtrCntOffset[i] = dxl_curr_position[i];   }
//     printf("Offset is %d %d %d\n", m_MtrCntOffset[0],m_MtrCntOffset[1],m_MtrCntOffset[2]);
//     //for(int i=0; i<3; i++)  {   m_TgtMtrCnt[i] = m_MtrCntOffset[i];   }
// }
