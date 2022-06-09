/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2021 Veysi ADIN, UST KIST
 *
 *  This file is part of the IgH EtherCAT master userspace program in the ROS2 environment.
 *
 *  The IgH EtherCAT master userspace program in the ROS2 environment is free software; you can
 *  redistribute it and/or modify it under the terms of the GNU General
 *  Public License as published by the Free Software Foundation; version 2
 *  of the License.
 *
 *  The IgH EtherCAT master userspace program in the ROS2 environment is distributed in the hope that
 *  it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with the IgH EtherCAT master userspace program in the ROS environment. If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 *  Contact information: veysi.adin@kist.re.kr
 *****************************************************************************/
/******************************************************************************
 *  \file   ecat_node.hpp
 *  \brief  IgH EtherCAT library functionality wrapper class header file.
 * 
 *   This header file contains blueprint of EthercatNode class which will be 
 *   responsible for encapsulating IgH EtherCAT library's functionality.
 *******************************************************************************/
#pragma once
/******************************************************************************/
#include "ecat_globals.hpp"
/******************************************************************************/
/// Forward declaration of EthercatSlave class.
class EthercatSlave ;
#include "ecat_slave.hpp"
/******************************************************************************/
/// ROS2 Headers
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joy.hpp"
/*****************************************************************************/
namespace EthercatCommunication
{
    class EthercatNode
    {
        public:
            EthercatNode();
            ~EthercatNode();
        EthercatSlave slaves_[NUM_OF_SLAVES];
        std::vector<SdoRequest> request_sdos_ ; 
    /**
     * @brief Requests master instance and creates a domain for a master.
     * @note  Keep in mind that created master and domain are global variables.
     * @return 0 if succesful otherwise -1.
     */
        int  ConfigureMaster();
    /**
     * @brief Defines default connected slaves based on number of slaves.
     *        Specifies its position, vendor id , product code etc.
     *  Default connected slaves considered implementation specific. In our case it will be 3 motors and 
     *  one EasyCAT slave.
     */
        void DefineDefaultSlaves();
    /**
     * @brief Passes your defined slave to EthercatNode class.
     * 
     * @param c_slave first create your own EthercatSlave instance and modify it then pass it to configuration.
     * @param position specify the physical connection position for your custom configured slave.
     */
        void SetCustomSlave(EthercatSlave c_slave, int position);
    /**
     * @brief Obtains slave configuration for all slaves w.r.t master.
     * @return 0 if successfull, otherwise -1. 
     */
        int  ConfigureSlaves();
    /**
     * @brief Set mode to ProfilePositionMode with specified parameters for servo drive on that position.
     *       
     * @param P Profile position parameter structure specified by user.
     * @param position Slave position
     * @return 0 if successfull, otherwise -1.
     */
        int  SetProfilePositionParameters(ProfilePosParam& P , int position);
    /**
     * @brief Set the mode to ProfilePositionMode with specified Parameters for all servo drives on the bus.
     * 
     * @param P Profile position parameter structure specified by user.
     * @return 0 if succesful, otherwise -1.
     */
        int SetProfilePositionParametersAll(ProfilePosParam& P);
    /**
     * @brief Set mode to ProfileVelocityMode with specified parameters for servo drive on that position.
     * 
     * @param P Profile velocity parameter structure specified by user.
     * @param position Slave position
     * @return 0 if succesful, -1 otherwise.
     */
        int SetProfileVelocityParameters(ProfileVelocityParam& P,int position);
    /**
     * @brief Set mode to ProfileVelocityMode with specified parameters for all servo drives on the bus
     * 
     * @param P Profile velocity parameter structure specified by user.
     * @return 0 if successfull, -1 otherwise.
     * @todo Add error code to all functions.Instead of returning -1. 
     */
        int SetProfileVelocityParametersAll(ProfileVelocityParam& P);
    /**
     * @brief Set the Cyclic Sync Position Mode Parameters for slave in specified physical position w.r.t. master.
     * 
     * @param P Cyclic Sync. Position Mode Parameters.
     * @param position Physical position of slave to be configured
     * @return 0 if sucessfull, otherwise -1.
     */
        int SetCyclicSyncPositionModeParameters(CSPositionModeParam& P, int position);
    /**
     * @brief Sets the Cyclic Synchronous Position Mode Parameters for all connected motor driver slaves
     * 
     * @return 0 if sucessful, otherwise -1.
     */
        int SetCyclicSyncPositionModeParametersAll(CSPositionModeParam &P);

    /**
     * @brief Set the Cyclic Sync Velocity Mode Parameters for slave in specified physical position w.r.t. master.
     * 
     * @param P Cyclic Sync. Velocity Mode Parameters.
     * @param position Physical position of slave to be configured
     * @return 0 if sucessfull, otherwise -1.
     */
        int SetCyclicSyncVelocityModeParameters(CSVelocityModeParam& P, int position);
    /**
     * @brief Sets the Cyclic Synchronous Velocity Mode Parameters for all connected motor driver slaves
     * 
     * @return 0 if sucessful, otherwise -1.
     */
        int SetCyclicSyncVelocityModeParametersAll(CSVelocityModeParam &P);

    /**
     * @brief Set the Cyclic Sync Torque Mode Parameters for slave in specified physical position w.r.t. master.
     * 
     * @param P Cyclic Sync. Torque Mode Parameters.
     * @param position Physical position of slave to be configured
     * @return 0 if sucessfull, otherwise -1.
     */
        int SetCyclicSyncTorqueModeParameters(CSTorqueModeParam& P, int position);

    /**
     * @brief Sets the Cyclic Synchronous Torque Mode Parameters for all connected motor driver slaves
     * 
     * @return 0 if sucessful, otherwise -1.
     */
        int SetCyclicSyncTorqueModeParametersAll(CSTorqueModeParam &P);
    /**
     * @brief Maps default SDOs which can be found @see SdoRequest struct.
     * 
     * @return 0 if successfull, otherwise -1.
     */
        int MapDefaultSdos();
    /**
     * @brief Maps default PDOs for the application.
     * @note This method is specific to our implementation. However you can modify it to suit your needs.
     * If you have different topology or different servo drives use MapCustomPdos() 
     * function or modify this function based on your needs.
     * @return 0 if successfull, otherwise -1.
     */
        int MapDefaultPdos();
    /**
     * @brief Map Custom PDO based on your PDO mappings
     * @note  You have to specify slave syncs and slave pdo registers before using function
     * @param c_slave EthercatSlave instance
     * @param position Physical position of your slave w.r.t master
     * @return 0 if successfull, -1 otherwise.
     */
        int MapCustomPdos(EthercatSlave c_slave, int position);
    /**
     * @brief Configures DC sync for our default configuration
     * 
     */
        void ConfigDcSyncDefault();
    /**
     * @brief Configures DC synchronization for specified slave position
     * 
     * @param assign_activate Activating DC synchronization for slave.
     * 0x300 for Elmo | and same for EasyCAT
     * @note Assign activate parameters specified in slaves ESI file 
     * 
     * @param position
     */
        void ConfigDcSync(uint16_t assign_activate, int position);
    /**
     * @brief This function will check slave's application layer states. (INIT/PREOP/SAFEOP/OP)
     */
        void CheckSlaveConfigurationState();
    /**
     * @brief This function will check master's state, in terms of number of responding slaves and their application layer states
     * 
     * @return 0 if succesful, otherwise -1 
     * \see ec_master_state_t structure.
     **/
        int  CheckMasterState();
    /**
     * @brief  Reads the state of a domain.
     * Stores the domain state in the given state structure.
     * Using this method, the process data exchange can be monitored in realtime.
     * */
        void CheckMasterDomainState();
    /**
     * @brief Activates master, after this function call realtime operation can start.
     * \warning Before activating master all configuration should be done
     * \warning After calling this function you have to register domain(s) and start realtime task.
     * @return 0 if succesful, otherwise -1. 
     */
        int  ActivateMaster();
    /**
     * @brief Registers domain for each slave.
     *  This method has to be called after ecrt_master_activate() to get the mapped domain process data memory. 
     * @return 0 if succeful , otherwise -1 
     */
        int  RegisterDomain();
    /**
     * @brief Puts all slave to operational mode. User must call this before entering real-time operation.
     *        Reason for this function is that, master and slave has to do several exchange before becoming operational.
     *        So this function does exchange between master and slaves for up to 10 sec, could finish earlier.
     *        If timeout occurs it will return -1.
     * @return 0 if successfull, otherwise -1.
     */
        int  WaitForOperationalMode();

    /**
     * @brief Opens EtherCAT master via command line tool if it's not already on.
     * 
     * @return 0 if successfull, otherwise -1.
     */ 
        int OpenEthercatMaster();
    /**
     * @brief Get the Number Of physically Connected Slaves to the bus.And checks if specified NUM_OF_SLAVES
     *        is correct.
     * @return 0 if NUM_OF_SLAVES setting is correct, otherwise -1.
     */
        int GetNumberOfConnectedSlaves();
    /**
     * @brief Get the information of physically connected slaves to the master.
     *        This function will return connected slave's vendor id, product code.
     */
        void GetAllSlaveInformation();

    /**
     * @brief Deactivates master, and it is used to stop cyclic PDO exchange.
     * @note That calling this function means that your PDO exchange will stop.
     * @note Additionally all pointers created by requesting master are freed with this function./
     * If you want to resume your communication, you'll have to do the configuration again.
     */
        void DeactivateCommunication();
    /**
     * @brief Deactivates and releases master shouldn't be called in real-time.
     */
        void ReleaseMaster();
    /**
     * @brief Shutdowns EtherCAT master via command line tool if it's not already off.
     * 
     * @return 0 if successfull, otherwise -1.
     */ 
        int ShutDownEthercatMaster();
    /**
     * @brief Restarts Ethercat master via command line tool.
     * 
     * @return 0 if successfull, otherwise -1.
     */
        int RestartEthercatMaster();
    /**
     * @brief Writes SDO in real-time context.
     * 
     * @param req 
     * @param data 
     */
        void WriteSDO(ec_sdo_request_t *req, int32_t data,int size);
    /**
     * @brief Reads SDO in real-time context by creating read request.
     * 
     * @param req 
     */
        uint16_t ReadSDO(ec_sdo_request_t *req,uint16_t &status_word);
    /**
     * @brief This function reads data from specified index and subindex via SDO, returned data will be stored in 
     * pack.data which needs to be casted to correct data type afterwards.
     * \see @SDO_data struct
     * \note This is a blocking function, until response has been received.Don't use it in real-time context!!.
     * 
     * @param pack SDO_data struct which contains; index,subindex,data size, and data that will be used to store read data.
     * @return 0 if successfull, otherwise -1. 
     */
        int8_t SdoRead(SDO_data &pack);

    /**
     * @brief This function writes data to specified index and subindex via SDO.
     * \see @SDO_data struct
     * \note This is a blocking function, until response has been received.Don't use it in real-time context!!.
     * 
     * @param pack SDO_data struct which contains; index,subindex,data size and data to be written.
     * @return 0 if successfull, otherwise -1. 
     */
        int8_t SdoWrite(SDO_data &pack);

    /**
     * @brief Get the Status Word from CiA402 slaves in specified index via SDO communication.
     * @param index slave index
     * @return status word 
     */
        uint16_t ReadStatusWordViaSDO(int index);
    /**
     * @brief Writes control word to slave in specified index via SDO.
     * 
     * @param index slave index, @param control_word control word to be written 
     * @return 0 if successfull otherwise -1.
     */
        int16_t WriteControlWordViaSDO(int index,uint16_t control_word);
    /**
     * @brief Writes desired operational mode, to slave in specified index via SDO.
     * 
     * @param index slave index
     * @param op_mode desired operational mode. @see OpMode enum defined in @file ecat_definitions.hpp
     * @return 0 if successfull, otherwise -1. 
     */
        int16_t WriteOpModeViaSDO(int index,uint8_t op_mode);
    /**
     * @brief Reads current operational mode from slave in specified index via SDO.
     * 
     * @param index slave index
     * @return status word of selected slave.
     */
        uint8_t ReadOpModeViaSDO(int index);
    /**
     * @brief Reads actual velocity value from slave in specified index via SDO.
     * 
     * @param index slave index
     * @return actual velocity value of selected slave. 
     */
        int32_t ReadActualVelocityViaSDO(int index);
    /**
     * @brief Writes target velocity value via SDO to slave in specified index.
     * 
     * @param index slave index
     * @param target_vel desired target velocity val.
     * @return 0 if successfull, othewise -1. 
     */
        int16_t WriteTargetVelocityViaSDO(int index,int32_t target_vel);
    /**
     * @brief Read actual position from slave in specified index via SDO.
     * 
     * @param index slave index
     * @return actual position of selected slave.
     */
        int32_t ReadActualPositionViaSDO(int index);
    /**
     * @brief Writes target position value via SDO to slave in specified index.
     * 
     * @param index slave index.
     * @param target_pos desired target position value.
     * @return 0 if successfull, otherwise -1.
     */
        int16_t WriteTargetPositionViaSDO(int index,int32_t target_pos);
    /**
     * @brief Read actual torque value from slave in specified index via SDO.
     * 
     * @param index 
     * @return uint16_t 
     */
        int16_t ReadActualTorqueViaSDO(int index);
    /**
     * @brief Writes target torque value to slave in specified index via SDO.
     * 
     * @param index slave index
     * @param target_tor desired target torque value.
     * @return 0 if successfull, otherwise -1.
     */
        int16_t WriteTargetTorqueViaSDO(int index,uint16_t target_tor);
    /**
     * @brief Enable CiA402 supported motor drives in specified index via SDO.
     * 
     * @param index slave index
     * @return 0 if successfull, otherwise -1.
     */
        int16_t EnableDrivesViaSDO(int index);
    /**
     * @brief Disable CiA402 supported motor drives in specified index via SDO.
     * 
     * @param index slave index
     * @return 0 if successfull, otherwise -1.
     */        
        int16_t DisableDrivesViaSDO(int index);

        private:
        /// File descriptor to open and wake  master from CLI.
        int  fd;
        
    };
}