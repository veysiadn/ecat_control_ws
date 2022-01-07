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
/*****************************************************************************
 * \file  ecat_globals.hpp
 * \brief Header file for all include statements and global variables for EtherCAT
 *        communication.
 * 
 * This header file contains required include statements for IgH EtherCAT library,
 * global variables (e.g. ethercat master,master_state, domain,domain_state), 
 * structs for PDO offset and recieved data from slaves,
 * Communication period and number of slaves can be specified in here.
 *******************************************************************************/
#pragma once
#include "ecat_definitions.hpp"

/****************************************************************************/
                /// USER SHOULD DEFINE THIS AREAS ///
/// Number of connected servo drives.                
const uint32_t  g_kNumberOfServoDrivers = 1 ; 
/// Select operation mode for motors, default: Profile Velocity.
static int8_t   g_kOperationMode = kProfileVelocity ;  
#define NUM_OF_SLAVES     1  /// Total number of connected slave to the bus.
/// Set this to 1 if you have custom EtherCAT slave other than servo drive.
/// @note  That if you have different custom slave than EasyCAT you have to modify PDO mapping by yourself.
#define CUSTOM_SLAVE      0  
#define FREQUENCY       1000  /// Ethercat PDO exchange loop frequency in Hz
#define MEASURE_TIMING         0    /// If you want to measure timings leave it as one, otherwise make it 0.
/*****************************************************************************/
#define GEAR_RATIO          103
#define ENCODER_RESOLUTION  1024
#define INC_PER_ROTATION      GEAR_RATIO*ENCODER_RESOLUTION*4
#define FIVE_DEGREE_CCW      int(INC_PER_ROTATION/72)
#define THIRTY_DEGREE_CCW    int(INC_PER_ROTATION/12)
#define PERIOD_NS       (g_kNsPerSec/FREQUENCY)  /// EtherCAT communication period in nanoseconds.
#define PERIOD_US       (PERIOD_NS / 1000)
#define PERIOD_MS       (PERIOD_US / 1000)
#if CUSTOM_SLAVE
    #define FINAL_SLAVE     (NUM_OF_SLAVES-1)
#endif
const struct timespec       g_cycle_time = {0, PERIOD_NS} ;       // cycletime settings in ns. 

