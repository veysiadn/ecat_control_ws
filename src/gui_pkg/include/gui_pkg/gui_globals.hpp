#pragma once

#define ID_CAMERA 0

enum LifeCycleStates
{
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

struct ControlUIButtonData
{
  uint8_t b_init_ecat;
  uint8_t b_reinit_ecat;
  uint8_t b_stop_cyclic_pdo;
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
