# Per-control-step metadata published by sysid_data_collector alongside
# /safe_drive and the pose/twist/odom streams on every control tick.
#
# All fields describe the INTENDED/COMMANDED step; measured state is
# reconstructed offline from the pose stream using header.stamp and
# t_cmd_intended. header.stamp == t_cmd_intended, matching the /safe_drive
# message's own header stamp for bag-time correlation.

std_msgs/Header header

uint32 path_id
uint32 step_index

# State-machine phase. Values MUST mirror sysid_data_collector.State(IntEnum)
# exactly -- see sysid_data_collector.py.
uint8 phase
uint8 PHASE_IDLE=0
uint8 PHASE_GENERATE=1
uint8 PHASE_ARMED=2
uint8 PHASE_EXECUTE=3
uint8 PHASE_SETTLE=4
uint8 PHASE_DONE=5

float64 v_cmd
float64 delta_cmd
float64 kappa_cmd

int32 coverage_bin_v
int32 coverage_bin_kappa
int32 coverage_bin_id

float64 lat_accel_est

bool flag_sync_chirp
bool flag_high_lat_accel
bool flag_stale_pose

builtin_interfaces/Time t_cmd_intended
builtin_interfaces/Time t_cmd_published
float64 loop_jitter_s

float64 dt
uint32 rng_seed
