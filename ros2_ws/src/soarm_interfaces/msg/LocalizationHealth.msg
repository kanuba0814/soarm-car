uint8 INITIALIZING=0
uint8 HEALTHY=1
uint8 DEGRADED=2
uint8 RELOCALIZING=3
uint8 LOST=4

uint32 REASON_NONE=0
uint32 REASON_QUALITY=1
uint32 REASON_AMBIGUOUS=2
uint32 REASON_DEGENERATE=4
uint32 REASON_STALE_INPUT=8
uint32 REASON_INTERNAL=16

builtin_interfaces/Time stamp
uint64 sequence
uint8 state
uint32 reason_mask
builtin_interfaces/Time last_accepted_correction_stamp
uint32 consecutive_rejections
float32 fitness
float32 rmse
string map_id
