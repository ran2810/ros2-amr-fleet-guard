# Published by every robot agent on /<robot_id>/robot_state (2 Hz).

# mission / robot state
uint8 IDLE=0
uint8 NAVIGATING_TO_PICK=1
uint8 PICK=2
uint8 NAVIGATING_TO_DROP=3
uint8 DROP=4
uint8 NAVIGATING_TO_DOCK=5
uint8 CHARGE=6
uint8 FAULT=7
uint8 STARTING=8

# health
uint8 HEALTHY=0
uint8 DEGRADED=1
uint8 FAULTED=2

builtin_interfaces/Time stamp
string robot_id
uint8 state
uint8 health
float32 x                    # pose in the map frame
float32 y
float32 yaw
float32 battery              # state of charge [%]
float32 nearest_robot_m      # distance to the closest other robot (inf if unknown)
string assigned_task         # empty when none
string detail                # reason for DEGRADED / FAULTED / current dock-return cause
