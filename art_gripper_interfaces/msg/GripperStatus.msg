uint8 gripper_status        # Ready(BIT: 0)/Fault(BIT: 1)/In-motion(BIT: 2)/Contact(BIT: 3)
uint8 finger_width          # [mm], 0~100
uint8 finger_pose           # [deg], 0~180
uint16[4] status_word       #
int32[4] position           # [cnt]
int32[4] position_auxiliary # [cnt]
int32[4] velocity           # [cnt]
int16[4] current            # [TPU]