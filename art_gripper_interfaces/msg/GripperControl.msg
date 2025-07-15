uint8 control_word
# ----------------------------------------------------------------------------------------------------------------------
# BIT | ACTION                 | VALUE                                                               | DESC.
# ----------------------------------------------------------------------------------------------------------------------
# 0   | activate motor         | 0:activate, 1:deactivate                                            |
# 1   | activate target        | 0:ignore target, 1:update target                                    |
# 2   | (n/a)                  |                                                                     |
# 3   | (n/a)                  |                                                                     |
# 4   | (n/a)                  |                                                                     |
# 5   | reset friction model   | 1:reset                                                             |
# 6   | reset absolut encoder  | 1:reset                                                             |
# 7   | set control mode       | 0:basic mode, 1:direct CST(cyclic synchronous torque) control mode  |
# ----------------------------------------------------------------------------------------------------------------------

uint8 finger_width                  # [mm], 0~100
uint8 finger_pose                   # [deg], 0~180
uint8 finger_width_speed            # [mm/s], 1~200(default:150)
uint8 finger_pose_speed             # [deg/s], 1~255(default: 180)
uint8 gripping_force                # [N], 1~100(default:30)
uint8 contact_sensitivity           # 1~100(default:80)
int16[4] target_current             # 모터 제어 명령