# ART Gripper ROS Package

## Introduction

This is the EtherCAT driver and sample client package for the 2-finger/3-finger gripper developed by the Hyundai Motor Company RoboticsLab.

## Package and Node Configuration

The `art_gripper` package is structured as follows:

*   **`art_gripper` (C++ Package):** Contains the core ROS 2 nodes for controlling the gripper.
    *   **`gripper_ecat` Node:** This is the primary control node responsible for direct communication with the gripper hardware via EtherCAT. It publishes gripper status and EtherCAT state, and provides services for controlling the gripper.
    *   **`gripper_client` Node:** A client node that demonstrates how to interact with the `gripper_ecat` node's services and topics. It can be used for testing and sending commands to the gripper.
*   **`art_gripper_py` (Python Package):** Provides a Python interface for interacting with the gripper, including a Python client node (`gripper_client_py`) that mirrors the functionality of the C++ client.
*   **`art_gripper_interfaces`:** Defines the custom ROS 2 messages and service types used for communication within the `art_gripper` ecosystem.

## Colcon Build Instructions

To build this package using colcon, navigate to the root of your workspace (`gripper_ws`) and run the following command:

```bash
$ colcon build --symlink-install --packages-select art_gripper_interfaces art_gripper art_gripper_py
```

After building, source your workspace to make the ROS 2 packages available:

```bash
$ source install/setup.bash
```

## udev 설정

EtherCAT 디바이스에 대한 udev 규칙을 설정하여 적절한 권한을 부여해야 합니다. 다음 명령을 실행하여 udev 규칙을 추가합니다.

```bash
$ echo 'KERNEL=="EtherCAT0", MODE="0666"' | sudo tee /etc/udev/rules.d/99-ethercat.rules
```

규칙 추가 후에 `sudo ethercatctl restart`를 해줘야합니다.

## Running the `gripper_ecat` Node

To run the `gripper_ecat` node, use the following command:

```bash
$ ros2 run art_gripper gripper_ecat --ros-args -p gripper_status_publish_rate_hz:=<rate_in_hz> -p ethercat_state_publish_rate_hz:=<rate_in_hz>
```

**Arguments:**

*   `gripper_status_publish_rate_hz`: Frequency (in Hz) at which gripper status messages are published. Default is 100.
*   `ethercat_state_publish_rate_hz`: Frequency (in Hz) at which EtherCAT state messages are published. Default is 100.

---

## **ART Gripper ROS 서비스**

이 문서는 `art_gripper` 패키지에서 제공하고 사용하는 ROS 서비스들에 대한 명세를 다룹니다.

---

### **공통 응답 (Common Response)**

대부분의 서비스는 결과 코드를 포함하는 동일한 형식의 응답을 반환합니다.

- **`response`**
  - `int32 result`: 서비스 호출의 결과를 나타냅니다. `0`은 성공을 의미하며, 다른 값은 오류를 나타냅니다.

---

### **1. `GetGripperInfo`**

그리퍼의 기본 정보를 요청합니다.

- **서비스 이름:** `/get_gripper_info`
- **서비스 타입:** `art_gripper_interfaces/srv/GetGripperInfo`
- **요청 (Request):** 없음
- **응답 (Response)**
  | 필드명 | 타입 | 설명 |
  | --- | --- | --- |
  | `info` | `art_gripper_interfaces/msg/GripperInfo` | 그리퍼 정보 |

  GripperInfo
  | 필드명 | 타입 | 설명 |
  | --- | --- | --- |
  | `name` | `string` | 그리퍼 이름 |
  | `version` | `string` | 그리퍼 펌웨어 버전 |
  | `description` | `string` | 그리퍼 설명 | 

- **샘플 명령:**
  ```bash
  $ ros2 service call /get_gripper_info art_gripper_interfaces/srv/GetGripperInfo
  ```

### **2. `MotorOn`**

그리퍼의 모터를 켜거나 끕니다.

- **서비스 이름:** `/motor_on`
- **서비스 타입:** `art_gripper_interfaces/srv/MotorOn`
- **요청 (Request)**
  | 필드명 | 타입 | 설명 |
  | --- | --- | --- |
  | `on` | `uint8` | 모터 상태. `1` for On, `0` for Off. |

- **샘플 명령:**
  ```bash
  # 모터 켜기
  $ ros2 service call /motor_on art_gripper_interfaces/srv/MotorOn "{'on': 1}"

  # 모터 끄기
  $ ros2 service call /motor_on art_gripper_interfaces/srv/MotorOn "{'on': 0}"
  ```

### **3. `ResetAbsEncoder`**

절대 엔코더를 리셋합니다.

- **서비스 이름:** `/reset_abs_encoder`
- **서비스 타입:** `art_gripper_interfaces/srv/ResetAbsEncoder`
- **요청 (Request):** 없음

- **샘플 명령:**
  ```bash
  $ ros2 service call /reset_abs_encoder art_gripper_interfaces/srv/ResetAbsEncoder
  ```

### **4. `ResetFrictionModel`**

마찰 모델을 리셋합니다.

- **서비스 이름:** `/reset_friction_model`
- **서비스 타입:** `art_gripper_interfaces/srv/ResetFrictionModel`
- **요청 (Request):** 없음

- **샘플 명령:**
  ```bash
  $ ros2 service call /reset_friction_model art_gripper_interfaces/srv/ResetFrictionModel
  ```

### **5. `SetContactSensitivity`**

그리퍼의 접촉 감도를 설정합니다.

- **서비스 이름:** `/set_contact_sensitivity`
- **서비스 타입:** `art_gripper_interfaces/srv/SetContactSensitivity`
- **요청 (Request)**
  | 필드명 | 타입 | 설명 |
  | --- | --- | --- |
  | `contact_sensitivity` | `uint8` | 접촉 감도 값. (범위: 1 ~ 100, 기본값: 80) |

- **샘플 명령:**
  ```bash
  $ ros2 service call /set_contact_sensitivity art_gripper_interfaces/srv/SetContactSensitivity "{'contact_sensitivity': 80}"
  ```

### **6. `SetGrippingForce`**

그리퍼가 물체를 잡는 힘(악력)을 설정합니다.

- **서비스 이름:** `/set_gripping_force`
- **서비스 타입:** `art_gripper_interfaces/srv/SetGrippingForce`
- **요청 (Request)**
  | 필드명 | 타입 | 설명 |
  | --- | --- | --- |
  | `gripping_force` | `uint8` | 악력 값. 단위: N. (범위: 1 ~ 100, 기본값: 30) |

- **샘플 명령:**
  ```bash
  $ ros2 service call /set_gripping_force art_gripper_interfaces/srv/SetGrippingForce "{'gripping_force': 30}"s
  ```

### **7. `SetTarget`**

그리퍼의 여러 파라미터를 한번에 설정합니다.

- **서비스 이름:** `/set_target`
- **서비스 타입:** `art_gripper_interfaces/srv/SetTarget`
- **요청 (Request)**
  | 필드명 | 타입 | 설명 |
  | --- | --- | --- |
  | `finger_width` | `uint8` | 목표 핑거 너비. [mm], 0~100 |
  | `finger_pose` | `uint8` | 목표 핑거 회전 각도. [deg], 0~180 |
  | `finger_width_speed` | `uint8` | 핑거 개폐 속도. [mm/s], 1~200(default:150) |
  | `finger_pose_speed` | `uint8` | 핑거 회전 속도. [deg/s], 1~255(default: 180) |
  | `gripping_force` | `uint8` | 악력. [N], 1~100(default:30) |
  | `contact_sensitivity` | `uint8` | 접촉 감도. 1~100(default:80) |

- **샘플 명령:**
  ```bash
  $ ros2 service call /set_target art_gripper_interfaces/srv/SetTarget "{'finger_width': 50, 'finger_pose': 90, 'finger_width_speed': 100, 'finger_pose_speed': 150, 'gripping_force': 40, 'contact_sensitivity': 70}"
  ```

### **8. `SetTargetCurrent`**

각 모터의 목표 전류를 설정합니다.

- **서비스 이름:** `/set_target_current`
- **서비스 타입:** `art_gripper_interfaces/srv/SetTargetCurrent`
- **요청 (Request)**
  | 필드명 | 타입 | 설명 |
  | --- | --- | --- |
  | `target_current` | `int16[4]` | 4개 모터의 목표 전류. |

- **샘플 명령:**
  ```bash
  $ ros2 service call /set_target_current art_gripper_interfaces/srv/SetTargetCurrent "{'target_current': [10, 10, 10, 10]}"
  ```

### **9. `SetTargetFingerPose`**

목표 핑거 회전 각도를 설정합니다.

- **서비스 이름:** `/set_target_finger_pose`
- **서비스 타입:** `art_gripper_interfaces/srv/SetTargetFingerPose`
- **요청 (Request)**
  | 필드명 | 타입 | 설명 |
  | --- | --- | --- |
  | `finger_pose` | `uint8` | 목표 핑거 회전 각도. [deg], 0~180 |

- **샘플 명령:**
  ```bash
  # Grasp
  $ ros2 service call /set_target_finger_pose art_gripper_interfaces/srv/SetTargetFingerPose "{'finger_pose': 0}"

  # 3-finger
  $ ros2 service call /set_target_finger_pose art_gripper_interfaces/srv/SetTargetFingerPose "{'finger_pose': 90}"

  # 2-finger
  $ ros2 service call /set_target_finger_pose art_gripper_interfaces/srv/SetTargetFingerPose "{'finger_pose': 180}"
  ```

### **10. `SetTargetFingerPoseSpeed`**

핑거의 회전(Pose) 속도를 설정합니다.

- **서비스 이름:** `/set_target_finger_pose_speed`
- **서비스 타입:** `art_gripper_interfaces/srv/SetTargetFingerPoseSpeed`
- **요청 (Request)**
  | 필드명 | 타입 | 설명 |
  | --- | --- | --- |
  | `finger_pose_speed` | `uint8` | 핑거 회전 속도. [deg/s], 1~255(default: 180) |

- **샘플 명령:**
  ```bash
  $ ros2 service call /set_target_finger_pose_speed art_gripper_interfaces/srv/SetTargetFingerPoseSpeed "{'finger_pose_speed': 100}"
  ```

### **11. `SetTargetFingerPoseWithSpeed`**

목표 핑거 회전 각도와 속도를 함께 설정하여 즉시 이동시킵니다.

- **서비스 이름:** `/set_target_finger_pose_with_speed`
- **서비스 타입:** `art_gripper_interfaces/srv/SetTargetFingerPoseWithSpeed`
- **요청 (Request)**
  | 필드명 | 타입 | 설명 |
  | --- | --- | --- |
  | `finger_pose` | `uint8` | 목표 핑거 회전 각도. [deg], 0~180 |
  | `finger_pose_speed` | `uint8` | 핑거 회전 속도. [deg/s], 1~255(default: 180) |

- **샘플 명령:**
  ```bash
  $ ros2 service call /set_target_finger_pose_with_speed art_gripper_interfaces/srv/SetTargetFingerPoseWithSpeed "{'finger_pose': 90, 'finger_pose_speed': 100}"
  ```

### **12. `SetTargetFingerWidth`**

목표 핑거 너비를 설정합니다.

- **서비스 이름:** `/set_target_finger_width`
- **서비스 타입:** `art_gripper_interfaces/srv/SetTargetFingerWidth`
- **요청 (Request)**
  | 필드명 | 타입 | 설명 |
  | --- | --- | --- |
  | `finger_width` | `uint8` | 목표 핑거 너비. [mm], 0~100 |

- **샘플 명령:**
  ```bash
  # Grasp
  $ ros2 service call /set_target_finger_width art_gripper_interfaces/srv/SetTargetFingerWidth "{'finger_width': 0}"

  # Open
  $ ros2 service call /set_target_finger_width art_gripper_interfaces/srv/SetTargetFingerWidth "{'finger_width': 100}"
  ```

### **13. `SetTargetFingerWidthSpeed`**

그리퍼 핑거의 개폐(Width) 속도를 설정합니다.

- **서비스 이름:** `/set_target_finger_width_speed`
- **서비스 타입:** `art_gripper_interfaces/srv/SetTargetFingerWidthSpeed`
- **요청 (Request)**
  | 필드명 | 타입 | 설명 |
  | --- | --- | --- |
  | `finger_width_speed` | `uint8` | 핑거 개폐 속도. [mm/s], 1~200(default:150) |

- **샘플 명령:**
  ```bash
  $ ros2 service call /set_target_finger_width_speed art_gripper_interfaces/srv/SetTargetFingerWidthSpeed "{'finger_width_speed': 100}"
  ```

### **14. `SetTargetFingerWidthWithSpeed`**

목표 핑거 개폐 너비와 속도를 함께 설정하여 즉시 이동시킵니다.

- **서비스 이름:** `/set_target_finger_width_with_speed`
- **서비스 타입:** `art_gripper_interfaces/srv/SetTargetFingerWidthWithSpeed`
- **요청 (Request)**
  | 필드명 | 타입 | 설명 |
  | --- | --- | --- |
  | `finger_width` | `uint8` | 목표 핑거 너비. [mm], 0~100 |
  | `finger_width_speed` | `uint8` | 핑거 개폐 속도. [mm/s], 1~200(default:150) |

- **샘플 명령:**
  ```bash
  $ ros2 service call /set_target_finger_width_with_speed art_gripper_interfaces/srv/SetTargetFingerWidthWithSpeed "{'finger_width': 50, 'finger_width_speed': 100}"
  ```

---

## **ROS Publishers**

### **1. `GripperStatus`**

그리퍼의 현재 상태를 게시합니다.

- **토픽 이름:** `/gripper_status`
- **메시지 타입:** `art_gripper_interfaces/msg/GripperStatus`

| 필드명 | 타입 | 설명 |
| --- | --- | --- |
| `gripper_status` | `uint16` | 그리퍼의 현재 상태 |
| `finger_width` | `uint8` | 현재 핑거 너비 [mm] |
| `finger_pose` | `uint8` | 현재 핑거 회전 각도 [deg] |
| `status_word` | `uint16` | 상태 워드 |
| `position` | `int32[4]` | 각 모터의 위치 |
| `position_auxiliary` | `int32[4]` | 각 모터의 보조 위치 |
| `velocity` | `int16[4]` | 각 모터의 속도 |
| `current` | `int16[4]` | 각 모터의 전류 |

### **2. `EthercatState`**

EtherCAT 마스터의 상태를 게시합니다.

- **토픽 이름:** `/ethercat_state`
- **메시지 타입:** `art_gripper_interfaces/msg/EthercatState`

| 필드명 | 타입 | 설명 |
| --- | --- | --- |
| `slaves_responding` | `uint32` | 응답하는 슬레이브의 합계 |
| `al_states` | `uint8` | 모든 슬레이브의 애플리케이션 계층 상태 |
| `link_up` | `uint8` | 하나 이상의 이더넷 링크가 활성화된 경우 true |
| `working_counter` | `uint32` | 마지막 작업 카운터의 값 |
| `wc_state` | `uint8` | 작업 카운터 해석 |
| `redundancy_active` | `uint32` | 중복 링크가 사용 중임 |


## Dual-Gripper support with dual EtherCAT master

두 대의 gripper를 동시에 운용하기 위해 PC에 두 개의 EtherCAT master를 이용합니다.

### EtherCAT Master 설정

#### EtherCAT1 활성화

/opt/etherlab/etc/ethercat.conf 파일을 열어 다음과 같이 EtherCAT1을 활성화합니다.

```
MASTER0_DEVICE="04:bf:1b:27:de:00"
MASTER1_DEVICE="20:16:06:02:02:00"
```

여기서 MASTER0_DEVICE, MASTER1_DEVICE 값에는 실제 시스템에 설치된 네트워크 카드의 mac address를 입력합니다.

#### udev 설정 추가

/etc/udev/rules.d/99-ethercat.rules 를 열어 다음과 같이 EtherCAT1에 대한 권한을 추가합니다.

```bash
KERNEL=="EtherCAT0", MODE="0666"
KERNEL=="EtherCAT1", MODE="0666"
```

#### EtherCAT 서비스 재시작

```bash
$ sudo ethercatctl restart
```

#### EtherCAT 마스터 확인

다음과 같이 두 개의 EtherCAT Master가 활성화된 것을 확인합니다.

```bash
$ ehtercat-tool master

Master0
  Phase: Idle
  Active: no
  Slaves: 0
  Ethernet devices:
    Main: 04:bf:1b:27:de:75 (attached)
      Link: DOWN
      Tx frames:   0
      Tx bytes:    0
      Rx frames:   0
      Rx bytes:    0
      Tx errors:   0
      Tx frame rate [1/s]:      0      0      0
      Tx rate [KByte/s]:      0.0    0.0    0.0
      Rx frame rate [1/s]:      0      0      0
      Rx rate [KByte/s]:      0.0    0.0    0.0
    Common:
      Tx frames:   0
      Tx bytes:    0
      Rx frames:   0
      Rx bytes:    0
      Lost frames: 0
      Tx frame rate [1/s]:      0      0      0
      Tx rate [KByte/s]:      0.0    0.0    0.0
      Rx frame rate [1/s]:      0      0      0
      Rx rate [KByte/s]:      0.0    0.0    0.0
      Loss rate [1/s]:          0      0      0
      Frame loss [%]:         0.0    0.0    0.0
  Distributed clocks:
    Reference clock:   None
    DC reference time: 0
    Application time:  0
                       2000-01-01 00:00:00.000000000
Master1
  Phase: Idle
  Active: no
  Slaves: 0
  Ethernet devices:
    Main: 20:16:06:02:02:a1 (attached)
      Link: DOWN
      Tx frames:   0
      Tx bytes:    0
      Rx frames:   0
      Rx bytes:    0
      Tx errors:   0
      Tx frame rate [1/s]:      0      0      0
      Tx rate [KByte/s]:      0.0    0.0    0.0
      Rx frame rate [1/s]:      0      0      0
      Rx rate [KByte/s]:      0.0    0.0    0.0
    Common:
      Tx frames:   0
      Tx bytes:    0
      Rx frames:   0
      Rx bytes:    0
      Lost frames: 0
      Tx frame rate [1/s]:      0      0      0
      Tx rate [KByte/s]:      0.0    0.0    0.0
      Rx frame rate [1/s]:      0      0      0
      Rx rate [KByte/s]:      0.0    0.0    0.0
      Loss rate [1/s]:          0      0      0
      Frame loss [%]:         0.0    0.0    0.0
  Distributed clocks:
    Reference clock:   None
    DC reference time: 0
    Application time:  0
                       2000-01-01 00:00:00.000000000
```

### art gripper 시작

art_gripper 패키지에 새로 추가된 launch 파일을 이용하여 두개의 EtherCAT master와 연결되는 art gripper driver node를 실행할 수 있습니다.

각 노드 시작시 전달할 수 있는 추가적인 argument는 다음과 같습니다.

| 이름 | 타입 | 설명 | 종류 |
| --- | --- | --- | --- | 
| `master` | `uint8`  | EtherCAT master index (0, 1, ...) | 일반 |
| `__ns` | `string` | 노드 namespace (모든 topic 및 service 이름 remapping 사용) | ROS 아규먼트 |


다음은 두 대의 gripper를 동작하기 위해 두 개의 ROS nodes를 실행하는 art_gripper 패키지의 gripper_ecat_dual.launch.py 파일입니다.

launch 파일 실행시 두 개의 node(gripper_ecat_driver_left, gripper_ecat_driver_right)가 실행되며,

* gripper_ecat_driver_left node는 EtherCAT Master 0 와 연결되며, ag_left namespace를 사용
* gripper_ecat_driver_right node는 EtherCAT Master 1 와 연결되며, ag_right namespace를 사용

합니다.


```
#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():

    # create and return launch description object
    # ros2 run art_gripper gripper_ecat --ros-args -p gripper_status_publish_rate_hz:=1 -p ethercat_state_publish_rate_hz:=1
    return LaunchDescription(
        [
            Node(
                package="art_gripper",
                executable="gripper_ecat",
                name="gripper_ecat_driver_left",
                output="screen",
                parameters=[
                    {"gripper_status_publish_rate_hz": 1},
                    {"ethercat_state_publish_rate_hz": 1},
                ],
                arguments=["--master", "0", "--ros-args", "-r", "__ns:=/ag_left"],
            ),
            Node(
                package="art_gripper",
                executable="gripper_ecat",
                name="gripper_ecat_driver_right",
                output="screen",
                parameters=[
                    {"gripper_status_publish_rate_hz": 1},
                    {"ethercat_state_publish_rate_hz": 1},
                ],
                arguments=["--master", "1", "--ros-args", "-r", "__ns:=/ag_right"],
            )
        ]
    )

```

실행 후 다음과 같이 ROS2 서비스명을 확인할 수 있습니다.
```bash
$ ros2 service list
/ag_left/get_gripper_info
/ag_left/gripper_ecat_driver_left/describe_parameters
/ag_left/gripper_ecat_driver_left/get_parameter_types
/ag_left/gripper_ecat_driver_left/get_parameters
/ag_left/gripper_ecat_driver_left/list_parameters
/ag_left/gripper_ecat_driver_left/set_parameters
/ag_left/gripper_ecat_driver_left/set_parameters_atomically
/ag_left/motor_on
/ag_left/reset_abs_encoder
/ag_left/reset_friction_model
/ag_left/set_contact_sensitivity
/ag_left/set_gripping_force
/ag_left/set_target
/ag_left/set_target_current
/ag_left/set_target_finger_pose
/ag_left/set_target_finger_pose_speed
/ag_left/set_target_finger_pose_with_speed
/ag_left/set_target_finger_width
/ag_left/set_target_finger_width_speed
/ag_left/set_target_finger_width_with_speed
/ag_right/get_gripper_info
/ag_right/gripper_ecat_driver_right/describe_parameters
/ag_right/gripper_ecat_driver_right/get_parameter_types
/ag_right/gripper_ecat_driver_right/get_parameters
/ag_right/gripper_ecat_driver_right/list_parameters
/ag_right/gripper_ecat_driver_right/set_parameters
/ag_right/gripper_ecat_driver_right/set_parameters_atomically
/ag_right/motor_on
/ag_right/reset_abs_encoder
/ag_right/reset_friction_model
/ag_right/set_contact_sensitivity
/ag_right/set_gripping_force
/ag_right/set_target
/ag_right/set_target_current
/ag_right/set_target_finger_pose
/ag_right/set_target_finger_pose_speed
/ag_right/set_target_finger_pose_with_speed
/ag_right/set_target_finger_width
/ag_right/set_target_finger_width_speed
/ag_right/set_target_finger_width_with_speed
```

하나의 gripper node를 실행할 때와 다르게 왼손을 제어하기 위해서는 /ag_left namespace를, 오른손을 제어하기 위해서는 /ag_right namespace를 사용해야 합니다.

