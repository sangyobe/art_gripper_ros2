## **ART Gripper ROS 서비스 문서**

이 문서는 `art_gripper` 패키지에서 제공하고 사용하는 ROS 서비스들에 대한 명세를 다룹니다.

---

### **공통 응답 (Common Response)**

대부분의 서비스는 결과 코드를 포함하는 동일한 형식의 응답을 반환합니다.

- **`response`**
  - `int32 result`: 서비스 호출의 결과를 나타냅니다. `0`은 성공을 의미하며, 다른 값은 오류를 나타냅니다.

---

### **1. `GetGripperInfo`**

그리퍼의 기본 정보를 요청합니다.

- **서비스 이름:** `/GetGripperInfo`
- **서비스 타입:** `art_gripper_interfaces/srv/GetGripperInfo`
- **요청 (Request):** 없음
- **응답 (Response)**
  | 필드명 | 타입 | 설명 |
  | --- | --- | --- |
  | `info` | `art_gripper_interfaces/msg/GripperInfo` | 그리퍼 정보 (이름, 버전, 설명) | 

- **샘플 명령:**
  ```bash
  ros2 service call /GetGripperInfo art_gripper_interfaces/srv/GetGripperInfo
  ```

### **2. `MotorOn`**

그리퍼의 모터를 켜거나 끕니다.

- **서비스 이름:** `/MotorOn`
- **서비스 타입:** `art_gripper_interfaces/srv/MotorOn`
- **요청 (Request)**
  | 필드명 | 타입 | 설명 |
  | --- | --- | --- |
  | `on` | `uint8` | 모터 상태. `1` for On, `0` for Off. |

- **샘플 명령:**
  ```bash
  # 모터 켜기
  ros2 service call /MotorOn art_gripper_interfaces/srv/MotorOn "{'on': 1}"

  # 모터 끄기
  ros2 service call /MotorOn art_gripper_interfaces/srv/MotorOn "{'on': 0}"
  ```

### **3. `ResetAbsEncoder`**

절대 엔코더를 리셋합니다.

- **서비스 이름:** `/ResetAbsEncoder`
- **서비스 타입:** `art_gripper_interfaces/srv/ResetAbsEncoder`
- **요청 (Request):** 없음

- **샘플 명령:**
  ```bash
  ros2 service call /ResetAbsEncoder art_gripper_interfaces/srv/ResetAbsEncoder
  ```

### **4. `ResetFrictionModel`**

마찰 모델을 리셋합니다.

- **서비스 이름:** `/ResetFrictionModel`
- **서비스 타입:** `art_gripper_interfaces/srv/ResetFrictionModel`
- **요청 (Request):** 없음

- **샘플 명령:**
  ```bash
  ros2 service call /ResetFrictionModel art_gripper_interfaces/srv/ResetFrictionModel
  ```

### **5. `SetContactSensitivity`**

그리퍼의 접촉 감도를 설정합니다.

- **서비스 이름:** `/SetContactSensitivity`
- **서비스 타입:** `art_gripper_interfaces/srv/SetContactSensitivity`
- **요청 (Request)**
  | 필드명 | 타입 | 설명 |
  | --- | --- | --- |
  | `contact_sensitivity` | `uint8` | 접촉 감도 값. (범위: 1 ~ 100, 기본값: 80) |

- **샘플 명령:**
  ```bash
  ros2 service call /SetContactSensitivity art_gripper_interfaces/srv/SetContactSensitivity "{'contact_sensitivity': 80}"
  ```

### **6. `SetGrippingForce`**

그리퍼가 물체를 잡는 힘(악력)을 설정합니다.

- **서비스 이름:** `/SetGrippingForce`
- **서비스 타입:** `art_gripper_interfaces/srv/SetGrippingForce`
- **요청 (Request)**
  | 필드명 | 타입 | 설명 |
  | --- | --- | --- |
  | `gripping_force` | `uint8` | 악력 값. 단위: N. (범위: 1 ~ 100, 기본값: 30) |

- **샘플 명령:**
  ```bash
  ros2 service call /SetGrippingForce art_gripper_interfaces/srv/SetGrippingForce "{'gripping_force': 50}"
  ```

### **7. `SetTarget`**

그리퍼의 여러 파라미터를 한번에 설정합니다.

- **서비스 이름:** `/SetTarget`
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
  ros2 service call /SetTarget art_gripper_interfaces/srv/SetTarget "{'finger_width': 50, 'finger_pose': 90, 'finger_width_speed': 100, 'finger_pose_speed': 150, 'gripping_force': 40, 'contact_sensitivity': 70}"
  ```

### **8. `SetTargetCurrent`**

각 모터의 목표 전류를 설정합니다.

- **서비스 이름:** `/SetTargetCurrent`
- **서비스 타입:** `art_gripper_interfaces/srv/SetTargetCurrent`
- **요청 (Request)**
  | 필드명 | 타입 | 설명 |
  | --- | --- | --- |
  | `target_current` | `int16[4]` | 4개 모터의 목표 전류. |

- **샘플 명령:**
  ```bash
  ros2 service call /SetTargetCurrent art_gripper_interfaces/srv/SetTargetCurrent "{'target_current': [100, 100, 100, 100]}"
  ```

### **9. `SetTargetFingerPose`**

목표 핑거 회전 각도를 설정합니다.

- **서비스 이름:** `/SetTargetFingerPose`
- **서비스 타입:** `art_gripper_interfaces/srv/SetTargetFingerPose`
- **요청 (Request)**
  | 필드명 | 타입 | 설명 |
  | --- | --- | --- |
  | `finger_pose` | `uint8` | 목표 핑거 회전 각도. [deg], 0~180 |

- **샘플 명령:**
  ```bash
  ros2 service call /SetTargetFingerPose art_gripper_interfaces/srv/SetTargetFingerPose "{'finger_pose': 90}"
  ```

### **10. `SetTargetFingerPoseSpeed`**

핑거의 회전(Pose) 속도를 설정합니다.

- **서비스 이름:** `/SetTargetFingerPoseSpeed`
- **서비스 타입:** `art_gripper_interfaces/srv/SetTargetFingerPoseSpeed`
- **요청 (Request)**
  | 필드명 | 타입 | 설명 |
  | --- | --- | --- |
  | `finger_pose_speed` | `uint8` | 핑거 회전 속도. [deg/s], 1~255(default: 180) |

- **샘플 명령:**
  ```bash
  ros2 service call /SetTargetFingerPoseSpeed art_gripper_interfaces/srv/SetTargetFingerPoseSpeed "{'finger_pose_speed': 100}"
  ```

### **11. `SetTargetFingerPoseWithSpeed`**

목표 핑거 회전 각도와 속도를 함께 설정하여 즉시 이동시킵니다.

- **서비스 이름:** `/SetTargetFingerPoseWithSpeed`
- **서비스 타입:** `art_gripper_interfaces/srv/SetTargetFingerPoseWithSpeed`
- **요청 (Request)**
  | 필드명 | 타입 | 설명 |
  | --- | --- | --- |
  | `finger_pose` | `uint8` | 목표 핑거 회전 각도. [deg], 0~180 |
  | `finger_pose_speed` | `uint8` | 핑거 회전 속도. [deg/s], 1~255(default: 180) |

- **샘플 명령:**
  ```bash
  ros2 service call /SetTargetFingerPoseWithSpeed art_gripper_interfaces/srv/SetTargetFingerPoseWithSpeed "{'finger_pose': 90, 'finger_pose_speed': 100}"
  ```

### **12. `SetTargetFingerWidth`**

목표 핑거 너비를 설정합니다.

- **서비스 이름:** `/SetTargetFingerWidth`
- **서비스 타입:** `art_gripper_interfaces/srv/SetTargetFingerWidth`
- **요청 (Request)**
  | 필드명 | 타입 | 설명 |
  | --- | --- | --- |
  | `finger_width` | `uint8` | 목표 핑거 너비. [mm], 0~100 |

- **샘플 명령:**
  ```bash
  ros2 service call /SetTargetFingerWidth art_gripper_interfaces/srv/SetTargetFingerWidth "{'finger_width': 50}"
  ```

### **13. `SetTargetFingerWidthSpeed`**

그리퍼 핑거의 개폐(Width) 속도를 설정합니다.

- **서비스 이름:** `/SetTargetFingerWidthSpeed`
- **서비스 타입:** `art_gripper_interfaces/srv/SetTargetFingerWidthSpeed`
- **요청 (Request)**
  | 필드명 | 타입 | 설명 |
  | --- | --- | --- |
  | `finger_width_speed` | `uint8` | 핑거 개폐 속도. [mm/s], 1~200(default:150) |

- **샘플 명령:**
  ```bash
  ros2 service call /SetTargetFingerWidthSpeed art_gripper_interfaces/srv/SetTargetFingerWidthSpeed "{'finger_width_speed': 100}"
  ```

### **14. `SetTargetFingerWidthWithSpeed`**

목표 핑거 개폐 너비와 속도를 함께 설정하여 즉시 이동시킵니다.

- **서비스 이름:** `/SetTargetFingerWidthWithSpeed`
- **서비스 타입:** `art_gripper_interfaces/srv/SetTargetFingerWidthWithSpeed`
- **요청 (Request)**
  | 필드명 | 타입 | 설명 |
  | --- | --- | --- |
  | `finger_width` | `uint8` | 목표 핑거 너비. [mm], 0~100 |
  | `finger_width_speed` | `uint8` | 핑거 개폐 속도. [mm/s], 1~200(default:150) |

- **샘플 명령:**
  ```bash
  ros2 service call /SetTargetFingerWidthWithSpeed art_gripper_interfaces/srv/SetTargetFingerWidthWithSpeed "{'finger_width': 50, 'finger_width_speed': 100}"
  ```

---

### **ROS Publishers**

#### **1. `GripperStatus`**

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

#### **2. `EthercatState`**

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
