# omp_tp — Core Features Summary

## 1. USV Simulation (Ignition Gazebo)

`tp_worlds/tp_world.sdf` 에 정의된 무인 수상정(USV) 시뮬레이션 환경.

- **모델 구성**: 선체(hull), 좌우 thruster, 카메라 센서로 이루어진 링크/조인트 구조
- **적용 플러그인**: Buoyancy, Hydrodynamics, Thruster, Camera
- Ignition Gazebo Fortress 위에서 동작하며, `ign gazebo tp_world.sdf` 로 실행

---

## 2. ROS 2 — Gazebo 브릿지 (`ros_gz_bridge`)

`omp_tp.launch.py` 에서 `parameter_bridge` 노드를 기동해 ROS 2와 Gazebo 간 토픽을 연결.

| 방향 | 토픽 | 메시지 타입 |
|------|------|-------------|
| Gazebo → ROS 2 | `/image` | `sensor_msgs/Image` |
| ROS 2 → Gazebo | `/my_boat/thrust1` | `std_msgs/Float64` |
| ROS 2 → Gazebo | `/my_boat/thrust2` | `std_msgs/Float64` |

---

## 3. 카메라 기반 장애물 감지

`_on_image()` 콜백에서 OpenCV로 실시간 처리.

```
BGR 이미지 수신
  → 그레이스케일 변환
  → 임계값(50) 이진화 (어두운 픽셀 추출)
  → 어두운 픽셀 수 > 10,000 이면 장애물로 판정
  → /gray_image 토픽으로 전처리 이미지 퍼블리시 (디버그용)
```

---

## 4. FSM 기반 장애물 회피 (`obs_avoid_3.py`)

두 상태(`NAVIGATING` / `AVOIDING`)와 고정 회피 시퀀스로 동작.

```
State.NAVIGATING
  ├─ 장애물 미감지 → 직진 (FWD_SPEED = 0.8)
  └─ 장애물 감지   → State.AVOIDING 전환

State.AVOIDING  (7단계 시퀀스, 각 단계 타이머로 전환)
  ① 좌회전  3.0 s
  ② 직진    7.0 s
  ③ 우회전  3.0 s
  ④ 직진    4.0 s
  ⑤ 우회전  3.0 s
  ⑥ 직진    7.0 s
  ⑦ 좌회전  3.0 s
  └─ 완료   → State.NAVIGATING 복귀
```

차동 추진(differential thrust)으로 방향 제어:
- 직진: `(+0.8, +0.8)`
- 좌회전: `(-1.5, +1.5)`
- 우회전: `(+1.5, -1.5)`

---

## 5. 노드 개발 이력

| 파일 | 설명 |
|------|------|
| `obs_avoid_1.py` | 초기 하드코딩 구현 — 8개 State Enum, 상태별 분기 |
| `obs_avoid_2.py` | 1차 리팩토링 |
| `obs_avoid_3.py` | 2차 리팩토링 — 시퀀스 리스트 기반 FSM, **현재 사용** |

---

## 6. 시스템 구성도

```
Ignition Gazebo
  └─ tp_world.sdf (USV 모델)
       ├─ Camera Plugin  ──────────► ros_gz_bridge ──► /image
       └─ Thruster Plugin ◄────────  ros_gz_bridge ◄── /my_boat/thrust1,2

obs_avoid_3 Node
  ├─ Subscribe: /image
  ├─ Publish:   /my_boat/thrust1, /my_boat/thrust2
  └─ Publish:   /gray_image

rviz2  ◄──── /gray_image (카메라 영상 모니터링)
```
