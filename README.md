# NeuroVector Drone

드론 하드웨어 제어 계층(HAL)입니다. C 펌웨어 레이어와 C++ 비행 제어 로직을 함께 제공합니다.

- **C11** — PWM/ESC 출력 (`nv_pwm`)
- **C++20** — 모터 믹싱, IMU, 비행 컨트롤러 (`drone_hal`)
- **Conan 2** — 의존성/패키지 관리
- **CMake 3.25+**

## 구조

```
include/neurovector/drone_hal.hpp   # C++ HAL 공개 헤더
src/drone_hal.cpp                   # 모터/IMU/비행 컨트롤러 구현
firmware/include/nv_pwm.h           # C PWM API
firmware/src/nv_pwm.c               # C PWM 구현
app/main.cpp                        # 데모 실행 파일
tests/test_drone_hal.cpp            # 단위 테스트
test_package/                       # conan create 검증용
conanfile.py                        # Conan 2 레시피
```

## 요구 사항

- CMake 3.25+
- C/C++ 컴파일러 (GCC 14 / Clang 18 이상 권장)
- [Conan 2](https://docs.conan.io/2/)

```bash
pip install conan
# 또는 릴리스 바이너리 사용
# https://github.com/conan-io/conan/releases
```

## 빌드

Conan 프로필이 없다면 먼저 생성합니다.

```bash
conan profile detect --force
```

### 일반 CMake 빌드

```bash
cmake -S . -B build -DNEUROVECTOR_BUILD_TESTS=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/drone_demo
```

### Conan 으로 빌드

```bash
conan build .
```

### 패키지 생성 + test_package 검증

```bash
conan create .
```

### 로컬 캐시에 패키징만

```bash
conan export-pkg .
```

## 소비하는 쪽 (Conan)

```python
# conanfile.py
def requirements(self):
    self.requires("neurovector-drone/0.1.0")

def build(self):
    cmake = CMake(self)
    cmake.configure()
    cmake.build()
```

```cmake
find_package(neurovector-drone REQUIRED)
target_link_libraries(my_app PRIVATE neurovector::drone_hal neurovector::nv_pwm)
```

## 안전 장치

- 시동(`arm`) 전에는 스로틀 변경이 무시됩니다.
- `disarm` 시 모든 모터 출력이 0 이 됩니다.
- PWM 값은 1000–2000µs 로 클램프됩니다.
- 전력은 `DroneConfig::max_power_watts` 로 제한됩니다.

## 라이선스

MIT

## 경로 교차 검증과 로컬 회피 판단

- [독립 임무 검증기](docs/mission.md): 백엔드 계획의 거리·에너지·금지구역·창문 바깥 정지 지점을 C++에서 다시 계산합니다.
- [위험도별 회피 판단](docs/avoidance.md): 1·2·3단계 감지와 실제 남은 경로, 거리·상대 속도·관측 신선도를 이용해 진행·감속·정지·재계획을 판단합니다.
- 공개 헤더: `neurovector/mission.hpp`, `neurovector/avoidance.hpp`
- [창문 층수·GPS·돌풍·Wi-Fi 통합 판단](docs/delivery-navigation.md): 기준층부터 창문 행을 확인하고 GPS 오차 내에서 목표 창문을 선택합니다. `nv_delivery_demo`로 합성 통합 시나리오를 실행할 수 있습니다. 실제 센서·모델·비행 제어 연결은 별도 필요합니다.
- CMake 대상: `neurovector::nv_mission`, `neurovector::nv_avoidance`, 실행 파일 `nv_route_check`

현재 HAL에는 임무 통신·GPS 항법·자동 비행 제어기가 없으므로 새 계층은 계산과 결정을 반환합니다.
상위 항법 계층이 검증된 경로와 실시간 관측을 연결한 뒤 명령을 실행해야 합니다. 기존 데모의
`arm()`을 경로 검증이나 회피 실행의 대체물로 취급하지 않습니다.
