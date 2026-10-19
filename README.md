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
