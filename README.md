# robomation

<p>
  <a href="https://pypi.org/project/robomation/"><img src="https://img.shields.io/badge/python-3.9+-blue.svg" alt="Python versions"></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-LGPL--2.1--or--later-blue.svg" alt="License"></a>
</p>

로보메이션(Robomation) 로봇과 AI 확장모듈을 위한 파이썬 패키지입니다.  
햄스터 S, 햄스터, 삐오봇, 터틀, Beagle, 라쿤봇, 치즈스틱 등 로보메이션 로봇을 파이썬 코드로 제어하고,  
카메라·마이크 기반의 AI 인식 기능(얼굴/손/몸/사물/색깔/마커 인식, 음성 인식, 자율주행)을 손쉽게 사용할 수 있습니다.

Windows / macOS / Chromebook 어느 환경에서든, VSCode 같은 파이썬 환경에서 바로 동작합니다.

<br>

## 설치

```bash
pip install -U robomation
```

<br>

### 요구 사항

- Python 3.9 이상
- 로봇 사용 시: USB 동글 또는 시리얼 연결
- AI 모듈 사용 시: 카메라 / 마이크, 그리고 OS의 카메라·마이크 권한 허용

<br>

주요 의존성(`pip install -U robomation` 시 자동 설치됩니다):  
`pyserial`, `numpy`, `opencv-contrib-python`, `mediapipe`, `SpeechRecognition`, `sounddevice` (Windows는 `pygrabber` 추가).

<br>

## 지원 라이브러리

### 공통

| 모듈 | 설명 |
|---|---|
| Utils | 유틸리티 |

### 로봇

| 모듈 | 설명 |
|---|---|
| HamsterS | 햄스터 S |
| Hamster | 햄스터 |
| Pio | 삐오봇 |
| Turtle | 터틀 |
| Beagle | 비글 |
| RaccoonBot | 라쿤봇 |
| CheeseStick | 치즈스틱 |

### AI 확장모듈

| 모듈 | 설명 |
|---|---|
| ASR | 음성 인식 |
| FaceDetection | 얼굴 찾기 |
| DetailedFaceDetection | 상세하게 얼굴 찾기 |
| FaceExpression | 나이, 성별, 표정 |
| HandDetection | 손 찾기 |
| BodyDetection | 몸 찾기 |
| ObjectDetection | 사물 인식 찾기 |
| ColorDetection | 색깔 찾기 |
| ArucoMarker | ArUco 마커 찾기 |
| SelfDriving | 카메라 자율주행하기 |

### 치즈스틱 확장모듈

| 모듈 | 설명 |
|---|---|
| CSD01 | 스위치 |
| CSD02 | RGB LED |
| CSD03 | 로터리 퍼텐쇼미터 |
| CSD07 | 소리 센서 |
| CSD09 | 모터 |
| CSD10 | 조도 센서 |
| NeoPixel | 네오픽셀 |
| HAT010 | HAT-010 5x5 매트릭스 |
| HAT022 | HAT-022 터치 피아노 |
| PID13 | PID-13 조이스틱과 버튼 |
| PID26 | PID-26 환경 센서 |

<br>

## 사용법 (Usage)

### 로봇 제어

```python
from robomation import *

hamster_s = HamsterS()
hamster_s.set_wheel_speed('both', 30)   
```

#### 인스턴스 선언

```python
hamster_s = HamsterS()                                 # 자동 탐색
hamster_s = HamsterS(0)                                # 인스턴스 번호
hamster_s = HamsterS("COM3")                           # 포트 이름
hamster_s = HamsterS("D9:4B:8B:A4:E1:67")              # 제품 고유 주소
hamster_s = HamsterS(0, port_name="COM3")              # 번호 + 포트
hamster_s = HamsterS(0, address="D9:4B:8B:A4:E1:67")   # 번호 + 주소
hamster_s = HamsterS(0, "COM3", "D94B8BA4E167")        # 번호 + 포트 + 주소
```

| 이름 | 구분 | 설명 | 범위 / 종류 | 기본값 |
| --- | --- | --- | --- | --- |
| index | 입력값 | 인스턴스 번호 (0부터 시작) | 0 이상 정수 | 0 |
| port_name | 입력값 | 연결할 포트 이름. 해당 포트만 사용. | 문자열 (`COM3`, `/dev/cu.usbmodem1101` 등) | None (자동 탐색) |
| address | 입력값 | 연결할 로봇의 제품 고유 주소. 주소가 일치하는 로봇만 연결. | 문자열 (16진수 12자리, `:` 는 생략 가능) | None (기종만 확인) |

제품 고유 주소는 로봇이 연결될 때 출력되는 메시지에서 확인할 수 있습니다.
제품 라벨에 인쇄된 주소와 같습니다.

```
HamsterS[0] Connected: COM3 D9:4B:8B:A4:E1:67
                            └── 이 값을 그대로 입력합니다
```

로봇을 2대 이상 사용할 때는 **인스턴스 번호를 서로 다르게 지정해야 합니다.**

```python
hamster_s1 = HamsterS(0, address="D9:4B:8B:A4:E1:67")
hamster_s2 = HamsterS(1, address="AA:BB:CC:DD:EE:FF")
```

제품 고유 주소를 지정하지 않으면 어느 인스턴스가 어느 로봇에 연결될지 실행할 때마다
달라질 수 있습니다. 여러 대의 로봇을 사용할 때는 주소를 지정하는 것을 권장합니다.

지정한 주소의 로봇이 없으면 **다른 로봇에 연결되지 않고** 그 인스턴스만 연결에 실패하며,
연결에 성공한 다른 로봇들은 정상적으로 동작합니다.

### AI 확장 모듈 — 얼굴 인식

```python
from robomation import *

face_detection = FaceDetection(0)
face_detection.device(0)                
face_detection.load_model()             
face_detection.detect_continuous()      

while True:
    if face_detection.detected():
        print(face_detection.face('x'), face_detection.face('y'))
    Utils.wait(0.1)                 
```

### 치즈스틱 확장 모듈 - 네오픽셀

```python
from robomation import *

cheesestick = CheeseStick()
neopixel = cheesestick.NeoPixel()

neopixel.set_range_pattern(1, 1, '3_colors')
```
<br>

## 참고 사항 (Notes)

- 카메라/마이크는 OS 권한 허용이 필요합니다.

<br>

## 문서 (Documentation)

자세한 사용법과 예제는 [Github Wiki](https://github.com/RobomationLAB/Robomation_Python/wiki)에서 확인할 수 있습니다.  

<br>

## 라이선스 (License)

본 라이브러리는 **GNU Lesser General Public License v2.1 이상**(LGPL-2.1-or-later) 하에 배포됩니다.  
자세한 내용은 [LICENSE](LICENSE) 파일을 참고하세요.  

Copyright (c) 2016 Kwang-Hyun Park.  
Copyright (c) 2026 Robomation.