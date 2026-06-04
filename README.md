# Spot Arm Pick & Place 시뮬레이션 구축 (cobot_0605)
> **조 이름:** [A-1 - ROKEY]
> **팀원:** [윤선재_김두산_이로키_박캠프] *(본인 및 팀원 이름으로 수정해주세요)*

*※ 본 프로젝트는 현재 진행 중인 미완성 프로젝트로, Isaac Sim 내에서 Spot 로봇과 매니퓰레이터의 물리적 안정화 및 조작을 구현하는 과정에 있습니다.*

---

## 1. 🎨 시스템 설계 및 플로우 차트
프로젝트의 전체적인 구조와 소프트웨어 흐름도입니다.

### 1-1. 시스템 설계도 (System Architecture)
<p align="center">
  <img src="./images/system_design.png" alt="시스템 설계도 이미지" width="400">
</p>
* *설명: Isaac Sim 환경 내에서 Spot 4족 보행 로봇과 7자유도 매니퓰레이터의 물리(PhysX) 시뮬레이션 및 제어 통신 구조를 나타냅니다. (추후 완성된 아키텍처 이미지로 교체 요망)*

### 1-2. 플로우 차트 (Flow Chart)
<p align="center">
  <img src="./images/flow_chart.png" alt="플로우 차트 이미지" width="300" height="300">
</p>
* *설명: 시뮬레이션 환경 로드, 동상 모드(Base Lock) 활성화, 물체 인식 및 Pick & Place 시퀀스 진행 프로세스를 나타냅니다. (추후 완성된 플로우 차트로 교체 요망)*

---

## 2. 🖥️ 운영체제 환경 (OS Environment)
이 프로젝트는 다음 환경에서 개발하였습니다.

* **OS:** Ubuntu 22.04 LTS
* **Simulator:** NVIDIA Isaac Sim 5.1.0
* **Language:** Python 3.10
* **IDE:** VS Code

---

## 3. 🛠️ 사용 장비 목록 (Hardware/Software List)
프로젝트(시뮬레이션)에 사용된 주요 구성 요소입니다.

| 장비/모델명 (Model) | 수량 | 비고 |
|:---:|:---:|:---|
| Spot (4족 보행 베이스) | 1 | Isaac Sim USD 모델 |
| 7-DOF Robotic Arm | 1 | Spot 등 위에 장착된 매니퓰레이터 |
| 가상 오브젝트 (Cube) | 2 | 마스크(Mask), 소화기(Extinguisher) 구현용 |

---

## 4. 📦 의존성 (Dependencies)
프로젝트 실행에 필요한 라이브러리입니다.

* Python 3.10 (Isaac Sim 내장 환경)
* isaacsim (Isaac Sim Core & Robot API)
* pxr (USD 조작 및 물리 엔진 인터페이스)
* numpy

---

## 5. ▶️ 실행 순서 (Usage Guide)
프로젝트를 실행하기 위한 순서입니다. 터미널 명령어를 순서대로 입력해 주세요.

### Step 1. 시뮬레이션 환경 및 로봇 실행
로봇을 스폰하고 가상의 작업 환경을 엽니다.
```bash
./run_isaac.sh applications/stow_cubes.py
```

### Step 2. 로봇 제어 (키보드 조작)
Isaac Sim 뷰포트 창을 클릭한 후, 아래 키보드 단축키를 이용해 기능을 실행합니다.

* `*` (숫자패드 별표) : **동상 모드 (Base Lock) 활성화/비활성화** 
  *(팔이 움직일 때 로봇 베이스가 흔들리지 않도록 가상의 핀(FixedJoint)을 월드에 꽂아 고정합니다.)*
* `1` : 마스크(Mask) 집기 (Pick)
* `2` : 소화기(Extinguisher) 집기 (Pick)
* `3` : 마스크 놓기 (Place)
* `4` : 소화기 놓기 (Place)
