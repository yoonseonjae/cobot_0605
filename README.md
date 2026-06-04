# Isaac Sim 디지털 트윈 기반 화재 대응 로봇 자동화 시뮬레이션 (cobot_0605)
> **조 이름:** [A-1 - ROKEY]
> **팀원:** [윤선재_김두산_이로키_박캠프]

*※ 본 프로젝트는 현재 진행 중인 미완성 프로젝트로, Isaac Sim 내에서 다중 Spot 로봇과 매니퓰레이터의 물리적 안정화 및 조작을 구현하는 과정에 있습니다.*

---

## 1. 📌 프로젝트 개요
* **주제:** 디지털 트윈 기반 화재 대응 로봇 자동화 시뮬레이션
* **환경:** Isaac Sim 5.1 / Ubuntu 22.04 / ROS2 Humble / Python 3.11

---

## 2. 🎭 시나리오 및 역할 분담
화재가 발생한 건물 1층에서 2대의 `Spot with Arm` 로봇이 역할을 분담하여 대응합니다.

### 🚑 Robot 1 (인명구조)
- [ ] 평소 PATROL 상태로 순찰
- [ ] 화재 트리거(`/fire_alert` 토픽) 수신
- [ ] 산소마스크 위치로 Nav2 이동
- [x] **Fake Grasp로 마스크 Pick** (등에 고정된 큐브를 EE로 집음)
- [ ] YOLO로 사람 탐색 (플랜B: ArUco)
- [ ] 사람 발견 → 마스크 전달 (Place)
- [ ] 출구까지 경로 안내 (사람 follower 스크립트)

### 🧯 Robot 2 (화재진압)
- [ ] 평소 PATROL 상태로 순찰
- [ ] 화재 트리거 수신
- [ ] 소화기 위치로 Nav2 이동
- [x] **Fake Grasp로 소화기 Pick** (등에 고정된 큐브를 EE로 집음)
- [ ] 화재 방으로 이동
- [ ] RL 학습된 `.pt`로 소화기 투척
- [ ] 빨간 큐브(불꽃)에 Contact 감지 → invisible 처리 (진압 연출)

---

## 3. 🛠️ 확정된 기술 스택
* **로봇:** `spot_with_arm.usd` × 2 (멀티로봇 네임스페이스 분리 `/robot1`, `/robot2`)
* **보행:** IsaacLab 사전학습 Spot Flat Terrain Policy (`.pt`)
* **팔 제어:** Joint Space Lerp 직접 제어 (MoveIt2 미사용)
* **Pick & Place:** Fake Grasp (USD Reparent, `keep_transform=True`) 및 Base Lock(동상 모드) 적용
* **RL 설계 (투척):** IsaacLab Manager-Based RL Env 활용 (Action: release 시점의 velocity vector)
* **비전/인지:** YOLO (플랜 B: ArUco 병행)

---

## 4. 🚀 10일 개발 로드맵 및 진행 상황

### 🟢 1일차 (Day 1) 세션 진행도
- [x] **Session 1:** `spot_with_arm` DOF 확인 + EE가 등에 닿는지 확인 (최우선)
- [x] **추가 달성:** 물리 엔진 폭발(Physics Explosion) 방지를 위한 동상 모드(Base Lock) 완벽 구현
- [ ] **Session 2:** usdz 맵 임포트 및 충돌 메쉬 검증
- [ ] **Session 3:** Spot `.pt` 보행 예제 실행 확인
- [ ] **Session 4:** YOLO 설치 및 사람 모델 탐지 테스트
- [x] **Session 5:** 오브젝트 배치 및 큐브 등 고정 테스트 (마스크, 소화기 구현)
- [ ] **Session 6:** Day 1 결과 정리 및 Day 2 계획 확정

### 📅 전체 일정
- **Day 1:** 환경 검증 (GO/NOGO) **← (현재 진행 중)**
- **Day 2:** 단일 Spot 기초 세팅
- **Day 3:** 단일 Spot 핵심 기능 구현
- **Day 4:** Robot 1 완성 + RL 학습 시작 (headless, 256 envs 병렬)
- **Day 5:** Robot 2 구현 + RL 모니터링
- **Day 6:** 멀티 로봇 통합
- **Day 7:** 전체 시나리오 1회 완주
- **Day 8:** RL `.pt` 적용 + 버그 수정
- **Day 9:** 안정화 + 시연 준비
- **Day 10:** 최종 리허설 + 발표

---

## 5. 🎨 시스템 설계 및 플로우 차트
*(추후 완성된 아키텍처 이미지로 교체 요망)*

### 5-1. 시스템 설계도 (System Architecture)
<p align="center">
  <img src="./images/system_design.png" alt="시스템 설계도 이미지" width="400">
</p>

### 5-2. 플로우 차트 (Flow Chart)
<p align="center">
  <img src="./images/flow_chart.png" alt="플로우 차트 이미지" width="300" height="300">
</p>

---

## 6. ▶️ 실행 순서 (Usage Guide)
현재까지 구현된 **Spot Arm 기초 테스트 (Pick & Place 및 Base Lock)** 실행 방법입니다.

### Step 1. 시뮬레이션 환경 및 로봇 실행
로봇을 스폰하고 가상의 작업 환경을 엽니다.
```bash
./run_isaac.sh applications/stow_cubes.py
```

### Step 2. 로봇 제어 (키보드 조작)
Isaac Sim 뷰포트 창을 클릭한 후, 아래 키보드 단축키를 이용해 기능을 실행합니다.

* `*` (숫자패드 별표) : **동상 모드 (Base Lock) 토글** 
  *(팔이 움직일 때 로봇 베이스가 흔들리지 않도록 가상의 핀을 월드에 꽂아 물리적으로 굳게 고정합니다.)*
* `1` : 마스크(Mask) 집기 (Pick)
* `2` : 소화기(Extinguisher) 집기 (Pick)
* `3` : 마스크 놓기 (Place)
* `4` : 소화기 놓기 (Place)
