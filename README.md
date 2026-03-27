# TeamKoreaPhysicalAI Testbed Simulator

OHT + AGV + 3DS Shuttle 통합 시뮬레이터.
TAPG(Temporal Action Precedence Graph) 기반 DES(Discrete Event Simulation)로 다중 AGV 교착 회피를 구현합니다.

## Quick Start

```bash
git clone https://github.com/ChungChanghyun/TeamKoreaPhysicalAI_Testbed.git
cd TeamKoreaPhysicalAI_Testbed
pip install -r requirements.txt
python vis_combined.py
```

## 조작법

| 키 | 기능 |
|----|------|
| SPACE | 시작 / 일시정지 |
| R | 리셋 |
| S | 경로 재배치 (Shuffle) |
| O / L | OHT 차량 추가 / 제거 |
| N / P | AGV 차량 추가 / 제거 |
| +/- | 시뮬레이션 속도 조절 |
| 마우스 드래그 | 화면 이동 |
| 휠 | 줌 |
| Q / ESC | 종료 |

## 문서

- [TAPG DES 엔진 명세](TAPG_DES_SPEC.md) — DES 이벤트 정의, 상태 머신, TAPG 구조
- [시뮬레이터 가이드](SIMULATOR_GUIDE.md) — 사용법 안내
- [OHT DES 명세](OHT_DES_SPEC.md) — OHT 세그먼트 큐 + 전방감지 모델

## 주요 파일

| 파일 | 역할 |
|------|------|
| `vis_combined.py` | 통합 시뮬레이터 (메인 진입점) |
| `env_tapg.py` | TAPG 기반 AGV DES 엔진 |
| `env_oht_des.py` | OHT DES 환경 (세그먼트 큐 + 전방감지) |
| `env_3ds.py` | 3DS 셔틀 환경 |
| `elevator.py` | 층간 엘리베이터 DES |
| `pkl_loader.py` | 충돌 프로파일(.pkl) 로더 |
| `pkl_prioritized_planner.py` | Prioritized SIPP 경로 플래너 |
| `gen_collision_profile.py` | 충돌 프로파일 생성 (회전 포함) |
| `gen_collision_profile_no_rot.py` | 충돌 프로파일 생성 (회전 없음) |

## 의존 패키지

```
networkx, shapely, rtree, scipy, matplotlib, numpy, openpyxl, pygame
```
