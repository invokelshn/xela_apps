# Phase 0 조사 결과 — index.html 내 Operator 코드/상태/토픽 인벤토리

대상: `xela_taxel_sidecar_dg5f/web/taxel_sidecar/index.html` (4375줄), 2026-09-09 조사.
목적: `xela_taxel_operator_dg5f` 신규 구현 시 무엇을 옮기고(이식), 무엇을 새로 설계해야 하는지(재설계) 판단하기 위한 근거 자료. dg5f 원본은 무수정, 읽기 전용 조사만 수행.

## 1. Operator 전용 코드 구간

| 구분 | 이름 | 대략 줄 번호 |
|---|---|---|
| CSS | `.operator-only`, `.app.operator-active .dev-only`, `#operatorAlertsHost`, `#operatorGraphHost .taxel-session-*`, `.operator-viz-row` | 135–213, 367–377 |
| DOM | `operatorBtn`, `operatorStatusBarHost`, `operatorModulesHost`, `operatorVizControlsHost`, `.operator-viz-row`, `operatorAlertsHost`, `operatorMainViewMarkersHost`, `operatorGraphHost`, `operatorFilmstripHost` | 559, 655–712 |
| import | `js/operator/*.js` 6개 컴포넌트 factory | 725–730 |
| 상수/상태 | `OPERATOR_VIEW_MARKER_*`, `operatorViewMarkerState` | 1234–1292 |
| 함수 | `buildOperatorViewMarker` | 1241–1287 |
| 함수 | `publishOperatorViewMarkerMsg` | 1297–1298 |
| 함수 | `deleteOperatorViewMarkerSlotId` | 1310–1327 |
| 함수 | `operatorViewMarkerLifetimeMsFor` | 1332~ |
| 함수 | `publishOperatorViewMarker` | 1343–1367 |
| 함수 | `republishOperatorViewMarkerSlotsWithLayout` | 1371–1379 |
| 함수 | `clearAllOperatorViewMarkerSlots` | 1381–1388 |
| 인스턴스 초기화 | `operatorStatusBar/ModuleSelect/VizControls/AlertCards/MainViewMarkerSettings/Filmstrip` | 3419–3492 |
| 함수 | `operatorAdjustNorm` | 3494–3500 |
| 상태/함수 | `operatorGraphsShown`, `setOperatorGraphsShown` | 3365–3410 |
| 상태/함수 | `preOperatorUiMode`, `preOperatorSensorViewMode`, `setOperatorMode` | 3572–3617 |
| 이벤트 배선 | grasp_event 핸들러 내 Operator 컴포넌트 콜백 | 3212–3227 |
| tick 루프 | `operatorStatusBar?.updateHealth`, `operatorFilmstrip?.tick` | 4038–4047 |
| 이벤트 바인딩 | `operatorBtn` 클릭 → `setOperatorMode` | 4166–4167 |

외부 파일(별도, 심볼릭 링크): `operator_status_bar.js`, `operator_module_select.js`, `operator_viz_controls.js`, `operator_alert_cards.js`, `operator_main_view_markers.js`, `operator_filmstrip.js`.

## 2. 공유 state 필드

Operator 전용 지역 상태(`operatorViewMarkerState`, `operatorGraphsShown`, `operatorSensitivityLevel`, `preOperatorUiMode`, `preOperatorSensorViewMode`)는 모듈 스코프로 분리돼 있으나, 아래 전역 `state` 필드는 Admin과 공유:

| 필드 | 공유 여부 |
|---|---|
| `state.uiMode` | 공유 — Admin 전역에서 읽고 씀 |
| `state.showModules` | Admin 소유, Operator는 비교/회피 목적으로만 참조 |
| `state.payload` | 공유 — Admin도 파싱 |
| `state.sensorViewMode` | 공유 — `setOperatorMode` 진입/이탈 시 저장·복원 |
| `state.ws`(간접, rosbridgeClient 경유) | 공유(공용 WebSocket 연결) |

## 3. Operator DOM

`#operatorBtn`, `.operator-only`, `#operatorStatusBarHost`, `#operatorModulesHost`, `#operatorVizControlsHost`, `.operator-viz-row`, `#operatorAlertsHost`, `#operatorMainViewMarkersHost`, `#operatorGraphHost`, `#operatorFilmstripHost`, `.app.operator-active`(Admin/Operator 전환 핵심 게이트 클래스).

## 4. rosbridge 토픽/서비스

| 토픽 | 용도 | Operator 전용 여부 |
|---|---|---|
| `/atag/grasp_event` | Operator 상태바/알림카드/필름스트립 갱신 + 뷰마커 발행 트리거 | 구독 자체는 공용, Operator 렌더링만 조건부 |
| `/visual_markers` (publish) | Operator 뷰마커 표시 | Operator 전용 발행 (Admin/Operator 모드 무관하게 항상 발행되도록 변경됨, 2026-09-08) |
| `robotDescriptionTopic`, `tfTopic`, `tfStaticTopic` | Admin 3D URDF/TF 렌더링 | **Admin 전용** — Operator 코드는 직접 구독하지 않음 |

## 5. 캐시(`frameAliasCache`/`materialCache`)와 Operator의 관계 — 중요 발견

두 캐시는 Admin 전용 3D URDF 렌더 파이프라인(`state.urdfSources`, `urdfMesh` 등)에서만 사용되고 Operator 함수들은 이를 전혀 참조하지 않는다. **그러나 Operator 모드는 별도의 3D 뷰를 갖지 않고, Admin의 3D 캔버스/렌더 파이프라인을 "그대로 재사용"한다(index.html 3568행 주석: "동일한 XelaModel 3D 뷰/캔버스를 재사용").**

→ 이는 기존 계획서 Phase 2에서 세운 가정("운영자 전용으로 더 가벼운 TF/마커 계산 재설계")에 대한 중요한 전제 수정이 필요함을 뜻함: 지금 Operator 화면이 "가볍다"고 느껴지는 부분은 오버레이 위젯(상태바/필름스트립/알림카드)뿐이고, 그 뒤에 깔리는 3D 뷰 자체는 Admin과 동일한 `frameAliasCache`/`materialCache`/URDF 렌더러를 그대로 쓰고 있다. 따라서 `xela_taxel_operator_dg5f`가 실제로 성능 이득을 보려면, Operator 전용 3D 뷰(또는 더 단순화된 뷰)까지 Phase 2 범위에 명시적으로 포함해야 하며, 단순히 오버레이 위젯만 옮기는 것으로는 목표한 성능 개선이 나오지 않을 가능성이 높다.

**⚠️ 2026-09-10 추가 정정**: 위 5절의 결론("Operator 전용 3D 뷰를 더 가볍게(축소해서) 재설계해야 함")은 이후 폐기됨. 개별 taxel 센서 마커(124개)는 운영자 핵심 기능이라 축소 불가로 정정, 실제 재설계 대상은 taxel/모듈 개수가 아니라 Admin 전용 오버헤드(2D grid, follow-cam, raw 디버그 패널) 제거 쪽으로 방향이 바뀜. 최신 결론은 `dg5f_operator_split_phase2_design.md`(v2) 참고. 이 문서(Phase 0)의 나머지 사실 인벤토리(1~4절, 코드 위치/줄번호)는 여전히 유효함.

## Phase 0 체크리스트 결과

- [x] 두 신규 패키지 디렉토리 위치/이름 확정 (`xela_apps/xela_taxel_viz_core`, `xela_apps/xela_taxel_operator_dg5f`)
- [x] Operator 코드 구간 및 참조 state 필드 목록 작성 완료 (본 문서)
- [x] 운영자 전용 토픽/서비스 목록 확정 (본 문서 4절) — 단, 3D 뷰 관련 토픽(robotDescription/tf)이 실제로는 Operator 화면에도 필요하다는 점이 새로 확인됨 (5절)
- [x] `xela_taxel_sidecar_dg5f` 무수정 확인 — 본 조사는 전부 읽기 전용, `git status`로 변경 없음 확인
- [ ] **사용자 확인 필요**: 5절의 발견(Operator가 Admin 3D 뷰를 재사용 중)에 따라 Phase 2 범위를 어떻게 조정할지 결정
