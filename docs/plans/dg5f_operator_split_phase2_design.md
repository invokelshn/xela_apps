# Phase 2 설계 — 운영자 전용 3D 뷰 (v2, 2026-09-10 전면 정정)

**이 문서의 v1(2026-09-09 최초 작성)은 폐기합니다.** v1은 "Admin의 124개 taxel 프레임 대비 Operator는 16개 모듈 링크만 필요하므로 87% 감소"라고 주장했으나, 사용자 확인 과정에서 두 가지가 잘못됐음이 드러남:

1. **개별 taxel 센서 마커(접촉점/힘 벡터, 124개)는 운영자에게 축소해서는 안 되는 핵심 기능.** 사용자 확인: "센서정보는 운용자에게 가장 필요한 기능중 하나". 실측 결과 현재도 Admin/Operator 구분 없이 124개 taxel 전부가 3D 뷰에 점+화살표로 표시되고 있었고(`updateUrdfMarkers`), Operator에서 차단된 건 grid(2D 숫자) 뷰뿐이었음.
2. **"124→16 감소"라는 수치 자체가 서로 다른 두 개념을 혼동한 것.** 124는 taxel 센서 프레임 개수(마커용, 축소 불가), 16은 하우징 mesh(손가락/패치 외피 STL) 개수인데, 하우징은 애초부터 124개가 아니라 16개였음(xacro 확인). 즉 실제로는 아무것도 줄어들지 않음 — Admin과 Operator는 결국 동일하게 16개 mesh + 124개 taxel 마커를 그려야 함.

## 1. 성능 이득의 진짜 원천 (재정의)

taxel/모듈 개수를 줄이는 게 아니라:
1. TF 해석(`resolveChildFrameAlias`, `resolveFrameToFixed` 등) + taxel 마커 렌더링(`updateUrdfMarkers` 등)을 **무수정으로 이식** — Operator도 Admin과 동일한 완전한 센서 정보를 봐야 함
2. Operator 페이지는 Admin 전용 코드 자체를 아예 로드하지 않음 (2D grid 렌더러, follow-cam 계산, raw 디버그 체크박스 패널 등) — 계산량 감소가 아니라 "안 쓰는 코드가 같이 실행되던 부분" 제거
3. 별도 rosbridge 연결로 서비스 콜 경합 해소 (Phase 0 이전부터 알려진 이슈)

## 2. 이식 대상 범위 재확인 결과 — 예상보다 큼 (2026-09-10 조사)

TF 해석 + taxel 마커 렌더링 핵심 함수 9개(`resolveChildFrameAlias`, `getFrameLeafIndex`, `resolveFrameToFixed`, `selectFixedFrame`/`selectFixedFrameUncached`, `updateUrdfMarkers`, `ensureMarkerObject`, `resolveMarkerPose`, `resolveTaxelLocalOffsetViaXela`, `resolveTaxelFixedFramePoseRobust`)의 의존성을 전수 조사한 결과:

- 이 함수들은 index.html에만 존재하고 **`xela_taxel_viz_core`(Phase 1 산출물)에는 아직 없음** — Phase 1에서 옮긴 건 씬/카메라/mesh 로더 골격(`urdf_mesh_renderer.js`)뿐, TF 해석과 taxel 마커는 빠져 있었음
- 이 9개 함수의 유일한 호출부인 `updateUrdfMarkers`는 `renderUrdfMesh`(index.html 3105행, 매 rAF 프레임 실행되는 렌더 루프의 핵심)에서 호출되며, 이 `renderUrdfMesh`는 `updateRobotLinkTransforms`, `autoFrameUrdfMeshCamera`, `updateFollowCamera`, `computeModuleCenterFromTf`와도 순서/부작용 의존성으로 얽혀 있음
- 즉 9개 함수만 떼어낼 수 없고 **사실상 index.html의 3D 렌더 루프 전체(대략 800~1000줄)를 통째로 옮겨야** 인접 함수 의존성이 깨지지 않음. 추가로 헬퍼 6개(`getFrameLeafName`, `normalizeFrameId`, `getActiveUrdfSource`, `getUrdfSourceForUiMode`, `extractModuleIdFromName`, `clamp`, `operatorAdjustNorm`)와 모듈 스코프 캐시 변수 5개(`lastSelectFixedFrameKey` 등)도 함께 이식 필요
- `js/core/app_state.js`(Phase 1 산출물)에 `urdfSources.{xela,robot}.frameLeafIndex` 필드가 명시적으로 선언돼 있지 않음(암묵적 undefined) — 이식 시 스키마에 추가 필요

## 3. 결정 사항 (2026-09-10)

**이번 세션에서는 렌더 루프 전체 이식을 진행하지 않고 다음 세션 과제로 이월.** 사유: 800~1000줄 규모의 상호의존적 코드를 서두르지 않고, dg5f와의 정확도 비교 검증까지 포함해서 진행하기 위함(정확히 이번에 지적된 "센서 정보 정확도"가 걸린 영역이므로 신중하게 접근).

**Phase 2 재정의**: "16개 모듈로 축소"가 아니라 **"렌더 루프 전체를 index.html에서 그대로 이식"**으로 범위를 변경. `xela_taxel_operator_dg5f/web/js/viz/operator_viz.js`(2026-09-09 작성분)는 이 재정의 이전의 잘못된 전제(모듈 축소)로 작성된 것이므로 **폐기 대상** — 다음 세션에서 렌더 루프 이식과 함께 다시 작성.

## 4. 검증 체크리스트 상태 (계획서 반영용, v2)

- [x] (v1 폐기) ~~Admin 대비 계산량/캐시 대상 축소 근거 수치화~~ — 잘못된 전제로 폐기
- [x] TF 해석 + taxel 마커 렌더 루프 핵심부 이식 완료 (2026-09-10) — `xela_taxel_viz_core/web/js/render/taxel_marker_renderer.js` 신규 파일로 9개 핵심 함수 + 인접 호출부(`updateRobotLinkTransforms`/`autoFrameUrdfMeshCamera`/`updateFollowCamera`/`computeModuleCenterFromTf`) + 헬퍼 전부 원본 그대로 이식. `app_state.js`에 누락됐던 `urdfSources.{xela,robot}.frameLeafIndex` 필드도 추가.
- [x] dg5f와 실 GPU/rosbridge 환경 실측 비교 — **완료 (2026-09-10 추가 세션)**. 1차 세션 판단("실 rosbridge/GPU 환경 없음")은 재빌드 실패 경험에서 온 오판이었음이 확인됨: 이 워크스페이스용 도커 이미지는 이미 빌드되어 있고 `~/.config/moveit_pro/moveit_pro_config.yaml`도 이미 `ur7e_xdg5f_atag_right_sim`으로 설정돼 있어, 전체 재빌드 없이 `moveit_pro run`만으로 실 시뮬레이션 스택(rosbridge 9090, dg5f sidecar http 8765)이 정상 기동됨. 신규 패키지는 `colcon build --packages-select xela_taxel_viz_core`만으로 반영. `xela_taxel_viz_core/web/demo/real_dg5f_check.html`(신규 작성)이 같은 실 rosbridge의 `/xvizdg5f/tf`, `/xvizdg5f/tf_static`, `/x_taxel_dg5f/web_state`에 라이브 구독해 이식된 `taxel_marker_renderer.js` 함수들을 실 데이터로 구동 → `markerMapSize=124, visibleCount=124, nanCount=0, pass=true`, 샘플 좌표(z≈0.28~0.29m 등)가 물리적으로 타당함을 Playwright(headless)로 확인. dg5f Admin(`http://localhost:8765`)도 같은 스택에서 Simulate 모드로 124개 taxel 마커(화살표+색상)를 정상 렌더링 중임을 스크린샷으로 확인.
- [x] Operator 전용 3D 뷰가 실제 로봇/시뮬레이션 데이터로 Admin과 시각적으로 동등한 정보를 보여주는지 확인 — 위 실측으로 확인 완료(마커 개수/가시성/좌표 유효성/실좌표값 기준 일치). 카메라 앵글까지 맞춘 픽셀 단위 스크린샷 비교는 Phase 3(운영자 launch 통합) 이후 선택적으로 격상 가능.
- [x] `xela_atag_taxel_viewer` session/operator 컴포넌트와의 DOM/이벤트 인터페이스 연결 가능성 확인(컴포넌트 자체는 module_groups 단위로 이미 동작, 무수정 재사용 가능 — 이 결론은 유효, taxel 마커 이식과는 별개 사안)

## 5. Phase 2 실행 결과 및 남은 이슈 (2026-09-10, 이식 세션)

- 이식 파일: `xela_taxel_viz_core/web/js/render/taxel_marker_renderer.js` (신규). `createTaxelMarkerRenderer(deps)` 팩토리가 원본 index.html의 모듈 스코프 캐시 변수들을 클로저로 캡슐화하고, 원본이 페이지 전역(`state`, DOM, `followCamState` 등)을 직접 참조하던 부분만 의존성 주입으로 치환 — TF 해석/좌표 계산 로직 자체는 한 줄도 바꾸지 않음.
- `bindActiveUrdfSourceToMesh`는 원래 이식 대상 9+adjacent 목록에 없었지만, `resolveChildFrameAlias` 등이 의존하는 `m.tfEdges`/`m.frameAliasCache`/`m.frameLeafIndexSource` 바인딩을 담당하는 필수 glue라 함께 이식(TF 관련 필드만, 메시 재빌드 side-effect는 제외).
- **2026-09-10 후속 세션에서 이식 완료**: `refreshUrdfModuleHighlight`/`getLinkMaterial`(index.html 2435/2447) + 지원 함수 `collectEnabledModuleIdsFromUi`/`collectVisibleModuleIdsFromPayload`/`computeActiveModuleIds`/`resolveLinkColorForSelection`/`fallbackModuleColor`/`URDF_NEUTRAL_LINK_COLOR`를 `taxel_marker_renderer.js`에 무수정 이식. `collectEnabledModuleIdsFromUi`의 Operator 위젯 DOM 분기만 `getEnabledModuleIds` 콜백 주입으로 대체(기본값=Admin의 `state.showModules` 분기 그대로). 이식 과정에서 기존 `createExtractModuleIdFromName`에 하우징 링크(`base_<module>_link`) 매칭 분기가 누락돼 있던 것을 발견해 원본대로 보강(최초 이식 시 누락, 회귀 아님). `moveit_pro build` 성공, 데모(`urdf_taxel_markers.html`)를 `state.urdfMesh.lastHighlightSignature = null` 우회 대신 실제 `refreshUrdfModuleHighlight()` 호출 경로로 바꿔 재실행 → 124개 마커 전부 visible/NaN 없음/콘솔 에러 없음 확인.
- `xela_taxel_operator_dg5f/web/js/viz/operator_viz.js` 삭제 관련: 2026-09-10 확인 결과 이 파일은 `op-separate` 브랜치(및 전체 git 이력, 파일시스템)에 실존한 적이 없음 — `xela_taxel_operator_dg5f` 디렉토리 자체가 아직 생성 전(Phase 3 대상)이라 삭제할 파일이 없음. 계획서 기록과 실제 상태가 어긋나 있었던 것으로 정리, 조치 불필요.
- 검증 수준: 정적 코드 동일성(원본 그대로 복사, refreshUrdfModuleHighlight/getLinkMaterial 계열 포함 라인 단위 대조) + 합성 데이터 기반 Playwright 실행(124개 마커 생성/좌표 유효성/모듈 하이라이트 게이트 정상 통과) + 원본-vs-이식 수치 비교(2026-09-10, `compare_original_vs_ported.mjs`: 124개 전부 좌표 1e-9 오차 내 완전 일치) + **실 GPU/rosbridge 실측 비교(2026-09-10 추가 세션, 최종 완료)**: 위 세 단계에서 "환경 제약으로 미실시"라 기록했던 부분은 오판이었음 — 실제로는 이 워크스페이스의 도커 이미지가 이미 빌드돼 있었고 config.yaml도 이미 dg5f 로봇으로 설정돼 있어 `moveit_pro run`(전체 재빌드 없이) 한 번으로 실 스택이 기동됨. `colcon build --packages-select xela_taxel_viz_core`로 신규 패키지만 반영 후, dg5f Admin(8765)과 신규 실데이터 데모 `real_dg5f_check.html`(viz_core, 8899 정적서버)을 같은 rosbridge(9090)에 연결해 Playwright로 검증: **124개 taxel 마커 전부 생성/visible/좌표 유효(NaN 없음), 실좌표값(z≈0.28~0.29m 등)이 물리적으로 타당, dg5f Admin 화면도 Simulate 모드에서 동일하게 124개 마커를 정상 렌더링 중임을 스크린샷으로 확인 (PASS)**. `moveit_pro down`으로 정상 종료, `xela_taxel_sidecar_dg5f`는 `git status`/`git diff` 기준 무수정 확인.
