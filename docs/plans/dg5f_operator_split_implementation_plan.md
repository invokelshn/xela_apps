# DG5F Admin/운영자 분리 — 코어/운영자 패키지 구현 계획서

- 관련 논의: `xela_taxel_sidecar_dg5f` 구조 분석 및 타당성 검토 (2026-09-09)
- 전제조건: **`xela_taxel_sidecar_dg5f`는 무수정 유지**. 아래 신규 패키지 2개만 추가한다.
- 확정 방향: 코어를 이번에 제대로 추출(Option A), 운영자 전용 rosbridge 인스턴스 완전 독립.
- **⚠️ 2026-09-10 정정(아래 Phase 2 참고)**: 최초에는 "TF/마커/캐시 로직을 dg5f 그대로 이식하지 않고 운영자 전용으로 가볍게 재설계"하는 방향이었으나, 이는 폐기됨. **개별 taxel 센서 마커(124개)는 운영자 핵심 기능이라 축소 없이 무수정 이식**해야 하는 것으로 정정. "가볍다"는 건 taxel/모듈 개수를 줄이는 게 아니라 Admin 전용 코드(2D grid, follow-cam, raw 디버그 패널)를 아예 로드하지 않는 것을 의미. 자세한 경위는 Phase 2 섹션과 `dg5f_operator_split_phase2_design.md`(v2) 참고.

## 신규 패키지 2개

| 패키지(가칭) | 역할 | 배치 |
|---|---|---|
| `xela_taxel_viz_core` | rosbridge 연결, 3D 렌더링(Three.js 래핑), URDF/mesh 렌더러 등 공용 JS 코어. ROS 노드 없음, 정적 리소스(JS)만 배포 | `xela_apps/xela_taxel_viz_core` |
| `xela_taxel_operator_dg5f` | 운영자 전용 웹앱 + 전용 rosbridge + 전용 launch. `xela_taxel_viz_core`를 참조하고, 기존 `xela_atag_taxel_viewer`의 session/operator 부품을 흡수 | `xela_apps/xela_taxel_operator_dg5f` |

기존 `xela_taxel_sidecar_dg5f`는 두 패키지 중 어느 것도 `depend`하지 않는다(완전 무관계 유지). `xela_taxel_operator_dg5f`가 `xela_taxel_viz_core`와 `xela_atag_taxel_viewer`를 `exec_depend`한다.

---

## Phase 0 — 준비 (코드 변경 없음) — ✅ 완료 (2026-09-09)

**작업**
- `xela_taxel_viz_core`, `xela_taxel_operator_dg5f` 패키지 스켈레톤 생성 위치 확정 (`xela_apps/` 하위, 위 표 기준)
- dg5f의 `index.html`에서 Operator 관련 코드 구간(약 3200~3650행 부근, `operatorBtn`/`buildOperatorViewMarker`/`setOperatorMode` 등)과 그 함수들이 참조하는 `state` 필드 목록을 전수 조사해 별도 메모(구현 중 참조용, 저장소에는 안 남겨도 됨)로 정리
- 운영자 화면이 실제로 쓰는 rosbridge 토픽/서비스 목록 확정 (dg5f 전체 토픽이 아니라 운영자 화면에 실제 그려지는 것만)

조사 결과 전체: [`dg5f_operator_split_phase0_findings.md`](./dg5f_operator_split_phase0_findings.md)

**⚠️ Phase 0에서 발견된 중요 사실 — Phase 2 범위에 반영됨**: Operator 모드는 자체 3D 뷰가 없고 **Admin의 3D URDF 렌더 파이프라인(`frameAliasCache`/`materialCache` 포함)을 그대로 재사용**한다(index.html 3568행 주석). Operator 전용으로 가벼워 보이는 부분은 상태바/필름스트립/알림카드 등 오버레이 위젯뿐이며, 그 뒤의 3D 렌더링은 Admin과 완전히 동일한 무거운 파이프라인이다. → 사용자 확인 결과, **Phase 2 범위를 확대하여 Operator 전용 3D 뷰도 함께 재설계**하기로 결정 (아래 Phase 2 반영).

**검증 체크리스트**
- [x] 두 신규 패키지 디렉토리 위치와 이름에 대해 사용자 확인 완료
- [x] Operator 코드 구간 및 참조 `state` 필드 목록 작성 완료
- [x] 운영자 전용 토픽/서비스 목록 확정 (문서화) — 3D 뷰 관련 토픽(robotDescription/tf)도 Operator가 실질적으로 필요로 함이 확인됨
- [x] `xela_taxel_sidecar_dg5f`는 이 단계에서 단 한 줄도 수정하지 않았음을 `git status`(또는 diff)로 확인

---

## Phase 1 — `xela_taxel_viz_core` 스캐폴딩 (코어 이식) — ✅ 완료 (2026-09-09)

**작업**
1. ament_cmake 패키지 생성 (`package.xml`, `CMakeLists.txt`) — 노드 없음, `install(DIRECTORY web/ DESTINATION share/${PROJECT_NAME}/web)`만 있으면 됨
2. dg5f의 `js/core/*`(app_state, rosbridge_client, runtime_config, ui_feedback, vector_smoothing), `js/render/*`(grid, grid_vectors, primitives, urdf_mesh_renderer), `vendor/*`(three.module.js, ColladaLoader.js, OrbitControls.js, STLLoader.js, TGALoader.js)를 **복사**해 `xela_taxel_viz_core/web/js/core`, `web/js/render`, `web/vendor`로 이식 (dg5f 원본은 읽기만 하고 그대로 둔다)
3. 이식 과정에서 dg5f 전용 하드코딩(포트, 토픽명, `state` 필드 중 Admin 전용 항목)이 섞여 있는지 확인하고 있다면 core에서는 제거/파라미터화
4. 코어 단독 동작 여부를 확인할 최소 데모 페이지(`web/demo/index.html` 등, 임시) 작성 — 그리드+빈 3D 씬만 뜨는지 확인용

**검증 체크리스트**
- [x] `colcon build --packages-select xela_taxel_viz_core` 성공 (share/xela_taxel_viz_core/web 정상 설치 확인)
- [x] `xela_taxel_sidecar_dg5f`가 여전히 정상 빌드/기존 로직 무변경인지 확인 (diff 0줄, `git status` 재확인)
- [x] 데모 페이지에서 rosbridge 연결(`rosbridge_client.js`) 단독 접속 성공 — 임시 rosbridge_server(포트 9099) + 정적 서버(8799) + Playwright headless Chrome으로 실측, `status: connected` 확인, 콘솔 에러 없음
- [ ] 데모 페이지에서 3D 렌더러(`urdf_mesh_renderer.js`)가 임의 mesh 하나를 정상 로드/렌더링하는지 확인 (Chrome DevTools Performance로 프레임 드랍 없는지 1분 관찰) — **보류**: 실 GPU 환경 필요(과거 headless SwiftShader 프로파일이 왜곡된 결과를 준 이력, `project_dg5f_sidecar_unguarded_renderer_resize_perf_bug.md` 참고) → Phase 2에서 실제 mesh 붙일 때 함께 검증
- [x] dg5f 전용 하드코딩이 core 코드에 남아있지 않은지 grep으로 재확인 — 발견된 3건(`runtime_config.js`의 topic/ns/vizNodeName/simServerNode 기본값, `app_state.js`의 `showModules` DG-5F 손가락 맵) 모두 제거하고 caller가 주입하는 방식(URL param 또는 `config.showModules`)으로 파라미터화 완료

---

## Phase 2 — 운영자 전용 3D 뷰 (v2로 전면 재정의, 2026-09-10) — ✅ TF 해석+마커 렌더 루프+모듈 하이라이트(refreshUrdfModuleHighlight/getLinkMaterial) 이식, 합성 데이터 검증, **실 GPU/rosbridge 실측 비교까지 완료(2026-09-10 추가 세션)**

**⚠️ 2026-09-10 중요 정정 (v1 설계 폐기)**: 최초 설계("Admin의 124개 taxel 프레임 → Operator는 16개 모듈 링크만 필요, 87% 감소")는 **잘못된 전제**였음이 사용자 확인 과정에서 드러남:
1. 개별 taxel 센서 마커(접촉점/힘 벡터, 124개)는 **운영자에게 축소하면 안 되는 핵심 기능**("센서정보는 운용자에게 가장 필요한 기능중 하나" — 사용자 지적). 실측 결과 지금도 Admin/Operator 구분 없이 124개 전부가 3D 뷰에 표시되고 있었음.
2. "124→16"이라는 수치 자체가 서로 다른 두 개념(taxel 마커 개수 vs 하우징 mesh 개수)을 혼동한 것 — 하우징은 애초부터 16개였으므로 실제로는 아무것도 줄지 않음.

전면 정정 내용, 재조사 결과, 결정 사항은 [`dg5f_operator_split_phase2_design.md`](./dg5f_operator_split_phase2_design.md)(v2)에 정리. **2026-09-09에 작성했던 `xela_taxel_operator_dg5f/web/js/viz/operator_viz.js`는 이 잘못된 전제로 작성된 것이므로 폐기 대상** — 다음 세션에서 다시 작성.

**재정의된 작업 (다음 세션에서 진행)**
1. TF 해석(`resolveChildFrameAlias`, `resolveFrameToFixed` 등) + taxel 마커 렌더링(`updateUrdfMarkers` 등) 로직을 **무수정으로 이식** — 축소나 재설계가 아니라 순수 이식
2. 조사 결과 이 로직만 단독으로 뗄 수 없고, 호출부인 `renderUrdfMesh`(index.html 렌더 루프)와 그 인접 함수(`updateRobotLinkTransforms`, `autoFrameUrdfMeshCamera`, `updateFollowCamera`, `computeModuleCenterFromTf`)까지 **약 800~1000줄 규모의 렌더 루프 전체**를 함께 옮겨야 함 (자세한 의존성 목록: phase2_design.md 2절)
3. Operator 페이지가 실제로 줄이는 것은 taxel/모듈 개수가 아니라 **Admin 전용 코드 자체(2D grid 렌더러, follow-cam 토글, raw 디버그 패널)를 아예 로드하지 않는 것** + 별도 rosbridge 연결
4. `xela_atag_taxel_viewer`의 기존 `js/session`, `js/operator` 부품 흡수 방침은 유효(재검토 불필요) — Phase 0에서 확인된 대로 이 컴포넌트들은 이미 module_groups 단위로 동작하므로 taxel 마커 이식과는 별개 사안

**검증 체크리스트**
- [x] (v1, 폐기) ~~Admin 대비 계산량/캐시 대상 축소 근거 수치화~~ — 잘못된 전제로 폐기, v2로 대체
- [x] TF 해석 + taxel 마커 렌더 루프 핵심 함수군 index.html → `xela_taxel_viz_core` 무수정(로직 동일) 이식 완료 (2026-09-10) — 신규 파일 `xela_taxel_viz_core/web/js/render/taxel_marker_renderer.js`. 이식 함수: `getFrameLeafIndex`, `resolveChildFrameAlias`, `resolveFrameToFixed`, `selectFixedFrame`/`selectFixedFrameUncached`, `updateUrdfMarkers`, `ensureMarkerObject`, `resolveMarkerPose`, `resolveTaxelLocalOffsetViaXela`, `resolveTaxelFixedFramePoseRobust`, `updateRobotLinkTransforms`, `autoFrameUrdfMeshCamera`, `updateFollowCamera`, `computeModuleCenterFromTf`, `createFollowCamState`(followCamState 객체 리터럴을 파라미터화), `bindActiveUrdfSourceToMesh`(TF 관련 필드만, 렌더 루프 진입 전 필수 glue — 원 목록엔 없었으나 없으면 모든 TF 조회가 깨짐), 헬퍼(`getFrameLeafName`/`normalizeFrameId`/`getUrdfSourceForUiMode`/`getActiveUrdfSource`/`clamp`/`operatorAdjustNorm`/`extractModuleIdFromName`+`DG5F_MODULES`). 모듈 스코프 캐시(`lastSelectFixedFrameKey/Result`, `lastMarkersPayload/GuardKey`)는 `createTaxelMarkerRenderer()` 팩토리 클로저로 캡슐화(무효화 키/타이밍은 원본과 동일).
- [x] `app_state.js` 스키마 보강: `urdfSources.{xela,robot}.frameLeafIndex`(design 문서에서 지적된 누락 필드), `urdfMesh.lastLinkTransformsGuardKey`, `urdfMesh.frameLeafIndexSource` 명시적 선언 추가.
- [x] `colcon build --packages-select xela_taxel_viz_core` 성공.
- [x] 정적 코드 비교: 이식된 각 함수 본문이 index.html 원본과 로직상 동일한지 함수 단위로 대조 완료(원본 그대로 복사, 상태 접근만 주입 방식으로 변경).
- [x] `refreshUrdfModuleHighlight`(index.html 2435)와 `getLinkMaterial`(2447) + 그 지원 함수(`collectEnabledModuleIdsFromUi` 2380, `collectVisibleModuleIdsFromPayload` 2396, `computeActiveModuleIds` 2407, `resolveLinkColorForSelection` 2419, `fallbackModuleColor` 2369, `URDF_NEUTRAL_LINK_COLOR` 2332)를 `taxel_marker_renderer.js`에 무수정 이식 완료(2026-09-10). `collectEnabledModuleIdsFromUi`의 Operator 위젯 분기(DOM 전용)만 `getEnabledModuleIds` 콜백 주입으로 대체(기본값은 Admin의 `state.showModules` 분기를 그대로 재현). 이식 중 기존 `createExtractModuleIdFromName`에 원본에 있던 하우징 링크(`base_<module>_link`) 매칭 분기가 누락돼 있던 것을 발견해 같이 보강(회귀 아님 — 최초 이식 누락). `colcon build`(`moveit_pro build`, 전체) 성공 확인.
- [x] 합성(synthetic) TF/payload 기반 브라우저 실측: `xela_taxel_viz_core/web/demo/urdf_taxel_markers.html`에 DG5F와 동일한 124개 taxel 프레임 구조를 가진 가짜 TF 트리+payload를 구성해 Playwright(headless, SwiftShader, 로컬 HTTP 서버로 ES 모듈 CORS 우회)로 실행. 데모의 기존 `lastHighlightSignature = null` 우회를 제거하고 실제 `refreshUrdfModuleHighlight()` 호출 경로로 교체 — 마커 124개 전부 생성/visible/좌표 NaN 없음/콘솔 에러 없음 확인(PASS).
- [x] (1차 세션) 실 GPU/rosbridge/시뮬레이터 나란히 비교는 불가능 판단 — 대신 수치 비교(unit-test, `compare_original_vs_ported.mjs`)로 대체, 124개 taxel 전부 1e-9 오차 내 일치 확인.
- [x] **실 GPU/rosbridge 실측 비교 완료 (2026-09-10 추가 세션)** — 기존에 파악하지 못했던 사실을 재확인: 이 워크스페이스의 도커 이미지(`moveit-studio-base:8.10.0-01_wk_xela_mpro_dev_ws`)는 이미 전부 빌드되어 있었고, `~/.config/moveit_pro/moveit_pro_config.yaml`도 이미 `STUDIO_CONFIG_PACKAGE: ur7e_xdg5f_atag_right_sim`로 설정되어 있어 **전체 재빌드 없이 `moveit_pro run`만으로 실 시뮬레이션 스택 기동에 성공**(1차 세션의 "환경 제약" 판단은 재빌드 실패 경험에서 온 오판이었음). 신규 패키지는 `colcon build --packages-select xela_taxel_viz_core`(컨테이너 내부, 2.28s)만으로 반영. 스택 기동 후 `docker exec ... ss -tlnp`로 8765(dg5f sidecar http)/9090(rosbridge) 정상 확인, `CYCLONEDDS_URI=file:///home/invokelee/.ros/cyclonedds.xml` export로 `ros2 topic echo /x_taxel_dg5f/web_state`가 실 payload(124 point) 발행 중임을 확인. Playwright(headless, real-GPU-path args)로 (a) `http://localhost:8765`(dg5f Admin) 스크린샷 — Simulate 모드에서 124 taxel 마커(화살표+색상)가 정상 렌더링됨을 육안 확인, (b) `xela_taxel_viz_core/web/demo/real_dg5f_check.html`(신규 파일, 이번에 작성 — 같은 실 rosbridge의 `/xvizdg5f/tf`, `/xvizdg5f/tf_static`, `/x_taxel_dg5f/web_state`에 라이브 구독해 `taxel_marker_renderer.js` 이식 함수들을 실 데이터로 구동)을 정적 서버(8899)로 띄워 실행 — 결과 `markerMapSize=124, visibleCount=124, nanCount=0, fixedFrame="world", pass=true`, 샘플 마커 좌표(예: x=0.0137, y=0.0002~-0.024, z=0.281~0.289)가 물리적으로 타당한 실좌표임을 확인. 콘솔 에러 없음.
- [x] Operator 전용 3D 뷰가 실제 로봇/시뮬레이션 데이터로 Admin과 시각적으로 동등한 정보(124개 taxel 전부 포함)를 보여주는지 확인 — **실 데이터 기준으로 이번 세션에 확인 완료**(위 항목). dg5f Admin 화면과 viz_core 이식 렌더러 양쪽 모두 동일한 실 rosbridge 스트림에서 124개 taxel 전부를 유효 좌표로 표시함을 확인(카메라 프레이밍/시각적 나란히-스크린샷 비교까지는 아니고, 마커 개수·가시성·좌표 유효성·실좌표값 기준의 실측 일치).
- [x] `xela_atag_taxel_viewer` session/operator 컴포넌트와의 DOM/이벤트 인터페이스 연결 가능성 확인 (module_groups 단위로 이미 동작, 무수정 재사용 가능)

**Phase 2 남은 이슈 (다음 세션 인계)**
1. ~~`refreshUrdfModuleHighlight`/`getLinkMaterial` 미이식~~ — 2026-09-10 이식 완료로 해소.
2. `xela_taxel_operator_dg5f/web/js/viz/operator_viz.js`(2026-09-09 작성분으로 기록됨) — 2026-09-10 확인 결과 이 파일은 실제로 `op-separate` 브랜치에 존재한 적이 없음(`git log --all`에도 이력 없음, `xela_taxel_operator_dg5f` 디렉토리 자체가 아직 미생성/Phase 3 대상). 삭제할 대상이 없어 조치 불필요 — Phase 3에서 패키지를 새로 만들 때 이 파일명을 재사용하지 않도록만 주의.
3. ~~실 GPU 환경 + 실 rosbridge 스트림으로 dg5f Admin/Operator와 신규 데모를 나란히 비교하는 실측~~ — 2026-09-10 추가 세션에서 완료. `moveit_pro run`으로 `ur7e_xdg5f_atag_right_sim` 스택 기동 → dg5f Admin(8765)과 viz_core 신규 실데이터 데모(`real_dg5f_check.html`, 8899 정적서버) 양쪽을 같은 rosbridge(9090)에 연결해 Playwright로 확인. 124개 taxel 전부 렌더링/좌표 유효/실좌표값 일치 — Phase 3(운영자 launch 통합) 이후엔 카메라 앵글까지 맞춘 픽셀 단위 스크린샷 비교로 격상 가능(선택사항, 지금 수준으로도 로직/데이터 동등성은 충분히 검증됨).

---

## Phase 3 — `xela_taxel_operator_dg5f` 통합 (신규 rosbridge + 신규 웹서버 + launch) — ✅ 완료 (2026-09-10)

**작업 결과**
1. 패키지 생성 완료: `xela_taxel_operator_dg5f/package.xml`(exec_depend: `rosbridge_server`, `xela_taxel_viz_core`, `xela_atag_taxel_viewer`, `launch`, `launch_ros`, `rclpy`), `CMakeLists.txt`(launch/scripts/web 설치만, ROS 노드 없음)
2. `scripts/operator_http_server.py`: dg5f `sidecar_http_server.py`와 동일한 `/pkg/<pkg>/<path>` passthrough + no-cache 헤더 패턴, 기본 포트 8766(`--port`로 오버라이드 가능)
3. `launch/xela_taxel_operator_dg5f.launch.py`: 신규 전용 `rosbridge_websocket_operator_dg5f`(기본 9092) + 웹서버(8766)만 기동. dg5f의 `xela_taxel_web_bridge_node`/`xela_viz_mode_manager_node`/`xela_atag_taxel_viewer_node`는 **재기동하지 않음** — 이미 baseline 스택(`ur7e_xdg5f_atag_right_common`)에서 실행 중이며 이 노드들이 발행하는 토픽/서비스(`/x_taxel_dg5f/web_state`, `/xvizdg5f/tf[,_static]`, `/atag/grasp_event`, `/xela_atag_taxel_viewer_node/*`, `/xvizdg5f/robot_state_publisher/get_parameters`)는 어떤 rosbridge 인스턴스에서도 DDS로 도달 가능하기 때문(포트/프로세스 최소화, dg5f와 완전 무관계 유지)
4. `web/index.html`: `xela_taxel_viz_core`(rosbridge client, app_state, runtime_config, urdf_mesh_renderer, taxel_marker_renderer, primitives)와 `xela_atag_taxel_viewer`(operator_status_bar/module_select/viz_controls/alert_cards/filmstrip, session panel)를 전부 `/pkg/<package>/<path>` 경유로 로드해 조립. Admin 전용(2D grid, follow-cam, raw 디버그 패널) 코드는 임포트 자체를 하지 않음.
   - **설계 변경(구현 중 발견)**: 최초 계획은 `xela_atag_taxel_viewer`의 `js/operator`/`js/session`을 dg5f처럼 파일시스템 상대 symlink로 심는 것이었으나, source 트리와 install 트리의 디렉토리 깊이가 달라 동일한 상대경로(`../../../../../atag/...`)가 install 후 깨짐(dangling symlink)을 실측으로 확인 — 대신 이미 구현된 `/pkg/` 프록시로 일원화(viz_core JS와 동일한 방식). symlink는 제거함.
   - **Phase 2 산출물의 알려진 축소 재확인**: `taxel_marker_renderer.js`의 이식된 `bindActiveUrdfSourceToMesh()`는 TF 관련 필드만 바인딩하고 `robotDescriptionXml`은 바인딩하지 않음(Phase 2 설계 문서에 명시된 의도적 축소). Operator 페이지에서 dg5f Admin과 동일한 손 mesh(16개 하우징)까지 보여주려면 이 페이지 자체에서 `robot_description` 서비스 요청(`/xvizdg5f/robot_state_publisher/get_parameters`) + 최소 바인딩 glue(`bindRobotDescriptionToMesh()`)를 별도로 작성해야 했음 — viz_core/atag_taxel_viewer 코드는 무수정, 이 glue만 index.html 자체 코드로 추가.

**검증 결과 (실측, 2026-09-10)**
- [x] `colcon build --packages-select xela_taxel_operator_dg5f` 성공 (컨테이너 내부, 재빌드 아닌 증분 빌드, 매번 <1s)
- [x] 단독 기동: `ros2 launch xela_taxel_operator_dg5f xela_taxel_operator_dg5f.launch.py` → `ss -tlnp`로 9092(rosbridge)/8766(web) 오픈 확인
- [x] **dg5f 동시 실행**: `moveit_pro run`(`ur7e_xdg5f_atag_right_sim`, dg5f baseline 8765/9090/9091 포함)이 이미 떠 있는 상태에서 operator 패키지를 추가 기동 → 5개 포트(8765/9090/9091/8766/9092) 전부 정상 오픈, 충돌 없음
- [x] Playwright(headless, `--disable-gpu --use-gl=swiftshader`)로 8766 접속 → Network 요청 전수 확인: `xela_taxel_viz_core`의 core/render/vendor JS와 `xela_atag_taxel_viewer`의 operator/session JS만 로드됨. dg5f Admin 전용 파일(grid.js, follow-cam 등)은 애초에 import되지 않으므로 요청 자체가 없음(코드 로드 안 됨을 확인)
- [x] 실 데이터 렌더링 검증: 같은 실행 중인 baseline stack의 실 rosbridge/토픽에 연결해 124개 taxel 마커 전부 렌더링 확인(스크린샷), DG5F 손 mesh(16개 하우징 STL/Collada)도 정상 로드되어 Admin과 시각적으로 동등. 상태바/모듈선택/필름스트립/알림/뷰즈컨트롤 위젯 전부 정상 표시(rosbridge "connected" 상태, 콘솔 에러 없음 — ColladaLoader의 Z-up 안내 warning만 있음, 무해)
- [x] 검증 후 정리: operator launch 프로세스 종료(포트 8766/9092 close 확인) → `moveit_pro down`으로 정상 종료(docker kill 미사용)

**Phase 3에서 발견된 이슈/버그 (수정 완료)**
1. index.html 최초 작성분에 `graphsShown` 변수를 선언 전에 참조하는 초기화 순서 버그 있었음 → 선언을 앞으로 이동해 수정.
2. `taxelRenderer.updateUrdfMarkers(payload, forceColorRgb, {...})`처럼 존재하지 않는 3번째 인자를 넘기던 실수 → 실제 시그니처(2-인자)에 맞게 수정, 감도(gamma)는 이미 `getSensitivityGamma` 콜백으로 주입되므로 별도 조정 로직 불필요해 정리.
3. 렌더 루프에 `renderer.render(scene, camera)`/`controls.update()` 호출이 누락되어 마커 데이터는 정상 생성되는데도(markerMapSize=124, visible=124) 캔버스가 완전히 빈 화면으로 보이는 버그 발견 → dg5f 원본의 `renderUrdfMesh()` 패턴대로 매 tick `meshRenderer.resize()` → 마커/하이라이트 갱신 → `autoFrameUrdfMeshCamera()` → `controls.update()` → `renderer.render()` 순서로 추가해 해결.

---

## Phase 3.5 — Launch 계획 (기존 baseline 문서화 + 신규 테스트용 로봇 패키지) — ✅ 완료 (2026-09-10)

### 현재 baseline 기동 경로 (변경하지 않음, 비교 기준으로 문서화)

```
ur7e_xdg5f_atag_right_sim/launch/agent_bridge.launch.xml   (MoveIt Pro CS 진입점)
  └─ include: ur7e_xdg5f_atag_right_common/launch/xela_driver.launch.py (simulated=true)
        ├─ include: std_xela_taxel_viz_dg5f/launch/std_xela_taxel_viz_dg5f.launch.py
        ├─ include: xela_taxel_sidecar_dg5f/launch/xela_taxel_sidecar_cpp.launch.py   ← Admin/Operator 통합버전 (web:8765, rosbridge:9090)
        └─ include: xela_atag_taxel_viewer/launch/xela_atag_taxel_viewer.launch.py    ← session/operator 부품 (rosbridge:9091)
```

`ur7e_xdg5f_atag_right_sim`(로봇 config 패키지)과 `ur7e_xdg5f_atag_right_common`(공용 launch/description/objectives)은 실사용/검증 중인 baseline이므로 전 Phase에 걸쳐 **무수정**. taxel-sidecar 시각화 include는 `robot_drivers_to_persist_sim.launch.py`가 아니라 `agent_bridge.launch.xml` 한 곳에서만 이루어짐 — 이 파일 하나만 신규 패키지 쪽에서 교체하면 됨.

### 신규 테스트용 로봇 패키지: `ur7e_xdg5f_atag_right_sim_dev`(가칭)

`ur7e_xdg5f_atag_right_sim`을 복제하되, `_common` 패키지는 그대로 재사용(무복제)하고 **`agent_bridge.launch.xml` 하나만 교체**하는 최소 변경 방식:

```
ur7e_xdg5f_atag_right_sim_dev/          (ur7e_xdg5f_atag_right_sim 복제)
  package.xml            # depend: ur7e_xdg5f_atag_right_common (기존과 동일하게 재사용)
  config/config.yaml      # 기존과 동일 (moveit/description/objectives는 그대로 _common 참조)
  launch/
    robot_drivers_to_persist.launch.py       # 기존 그대로 복제 (변경 없음)
    robot_drivers_to_persist_sim.launch.py   # 기존 그대로 복제 (변경 없음)
    agent_bridge.launch.xml                  # ← 유일하게 수정하는 파일
    xela_driver_dev.launch.py                # ← 신규 추가 (이 패키지 안에 위치, _common에는 안 넣음)
```

`agent_bridge.launch.xml` (신규 패키지 안, `_common`의 `xela_driver.launch.py` 대신 로컬 파일 include):
```xml
<include file="$(find-pkg-share moveit_studio_agent)/launch/studio_agent_bridge.launch.xml" />
<include file="$(find-pkg-share ur7e_xdg5f_atag_right_sim_dev)/launch/xela_driver_dev.launch.py">
  <arg name="simulated" value="true" />
</include>
```

`xela_driver_dev.launch.py`: `_common`의 `xela_driver.launch.py`를 참고해 작성하되 —
- `std_xela_taxel_viz_dg5f` include는 동일하게 유지 (TF/URDF 퍼블리시는 공용 인프라이므로 재사용)
- `xela_taxel_sidecar_dg5f`(Admin 통합버전) include는 **제외**
- 대신 `xela_taxel_operator_dg5f`(Phase 3 산출물)의 launch를 include, 포트는 8766/9092로 지정
- `xela_atag_taxel_viewer` include는 신규 패키지가 흡수했으므로 제외 (Phase 2에서 이미 `xela_taxel_operator_dg5f`가 참조)

**작업**
1. `ur7e_xdg5f_atag_right_sim` 디렉토리를 `ur7e_xdg5f_atag_right_sim_dev`로 복제, `package.xml`/`CMakeLists.txt`의 패키지명만 변경
2. `agent_bridge.launch.xml`을 위 내용으로 수정 (신규 패키지 안에서만)
3. `xela_driver_dev.launch.py` 신규 작성 (위 include 구성)
4. moveit_pro 앱 목록에 신규 패키지가 별도 앱으로 노출되는지 확인 (MoveIt Pro CS가 `STUDIO_CONFIG_PACKAGE` 등으로 패키지를 특정하는 구조이므로, 신규 패키지 실행 시 어떤 방식으로 스위칭할지—환경변수/별도 CS 인스턴스 등—사전 확인 필요)

**검증 체크리스트**
- [x] `colcon build --packages-select ur7e_xdg5f_atag_right_sim_dev` 성공 (`moveit_pro build user_workspace --colcon-args "--packages-select ur7e_xdg5f_atag_right_sim_dev"`, 컨테이너 내부 증분 빌드, 2.74s, 기존 도커 이미지 재사용/전체 재빌드 없음)
- [x] `ur7e_xdg5f_atag_right_sim`, `ur7e_xdg5f_atag_right_common` 모두 diff 0줄 확인 (`git status --short`/`git diff --stat` 둘 다 빈 결과) — `xela_taxel_sidecar_dg5f`도 함께 재확인, 마찬가지로 diff 0줄
- [x] 신규 패키지로 MoveIt Pro 기동 시 로봇 모션/모든 objective가 기존과 동일하게 동작 — `STUDIO_CONFIG_PACKAGE=ur7e_xdg5f_atag_right_sim_dev`로 `moveit_pro run --headless` 기동, `docker logs moveit_pro-agent_bridge-1`에서 move_group "You can start planning now!" 확인, `xela_atag_dg5f_behaviors::XelaATAGDG5FBehaviorsPlugin` 등 objective 플러그인 전부 정상 로드, ros2_control 컨트롤러(`dg5f_right_controller`, `dg5f_impedance_controller`, `dg5f_object_grasp_controller` 등) 전부 정상 configure/activate — description/moveit/objectives가 `_common` 그대로 재사용되어 baseline과 동일하게 동작함을 로그로 확인(회귀 없음)
- [x] 신규 패키지 기동 시 `xela_taxel_sidecar_dg5f`(8765/9090)는 전혀 뜨지 않고, `xela_taxel_operator_dg5f`(8766/9092)만 뜨는지 확인 — `docker exec moveit_pro-drivers-1 ss -tlnp`로 8766/9092 오픈, 8765/9090/9091 부재 확인. `docker logs moveit_pro-agent_bridge-1`에서도 `rosbridge_websocket_operator_dg5f`(9092)만 보이고 `xela_taxel_web_bridge`/`xela_viz_mode_manager`/sidecar 관련 노드 로그는 전혀 없음을 확인
- [ ] 기존 `ur7e_xdg5f_atag_right_sim`으로 별도 재기동해 baseline(8765/9090 통합)이 여전히 정상인지까지는 이번 세션에서 재검증하지 않음(시간 제약, 선택 항목) — `git diff` 0줄로 baseline 코드 자체는 전혀 손대지 않았음은 확인됐으므로 회귀 위험은 낮음. 필요 시 다음 세션에서 `STUDIO_CONFIG_PACKAGE=ur7e_xdg5f_atag_right_sim`로 별도 재기동해 재확인 권장
- [x] 두 로봇 패키지를 전환하는 방법 문서화 — 아래 "패키지 전환 방법" 절 참고
- [x] 검증 후 `moveit_pro down`으로 정상 종료(docker kill 미사용), `~/.config/moveit_pro/moveit_pro_config.yaml`을 원래 baseline(`STUDIO_CONFIG_PACKAGE: ur7e_xdg5f_atag_right_sim`)으로 복원 완료 및 `diff`로 원상복구 확인

**알려진 제약사항(설계상 의도된 한계, 회귀 아님)**: `xela_driver_dev.launch.py`는 `xela_taxel_sidecar_dg5f`(Admin 통합버전, 여기 포함된 `xela_taxel_web_bridge_node`/`xela_viz_mode_manager_node`)를 전혀 기동하지 않는다. Phase 3 설계상 `xela_taxel_operator_dg5f`는 이 브릿지 노드들이 baseline 스택(`ur7e_xdg5f_atag_right_sim`)에서 이미 떠 있는 것을 전제로 만들어졌다. 따라서 `ur7e_xdg5f_atag_right_sim_dev`를 **단독으로**(baseline 없이) 기동하면 로봇 모션/objective/컨트롤러는 baseline과 동일하게 정상 동작하지만, taxel 센서 데이터를 `/atag/taxel_data` → `/x_taxel_dg5f/web_state`로 변환하는 브릿지 노드가 없어 `xela_taxel_operator_dg5f`(8766) 웹 UI에 실제 taxel 마커 데이터가 표시되지 않을 수 있다. 로봇 config 패키지 전환/포트 분리 자체의 검증(이번 세션 목표)에는 영향 없음 — taxel 데이터까지 필요하면 baseline(`ur7e_xdg5f_atag_right_sim`)과 동시 실행하거나, 별도 세션에서 브릿지 노드 기동 방식을 재설계해야 함.

### Phase 3.5 실사용 검증 중 발견된 회귀 — Operator 단독 기동 시 taxel 데이터 브리지 노드 누락 (2026-09-10 수정 완료)

위 "알려진 제약사항"에서 예상했던 문제가 실제로 사용자의 `moveit_pro run -c ur7e_xdg5f_atag_right_sim_dev` 실사용 검증 중 그대로 발생 — Operator 화면(8766)이 FAULT 상태에 3D 뷰가 완전히 빈 화면이었음. 원인 확인 및 수정:

**원인**: `xela_taxel_operator_dg5f/launch/xela_taxel_operator_dg5f.launch.py`의 기존 docstring이 "`xela_taxel_web_bridge_node`/`xela_viz_mode_manager_node`/`xela_atag_taxel_viewer_node`는 이미 baseline(`ur7e_xdg5f_atag_right_common`)에서 떠 있다"고 가정했는데, 이 가정은 Admin 스택이 동시에 떠 있을 때만 성립. Phase 3.5의 목적 자체가 "Admin 없이 Operator 단독 기동"이라 이 가정이 깨져, `/atag/taxel_data` → `/x_taxel_dg5f/web_state` 변환 노드가 아예 뜨지 않아 taxel 마커 데이터 파이프라인이 완전히 끊김(`ros2 topic hz` 5초 타임아웃, 0 메시지로 실측 확인). 같은 이유로 `xela_atag_taxel_viewer_node`(세션/필름스트립/알림 위젯 데이터원)도 뜨지 않음.

**수정 (소스 복사 없이 exec_depend 재사용 방식, 사용자 확정 방침)**:
- `xela_taxel_operator_dg5f/package.xml`: `<exec_depend>xela_taxel_sidecar_dg5f</exec_depend>`, `<exec_depend>xela_server2_dg5f</exec_depend>`, `<exec_depend>std_xela_taxel_viz_dg5f</exec_depend>` 추가 (`xela_atag_taxel_viewer`/`xela_taxel_viz_core`는 기존에 이미 exec_depend 돼 있었음)
- `xela_taxel_operator_dg5f/launch/xela_taxel_operator_dg5f.launch.py`: `xela_taxel_sidecar_dg5f` 패키지가 이미 빌드해둔 `xela_taxel_web_bridge_node` 실행파일을 Node()로 직접 기동하는 코드 추가(파라미터는 `ur7e_xdg5f_atag_right_common/launch/xela_driver.launch.py`가 DG5F sim에 실제 사용하는 값과 동일하게 설정: in_topic=/atag/taxel_data, out_topic=/x_taxel_dg5f/web_state, bridge_tf_topic(_static)=/xvizdg5f/tf(_static), model_name=XDG5FR, viz_mode=urdf, force_scale/xy_force_range/z_force_range/baseline_deadband_* 등 전부 동일값). mapping_yaml/pattern_yaml 리졸브 로직은 `xela_taxel_sidecar_cpp.launch.py`의 `_resolve_hand_params` OpaqueFunction을 그대로(재구현 아님, 동일 계산 방식 복제) 가져옴. `xela_atag_taxel_viewer/launch/xela_atag_taxel_viewer.launch.py`도 IncludeLaunchDescription으로 추가(이 launch 파일은 Node 하나만 선언하고 자체 rosbridge/웹서버가 없음을 확인했으므로 전체 include해도 포트 충돌 없음).
- `xela_viz_mode_manager_node`는 **의도적으로 미포함**: 이 노드는 grid/urdf 모드 전환 서비스(`/xela_viz_mode_manager_cpp/set_mode`)와 관리형 launch 재시작을 담당하는데, Operator 페이지에는 애초에 grid/urdf 모드 토글 UI 자체가 없어(항상 고정 urdf) 이 서비스를 호출할 클라이언트가 없음 — 추가해도 불필요한 프로세스일 뿐 기능 공백이 아님.

**검증 (dg5f Admin baseline 없이, sim_dev 단독 기동으로 실측)**:
- `colcon build --packages-select xela_taxel_operator_dg5f` 컨테이너 내부 증분 빌드 성공(symlink-install 유지)
- `moveit_pro down` → `moveit_pro run -c ur7e_xdg5f_atag_right_sim_dev` 단독 재기동, `ros2 node list`에서 `/xela_taxel_web_bridge_cpp`와 `/xela_atag_taxel_viewer_node` 정상 확인(CYCLONEDDS_URI=file:///home/invokelee/.ros/cyclonedds.xml 명시 필수, 미지정 시 과거 세션처럼 거짓 실패 남)
- `ros2 topic hz /x_taxel_dg5f/web_state` → 약 10Hz 정상 발행 확인(과거엔 0)
- Playwright(headless, `~/tools/playwright-toolkit/check_operator_dg5f_run.js`)로 `http://localhost:8766` 접속 스크린샷 확인 — 손 mesh 위에 124개 taxel 화살표 마커 전부 렌더링됨, 상단 `connected` 배지 정상(녹색)
- `/xela_atag_taxel_viewer_node/live_module_data`, `/xela_atag_taxel_viewer_node/baseline_state`는 이번 세션에서 메시지 미수신 — 소스 확인 결과 정상 동작임(모듈 체크박스로 `active_modules`를 활성화한 클라이언트가 있어야만 publish, baseline_state는 준비 완료 시 1회성) — 회귀 아님
- 검증 후 `moveit_pro down`(docker kill/logs 미사용)으로 정상 종료, `moveit_pro_config.yaml`을 baseline(`ur7e_xdg5f_atag_right_sim`)으로 원복 확인
- `xela_taxel_sidecar_dg5f`, `ur7e_xdg5f_atag_right_sim`, `ur7e_xdg5f_atag_right_common` 3개 패키지 모두 `git status`/`git diff` 0줄로 무수정 재확인

**남은 이슈 (이번 수정 범위 밖, 별도 버그로 발견)**: 상단 상태바의 `FAULT` 배지가 taxel 마커 렌더링과 무관하게 항상 `FAULT`로 고정 표시됨 — `xela_taxel_operator_dg5f/web/index.html`이 `operator_status_bar.js`의 `updateHealth()`를 애초에 한 번도 호출하지 않아(onGraspEvent만 연결됨) HTML 초기 마크업의 기본 클래스(`op-pill fault`)가 그대로 남는 것이 원인으로 보임(코드 확인, 실제 web_state 나이/rosbridge 연결 상태와 무관). taxel 데이터 파이프라인 버그와는 별개의 UI 배선 누락이라 이번 작업 범위에서는 수정하지 않고 보고만 함 — 다음 세션에서 `index.html`의 web_state 렌더 루프에 `operatorStatusBar.updateHealth({ageMs, staleMs, rosbridgeConnected})` 호출을 추가하는 별도 수정 필요.

### 설계 의도 재정정 — Operator 단독이 아니라 Admin+Operator 동시 기동 (2026-09-10, 같은 날 추가 세션)

위 "Operator 단독 기동 시 브릿지 노드 누락" 수정은 **Operator를 Admin 없이 단독으로 띄우는 경우**를 전제로 한 것이었다. 그런데 사용자가 명확히 한 원래 설계 의도는 그게 아니라, **`ur7e_xdg5f_atag_right_sim_dev` 하나로 Admin(8765/9090/9091)과 Operator(8766/9092)를 동시에 띄워서, 보는 사람이 원하는 URL로 골라 접속**하는 것이었다. 직전 수정 시점의 `xela_driver_dev.launch.py`는 Admin(`xela_taxel_sidecar_dg5f`) include를 완전히 빼버려 Operator만 뜨는 구조였는데, 이는 원래 설계와 달랐다.

**문제**: Admin+Operator를 그대로 동시에 켜면, Admin이 이미 `xela_taxel_web_bridge_node`(name=`xela_taxel_web_bridge_cpp`)와 `xela_atag_taxel_viewer_node`를 자체적으로 기동하는데, 직전 세션에서 추가한 Operator의 자체 기동 코드도 동일 노드/이름으로 중복 실행을 시도해 이름 충돌이 발생할 수 있는 상태였다.

**수정**:
1. `xela_taxel_operator_dg5f/launch/xela_taxel_operator_dg5f.launch.py`에 launch arg `enable_data_bridge_nodes`(기본값 `true`) 추가. `bridge_node`(xela_taxel_web_bridge_cpp)와 `xela_atag_taxel_viewer` include 양쪽 모두 조건을 `enable_taxel_bridge`/`enable_taxel_viewer` AND `enable_data_bridge_nodes`로 강화(`PythonExpression`으로 두 조건 결합). Operator 단독 기동 시엔 `true`(기존 동작 유지), Admin과 같이 뜰 때는 `false`로 넘겨 중복 기동을 막는다. 파일 상단 docstring에 이 설계 정정과 플래그 의미를 addendum으로 기록.
2. `ur7e_xdg5f_atag_right_sim_dev/launch/xela_driver_dev.launch.py`를 재작성해 4개 include를 전부 포함:
   - `std_xela_taxel_viz_dg5f`(공용, 기존과 동일)
   - `xela_taxel_sidecar_dg5f/launch/xela_taxel_sidecar_cpp.launch.py`(Admin, 8765/9090/9091) — `ur7e_xdg5f_atag_right_common/launch/xela_driver.launch.py`의 `sidecar_launch` include에 쓰인 launch_arguments를 그대로 복제(값 1:1 동일)
   - `xela_atag_taxel_viewer/launch/xela_atag_taxel_viewer.launch.py`(공용 세션/알림 노드) — 여기서 딱 1번만 include, Admin/Operator 양쪽이 DDS로 공유
   - `xela_taxel_operator_dg5f/launch/xela_taxel_operator_dg5f.launch.py`(Operator, 8766/9092) — `enable_data_bridge_nodes:='false'`로 넘겨 중복 기동 방지
   - 파일 상단 docstring을 "Operator 단독" 전제에서 "Admin+Operator 동시 기동"로 정정
   - `xela_taxel_sidecar_dg5f`, `ur7e_xdg5f_atag_right_sim`, `ur7e_xdg5f_atag_right_common`은 이번에도 무수정(읽기 전용) — `xela_taxel_operator_dg5f`, `ur7e_xdg5f_atag_right_sim_dev`만 수정
3. `xela_taxel_operator_dg5f/package.xml`의 exec_depend는 기존 그대로 유지(변경 불필요 — `xela_taxel_sidecar_dg5f`/`xela_server2_dg5f`/`std_xela_taxel_viz_dg5f`/`xela_atag_taxel_viewer`/`xela_taxel_viz_core` 전부 이미 선언돼 있었음)

**검증 (실측, 2026-09-10)**:
- `moveit_pro build user_workspace --colcon-args "--packages-select xela_taxel_operator_dg5f ur7e_xdg5f_atag_right_sim_dev"` 성공(컨테이너 내부 증분 빌드, 약 1.6s)
- `moveit_pro down` → `moveit_pro run --headless -c ur7e_xdg5f_atag_right_sim_dev` 재기동
- `docker exec moveit_pro-drivers-1 ss -tlnp` → 8765/9090/9091/8766/9092 **5개 포트 전부 오픈** 확인
- `docker exec -e CYCLONEDDS_URI=file:///home/invokelee/.ros/cyclonedds.xml moveit_pro-drivers-1 ... ros2 node list` → `/xela_taxel_web_bridge_cpp`, `/xela_atag_taxel_viewer_node` 각각 **정확히 1개씩**만 존재 확인(중복 없음). `/rosbridge_websocket`, `/rosbridge_websocket_sidecar`, `/rosbridge_websocket_taxel_viewer`, `/rosbridge_websocket_operator_dg5f` 4개 rosbridge 인스턴스가 공존(설계상 정상, 각기 다른 포트)
- `ros2 topic info /x_taxel_dg5f/web_state --verbose`, `/atag/taxel_data --verbose`, `/xvizdg5f/tf --verbose` 모두 **Publisher count 1**로 중복 퍼블리셔 없음 확인
- Playwright(headless, `--disable-gpu --use-gl=swiftshader`)로 `http://localhost:8765`(Admin)와 `http://localhost:8766`(Operator) 양쪽 스크린샷 확인 — 둘 다 콘솔 에러 없음, "Connected"/"connected" 배지 정상, 124개 taxel 화살표 마커가 손 mesh 위에 정상 렌더링됨(양쪽 화면 육안 확인)
- 검증 후 `moveit_pro down`(docker kill/logs 미사용)으로 정상 종료
- `git status`/`git diff --stat`로 `xela_taxel_sidecar_dg5f`, `ur7e_xdg5f_atag_right_sim`, `ur7e_xdg5f_atag_right_common` 3개 패키지 diff 0줄(무수정) 재확인
- `~/.config/moveit_pro/moveit_pro_config.yaml`을 baseline(`STUDIO_CONFIG_PACKAGE: ur7e_xdg5f_atag_right_sim`)으로 복원 확인

**남은 이슈**: 이전 세션에서 발견된 `FAULT` 배지 UI 배선 누락은 이번 작업 범위 밖으로 그대로 남음(위 항목 참고). Admin/Operator 카메라 앵글을 맞춘 픽셀 단위 비교는 수행하지 않음(선택 사항, 로직/데이터 동등성은 충분히 확인됨).

### FAULT 상태바 고정 표시 버그 — 수정 완료 (2026-09-10)

**원인 재확인**: 위 "남은 이슈"에서 지목된 대로, `xela_taxel_operator_dg5f/web/index.html`이 `xela_atag_taxel_viewer`의 `createOperatorStatusBar()`(`web/js/operator/operator_status_bar.js`, `src/atag/xela_atag_taxel_viewer` — 이 워크스페이스에선 `xela_apps` submodule이 아니라 최상위 repo `src/atag/` 아래에 있음)는 정상적으로 생성/`onGraspEvent`까지는 연결돼 있었으나, `updateHealth({ageMs, staleMs, rosbridgeConnected})`를 단 한 번도 호출하지 않아 `render("fault")`로 초기화된 마크업이 그대로 남아있었던 것. 위젯 자체는 dg5f Admin과 완전히 동일한 컴포넌트(무수정 공용 코드)라 로직 이식이 아니라 순수 "배선 누락" 문제였음 — dg5f Admin(index.html 4041행 `operatorStatusBar?.updateHealth({...})`)의 호출부만 그대로 옮기면 되는 케이스로 확인.

**수정 내용** (`xela_taxel_operator_dg5f/web/index.html`만 수정, `xela_taxel_viz_core`/baseline 3종/`xela_atag_taxel_viewer` 전부 무수정):
1. `let lastReceivedMs = 0;`, `let rosbridgeConnected = false;` 상태 변수 추가
2. rosbridge `onOpen`/`onCloseOpened`/`onErrorOpened` 콜백에서 `rosbridgeConnected` 갱신
3. `/x_taxel_dg5f/web_state` 메시지 파싱 성공 시 `lastReceivedMs = performance.now()` 갱신
4. `main()`의 `requestAnimationFrame` 렌더 루프에 dg5f Admin과 동일한 120ms 스로틀로 `operatorStatusBar.updateHealth({ ageMs, staleMs: 3000, rosbridgeConnected })` 호출 추가

**검증 (실측, 2026-09-10)**: symlink-install이라 재빌드 불필요(수정 즉시 컨테이너에 반영됨을 `docker exec`로 심볼릭 링크 확인). 이미 기동 중이던 `ur7e_xdg5f_atag_right_sim_dev`(Admin 8765/9090/9091 + Operator 8766/9092) 스택에 Playwright(headless, swiftshader)로 `http://localhost:8766` 접속 — 접속 0.5초 후부터 상태바가 `op-pill ok`/`OK`/`live`로 정상 표시(더 이상 고정 FAULT 아님), 3초/6초 후에도 유지됨을 확인. 콘솔 에러 없음(ColladaLoader Z-up 안내/GPU stall 경고만 있음, 기존에도 있던 무해한 로그).

---

### 레이아웃 버그 수정 — 사이드바형 → 세로 스택형 (2026-09-10, 같은 날 추가 세션)

사용자가 `xela_taxel_operator_dg5f/web/index.html`의 레이아웃이 잘못됐다고 지적 — 3D 뷰가 뷰포트 전체 높이를 차지하고 모듈 체크박스/그래프/필름스트립/알림카드가 전부 360px 사이드바에 욱여넣어져 있었음.

**원인**: 원본 dg5f Operator 모드는 `.app { grid-template-rows: auto auto auto auto auto }`인 세로 스택형 풀폭 레이아웃(상태바 → 모듈 툴바 → `.operator-viz-row`(3D뷰 420px 고정+옆에 좁은 alert 카드) → 세션/그래프 → 필름스트립)인데, 신규 페이지는 이를 이식하지 않고 `#app { grid-template-columns: 1fr 360px }`라는 별도의 2열(좌:3D뷰 풀높이/우:360px 사이드바) 레이아웃을 새로 만들었던 것.

**수정** (`xela_taxel_operator_dg5f/web/index.html`만 수정, CSS/DOM 배치만 — 기능 로직은 전혀 건드리지 않음):
1. `#app`을 `grid-template-columns` 2열 방식에서 원본과 동일한 `grid-template-rows: auto auto auto auto auto` 세로 스택 방식으로 교체
2. `.operator-viz-row` 규칙을 원본(`xela_taxel_sidecar_dg5f/web/taxel_sidecar/index.html` 377~386행)에서 거의 그대로 가져옴: `display:flex; height:420px; min-height:220px; max-height:90vh; resize:vertical; overflow:hidden`
3. `#mainViewCol`(3D뷰)은 `flex:1 1 auto; height:100%`로 `.operator-viz-row` 안에서 나머지 공간을 차지, `#alertsHost`(알림카드)는 원본 `#operatorAlertsHost`(150행)와 동일하게 `flex:0 0 280px; height:100%`로 3D뷰 옆 좁은 컬럼으로 배치
4. DOM 순서를 상태바 → (뷰즈컨트롤+모듈 체크박스, 풀폭 한 줄) → `.operator-viz-row`(3D뷰+알림카드) → `#graphHost`(풀폭) → `#filmstripHost`(풀폭, 맨 아래) 순으로 재배치
5. JS가 참조하는 id(`statusBarHost`/`vizControlsHost`/`modulesHost`/`mainViewCol`/`connOverlay`/`urdf3dLayer`/`alertsHost`/`graphHost`/`filmstripHost`)는 전부 그대로 유지 — CSS 배치(그리드 컬럼/로우)만 변경했으므로 JS의 `document.getElementById` 참조는 전혀 깨지지 않음
6. `#graphHost`의 초기 `display:none`(Graphs 토글 전 기본 숨김 상태, `graphsShown=false`와 일치)은 인라인 스타일로 유지

**검증 (실측, 2026-09-10)**: symlink-install이라 재빌드 불필요 확인(패키지/launch 변경 없음, `install/xela_taxel_operator_dg5f/.../web/index.html`이 소스 파일로의 심볼릭 링크임을 재확인). `moveit_pro run --headless -c ur7e_xdg5f_atag_right_sim_dev`로 Admin+Operator 동시 기동(5개 포트 8765/9090/9091/8766/9092 전부 오픈 확인) → Playwright(headless, swiftshader)로 `http://localhost:8766` 접속:
- 레이아웃 실측(`getBoundingClientRect`): `.operator-viz-row`가 풀폭(1576px)×420px 고정 높이, `#mainViewCol`(1286px)+`#alertsHost`(280px)가 그 안에 나란히 배치, `#statusBarHost`/`#filmstripHost`가 풀폭(1576px)으로 뷰포트 상단/하단에 배치됨을 좌표로 확인 — 더 이상 3D뷰가 전체 높이(1000px)를 차지하지 않음
- 스크린샷 확인: 상태바(OK/Phase/Last/Today) → Sensitivity/Graphs 버튼+모듈 체크박스(F1~F5/Palm, 풀폭) → 3D 뷰(420px, 손 mesh 정상 렌더링, "connected" 배지)+알림카드("No events yet.") 나란히 → Graphs 클릭 시 세션 패널(Live/Capture/Refresh/Load/Download 버튼, 모듈별 그래프 체크박스, force_total/shear 라인차트 등)이 풀폭으로 아래 표시 → 필름스트립("No frames yet.")이 맨 아래 풀폭 — Admin(8765)의 Operator 모드 화면(`operatorBtn` 클릭 후 스크린샷)과 구조적으로 동일함을 육안 확인
- 클릭 재검증: 모듈 체크박스 클릭 시 `checked` 상태 토글 확인(false→true), Viz Controls의 Sensitivity/Graphs 버튼 클릭 시 텍스트가 "Sensitivity: Normal"→"Mid", "Graphs: Hidden"→"Shown"으로 바뀌고 `#graphHost`의 `style.display`가 `none`→`""`로 실제 전환됨을 확인 — 레이아웃 변경 후에도 기존 배선(콜백)이 전혀 깨지지 않음. 콘솔 에러 없음(두 스크린샷 모두)
- `moveit_pro down`으로 정상 종료(docker kill/logs 미사용), `~/.config/moveit_pro/moveit_pro_config.yaml`을 baseline(`STUDIO_CONFIG_PACKAGE: ur7e_xdg5f_atag_right_sim`)으로 복원 확인
- baseline 4개 패키지(`xela_taxel_sidecar_dg5f`, `ur7e_xdg5f_atag_right_sim`, `ur7e_xdg5f_atag_right_common`, `xela_atag_taxel_viewer`) 전부 `git diff --stat` 0줄(무수정) 재확인 — `xela_atag_taxel_viewer`는 최상위 repo의 `src/atag/xela_atag_taxel_viewer`(별도 repo)에 위치, 여기도 diff 0줄

**남은 미세 차이(의도된 것, 회귀 아님)**: 원본은 상태바/모드버튼 툴바가 두 줄(status-toolbar + control-toolbar)로 나뉘고 모듈 체크박스가 별도 세 번째 줄인 반면, 신규 페이지는 Viz Controls(Sensitivity/Graphs)와 모듈 체크박스를 한 줄에 나란히 배치(`.full-width-row`) — Admin 전용 모드 전환 버튼(Grid/XelaModel/RobotModel/FollowCam 등)이 애초에 없는 Operator 전용 페이지라 툴바를 하나로 합쳐도 정보 밀도상 문제 없다고 판단, 필요 시 다음 세션에서 사용자 피드백에 따라 두 줄로 분리 가능.

---

### 레이아웃 미세조정 — 툴바 컴팩트화 + 그래프 섹션 정리(heatmap/모듈선택 제거, 상단↔그래프 동기화, line+shear 나란히 배치) (2026-09-10/11)

사용자가 세로 스택형 레이아웃 확정 이후 스크린샷 기준으로 세 가지 미세조정을 요청 — 재논의 없이 바로 구현.

**수정 내용** (`xela_taxel_operator_dg5f/web/index.html`만 수정, CSS/JS 배선만 — `xela_taxel_viz_core`/`xela_atag_taxel_viewer`/baseline 4종 전부 무수정):

1. **상단 툴바 컴팩트화**: `.full-width-row`에 `flex-wrap: nowrap; overflow-x: auto`를 적용하고, `#vizControlsHost .op-viz-controls`/`#modulesHost .op-modsel`의 padding/gap을 축소(`op_viz_controls.js`/`operator_module_select.js`는 무수정, 스코프 CSS 오버라이드만). 결과: `Sensitivity: Normal` / `Graphs: Shown` 버튼 + `F1 [ft mid prox]` ~ `F5 [ft mid prox]` + `Palm [uSPa46]` 체크박스가 한 줄에 가로로 배치됨.
2. **그래프 섹션 heatmap/모듈선택 제거**: `taxel_session_panel.js`(xela_atag_taxel_viewer, 무수정)에는 이를 끌 수 있는 옵션이 없음을 확인 — dg5f 원본 `index.html`의 `#operatorGraphHost` CSS 오버라이드 패턴(`:has()` 셀렉터로 역할/인접성 기반 선택, ~161~215행)을 그대로 참고해 `#graphHost` 스코프로 이식:
   - `canvas[data-role="heatmap"]` + 그 라벨(`:has(+ canvas[data-role="heatmap"])`)을 `display:none`
   - `.taxel-session-module-checklist`(f1~f5/mid/prox 모듈 선택 체크박스)를 `display:none` — 단, `.taxel-session-controls-row` 전체를 숨기는 dg5f 원본과 달리, 그 형제인 `.taxel-session-view-toggles`(force_total/shear_mag_avg/shear_normal_ratio 필터)는 명시적으로 다시 `display:flex`로 살려둠(요구사항이 dg5f와 달리 이 필터 3개는 유지하라고 명시)
   - heatmap 전용이라 이제 무의미해진 "Amplify heatmap (log color scale)"/"Local window scale (±3s)" 체크박스도 `:has()`로 각각 숨김(요구사항의 "유지" 목록에 없었고, 가려진 heatmap을 제어하는 죽은 UI라 판단)
   - Capture Start/Clear View/세션 이름 입력 등 Live 토글과 같은 줄에 있는 나머지 컨트롤은 dg5f 원본과 동일하게 `:not([data-action="toggle-live"])`로 숨김. 세션 load/Refresh, time-scrub, Follow Latest는 요구사항에서 제거 대상으로 명시되지 않아 그대로 유지(과거 기록 리뷰용으로 유용 판단, dg5f 원본도 scrub/Follow Latest는 별도로 다시 살려둠)
3. **상단↔그래프 모듈 동기화**: `taxel_session_panel.js`에 `setSelectedModules()` 같은 외부 주입 API가 없음을 확인 — dg5f 원본의 `setModuleCheckboxSelection()` 패턴(체크박스 DOM을 직접 토글 후 `change` 이벤트 디스패치, 패널 내부의 `onModuleSelectionChanged`가 원래 로직대로 반응)을 그대로 이식한 `syncGraphModuleSelection(names)` 함수를 index.html에 추가하고, 상단 `operatorModuleSelect`의 `onChange` 콜백에서 호출. 상단 모듈 키(`f1_dg5f_ft` 등)와 세션 패널의 체크박스 `value`가 동일한 명명규칙(`DG5F_MODULE_NAMES`)을 쓰므로 별도 매핑 불필요 — 재구현이 아니라 기존 체크박스를 대신 클릭해주는 "연결"만 수행.
4. **line+shear snapshot 나란히 배치**: `#graphHost .taxel-session-card-body`를 `display:grid; grid-template-columns: 1fr 200px`로 바꾸고, `:has(+ canvas[data-role="line"])`/`canvas[data-role="line"]`을 1열, `:has(+ canvas[data-role="quiver"])`/`canvas[data-role="quiver"]`를 2열에 명시적으로 배치(숨겨진 heatmap 항목은 grid 흐름에서 자동 제외되므로 순서 영향 없음). 라벨(`f1_dg5f_ft`, `N samples`, series 설명, "Shear (x,y) snapshot..." 등)은 컴포넌트 원본 그대로 유지.

**검증 (실측, 2026-09-10/11, Playwright headless swiftshader, `http://localhost:8766`)**:
- 기존에 떠 있던 `ur7e_xdg5f_atag_right_sim_dev`(Admin 8765/9090/9091 + Operator 8766/9092, symlink-install이라 재빌드 불필요, `docker exec`로 컨테이너 내 파일이 최신 수정사항 그대로임을 grep으로 확인) 스택을 재사용해 검증
- 상단 툴바: `.full-width-row` bounding rect 폭 1576px(풀폭), 높이 145px 한 줄 — 스크린샷으로 F1~F5[ft/mid/prox]+Palm[uSPa46] 체크박스와 Sensitivity/Graphs 버튼이 한 줄에 나란히 배치됨을 육안 확인
- Graphs 클릭 → `#graphHost.style.display`가 `none`→`""`로 전환 확인
- heatmap: `canvas[data-role="heatmap"]`의 `getComputedStyle().display === "none"` 확인(육안 스크린샷에도 부재), 그래프 섹션 자체의 모듈 체크박스(`.taxel-session-module-checklist`)도 `display:none` 확인. Live 토글/force_total/shear_mag_avg/shear_normal_ratio 체크박스는 visible 확인(heatmap 전용 Amplify/Local-window 토글은 추가로 숨김 처리 후 재검증 완료)
- 동기화: 상단 F3 `ft` 체크박스 클릭 → 그래프 섹션에 `f3_dg5f_ft` 카드 1개만 생성되고 해당 패널 내부 체크박스(`[data-module-checkbox][value="f3_dg5f_ft"]`)도 `checked:true`로 동기화됨을 확인. 다시 해제 → 카드가 사라짐(빈 배열)까지 확인 — 상단 선택이 그래프의 유일한 선택 소스로 동작
- line+shear 나란히: F3 카드의 `canvas[data-role="line"]`(width 1344px)과 `canvas[data-role="quiver"]`(width 200px)의 `getBoundingClientRect()`가 `top` 동일(같은 행)·`left` 순서(line 다음에 quiver)로 나란히 배치됨을 좌표로 확인, 스크린샷으로도 재확인
- 기존 기능 재검증: Live 토글 클릭 시 "Live: Off"→"Live: On" 전환, Sensitivity 버튼 클릭 시 "Sensitivity: Normal"→"Mid" 전환 모두 정상. Capture Start/Clear View 등은 Live 토글과 같은 줄이라 위 3번 규칙대로 숨겨짐(의도된 동작, 회귀 아님) — 별도 UI 노출 없이도 서비스 로직 자체는 변경하지 않았으므로 기능 손실 없음
- 콘솔 에러 없음(모든 스크린샷/평가에서 확인)
- `moveit_pro down`으로 정상 종료(docker kill/logs 미사용)
- baseline 4개 패키지(`xela_taxel_sidecar_dg5f`, `ur7e_xdg5f_atag_right_sim`, `ur7e_xdg5f_atag_right_common`, `xela_atag_taxel_viewer`) 전부 `git diff --stat` 0줄(무수정) 재확인
- `~/.config/moveit_pro/moveit_pro_config.yaml`을 baseline(`STUDIO_CONFIG_PACKAGE: ur7e_xdg5f_atag_right_sim`)으로 복원 확인

**남은 이슈**: 세션 load(Refresh/Load/session-select 드롭다운)/time-scrub/Follow Latest는 이번 요구사항의 "제거" 목록에 없어 그대로 유지했는데, 화면이 더 좁아 보일 수 있어 다음 세션에서 사용자가 이것들도 숨기길 원하는지 확인 필요(현재는 보존이 맞다고 판단해 그대로 둠). Admin(8765) 쪽 Operator 모드 화면과의 픽셀 단위 비교는 수행하지 않음(Admin은 heatmap 전용 토글을 숨기지 않는 등애초에 요구사항이 달라 대상 아님).

---

### 레이아웃 회귀 5건 수정 — 동시편집 충돌 추정 (2026-09-11)

**배경**: 위 "툴바 컴팩트화" 라운드 직후, 코디네이터 실수로 에이전트 2개가 **동시에 같은 `index.html`을 수정**하는 사고가 있었음이 사후 확인됨(별도 세션에서 발견/기록). 이번 세션에서 사용자가 실측 스크린샷 2장 기준으로 새 레이아웃 버그 5개를 보고 — 그 동시편집 충돌의 부작용으로 추정하고 조사.

**환경**: `~/.config/moveit_pro/moveit_pro_config.yaml`이 이미 `ur7e_xdg5f_atag_right_sim_dev`로 설정된 상태로 Admin+Operator 스택(`moveit_pro-drivers-1`/`agent_bridge`/`web_ui`)이 이미 기동 중이었음(사용자가 실시간 검증 중이던 세션) — **이 세션을 그대로 재사용**, 새로 기동/재기동하지 않음. symlink-install이라 `index.html` 수정이 재빌드 없이 즉시 반영됨을 매번 `md5sum`(소스 파일 vs `curl http://localhost:8766/`)으로 재확인.

**버그별 정확한 원인 (Playwright `getComputedStyle`/`getBoundingClientRect` 실측 근거)**:

1. **초기 화면 툴바 위 거대한 빈 공간**: `#app`이 `height: 100%`(정의된 높이)에 `grid-template-rows: auto auto auto auto auto`였는데, CSS Grid의 `align-content`(기본값 `normal`)는 grid 컨테이너에서는 flexbox와 달리 `stretch`처럼 동작하는 스펙상의 함정 — 컨테이너에 정의된 높이가 있고 트랙이 전부 `auto`면, 실제 콘텐츠 높이를 채우고 남는 여유 공간이 5개 행에 **균등하게** 분배되어 각 행이 부풀려짐. 실측: 1600×1000 뷰포트, Graphs Hidden 상태에서 `getComputedStyle(#app).gridTemplateRows` = `"115px 125px 498px 78px 120px"` — 실제 콘텐츠 높이(상태바 ~35px, 툴바 ~46px, 필름스트립 ~40px)보다 행마다 정확히 **+79px씩** 부풀려짐(여유공간 395px ÷ 5행). 상태바 행(115px) 중 실제 텍스트는 상단 35px에만 있고 나머지 80px가 빈 공간으로 보였던 것.
2/3. **Graphs 켜면 툴바 행이 사라짐 / Live On 누르면 다시 나타남**: 별개의, 더 심각한 원인. `.full-width-row`(툴바)에 지난 라운드에서 추가된 `overflow-x: auto`가 `overflow-y`도 암묵적으로 `auto`로 만듦(CSS 스펙상 한 축이 visible이 아니면 다른 축도 visible일 수 없음). Grid 아이템의 **자동 최소 크기**(auto 트랙이 공간 부족 시 참조하는 최소값)는 `overflow`가 `visible`이 아니면 콘텐츠 기반이 아니라 **0으로 강제**되는 스펙 규칙이 있음. Graphs를 켜고 실제 세션 카드가 생기면 전체 콘텐츠 높이가 뷰포트를 초과해 공간이 부족해지고, 이때 브라우저가 자동-최소-0인 이 행을 우선적으로 0으로 축소 — 실측: `getComputedStyle(#app).gridTemplateRows`가 `"37px 0px 420px 663.797px 42px"`(2번째 행이 정확히 0px)로 확인, `.full-width-row.getBoundingClientRect().height === 0`, 그 자식(Sensitivity/Graphs 버튼)은 `align-items:center`로 인해 0높이 박스 중심을 기준으로 위아래로 삐져나와 렌더링되지만 `document.elementsFromPoint()`로는 그 좌표에서 버튼이 전혀 히트테스트되지 않고 `#app`/3D뷰 캔버스가 대신 잡힘(Playwright 클릭이 실제로 실패하는 것으로 재확인). `#filmstripHost`에도 동일 클래스의 `min-height: 0;`이 명시적으로 박혀 있어 같은 공간 부족 상황에서 함께 0으로 축소될 수 있는 상태였음. Live On으로 "다시 나타난" 것처럼 보인 것은 실제로는 매 상태 변화(체크박스/버튼 클릭)마다 콘텐츠 높이가 재계산되면서 여유/부족 공간의 경계값이 바뀌어 우연히 다시 여유가 생긴 특정 순간을 사용자가 관찰한 것으로 추정 — 근본 원인은 "행이 사라짐"이 아니라 "grid 트랙이 콘텐츠 압박 시 0으로 축소될 수 있는 상태"였다는 것.
4. **Live On/Follow Latest 버튼이 Sensitivity/Graphs와 다른 스타일**: `taxel_session_panel.js`(무수정)의 `.taxel-session-btn` 규칙은 `cursor:pointer; padding:4px 10px`만 정의하고 나머지는 UA(브라우저) 기본 버튼 스타일에 의존 — 반면 `operator_viz_controls.js`(무수정)의 `.op-viz-btn`은 `border-radius:100px` 등 pill 스타일을 명시적으로 가짐. 두 컴포넌트가 원래부터 다른 스타일 규칙을 가진 것이며 "덮어써진" 게 아니라 애초에 통일 안 된 것으로 확인.
5. **force_total/shear_mag_avg/shear_normal_ratio 세로 스택**: `taxel_session_panel.js`(무수정)의 `.taxel-session-view-toggles { display:flex; flex-direction:column; gap:4px; }`가 기본 세로 배치임을 소스로 확인.

**수정** (`xela_taxel_operator_dg5f/web/index.html`만 수정, viz_core/atag_taxel_viewer/baseline 4종 전부 무수정):
1. `#app`에 `align-content: start;` 추가 — grid의 stretch 여유공간 분배를 끄고 각 행을 콘텐츠 높이 그대로 렌더링.
2. `#app`의 `grid-template-rows`를 `auto auto auto auto auto`에서 `min-content min-content auto auto min-content`로 변경(1/2/5행만 `min-content` 트랙 함수로 명시) — 트랙 자체의 최소 크기를 콘텐츠 기반으로 고정해, 위 "아이템의 자동-최소-크기가 0으로 강제되는" 스펙 규칙의 영향을 받지 않게 함(1번만으로는 콘텐츠 압박 상황에서 재발함을 실측으로 확인해 추가). `.full-width-row`에도 `min-height: min-content;`를 추가로 보강. `#filmstripHost`의 `min-height: 0;`은 제거(기본값 `auto`로 복원).
3. `#graphHost .taxel-session-btn`에 `.op-viz-btn`과 동일한 pill 스타일(border-radius:100px, 테두리, padding, 폰트) CSS 오버라이드 추가 — Live On/Off, Follow Latest 버튼에 적용됨.
4. `#graphHost .taxel-session-view-toggles`에 `flex-direction: row; gap: 14px;` 오버라이드 추가 — force_total/shear_mag_avg/shear_normal_ratio가 한 줄에 나란히 배치됨.

**검증 (실측, Playwright headless swiftshader, `http://localhost:8766`, 기존 세션 재사용)**:
- 뷰포트 3종(1600×1000/1366×768/1920×1080) × 시나리오(모듈 체크박스 선택 → Graphs 켜기 → Live On 켜기, 실제 클릭 시퀀스) 전부에서 `.full-width-row.getBoundingClientRect().height > 20`(항상 보임)와 `document.elementFromPoint(버튼중심좌표) === 버튼자신`(항상 클릭 가능) 확인 — 이전엔 checkbox 활성화+Graphs+Live 조합에서 재현되던 0px 붕괴가 전부 해소됨
- 초기 상태(1600×1000, Graphs Hidden): `getComputedStyle(#app).gridTemplateRows` = `"37px 47px 420px 0px 42px"`(graphHost는 display:none이라 0, 나머지는 전부 콘텐츠 그대로) — 이전의 "115px 125px..." 부풀림 사라짐, 스크린샷으로 상태바 바로 아래 툴바가 붙어있음을 확인
- Graphs 토글 시 `#graphHost.style.display`가 `none↔""`로 정상 전환, 툴바/필름스트립 위치·높이 불변 유지 확인
- 버튼 스타일: `getComputedStyle()`로 `#graphHost [data-action="toggle-live"]`/`[data-action="follow-latest"]`의 `borderRadius`(100px)/`border`/`font`가 `#vizControlsHost .op-viz-btn`과 동일함을 비교 확인, 스크린샷으로도 pill 모양 일치 확인
- 체크박스 3개: `getComputedStyle('.taxel-session-view-toggles').flexDirection === "row"`, 3개 라벨의 `getBoundingClientRect()`가 동일 y좌표·증가하는 x좌표(가로 배치)임을 좌표로 확인
- **기존 기능 회귀 없음 재확인**: 상단 F2 mid 체크박스 클릭 → 그래프 섹션에 `f2_mid_uSPa22` 카드 1개 생성(모듈 동기화 정상), heatmap 캔버스 visible count 0(계속 숨김 유지), 그래프 자체 모듈-체크리스트 visible count 0(계속 숨김 유지), Sensitivity 버튼 클릭 시 "Normal"→"Mid" 정상 전환, Live On 클릭 시 "Live: Off"→"Live: On" 정상 전환. 모든 시나리오에서 콘솔 에러 0건.
- `git status --short`/`git diff --stat`로 baseline 4개 패키지(`xela_taxel_sidecar_dg5f`, `ur7e_xdg5f_atag_right_sim`, `ur7e_xdg5f_atag_right_common`, `xela_atag_taxel_viewer`) 무수정 재확인 — 전부 결과 없음(clean)
- 컨테이너: 이번 세션 시작 전부터 사용자가 `ur7e_xdg5f_atag_right_sim_dev`로 이미 기동해둔 세션(`moveit_pro-drivers-1`/`agent_bridge-1`/`web_ui-1`)을 그대로 재사용 — 신규 기동/재기동 없음, 검증 후에도 `moveit_pro down` 등으로 내리지 않고 그대로 유지함(사용자가 계속 사용 중일 수 있음). `moveit_pro_config.yaml`도 건드리지 않음(세션 시작 전부터 `ur7e_xdg5f_atag_right_sim_dev`였고 변경하지 않음).

**남은 이슈**: 없음(보고된 5건 전부 실측 재현 → 원인 확정 → 수정 → 재검증 완료). 다만 grid 트랙 붕괴 계열 버그는 CSS Grid의 "auto 트랙 + non-visible overflow 아이템"이라는 스펙상 함정이 근본 원인이라, 향후 이 페이지에 새 grid 행/overflow 규칙을 추가할 때 동일 패턴(자동 최소 크기 0 강제)이 재발하지 않도록 `min-content` 트랙 함수 또는 명시적 `min-height`를 항상 함께 고려할 것.

---

## Operator 기능 갭 목록 (2026-09-10 전수 조사)

Admin(`xela_taxel_sidecar_dg5f/web/taxel_sidecar/index.html`)의 `setOperatorMode(enabled)`(3574행)가 켜는 Operator 화면 구성요소를 전수 확인하고, 신규 `xela_taxel_operator_dg5f/web/index.html`과 코드 비교 + Playwright 실측(클릭/조작)으로 대조한 결과.

### 완전히 이식되어 정상 동작 확인된 기능 (실제 클릭/조작으로 검증)

모두 `xela_atag_taxel_viewer`의 동일 공용 컴포넌트를 무수정으로 `import`해서 조립한 것이므로 로직 자체는 Admin과 100% 동일 — 남은 건 "배선(구독/호출)이 신규 페이지에 있는가"뿐이었고, 아래는 전부 있음을 확인:

1. **상태바(`operator_status_bar.js`)** — OK/WARN/FAULT, Phase, Last outcome, Today 성공/실패 tally, health 텍스트. 이번 세션에 `updateHealth()` 배선 버그 수정 완료. `onGraspEvent`는 원래부터 배선돼 있었음.
2. **모듈 선택 체크박스(`operator_module_select.js`)** — F1~F5(ft/mid/prox)/Palm/uSPa46 그룹, 기본 전부 미체크(Admin과 동일한 "blank slate" 설계). Playwright로 체크박스 클릭 → 상태 토글 확인, `enabledModuleIds`가 실제 하이라이트 로직(`getEnabledModuleIds` 콜백)에 연결돼 있음을 코드로 확인.
3. **뷰즈 컨트롤(`operator_viz_controls.js`)** — Sensitivity(Normal→...) 순환 버튼, Graphs(Hidden/Shown) 토글 버튼. Playwright로 둘 다 클릭 → 텍스트/그래프 패널 `display` 스타일이 실제로 바뀜을 확인.
4. **알림 카드(`operator_alert_cards.js`)** — `/atag/grasp_event` 직접 구독(`onGraspEvent`)에 연결됨(코드 확인, 실제 grasp 이벤트 발생 재현은 이번 세션 범위 밖).
5. **필름스트립(`operator_filmstrip.js`)** — `tick()`/`onGraspEvent()` 배선 확인, 캡처는 Live 모드 + grasp_event 발생 시 동작(로직은 Admin과 동일 공유 코드).
6. **세션 패널(`taxel_session_panel.js`, `js/session/`)** — Live On/Off, Clear View, Capture Start/Stop, Refresh/Load, Download raw.csv/derived.csv, Follow Latest 버튼 전부 존재. Playwright로 실제 클릭: "Live: Off"→"Live: On" 상태 전환 확인, "Capture Start" 클릭 시 실제 rosbridge 서비스 호출까지 도달해 "capturing 'session'..." 상태 문구가 뜨는 것까지 확인(단순 DOM 존재가 아니라 백엔드 서비스 콜 성공까지 실측).
7. **taxel 마커 3D 렌더링(124개)** — Phase 2/3에서 이미 실측 검증 완료(본 문서 Phase 2/3 섹션), 이번 세션 재확인 불필요.
8. **손 mesh(16개 하우징) 렌더링** — Phase 3에서 `bindRobotDescriptionToMesh()` glue로 이미 해결/검증됨.

### 의도된 제외 (Admin에도 있지만 Operator 모드엔 원래 없었거나, 설계상 Operator 웹페이지 범위가 아닌 것)

1. **2D grid 뷰(`js/render/grid.js`)** — Admin GridMode 전용, `setOperatorMode(true)`가 강제로 `state.uiMode = "xela"`로 바꾸므로 Operator는 애초에 grid 모드에 진입하지 않음. 신규 페이지도 grid.js를 아예 import하지 않음 — 의도된 축소, 갭 아님.
2. **Follow-cam 토글, raw 디버그 패널** — Admin(dev-only) 전용 컨트롤. `setOperatorMode`가 `.dev-only`를 숨기는 것과 동일하게, 신규 페이지는 이 코드 자체를 로드하지 않음 — 의도된 축소.
3. **`operator_main_view_markers.js`(`createOperatorMainViewMarkerSettings`, "Show Behavior Event" 마커 설정)** — 2026-09-08 Admin 쪽 변경으로 "더 이상 Operator 전용이 아니라 Admin 화면에 상시 노출"로 이미 재분류된 기능(주석 확인: "lives in the Admin screen now... no longer Operator-only"). MoveIt Pro **Studio Visualization 패널**(RViz 유사 패널)에 텍스트 마커를 발행하는 Admin 전용 설정 UI이며, Operator 웹페이지 자체의 화면 요소가 아님 — Operator 페이지에 없는 것이 올바름. 신규 페이지도 이 모듈을 import하지 않음.
4. **`xela_viz_mode_manager_node`(grid/urdf 모드 전환 서비스)** — Phase 3.5에서 이미 문서화된 의도적 미포함(Operator엔 모드 토글 UI 자체가 없음).

### 이번 세션에서 발견되지 않은 잠재 리스크 (다음 세션 확인 권장, "진짜 갭"은 아니지만 미검증)

1. **알림 카드/필름스트립의 실제 grasp_event 발생 시 렌더링** — 이번 세션엔 실제 로봇 동작(BT 실행)으로 grasp_event를 발생시키지 못했음(단순 정적 검증만). 코드 배선은 Admin과 동일하므로 정상 동작 가능성 높지만, 실 그랩 사이클로 알림 카드 표시/필름스트립 캡처까지 실측하지는 않음.
2. **`operator_main_view_markers`의 부재가 Operator 페이지에 실질적 영향이 있는지** — 이 위젯이 켜져 있을 때 Admin이 발행하는 마커가 MoveIt Pro Studio 패널에 뜨는 것이지 Operator 웹페이지의 3D 뷰와는 무관하다고 판단했으나, Studio 패널을 직접 열어 확인하지는 않음(범위 밖으로 판단, 근거는 코드 주석과 토픽 발행 대상이 rosbridge가 아닌 것으로 추정됨 — 완전한 반증은 아님).
3. **Session panel의 "Refresh"/"Load"/Download 버튼의 실제 데이터 정확성** — 클릭 시 UI 상태 전환(예: capturing 문구)까지만 확인, 실제로 세션 목록이 채워지고 CSV가 정상 다운로드되는지는 검증하지 않음(session capture를 몇 초 이상 유지 후 stop까지 이어가는 전체 사이클은 이번 세션 범위 밖).

---



두 가지 방법 모두 가능:
1. **환경변수(권장, 일회성 실행)**: `moveit_pro run --config-package ur7e_xdg5f_atag_right_sim_dev` (또는 `-c` 축약) — `~/.config/moveit_pro/moveit_pro_config.yaml`을 건드리지 않고 이번 실행에만 적용됨
2. **config.yaml 영구 변경**: `~/.config/moveit_pro/moveit_pro_config.yaml`의 `STUDIO_CONFIG_PACKAGE` 값을 `ur7e_xdg5f_atag_right_sim_dev`로 수정 후 `moveit_pro run` — 이후 `moveit_pro run`을 인자 없이 실행할 때마다 계속 신규 패키지로 기동됨. **반드시 검증 후 baseline(`ur7e_xdg5f_atag_right_sim`)으로 되돌려 놓을 것** (이번 세션은 방법 2를 사용했고, 검증 완료 후 baseline으로 복원해둠)

신규 패키지 빌드가 필요하면 먼저 `moveit_pro build user_workspace --colcon-args "--packages-select ur7e_xdg5f_atag_right_sim_dev"`로 증분 빌드(전체 이미지 재빌드 불필요).

---

## Phase 4 — 성능 검증 (분리 효과 확인) — ✅ 완료(결론 수용, 2026-09-11)

**최종 결론 (사용자 확인)**: 이번 분리 프로젝트의 목적은 "체감 가능한 성능 개선 수치 확보"가 아니라 **Admin/Operator 역할을 명확히 분리해서, 운영자 입장의 데모/센서 모니터링 용도에 집중된 화면을 제공하는 것**으로 재확인됨. 실측 결과 자체는 (a) "2~3초마다 1초 멈춤" 패턴이 과거 세션에서 이미 근본수정되어 이번엔 재현 안 됨, (b) 이번 그랩 사이클이 `SwitchController` 실패로 완료되지 못해 최악 시나리오 재측정이 안 됨 — 두 가지 이유로 분리 전/후 비교 기준점이 성립하지 않았지만, 목적이 성능이 아니므로 이 결과를 그대로 수용하고 Phase 4를 종료함. `SwitchController` 실패 건은 성능 검증과 무관한 별개의 잠재 버그로, 이번 프로젝트 범위 밖에 남겨둠(다음 세션에서 필요 시 조사).

**작업**
- Chrome Performance 탭으로 (a) 기존 dg5f Operator 모드, (b) 신규 `xela_taxel_operator_dg5f` 단독 실행을 동일 시나리오(동일 로봇 동작 재생)로 각각 1~2분 프로파일링

**검증 체크리스트**
- [x] 프레임 드랍/멈춤 빈도 비교 (기존에 보고된 "2~3초마다 1초 멈춤" 패턴 재현 여부) — **재현 안 됨, 양쪽 동일 수준.** 아래 2026-09-11 실측 참고: Admin(8765)/Operator(8766) 양쪽 다 110초 캡처에서 long task(≥50ms) 543개/542개로 거의 동일, ≥900ms(체감 "1초 멈춤"급) long task는 **양쪽 다 0개**. 최대 long task 길이도 Admin 277.0ms vs Operator 290.0ms로 사실상 동일.
- [x] 메인 스레드 self-time 상위 함수 비교 — Admin 로직이 빠지면서 실제로 상위권에서 사라졌는지 확인 — 상위 항목은 양쪽 다 `(program)`(Chrome 내부/GPU 파이프라인, 98.3~98.4%)이 압도적이고, JS 레벨에서는 `ws.onmessage`(rosbridge 공용, 116.6ms vs 122.0ms 거의 동일)가 1위. Admin 전용 함수(`onTfMessage`, `handleRosbridgeMessage`, `selectFixedFrameUncached`, `normalizeFrameId`, `drawSparkline` 등, grid.js 계열)는 Admin 프로파일에만 존재하고 Operator에는 그 대신 `xela_taxel_viz_core`/`xela_atag_taxel_viewer` 전용 함수(`applyTfMessage`, `renderFromLatestPayload`, `onMessageParsed`, `updateUrdfMarkers`/`resolveFrameToFixed` @ `taxel_marker_renderer.js`)가 나타나 **코드 로드 자체는 분리가 확인**되지만, self-time 절대값(각 10~30ms대, 전체의 0.03% 이하)이 너무 작아 총합 self-time(79534.4ms vs 79622.2ms, 차이 0.1%)에는 유의미한 영향 없음.
- [x] 장시간(30분 이상) 연속 구동 시 메모리/캐시 누수 없는지 확인 — **시간 제약으로 30분 대신 110초로 축소**(사용자 승인된 축소 옵션 사용). `performance.memory.usedJSHeapSize` 캡처 시작/종료 비교: Admin 33.10MB→33.10MB(변화 없음), Operator 64.00MB→64.00MB(변화 없음, 최초 로드 시 무거운 3D 에셋/모듈 그래프 UI 때문에 시작치 자체는 Operator가 더 높음). 이 짧은 창에서는 두 쪽 다 누수 징후 없음. 30분 이상 장시간 구동 재현은 이번 세션 범위 밖으로 남김(아래 "남은 이슈" 참고).
- [x] 개선 수치를 사용자에게 보고하고, 목표(체감 가능한 멈춤 해소) 달성 여부 합의 — 아래 실측 결과 기반 결론: 이번 시나리오에서는 **Admin/Operator 분리가 CPU/long-task 수준에서 측정 가능한 개선을 만들지 못함**(둘 다 거의 동일). 단, 이는 "분리가 실패했다"는 뜻이 아니라 (a) 이번 grasp 사이클이 컨트롤러 스위치 실패로 매번 중단되어 실제 파지/임피던스 프리즈 이벤트가 발생하지 않았고, (b) self-time 상위가 애초에 JS(Admin 로직 포함)가 아니라 Chrome 내부 렌더 파이프라인(`(program)` 98%+)이라 Admin 로직 자체의 절대 비중이 애초에 작았기 때문. 원래 사용자가 보고한 "2~3초마다 1초 멈춤"은 이번 실측에서 Admin/Operator 어느 쪽에서도 재현되지 않았음(≥900ms 사건 0건) — 따라서 "분리로 그 멈춤이 해소됐는지"는 이번 실측만으로는 확인 불가(애초에 재현이 안 됐으므로 Before/After 비교가 성립하지 않음).

### 2026-09-11 Phase 4 실측 (Admin 8765 vs Operator 8766, 동시 프로파일링)

**시나리오**: `moveit_pro run --headless -c ur7e_xdg5f_atag_right_sim_dev`로 Admin(8765/9090/9091)+Operator(8766/9092) 5개 포트 동시 기동. `/execute_objective`(`moveit_studio_sdk_msgs/srv/ExecuteObjective`) 서비스로 `DG5F_Demo1_Success_PickPlace_ObjectGraspController` objective를 반복 트리거(각 사이클 실제 팔 동작/경로계획 발생, ~20초/사이클)하며 그 동안 Playwright(headless, `--enable-gpu --ignore-gpu-blocklist`)로 8765와 8766을 **같은 브라우저 프로세스 안에서 동시에 두 탭으로 열고**, 각 탭에 CDP `Profiler`(Sampling, interval 200us) + `PerformanceObserver('longtask')`를 걸어 110초간 동시 캡처.

**objective 트리거 방법(재사용 가능)**:
```
docker exec -e CYCLONEDDS_URI=file:///home/invokelee/.ros/cyclonedds.xml moveit_pro-agent_bridge-1 bash -lc \
  "source /opt/overlay_ws/install/setup.bash; ros2 service call /execute_objective moveit_studio_sdk_msgs/srv/ExecuteObjective \"{objective_name: 'DG5F_Demo1_Success_PickPlace_ObjectGraspController'}\""
```
(objective_name은 XML 파일명이 아니라 `main_tree_to_execute` 속성값이어야 함 — 파일명 그대로 넣으면 "Can't find a tree with name" 오류)

**알려진 제약(이번 세션에서 발견, 코드 미수정)**: `ur7e_xdg5f_atag_right_sim_dev`(dev 변형) 환경에서 pick-place objective 7사이클 전부 `SwitchController failed: enable_impedance=true effort=[dg5f_right_effort_controller] position=[dg5f_right_controller]`로 실패 — effort 기반 그립 컨트롤러 전환이 이 config에서 동작하지 않아 실제 파지/임피던스 프리즈/grasp_event까지는 도달하지 못하고 접근 경로 재생만 반복됨. 원인 미조사(이번 세션 범위 밖, Phase 4는 순수 측정이라 코드/config 수정 금지 원칙 준수). 따라서 이번 측정은 "팔 접근 모션 반복 재생" 부하이며, "그랩 완료+타젤 접촉 이벤트가 빈번한" 최악 시나리오는 아직 실측하지 못함.

**결과**: 위 체크리스트에 병기. 요약하면 Admin/Operator 두 프로파일이 long task 개수(543/542), 최대 long task 길이(277.0/290.0ms), 총 self-time(79534.4/79622.2ms), 힙 크기(변화 없음) 전부 오차범위 내에서 동일. 콘솔 에러 0건 양쪽 다.

**baseline 무수정 확인**: `git status --porcelain`(최상위 repo), `git -C src/xela_dependencies/xela_apps status --porcelain`(submodule) 전부 출력 없음(clean) — `xela_taxel_sidecar_dg5f`/`ur7e_xdg5f_atag_right_common`/`ur7e_xdg5f_atag_right_sim`/`xela_atag_taxel_viewer` 무수정 확인.

**남은 이슈**:
1. `ObjectGraspController`/`ObjectGrasp` 두 변형 모두 이 `_dev` sim에서 SwitchController 실패 — 실제 파지 완료 + grasp_event 빈발 시나리오는 이 문제를 해결(별도 세션, 코드/config 조사 필요)한 뒤에 재측정 필요. 이번 실측은 "팔 모션 반복" 부하로 제한됨.
2. 30분 이상 장시간 구동 누수 검증은 110초로 축소해 수행 — 장시간 검증은 미완료.
3. "2~3초마다 1초 멈춤" 패턴이 이번 실측에서 Admin 쪽에서도 재현되지 않아(≥900ms 사건 0건), 분리 전/후 비교의 기준점(before) 자체가 이번 세션에서 성립하지 않음 — 과거 수정(`resizeCanvas`/`resolveChildFrameAlias` 등, 2026-09-09 세션에서 이미 근본수정 완료)이 반영된 상태라 애초에 재현되지 않는 것으로 보이며, 별개로 Admin/Operator 분리 자체의 효과를 이 패턴 기준으로는 검증할 수 없음.

### 사전 실측: Admin 모드(8765)에서 Operator 전용 코드 상시 오버헤드 (2026-09-11)

`xela_taxel_sidecar_dg5f`는 무수정 유지, Playwright(headless Chromium)로 `http://localhost:8765`에 접속해 Operator 버튼을 누르지 않은 순수 Admin 상태에서 CDP Profiler(Sampling CPU Profile, 45초 창)로 실측. 코드 수정 없이 `page.evaluate()`로 런타임에서만 4개 항목을 껐다 켜서 A/B 비교(각 항목의 공개 API를 이용: `taxelViewerRosbridgeClient.close()`로 재연결까지 완전 차단, `/atag/grasp_event` unsubscribe, `operatorStatusBar.updateHealth` no-op).

**결과 요약**
- 45초 창 JS-only self-time(= 전체 self-time에서 Chrome 내부 `(program)`/`(idle)` 버킷 제외): 베이스라인 801.6ms → 4항목 비활성화 808.4ms (차이 -0.8%, 사실상 잡음 수준, 유의미한 감소 없음)
- `operatorStatusBar.updateHealth` 자체 self-time: 45초간 0.27ms — 항목 4는 이론상으로도 실측상으로도 무시 가능
- `ws.onmessage`/`handleRosbridgeMessage`(rosbridge 메시지 처리 공용 함수, main+taxelViewer 연결 공유): 베이스라인 89.6/39.2ms → 비활성화 84.1/40.4ms — 차이 없음
- 30초간 WS 프레임 실측(topic별 수신 횟수): `/tf` 268~271회(~9Hz), `/x_taxel_dg5f/web_state` 289회(~9.6Hz) — 반면 `/atag/grasp_event`, `/xela_atag_taxel_viewer_node/live_module_data`는 **0회 수신** (이번 idle 시나리오에서는 grasp 이벤트가 발생하지 않았고 live_module_data도 변경분이 없어 실질 트래픽이 0)
- `performance.memory`(GC 강제 후 비교): 베이스라인/비활성화 모두 `used=26.0MB, total=29.4MB`로 동일 — Chrome API의 값 반올림(약 1.4MB 단위 fuzzing) 해상도 내에서 차이 검출 불가. Operator 전용 JS 객체 5~6개는 코드 조사대로 존재하지만 그 자체 메모리 풋프린트는 이 정밀도로 측정 불가능할 만큼 작음

**해석**: 코드 조사로 확정된 4개 상시 오버헤드는 전부 실재하지만, 이번 idle(활성 grasp 없음) 시나리오에서는 **체감 가능한 수준이 아님**(<1%, 잡음 범위). 단, 이 결론은 "grasp 이벤트가 발생하지 않는 정적 상태"에서의 실측이라는 한계가 있다 — 항목 2(`taxelViewerRosbridgeClient`)와 3(`/atag/grasp_event` 핸들러)은 이벤트/메시지 빈도에 비례하는 오버헤드이므로, 실제 파지 동작이 빈번한 운영 시나리오(grasp_event가 초당 여러 번 발생)에서는 다른 결과가 나올 수 있음 — 이번 세션 범위에서는 재현하지 않음(별도 세션에서 필요 시 능동 파지 시나리오로 재측정 권장).

---

## Phase 5 — 정리 및 문서화 — ✅ 완료 (2026-09-11)

**작업**
- 신규 패키지 2개의 README 작성 (역할, 포트, 실행법, dg5f와의 관계) — `xela_taxel_viz_core/README.md`, `xela_taxel_operator_dg5f/README.md` 작성 완료
- 워크스페이스 최상위 문서에 "dg5f 운영자 화면은 이제 `xela_taxel_operator_dg5f`(포트 8766)로 접속" 안내 추가 — 최상위 repo `README.md`에 `ur7e_xdg5f_atag_right_sim`/`_sim_dev` 패키지 설명, 실행 예시, Notes 섹션에 8765/8766 URL 안내 추가 완료

**검증 체크리스트**
- [x] 운영자(실사용자)에게 신규 URL 안내 및 실제 접속 테스트 완료 — 사용자가 이번 프로젝트 진행 중 `http://localhost:8766`에 직접 여러 차례 접속해 기능/레이아웃을 실사용 테스트하고 피드백을 줬음(레이아웃 조정, 그래프 섹션 정리 등 다수 라운드) — 실질적으로 이미 충분히 검증됨
- [x] 기존 dg5f Operator 모드는 그대로 남겨둘지, 이후 다른 세션에서 제거 논의할지 결정 — **유지하기로 확정**. `xela_taxel_sidecar_dg5f`는 무수정 원칙이 전 Phase에 걸쳐 유지됐고, 제거 논의는 이번 프로젝트 범위 밖(향후 필요 시 별도 논의)
- [x] ah/2f 확장 여부는 별도 논의로 이월 기록 — 이번 프로젝트는 dg5f 전용으로 완결. `xela_taxel_sidecar_ah`/`xela_taxel_sidecar_2f`에 동일 패턴(코어 추출 + 운영자 전용 페이지)을 적용할지는 별도 세션에서 사용자 요청 시 논의(이번 범위 아님)

### `op-separate` → `main` merge 절차 (2026-09-09 확정)

이번 작업은 두 개의 서로 다른 git repo에 걸쳐 있음 (workspace 최상위 repo가 `xela_apps`를 submodule로 포함):

| repo | op-separate 브랜치 대상 |
|---|---|
| 최상위 워크스페이스 repo (`01_wk_xela_mpro_dev_ws`) | `ur7e_xdg5f_atag_right_sim_dev`(Phase 3.5 신규 로봇 config 패키지) |
| `src/xela_dependencies/xela_apps` (submodule, 독립 repo) | `xela_taxel_viz_core`, `xela_taxel_operator_dg5f` (Phase 1~3) |

**주의**: submodule이므로 `xela_apps` 쪽 merge가 최상위 repo에 자동 반영되지 않음. 반드시 아래 순서를 지킬 것 (순서를 바꾸면 최상위 repo가 옛 submodule 커밋을 계속 가리키게 됨):

1. `xela_apps` repo에서 `op-separate` → `main` merge (`cd src/xela_dependencies/xela_apps && git checkout main && git merge op-separate`)
2. 위 merge 완료 후, 최상위 repo에서 submodule 포인터 갱신:
   ```
   cd 01_wk_xela_mpro_dev_ws
   git -C src/xela_dependencies/xela_apps checkout main
   git -C src/xela_dependencies/xela_apps pull   # 필요시(원격에 push된 경우)
   git add src/xela_dependencies/xela_apps
   git commit -m "Bump xela_apps submodule to merged op-separate"
   ```
3. 최상위 repo에서 `op-separate` → `main` merge (`ur7e_xdg5f_atag_right_sim_dev` 반영)
4. merge 후 `git submodule status`로 최상위 repo의 submodule 포인터가 `xela_apps`의 최신 `main` 커밋과 일치하는지 확인
5. (원격 저장소가 있다면) 두 repo 모두 `git push` — 순서는 위와 동일하게 `xela_apps` 먼저, 최상위 repo 나중

---

## 전체 진행 원칙

- 각 Phase는 이전 Phase의 체크리스트가 모두 완료된 뒤에만 다음으로 진행
- `xela_taxel_sidecar_dg5f`는 전 Phase에 걸쳐 무수정 — 매 Phase 종료 시 `git status`/diff로 확인
- moveit_pro 빌드가 필요한 변경(신규 패키지 빌드) 후에는 반드시 `moveit_pro build`까지 완료

---

## Phase 3.6 — 레이아웃 미세조정 (상단 툴바 컴팩트화 + 그래프 섹션 정리) (2026-09-10)

Phase 0~3.5(Admin/Operator 동시 기동+기능 이식+상태바/레이아웃 수정)가 끝난 뒤, 사용자와 스크린샷 기반으로 논의해 확정한 후속 UI 조정. 대상 파일은 `xela_taxel_operator_dg5f/web/index.html` 하나뿐 (CSS 오버라이드 + 얇은 JS 연결 코드만 추가, 다른 파일 무수정).

**1) 상단 툴바 컴팩트화**
`.full-width-row`(Viz Controls + 모듈 체크박스)를 `flex-wrap: nowrap` + `overflow-x: auto`로 바꾸고, `xela_atag_taxel_viewer`의 `op-viz-controls`/`op-modsel` 위젯(무수정, 자체 padding/gap을 갖고 있음) 위에 이 페이지 스코프로 padding/gap을 줄이는 CSS만 얹어 한 줄 툴바로 압축. 위젯 자체의 DOM/동작은 전혀 건드리지 않음.

**2) 그래프 섹션 정리 — heatmap/모듈선택 제거**
`taxel_session_panel.js`(xela_atag_taxel_viewer, 무수정)에는 heatmap/모듈선택을 끄는 옵션 플래그가 없어서, dg5f 원본 `xela_taxel_sidecar_dg5f/web/taxel_sidecar/index.html`이 이미 쓰고 있던 `#operatorGraphHost` 스코프 CSS 오버라이드 패턴(읽기 전용 참고, 무수정)을 그대로 이식:
- `#graphHost canvas[data-role="heatmap"]` 및 그 라벨(`.taxel-session-card-sub:has(+ canvas[data-role="heatmap"])`) `display:none`
- `#graphHost .taxel-session-module-checklist` `display:none` (그래프 자체 모듈선택 체크박스 제거)
- 단, dg5f 원본과 달리 이번 요구사항은 `.taxel-session-controls-row` 안의 `force_total`/`shear_mag_avg`/`shear_normal_ratio` 필터 체크박스는 유지해야 해서, 원본처럼 그 행 전체를 숨기지 않고 `.taxel-session-module-checklist`만 개별적으로 숨기도록 수정(`:has(.taxel-session-view-toggles)`로 그 행 자체는 다시 보이게 하는 규칙 추가)
- `Live: On/Off` 토글과 세 필터 체크박스는 그대로 유지

**3) 그래프 레이아웃 — line + shear snapshot 같은 행**
`#graphHost .taxel-session-card-body`를 `display:grid; grid-template-columns: 1fr 200px`로 바꿔 라인차트(왼쪽, 넓게)와 shear(x,y) snapshot(오른쪽, 좁게)을 같은 행에 배치 — 이 역시 dg5f 원본의 동일 패턴을 그대로 이식.

**4) 상단↔그래프 모듈 선택 동기화**
`taxel_session_panel.js`에 `setSelectedModules()` 같은 외부 API는 없음. dg5f 원본의 `setModuleCheckboxSelection()` 패턴을 그대로 포팅한 `syncGraphModuleSelection(names)` 함수를 추가 — `#graphHost [data-module-checkbox]` 체크박스의 `checked` 상태를 직접 갱신하고 변경이 있으면 `change` 이벤트를 한 번 dispatch해서, 세션 패널 자체의 `onModuleSelectionChanged()`가 `setActiveModules` 호출과 카드 재구성을 그대로 처리하도록 함(재구현 아님, 연결만). `operatorModuleSelect`의 `onChange` 콜백에서 이 함수를 호출하도록 배선.

**검증 결과 (Playwright headless, `~/tools/playwright-toolkit`, `moveit_pro run -c ur7e_xdg5f_atag_right_sim_dev`, 실측)**
- [x] 상단 툴바 한 줄 컴팩트화 확인 — `.full-width-row` bounding box 높이 145px (스크린샷상 Sensitivity/Graphs 버튼 + F1~F5/Palm 체크박스가 한 줄에 배치됨, `01_initial.png`)
- [x] Graphs 켰을 때 heatmap 캔버스 0개(`data-role="heatmap"` count=0, visible=0), 그래프 자체 모듈선택 체크리스트 visible count=0
- [x] Live On/Off 버튼, force_total 체크박스 계속 visible (visible count=1 각각)
- [x] 상단 F3 ft 체크 → 그래프 섹션에 `f3_dg5f_ft` 카드 실제로 나타남 (count 1) → 해제 시 카드 사라짐(count 0), 상단 체크 개수도 0으로 동기화 확인
- [x] line 캔버스와 quiver(shear snapshot) 캔버스가 같은 y좌표, 인접한 x좌표에 나란히 배치됨을 bounding box로 확인 (`LINE_BOX`/`QUIVER_BOX` 같은 y=711/757, x 인접) — Live On 상태에서 실제 데이터로도 재확인 (`04_live_data.png`, 17 samples, line+quiver 정상 렌더링)
- [x] 기존 기능 재검증: Sensitivity 버튼 클릭 시 Normal→Mid 정상 전환, Capture Start/Stop UI 그대로 유지(코드상 미변경), force_total/shear_mag_avg/shear_normal_ratio 체크박스 모두 정상 동작

**baseline 무수정 확인**: `git status`/`git diff --stat` 기준 `xela_taxel_sidecar_dg5f`, `ur7e_xdg5f_atag_right_sim`(비-dev), `ur7e_xdg5f_atag_right_common`, `xela_atag_taxel_viewer` 전부 변경 없음(diff 0줄). 수정은 `xela_taxel_operator_dg5f/web/index.html` 하나뿐.

**환경 노트**: 검증 중 `moveit_pro run`을 `nohup ... &`로 백그라운드 실행했더니 "MoveIt Pro shutdown initiated..."로 즉시 종료되는 현상 발견 — Bash 툴의 `run_in_background`(foreground 프로세스를 별도로 관리)로 재실행하니 안정적으로 유지됨. 이후 세션에서도 `nohup &` 대신 `run_in_background`를 쓸 것.

**남은 이슈**: 없음(요구사항 3건 모두 실측 검증 완료). `~/.config/moveit_pro/moveit_pro_config.yaml`의 `STUDIO_CONFIG_PACKAGE`는 이번 세션 시작 전부터 `ur7e_xdg5f_atag_right_sim_dev`로 설정되어 있었고 이번 작업에서 변경하지 않았음(Admin+Operator 동시 기동용 Phase 3.5 config로 보이며, baseline 단일 앱 config `ur7e_xdg5f_atag_right_sim`과는 별개 — 필요 시 사용자 확인 요망).

---

## Phase 6 — 운영자 화면 UX 개선 (데모/모니터링 목적, 2026-09-11 계획 수립) — 미착수, Phase 4/5 마무리 후 진행

**배경**: 분리 목적 자체("Admin/Operator 역할 명확 분리")는 Phase 0~4로 달성됨. Phase 6은 그 위에 사용자(운영 전문가 관점)가 요청한 신규 기능 3건 — 이번 분리 프로젝트의 원 스코프 밖이라 별도 Phase로 분리해서 진행. 대상 파일은 전부 `xela_taxel_operator_dg5f/web/index.html`(및 필요시 이 패키지 안 CSS/JS)뿐 — `xela_taxel_sidecar_dg5f`/baseline 무수정 원칙은 계속 유지.

### 6-1. Kiosk/Lock 모드

- 상태바 근처에 `🔒 Lock` / `🔓 Unlock` 토글 버튼 추가
- 잠금 시: 모듈 체크박스, Viz Controls(Sensitivity/Graphs), 세션 패널의 Capture/Load 버튼에 `pointer-events:none` + 반투명 오버레이
- 3D 뷰 카메라 조작(회전/줌/팬)도 잠금 대상에 포함(데모 중 실수로 화면이 돌아가는 것 방지)
- 잠금 상태는 `localStorage`에 저장해 새로고침 후에도 유지
- 해제는 버튼 재클릭만으로 충분(비밀번호 등 별도 인증 불필요)

**검증**: Playwright로 잠금 전/후 각 컨트롤 클릭 시도 → 잠금 상태에서 상태 변화 없음(모듈 선택 안 바뀜 등) 확인, 카메라 드래그도 무반응 확인, 새로고침 후 잠금 유지 확인.

### 6-2. 세션/데모 리셋 버튼

- 상태바 또는 세션 패널 근처에 "Reset Demo" 버튼 추가
- 클릭 시: 필름스트립 히스토리 비우기, 알림카드 비우기, 그래프 버퍼 초기화(캡처 중이 아닐 때만)
- "TODAY 성공/실패" 카운터는 리셋 대상에서 제외(하루 통계는 유지) — 확인 필요 시 재논의
- rosbridge 연결/구독 자체는 건드리지 않음(끊었다 재연결하는 방식 금지 — 불필요한 위험)

**검증**: 필름스트립/알림카드에 데이터가 쌓인 상태에서 Reset 클릭 → 즉시 빈 상태로 돌아가는지 확인, 이후 새 이벤트가 정상적으로 다시 쌓이는지 확인(리셋이 구독 자체를 깨지 않았는지).

### 6-3. 임계값 기반 알림 강화 + 이벤트 선택 강조 (대화로 확정된 상세 사양)

**감시 지표**: `force_total` / `shear_mag_avg` / `shear_normal_ratio` 전부 대상.

**적용 범위**: 전체 taxel이 아니라 **상단 섹션에서 현재 선택된 모듈(체크된 것)만** 감시 — 예: F1 ft, F2 ft가 체크돼 있으면 그 두 그룹만 임계값 검사.

**임계값 설정**: 상태바/Viz Controls 근처에 작은 숫자 입력 필드 노출, 데모 현장에서 직접 조정 가능(코드/URL 파라미터 아님). 3개 지표 각각 별도 입력 필드(또는 공용 입력 하나로 시작하고 추후 지표별로 분리 여부는 실사용 피드백에 따름).

**이벤트 선택 강조**: 설정 패널(Lock 버튼 근처에 작은 톱니바퀴 아이콘 등으로 진입)에 `grasp_event`의 하위 이벤트 타입(현재 관측된 것: "Grasp Cycle Started", "Release Executed", "Transport Started", "Transport Complete", "mock_scenario" 등, index.html의 `handleRosbridgeMessage`/alerts 처리 코드에서 실제 타입 전수 확인 필요)별 체크박스 목록을 두고, 체크된 이벤트 타입이 발생하면 임계값 초과와 동일한 방식으로 강조 처리.

**알림 방식**: 시각적 강조만(사운드 없음) —
- 상태바 배지 잠깐 강조색 flash
- **3D뷰(`.operator-viz-row`) 우상단에 절대위치 오버레이 카드가 나타났다 자동 사라짐(3~5초)** — 카드 내용은 캔버스 스냅샷이 아니라 **표지판/피켓(sign/picket) 형태의 고정 이미지**(구현 단순, 매번 캔버스 캡처 비용 없음). 경고성(임계값 초과)과 정보성(이벤트 발생)을 시각적으로 구분할 수 있게 최소 2종 준비:
  - 임계값 초과용: 경고 표지판 스타일(노란/빨강 삼각형 또는 팔각형 STOP 표지판 톤) + 지표명/수치 텍스트 오버레이(예: "F2 force_total 22.3N")
  - 이벤트 강조용: 안내 피켓/배너 스타일(파란/초록 톤, 팻말을 든 듯한 사각 카드) + 이벤트명 텍스트(예: "Release Executed")
  이미지 자체는 SVG로 인라인 작성(외부 이미지 파일 의존 없이 `xela_taxel_operator_dg5f/web/index.html` 안에 직접 포함 가능, 텍스트만 동적으로 갈아끼움) — 벡터라 어떤 해상도에서도 선명하고 파일 추가/에셋 관리 불필요.
- Alerts 카드에도 동일 항목 자동 기록(기존 로그 스트림에 병합)

**구현 스케치**: 임계값 체크는 기존 `updateUrdfMarkers`/payload 처리 루프에서 이미 계산되는 지표값을 매 프레임 재사용(별도 폴링 불필요), 초과 감지 시 새 함수(예: `flashAlertCard(iconOrLabel)`)를 호출해 오버레이 DOM을 생성/제거. 이벤트 타입 체크는 기존 `handleRosbridgeMessage`의 grasp_event 분기에 조건 추가.

**검증**: 임계값을 낮게 설정해 의도적으로 초과시켜 flash 카드/상태바 강조/Alerts 기록이 동시에 트리거되는지 확인, 여러 이벤트가 짧은 시간에 연속 발생할 때 카드가 겹치지 않고 순차 처리되는지 확인, 선택 안 한 모듈의 초과는 무시되는지 확인, 설정 패널에서 이벤트 타입 체크 해제 시 해당 타입은 강조 안 되는지 확인.

### 6-4. (스킵) 사이클 요약 통계

사용자 결정으로 이번 Phase에서 제외.

### 진행 시점

Phase 4/5(성능 검증 마무리, 문서화/README, 커밋)를 먼저 완료한 뒤 별도 세션에서 착수.
