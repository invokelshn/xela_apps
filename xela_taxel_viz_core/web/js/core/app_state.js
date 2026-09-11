export function createAppState(config) {
  return {
    ws: null,
    connected: false,
    payload: null,
    lastReceivedMs: 0,
    activeWsUrl: null,
    uiMode: "grid",
    modeOverriddenByUser: false,
    requestedVizMode: "grid",
    sidecarRenderMode: "grid-2d",
    modelName: "",
    // Module id -> enabled flag. This is model-specific (finger/sensor naming
    // differs per hand/gripper), so the core has no built-in default: callers pass
    // their own module map via config.showModules (falls back to empty, i.e. no
    // modules toggled, if omitted).
    showModules: { ...(config.showModules || {}) },
    sensorViewMode: "all",
    pendingSwitch: false,
    gridVectorMode: config.gridVectorMode || "xy",
    gridVectorZProjX: config.vector3dProjectX,
    gridVectorZProjY: config.vector3dProjectY,
    vectorEma: new Map(),
    lastStatsPaintMs: 0,
    lastStatsText: "",
    lastHintText: "",
    lastModeInfoText: "",
    camera: {
      yaw: 0.0,
      pitch: 0.0,
      zoom: 1.35,
      panX: 0.0,
      panY: 0.0,
    },
    drag: null,
    lastSceneExtent: 0.12,
    sourceTopicSubscribed: false,
    sourceByService: new Map(),
    sourceByServiceCallId: new Map(),
    serviceCallSeq: 0,
    urdfSources: {
      xela: {
        name: "xela",
        robotDescriptionTopic: config.xelaRobotDescriptionTopic,
        tfTopic: config.xelaTfTopic,
        tfStaticTopic: config.xelaTfStaticTopic,
        robotDescriptionService: config.xelaRobotDescriptionService,
        robotDescriptionXml: "",
        robotDescriptionVersion: 0,
        robotRootLink: "",
        tfEdges: new Map(),
        // Bumped by onTfMessage whenever a non-empty TF message actually arrives -- lets
        // updateRobotLinkTransforms() skip its per-link TF walk when nothing changed since the
        // last render tick, instead of redoing it unconditionally on every rAF (2026-09-08 perf
        // fix, see updateRobotLinkTransforms's own comment).
        tfVersion: 0,
        frameAliasCache: new Map(),
        // Phase 2 (2026-09-10): index.html's getFrameLeafIndex() lazily assigns this field onto
        // whichever source object it's given (src.frameLeafIndex = index), but app_state.js never
        // declared it explicitly -- flagged in dg5f_operator_split_phase2_design.md as an
        // undeclared-schema gap. Declaring it null here doesn't change behavior (the ported
        // getFrameLeafIndex still lazily populates it the same way), just documents the field.
        frameLeafIndex: null,
        pendingRobotDescriptionService: false,
        lastRobotDescriptionServiceReqMs: 0,
      },
      robot: {
        name: "robot",
        robotDescriptionTopic: config.robotRobotDescriptionTopic,
        tfTopic: config.robotTfTopic,
        tfStaticTopic: config.robotTfStaticTopic,
        robotDescriptionService: config.robotRobotDescriptionService,
        robotDescriptionXml: "",
        robotDescriptionVersion: 0,
        robotRootLink: "",
        tfEdges: new Map(),
        tfVersion: 0,
        frameAliasCache: new Map(),
        frameLeafIndex: null,
        pendingRobotDescriptionService: false,
        lastRobotDescriptionServiceReqMs: 0,
      },
    },
    lastRosbridgeErrorText: "",
    urdfRenderEpoch: 0,
    urdfMesh: {
      loading: false,
      ready: false,
      failed: false,
      failReason: "",
      THREE: null,
      renderer: null,
      scene: null,
      camera: null,
      controls: null,
      stlLoader: null,
      colladaLoader: null,
      rootGroup: null,
      robotGroup: null,
      markerGroup: null,
      linkGroups: new Map(),
      materialCache: new Map(),
      meshCache: new Map(),
      markerMap: new Map(),
      worldGrid: null,
      tfEdges: new Map(),
      frameAliasCache: new Map(),
      tfParentFrames: new Set(),
      linkModuleMap: new Map(),
      linkUrdfColorMap: new Map(),
      linkMeshMap: new Map(),
      activeModuleIds: new Set(),
      lastHighlightSignature: "",
      robotDescriptionXml: "",
      boundSourceName: "",
      boundRobotDescriptionVersion: -1,
      robotBuildToken: 0,
      robotBuilt: false,
      autoFrameDone: false,
      fixedFrame: config.defaultFixedFrame,
      tfFallbackActive: false,
      lastTfStats: null,
      // Phase 2 (2026-09-10): guard key for updateRobotLinkTransforms()'s tfVersion-based skip
      // (ported from index.html; see taxel_marker_renderer.js's updateRobotLinkTransforms).
      lastLinkTransformsGuardKey: null,
      // Phase 2 (2026-09-10): frameLeafIndexSource is set by taxel_marker_renderer.js's
      // bindActiveUrdfSourceToMesh() (ported from index.html 1754-1779, trimmed to TF-relevant
      // fields) so resolveChildFrameAlias's getFrameLeafIndex() call always has an explicit
      // source object to read/write frameLeafIndex on, rather than relying on it being set
      // implicitly by other mesh-build code.
      frameLeafIndexSource: null,
    },
  };
}
