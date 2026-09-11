// Ported (2026-09-10, Phase 2) from xela_taxel_sidecar_dg5f/web/taxel_sidecar/index.html.
//
// SOURCE OF TRUTH: xela_taxel_sidecar_dg5f/web/taxel_sidecar/index.html is READ-ONLY for this
// port -- every function body below is copied verbatim (only variable capture/injection changed
// to remove the direct references to index.html's page-global `state`/DOM/UI helpers). Do not
// "clean up" or re-derive the TF/marker math here without re-checking it against the original;
// see docs/plans/dg5f_operator_split_phase2_design.md for why this must be a pure copy.
//
// Ported functions (original index.html line numbers as of the 2026-09-10 survey):
//   getFrameLeafIndex (1799), resolveChildFrameAlias (1820), resolveFrameToFixed (2692),
//   selectFixedFrame (1844), selectFixedFrameUncached (1855), updateUrdfMarkers (2941),
//   ensureMarkerObject (2801), resolveMarkerPose (2833), resolveTaxelLocalOffsetViaXela (2862),
//   resolveTaxelFixedFramePoseRobust (2899), updateRobotLinkTransforms (2749),
//   autoFrameUrdfMeshCamera (2624), updateFollowCamera (2013), computeModuleCenterFromTf (1978),
//   bindActiveUrdfSourceToMesh (1754, glue needed by renderUrdfMesh/resolveChildFrameAlias --
//   not in the original enumerated list of 9+adjacent functions, but required so
//   state.urdfMesh.tfEdges/frameAliasCache/frameLeafIndexSource are bound to the active source
//   before any of the above run; without it every TF lookup silently operates on stale/empty
//   data. See docs/plans/dg5f_operator_split_phase2_design.md, "남은 이슈" section).
//   Helpers: getFrameLeafName (1781), normalizeFrameId (1741), getUrdfSourceForUiMode (1746),
//   getActiveUrdfSource (1750), extractModuleIdFromName (2345), clamp (896),
//   operatorAdjustNorm (3495, rewired below to take an injected gamma getter instead of reading
//   DOM class + module-scope sensitivity level directly).
//
// refreshUrdfModuleHighlight (2435) + getLinkMaterial (2447) + their support functions
// collectEnabledModuleIdsFromUi (2380), collectVisibleModuleIdsFromPayload (2396),
// computeActiveModuleIds (2407), resolveLinkColorForSelection (2419), fallbackModuleColor (2369)
// and URDF_NEUTRAL_LINK_COLOR (2332) are now ported below too (2026-09-10, follow-up). The
// original's collectEnabledModuleIdsFromUi reads page-global DOM (appEl.classList,
// operatorModuleSelect widget) that this file has no access to -- ported as-is but with that one
// branch replaced by an injected `getEnabledModuleIds()` callback; a caller that wants the
// Operator widget behavior passes its own, and a caller that wants the plain
// state.showModules-driven Admin behavior can omit it (the default below reproduces that branch
// verbatim).
//
// NOT ported here (out of scope for this file, still index.html-only / caller-owned):
//   renderUrdfMesh's mesh-build steps (buildUrdfRobotFromDescription / resizeUrdfMeshRenderer --
//   already live as buildRobotFromDescription/resize in urdf_mesh_renderer.js from Phase 1).
//
// Caches/state kept module-scoped in the original (frameAliasCache, frameLeafIndex,
// lastSelectFixedFrameKey/Result, lastMarkersPayload/GuardKey, lastLinkTransformsGuardKey) are
// encapsulated per-instance below (closure over the factory call) rather than as file-level
// globals, so multiple renderers (e.g. Admin + Operator, or two demo instances) don't share
// cache state -- this changes *scope*, not cache *semantics*: invalidation keys/timing are
// unchanged from the original.

// Ported from index.html's extractModuleIdFromName (2345) + its DG5F_MODULES list (2338-2343).
// DG-5F module names come from r_server_model_joint_map.yaml; frame/link ids look like
// x_taxel_1_f3_dg5f_ft_09_joint / _link, so the module name is the text between the
// "x_taxel_<n>_" prefix and the trailing "_<NN>_joint|link" taxel index suffix. Exposed as a
// standalone factory (rather than hardcoded) so a non-DG5F caller can pass its own module list.
export function createExtractModuleIdFromName(moduleList, normalizeFrameIdFn) {
  const modules = Array.isArray(moduleList) ? moduleList : [];
  const normalize = typeof normalizeFrameIdFn === "function"
    ? normalizeFrameIdFn
    : (frameId) => (frameId ? String(frameId).replace(/^\/+/, "").trim() : "");
  return function extractModuleIdFromName(name) {
    const normalized = normalize(name).toLowerCase();
    if (!normalized) return "";
    const taxelMatch = normalized.match(/(?:^|_)x_taxel_\d+_(.+)_\d{1,2}_(?:joint|link)$/);
    if (taxelMatch && taxelMatch[1]) {
      const found = modules.find((mod) => mod.toLowerCase() === taxelMatch[1]);
      if (found) return found;
    }
    // Sensor HOUSING links (the STL mesh bodies, e.g. "base_f3_dg5f_ft_link",
    // "base_palm_uSPa46_link") come straight from x_dg5f_hand.xacro's own naming
    // (base_<module>_link, no x_taxel_<n>_ prefix, no taxel-index suffix) -- matched separately
    // so resolveLinkColorForSelection can also resolve a moduleId for these, not just the taxel
    // dot links (index.html 2345, housingMatch branch; missing here until 2026-09-10 follow-up).
    const housingMatch = normalized.match(/^(?:base_)?(.+)_link$/);
    if (housingMatch && housingMatch[1]) {
      const found = modules.find((mod) => mod.toLowerCase() === housingMatch[1]);
      if (found) return found;
    }
    return "";
  };
}

// Ported from index.html's URDF_NEUTRAL_LINK_COLOR (2332) + fallbackModuleColor (2369).
export const URDF_NEUTRAL_LINK_COLOR = 0x6b7280;

export function fallbackModuleColor(moduleId) {
  // 5 finger colors + 1 palm color (extends AH's original 4-finger+palm palette by one hue).
  if (moduleId.startsWith("f1_")) return 0xf59e0b; // f1 - amber
  if (moduleId.startsWith("f2_")) return 0x84cc16; // f2 - lime
  if (moduleId.startsWith("f3_")) return 0x38bdf8; // f3 - sky
  if (moduleId.startsWith("f4_")) return 0xfb7185; // f4 - rose
  if (moduleId.startsWith("f5_")) return 0xa78bfa; // f5 - violet (new hue vs. AH's 4-finger set)
  if (moduleId.startsWith("palm")) return 0x2dd4bf; // palm - teal
  return 0x93c5fd;
}

export const DG5F_MODULES = [
  "f3_dg5f_ft", "f4_dg5f_ft", "f2_dg5f_ft", "f5_dg5f_ft",
  "f3_mid_uSPa22", "f4_mid_uSPa22", "f2_mid_uSPa22", "f5_mid_uSPa22",
  "f3_prox_uSPa22", "f1_dg5f_ft", "f4_prox_uSPa22", "f2_prox_uSPa22",
  "f5_prox_uSPa22", "f1_mid_uSPa22", "palm_uSPa46", "f1_prox_uSPa22",
];

export function createTaxelMarkerRenderer({
  state,
  defaultFixedFrame,
  robotModelFixedFrame,
  applyOrbitProfileForMode,
  getSensitivityGamma,
  moduleIdFromNameFn,
  onFollowDebugText,
  urdfMeshRenderer,
  getEnabledModuleIds,
} = {}) {
  const resolveModuleId = typeof moduleIdFromNameFn === "function"
    ? moduleIdFromNameFn
    : createExtractModuleIdFromName(DG5F_MODULES, normalizeFrameId);
  const getGamma = typeof getSensitivityGamma === "function" ? getSensitivityGamma : () => 1.0;
  const notifyFollowDebug = typeof onFollowDebugText === "function" ? onFollowDebugText : () => {};
  // Ported from index.html's collectEnabledModuleIdsFromUi (2380). The original's
  // operator-widget branch (appEl.classList.contains("operator-active") && operatorModuleSelect)
  // is page-global DOM/UI state this file has no access to, so it is replaced by an injected
  // callback; the default below reproduces the plain Admin branch verbatim (reads
  // state.showModules against DG5F_MODULES).
  const collectEnabledModuleIdsFromUi = typeof getEnabledModuleIds === "function"
    ? getEnabledModuleIds
    : function defaultCollectEnabledModuleIdsFromUi() {
      const out = new Set();
      for (const mod of DG5F_MODULES) {
        if (state.showModules?.[mod] !== false) {
          out.add(mod);
        }
      }
      return out;
    };

  function clamp(v, lo, hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  function normalizeFrameId(frameId) {
    if (!frameId) return "";
    return String(frameId).replace(/^\/+/, "").trim();
  }

  function getUrdfSourceForUiMode() {
    return state.uiMode === "robot" ? "robot" : "xela";
  }

  function getActiveUrdfSource() {
    return state.urdfSources[getUrdfSourceForUiMode()];
  }

  // Ported from index.html's bindActiveUrdfSourceToMesh (1754) -- trimmed to just the
  // TF/frame-resolution fields this module's functions actually read (tfEdges,
  // frameAliasCache, frameLeafIndexSource). The mesh-rebuild side effects (clearing
  // markerMap/linkGroups/etc. on source change) stay the caller's responsibility via
  // urdf_mesh_renderer.js's own bindActiveUrdfSourceToMesh-equivalent, to avoid this module
  // reaching into mesh-build state it doesn't own.
  function bindActiveUrdfSourceToMesh() {
    const src = getActiveUrdfSource();
    const m = state.urdfMesh;
    m.tfEdges = src.tfEdges;
    m.frameAliasCache = src.frameAliasCache;
    m.frameLeafIndexSource = src;
  }

  function getFrameLeafName(frame) {
    const text = normalizeFrameId(frame);
    if (!text) return "";
    const parts = text.split(/[/:]/).filter(Boolean);
    return parts.length ? parts[parts.length - 1] : text;
  }

  // 2026-09-09 perf fix (see original index.html comment above getFrameLeafIndex, 1788-1798):
  // build a leaf-name index once per structural change (O(edges)) instead of once per queried
  // frame per structural change (O(edges * frames)).
  function getFrameLeafIndex(src) {
    if (src.frameLeafIndex) {
      return src.frameLeafIndex;
    }
    const index = new Map();
    const ambiguous = new Set();
    for (const key of src.tfEdges.keys()) {
      const keyLeaf = getFrameLeafName(key);
      if (!index.has(keyLeaf)) {
        index.set(keyLeaf, key);
      } else if (index.get(keyLeaf) !== key) {
        ambiguous.add(keyLeaf);
      }
    }
    for (const leaf of ambiguous) {
      index.set(leaf, "");
    }
    src.frameLeafIndex = index;
    return index;
  }

  function resolveChildFrameAlias(frame) {
    const m = state.urdfMesh;
    const clean = normalizeFrameId(frame);
    if (!clean) return "";
    if (m.tfEdges.has(clean)) {
      return clean;
    }
    if (m.frameAliasCache.has(clean)) {
      return m.frameAliasCache.get(clean);
    }
    const leaf = getFrameLeafName(clean);
    const index = getFrameLeafIndex(m.frameLeafIndexSource || getActiveUrdfSource());
    const match = index.has(leaf) ? index.get(leaf) : "";
    m.frameAliasCache.set(clean, match || "");
    return match || "";
  }

  // 2026-09-08 perf fix, step 3 (see original comment above selectFixedFrame, 1837-1841).
  let lastSelectFixedFrameKey = null;
  let lastSelectFixedFrameResult = null;
  function selectFixedFrame(activeSource, payloadFixedFrame) {
    const cacheKey = `${activeSource?.name}|${activeSource?.tfVersion || 0}|${payloadFixedFrame}|${state.uiMode}`;
    if (cacheKey === lastSelectFixedFrameKey) {
      return lastSelectFixedFrameResult;
    }
    const result = selectFixedFrameUncached(activeSource, payloadFixedFrame);
    lastSelectFixedFrameKey = cacheKey;
    lastSelectFixedFrameResult = result;
    return result;
  }

  function selectFixedFrameUncached(activeSource, payloadFixedFrame) {
    const m = state.urdfMesh;
    const payloadFixed = normalizeFrameId(payloadFixedFrame || "");
    const childSet = new Set(m.tfEdges.keys());
    const parentSet = new Set();
    const childrenByParent = new Map();
    for (const [child, edge] of m.tfEdges.entries()) {
      const parent = normalizeFrameId(edge.parent);
      if (!parent) continue;
      parentSet.add(parent);
      if (!childrenByParent.has(parent)) {
        childrenByParent.set(parent, []);
      }
      childrenByParent.get(parent).push(normalizeFrameId(child));
    }
    const frameExists = (name) => {
      if (!name) return false;
      return childSet.has(name) || parentSet.has(name);
    };

    const preferred = [];
    if (state.uiMode === "robot") {
      const rootLink = normalizeFrameId(activeSource?.robotRootLink || "");
      if (rootLink) {
        const aliasedRoot = resolveChildFrameAlias(rootLink) || rootLink;
        preferred.push(aliasedRoot);
      }
      if (robotModelFixedFrame) preferred.push(robotModelFixedFrame);
      if (payloadFixed) preferred.push(payloadFixed);
      preferred.push("world", "base_link");
    } else {
      if (payloadFixed) preferred.push(payloadFixed);
      preferred.push(defaultFixedFrame);
    }
    for (const name of preferred) {
      const clean = normalizeFrameId(name);
      if (frameExists(clean)) return clean;
    }

    const roots = [];
    for (const p of parentSet) {
      if (!childSet.has(p)) roots.push(p);
    }
    if (roots.length > 0) {
      const memo = new Map();
      const visit = new Set();
      const scoreRoot = (root) => {
        if (memo.has(root)) return memo.get(root);
        if (visit.has(root)) return 0;
        visit.add(root);
        let total = 1;
        const children = childrenByParent.get(root) || [];
        for (const child of children) {
          if (child) total += scoreRoot(child);
        }
        visit.delete(root);
        memo.set(root, total);
        return total;
      };
      roots.sort((a, b) => scoreRoot(b) - scoreRoot(a) || a.localeCompare(b));
      return roots[0];
    }

    if (childSet.size > 0) {
      return Array.from(childSet).sort()[0];
    }
    return payloadFixed || normalizeFrameId(defaultFixedFrame || "world") || "world";
  }

  function resolveFrameToFixed(frame, fixedFrame, cache, visiting, allowLooseRoot = false) {
    const m = state.urdfMesh;
    const fixed = normalizeFrameId(fixedFrame);
    const rawFrame = normalizeFrameId(frame);
    if (!rawFrame) return null;
    const resolvedFrame = resolveChildFrameAlias(rawFrame) || rawFrame;
    if (resolvedFrame === fixed || rawFrame === fixed) {
      return { tx: 0, ty: 0, tz: 0, qx: 0, qy: 0, qz: 0, qw: 1 };
    }
    if (cache.has(resolvedFrame)) return cache.get(resolvedFrame);
    if (visiting.has(resolvedFrame)) return null;
    visiting.add(resolvedFrame);
    const edge = m.tfEdges.get(resolvedFrame);
    if (!edge) {
      visiting.delete(resolvedFrame);
      if (allowLooseRoot) {
        const isKnownGraphFrame = m.tfParentFrames.has(resolvedFrame) || m.tfEdges.has(resolvedFrame);
        if (!isKnownGraphFrame) {
          return null;
        }
        const root = { tx: 0, ty: 0, tz: 0, qx: 0, qy: 0, qz: 0, qw: 1 };
        cache.set(resolvedFrame, root);
        return root;
      }
      return null;
    }
    const parentTf = resolveFrameToFixed(edge.parent, fixed, cache, visiting, allowLooseRoot);
    visiting.delete(resolvedFrame);
    if (!parentTf) return null;

    const THREE = m.THREE;
    const parentQ = new THREE.Quaternion(parentTf.qx, parentTf.qy, parentTf.qz, parentTf.qw);
    const localQ = new THREE.Quaternion(edge.qx, edge.qy, edge.qz, edge.qw);
    const combinedQ = parentQ.clone().multiply(localQ);
    const localT = new THREE.Vector3(edge.tx, edge.ty, edge.tz).applyQuaternion(parentQ);
    const combinedT = localT.add(new THREE.Vector3(parentTf.tx, parentTf.ty, parentTf.tz));
    const out = {
      tx: combinedT.x,
      ty: combinedT.y,
      tz: combinedT.z,
      qx: combinedQ.x,
      qy: combinedQ.y,
      qz: combinedQ.z,
      qw: combinedQ.w,
    };
    cache.set(resolvedFrame, out);
    return out;
  }

  // 2026-09-08 perf fix, step 2 (see original comment above updateRobotLinkTransforms,
  // 2741-2748): guard the full per-link TF walk behind a tfVersion-keyed cache so idle frames
  // (no new TF) skip it entirely.
  function updateRobotLinkTransforms(fixedFrame, allowLooseRootFallback = true) {
    const m = state.urdfMesh;
    if (!m.ready || !m.robotBuilt) return { total: 0, visibleCount: 0, missingCount: 0, fallbackVisibleCount: 0 };
    const activeSrc = getActiveUrdfSource();
    const guardKey = `${activeSrc.name}|${activeSrc.tfVersion || 0}|${fixedFrame}|${allowLooseRootFallback}`;
    if (guardKey === m.lastLinkTransformsGuardKey) {
      return m.lastTfStats || { total: m.linkGroups.size, visibleCount: 0, missingCount: 0, fallbackVisibleCount: 0 };
    }
    m.lastLinkTransformsGuardKey = guardKey;
    const parentFrames = new Set();
    for (const edge of m.tfEdges.values()) {
      if (edge.parent) parentFrames.add(normalizeFrameId(edge.parent));
    }
    m.tfParentFrames = parentFrames;
    const strictCache = new Map();
    const strictVisiting = new Set();
    const looseCache = new Map();
    const looseVisiting = new Set();
    let visibleCount = 0;
    let missingCount = 0;
    let fallbackVisibleCount = 0;
    for (const [linkName, group] of m.linkGroups.entries()) {
      let tf = resolveFrameToFixed(linkName, fixedFrame, strictCache, strictVisiting, false);
      let usedFallback = false;
      if (!tf && allowLooseRootFallback) {
        tf = resolveFrameToFixed(linkName, fixedFrame, looseCache, looseVisiting, true);
        usedFallback = !!tf;
      }
      if (!tf) {
        group.visible = false;
        missingCount += 1;
        continue;
      }
      group.visible = true;
      group.position.set(tf.tx, tf.ty, tf.tz);
      group.quaternion.set(tf.qx, tf.qy, tf.qz, tf.qw);
      visibleCount += 1;
      if (usedFallback) {
        fallbackVisibleCount += 1;
      }
    }
    const stats = {
      total: m.linkGroups.size,
      visibleCount,
      missingCount,
      fallbackVisibleCount,
    };
    m.lastTfStats = stats;
    m.tfFallbackActive = fallbackVisibleCount > 0;
    return stats;
  }

  function ensureMarkerObject(key) {
    const m = state.urdfMesh;
    const THREE = m.THREE;
    if (m.markerMap.has(key)) {
      return m.markerMap.get(key);
    }
    const group = new THREE.Group();
    const sphere = new THREE.Mesh(
      new THREE.SphereGeometry(0.002, 24, 16),
      new THREE.MeshStandardMaterial({ color: 0x60a5fa, roughness: 0.2, metalness: 0.0 }),
    );
    const arrow = new THREE.ArrowHelper(
      new THREE.Vector3(1, 0, 0),
      new THREE.Vector3(0, 0, 0),
      0.006,
      0xf8fafc,
      0.0024,
      0.0016,
    );
    group.add(sphere);
    group.add(arrow);
    m.markerGroup.add(group);
    const marker = { group, sphere, arrow };
    m.markerMap.set(key, marker);
    return marker;
  }

  function resolveMarkerPose(frameId, fixedFrame, strictCache, strictVisiting, looseCache, looseVisiting) {
    const frame = resolveChildFrameAlias(frameId) || normalizeFrameId(frameId);
    if (!frame) return null;
    let tf = resolveFrameToFixed(frame, fixedFrame, strictCache, strictVisiting, false);
    if (tf) return tf;
    return resolveFrameToFixed(frame, fixedFrame, looseCache, looseVisiting, true);
  }

  function resolveTaxelLocalOffsetViaXela(frameId, housingLinkName) {
    const m = state.urdfMesh;
    const frame = resolveChildFrameAlias(frameId) || normalizeFrameId(frameId);
    if (!frame || !housingLinkName) return null;
    const savedEdges = m.tfEdges;
    const savedParents = m.tfParentFrames;
    m.tfEdges = state.urdfSources.xela.tfEdges;
    const parentFrames = new Set();
    for (const edge of m.tfEdges.values()) {
      if (edge.parent) parentFrames.add(normalizeFrameId(edge.parent));
    }
    m.tfParentFrames = parentFrames;
    try {
      const cache = new Map();
      const visiting = new Set();
      return resolveFrameToFixed(frame, housingLinkName, cache, visiting, false);
    } finally {
      m.tfEdges = savedEdges;
      m.tfParentFrames = savedParents;
    }
  }

  function resolveTaxelFixedFramePoseRobust(frameId, fixedFrame, strictCache, strictVisiting) {
    const m = state.urdfMesh;
    const frame = resolveChildFrameAlias(frameId) || normalizeFrameId(frameId);
    if (!frame) return null;
    const direct = resolveFrameToFixed(frame, fixedFrame, strictCache, strictVisiting, false);
    if (direct) return direct;
    if (state.uiMode !== "robot") return null;
    const moduleId = resolveModuleId(frameId);
    const housingLinkName = moduleId ? `base_${moduleId}_link` : "";
    if (!housingLinkName) return null;
    const housingTf = resolveFrameToFixed(housingLinkName, fixedFrame, strictCache, strictVisiting, false);
    if (!housingTf) return null;
    const localOffset = resolveTaxelLocalOffsetViaXela(frameId, housingLinkName);
    if (!localOffset) return null;
    const THREE = m.THREE;
    const housingQ = new THREE.Quaternion(housingTf.qx, housingTf.qy, housingTf.qz, housingTf.qw);
    const localQ = new THREE.Quaternion(localOffset.qx, localOffset.qy, localOffset.qz, localOffset.qw);
    const combinedQ = housingQ.clone().multiply(localQ);
    const localT = new THREE.Vector3(localOffset.tx, localOffset.ty, localOffset.tz).applyQuaternion(housingQ);
    const combinedT = localT.add(new THREE.Vector3(housingTf.tx, housingTf.ty, housingTf.tz));
    return {
      tx: combinedT.x,
      ty: combinedT.y,
      tz: combinedT.z,
      qx: combinedQ.x,
      qy: combinedQ.y,
      qz: combinedQ.z,
      qw: combinedQ.w,
    };
  }

  // Ported from index.html's operatorAdjustNorm (3495) -- the original reads a DOM class
  // (appEl.classList.contains("operator-active")) and a module-scope operatorSensitivityLevel
  // directly; here the caller injects getSensitivityGamma() so this module has no DOM/page-global
  // coupling. Pass a getter that returns 1.0 for Admin (no-op) and the OPERATOR_SENSITIVITY_GAMMA
  // lookup for Operator, matching the original's "only applied in Operator mode" behavior.
  function operatorAdjustNorm(norm) {
    const gamma = getGamma();
    if (!gamma || gamma === 1.0) {
      return norm;
    }
    return Math.pow(Math.max(0, Math.min(1, norm)), gamma);
  }

  // Ported from index.html's collectVisibleModuleIdsFromPayload (2396).
  function collectVisibleModuleIdsFromPayload(payload) {
    const out = new Set();
    const points = Array.isArray(payload?.urdf?.points) ? payload.urdf.points : [];
    for (const p of points) {
      const moduleId = resolveModuleId(p?.frame_id);
      if (!moduleId) continue;
      out.add(moduleId);
    }
    return out;
  }

  // Ported from index.html's computeActiveModuleIds (2407).
  function computeActiveModuleIds(payload) {
    const uiEnabledModules = collectEnabledModuleIdsFromUi();
    if (state.sensorViewMode === "all") {
      return uiEnabledModules;
    }
    const visibleModules = collectVisibleModuleIdsFromPayload(payload);
    if (visibleModules.size > 0) {
      return visibleModules;
    }
    return uiEnabledModules;
  }

  // Ported from index.html's resolveLinkColorForSelection (2419).
  function resolveLinkColorForSelection(linkName) {
    const m = state.urdfMesh;
    const normalizedLinkName = normalizeFrameId(linkName);
    const moduleId =
      m.linkModuleMap.get(normalizedLinkName) ||
      resolveModuleId(normalizedLinkName);
    if (!moduleId) {
      return URDF_NEUTRAL_LINK_COLOR;
    }
    if (!m.activeModuleIds.has(moduleId)) {
      return URDF_NEUTRAL_LINK_COLOR;
    }
    const urdfColor = m.linkUrdfColorMap.get(normalizedLinkName);
    return Number.isInteger(urdfColor) ? urdfColor : fallbackModuleColor(moduleId);
  }

  // Ported from index.html's refreshUrdfModuleHighlight (2435). urdfMeshRenderer must expose
  // refreshLinkMaterials(colorResolverFn) (see urdf_mesh_renderer.js) -- pass it in via the
  // factory's `urdfMeshRenderer` option; without it this is a no-op (no housing mesh to recolor
  // in "xela" ui mode demos, matching the original's behavior of only mattering in "robot" mode).
  function refreshUrdfModuleHighlight(payload) {
    const m = state.urdfMesh;
    const activeModules = computeActiveModuleIds(payload);
    const signature = [...activeModules].sort().join(",");
    if (signature === m.lastHighlightSignature) {
      return;
    }
    m.activeModuleIds = activeModules;
    m.lastHighlightSignature = signature;
    if (urdfMeshRenderer && typeof urdfMeshRenderer.refreshLinkMaterials === "function") {
      urdfMeshRenderer.refreshLinkMaterials(resolveLinkColorForSelection);
    }
  }

  // Ported from index.html's getLinkMaterial (2447).
  function getLinkMaterial(THREE, linkName, colorOverride = null) {
    const m = state.urdfMesh;
    const color = Number.isInteger(colorOverride)
      ? colorOverride
      : resolveLinkColorForSelection(linkName);
    const key = String(color);
    if (!m.materialCache.has(key)) {
      // Matches AH's getLinkMaterial exactly (2026-08-24). The washout symptoms blamed on this
      // material/lighting for most of a day turned out to be several DG-5F .dae files' embedded
      // PointLights (see the embeddedLights strip in urdf_mesh_renderer.js's
      // buildRobotFromDescription()) -- once those were actually removed, AH's original
      // roughness/metalness recipe renders correctly.
      m.materialCache.set(key, new THREE.MeshStandardMaterial({
        color,
        roughness: 0.72,
        metalness: 0.08,
      }));
    }
    return m.materialCache.get(key);
  }

  const _arrowHsl = { h: 0, s: 0, l: 0 };
  let lastMarkersPayload = null;
  let lastMarkersGuardKey = null;
  function updateUrdfMarkers(payload, forceColorRgb) {
    const m = state.urdfMesh;
    if (!m.ready) return;
    const guardKey = `${state.uiMode}|${m.lastHighlightSignature}|${getGamma()}`;
    if (payload === lastMarkersPayload && guardKey === lastMarkersGuardKey) {
      return;
    }
    lastMarkersPayload = payload;
    lastMarkersGuardKey = guardKey;
    const points = (payload?.urdf?.points && Array.isArray(payload.urdf.points)) ? payload.urdf.points : [];
    const liveKeys = new Set();
    const xyRange = Math.max(1e-6, Number(payload?.meta?.xy_force_range) || 0.8);
    const zRange = Math.max(1e-6, Number(payload?.meta?.z_force_range) || 14.0);
    const xyArrowGain = 3.0;
    const fixedFrame = normalizeFrameId(m.fixedFrame || payload?.fixed_frame || defaultFixedFrame);
    const strictCache = new Map();
    const strictVisiting = new Set();
    const looseCache = new Map();
    const looseVisiting = new Set();

    for (const p of points) {
      const key = `${p.module}:${p.sensor_index}`;
      liveKeys.add(key);
      const marker = ensureMarkerObject(key);

      const markerModuleId = resolveModuleId(p.frame_id);
      if (markerModuleId && m.lastHighlightSignature != null && !m.activeModuleIds.has(markerModuleId)) {
        marker.group.visible = false;
        continue;
      }
      marker.group.visible = true;

      let placedViaHousing = false;
      if (state.uiMode === "robot") {
        const moduleId = resolveModuleId(p.frame_id);
        const housingLinkName = moduleId ? `base_${moduleId}_link` : "";
        const housingGroup = housingLinkName ? m.linkGroups.get(housingLinkName) : null;
        if (housingGroup) {
          if (marker.robotHousingLink !== housingLinkName) {
            marker.robotLocalOffset = resolveTaxelLocalOffsetViaXela(p.frame_id, housingLinkName);
            marker.robotHousingLink = housingLinkName;
          }
          if (marker.robotLocalOffset) {
            if (marker.group.parent !== housingGroup) {
              housingGroup.add(marker.group);
            }
            marker.group.position.set(
              marker.robotLocalOffset.tx,
              marker.robotLocalOffset.ty,
              marker.robotLocalOffset.tz,
            );
            marker.group.quaternion.set(
              marker.robotLocalOffset.qx,
              marker.robotLocalOffset.qy,
              marker.robotLocalOffset.qz,
              marker.robotLocalOffset.qw,
            );
            placedViaHousing = true;
          }
        }
      }
      if (!placedViaHousing) {
        if (marker.group.parent !== m.markerGroup) {
          m.markerGroup.add(marker.group);
        }
        const tfPose = resolveMarkerPose(
          p.frame_id,
          fixedFrame,
          strictCache,
          strictVisiting,
          looseCache,
          looseVisiting,
        );
        if (tfPose) {
          marker.group.position.set(tfPose.tx, tfPose.ty, tfPose.tz);
          marker.group.quaternion.set(tfPose.qx, tfPose.qy, tfPose.qz, tfPose.qw);
        } else {
          marker.group.position.set(Number(p.x) || 0, Number(p.y) || 0, Number(p.z) || 0);
          marker.group.quaternion.set(0, 0, 0, 1);
        }
      }
      // uSPa22 (mid/prox, 2x2) and uSPa46 (palm, 4x6) taxel dots read as floating/protruding
      // above their housing (2026-08-24 user feedback) -- fingertip taxels looked fine as-is.
      const frameIdStr = String(p.frame_id || "");
      if (frameIdStr.includes("uSPa46")) {
        marker.sphere.position.set(0, 0, -0.0028);
        marker.sphere.scale.setScalar(1);
      } else if (frameIdStr.includes("uSPa22")) {
        marker.sphere.position.set(-0.0001, 0.0003, -0.0014);
        marker.sphere.scale.setScalar(0.85);
      } else {
        marker.sphere.position.set(0, 0, 0);
        marker.sphere.scale.setScalar(1);
      }

      const norm = operatorAdjustNorm(Math.max(0, Math.min(1, Number(p.norm) || 0)));
      const rgb = forceColorRgb(norm);
      marker.sphere.material.color.setRGB(rgb.r / 255, rgb.g / 255, rgb.b / 255);

      // Sidecar rendering convention: X axis is mirrored vs sensor stream.
      const fxRaw = -(Number(p.fx) || 0);
      const fyRaw = Number(p.fy) || 0;
      const fz = Number(p.fz) || 0;
      const dir = new m.THREE.Vector3(
        (fxRaw / xyRange) * xyArrowGain,
        (fyRaw / xyRange) * xyArrowGain,
        fz / zRange,
      );
      const mag = dir.length();
      if (mag < 0.012) {
        marker.arrow.visible = false;
        continue;
      }
      marker.arrow.visible = true;
      dir.normalize();
      const length = clamp(0.004 + mag * 0.012, 0.004, 0.028);
      marker.arrow.setDirection(dir);
      marker.arrow.setLength(length, Math.min(0.011, length * 0.6), Math.min(0.007, length * 0.4));
      const arrowColor = new m.THREE.Color(rgb.r / 255, rgb.g / 255, rgb.b / 255);
      arrowColor.getHSL(_arrowHsl);
      arrowColor.setHSL(_arrowHsl.h, 1.0, Math.min(0.62, Math.max(0.42, _arrowHsl.l)));
      marker.arrow.setColor(arrowColor);
    }

    for (const [key, marker] of m.markerMap.entries()) {
      if (!liveKeys.has(key)) {
        marker.group.visible = false;
      }
    }
  }

  function autoFrameUrdfMeshCamera(force = false) {
    const m = state.urdfMesh;
    if (!m.ready || !m.robotBuilt) return;
    if (!force && m.autoFrameDone) return;

    const THREE = m.THREE;
    const box = new THREE.Box3();
    let hasVisible = false;
    for (const group of m.linkGroups.values()) {
      if (!group.visible) continue;
      box.expandByObject(group);
      hasVisible = true;
    }
    if (!hasVisible || box.isEmpty()) return;

    const center = new THREE.Vector3();
    const size = new THREE.Vector3();
    box.getCenter(center);
    box.getSize(size);
    const radius = Math.max(size.length() * 0.45, 0.02);

    if (state.uiMode === "robot") {
      const src = getActiveUrdfSource();
      const preferredBaseLinks = [
        normalizeFrameId(src?.robotRootLink || ""),
        "base_link",
        "base_link_inertia",
        "base",
        "base_footprint",
      ].filter((n, idx, arr) => n && arr.indexOf(n) === idx);
      let foundBase = false;
      for (const name of preferredBaseLinks) {
        const group = m.linkGroups.get(name);
        if (!group || !group.visible) continue;
        const basePos = new THREE.Vector3();
        group.getWorldPosition(basePos);
        center.set(basePos.x, basePos.y, 0.0);
        foundBase = true;
        break;
      }
      if (!foundBase) {
        center.set(0, 0, 0);
      }
    }

    m.controls.target.copy(center);
    m.camera.position.set(
      center.x + radius * 2.4,
      center.y - radius * 0.5,
      center.z + radius * 0.6,
    );
    m.camera.near = Math.max(radius / 1000.0, 0.0005);
    m.camera.far = Math.max(radius * 600.0, 20.0);
    m.camera.updateProjectionMatrix();
    if (typeof applyOrbitProfileForMode === "function") {
      applyOrbitProfileForMode();
    }
    m.controls.update();
    m.autoFrameDone = true;
  }

  function computeModuleCenterFromTf(points, moduleId, fixedFrame, strictCache, strictVisiting) {
    let x = 0.0;
    let y = 0.0;
    let z = 0.0;
    let n = 0;
    const seenFrames = new Set();
    for (const p of points) {
      const idFromFrame = resolveModuleId(p?.frame_id);
      if (idFromFrame !== moduleId) continue;
      const rawFrame = normalizeFrameId(p?.frame_id);
      if (!rawFrame) continue;
      const frame = resolveChildFrameAlias(rawFrame) || rawFrame;
      if (!frame || seenFrames.has(frame)) continue;
      seenFrames.add(frame);
      let tf = resolveTaxelFixedFramePoseRobust(frame, fixedFrame, strictCache, strictVisiting);
      if (!tf) {
        tf = resolveFrameToFixed(frame, fixedFrame, strictCache, strictVisiting, true);
      }
      if (!tf) continue;
      x += tf.tx;
      y += tf.ty;
      z += tf.tz;
      n += 1;
    }
    if (n <= 0) return null;
    return { x: x / n, y: y / n, z: z / n };
  }

  // Ported from index.html's updateFollowCamera (2013). followCamState is created by
  // createFollowCamState() below (caller-owned instance, DG5F pinch/palm module ids and tuning
  // constants passed in as config -- see that factory) instead of a page-global const, and debug
  // text goes through the injected onFollowDebugText callback instead of a direct DOM write.
  function updateFollowCamera(nowMs, followCamState, formatVec3Fn) {
    const formatVec3 = typeof formatVec3Fn === "function"
      ? formatVec3Fn
      : (vec) => (vec ? `[${vec.x.toFixed(3)},${vec.y.toFixed(3)},${vec.z.toFixed(3)}]` : "[-,-,-]");
    const setFollowDebugTextThrottled = (nowMsInner, text) => {
      if ((nowMsInner - followCamState.debugLastUpdateMs) < followCamState.debugUpdatePeriodMs) {
        return;
      }
      followCamState.debugLastUpdateMs = nowMsInner;
      if (followCamState.debugText === text) return;
      followCamState.debugText = text;
      notifyFollowDebug(text);
    };

    if (!followCamState.enabled || state.uiMode !== "robot") {
      return;
    }
    const m = state.urdfMesh;
    if (!m.ready || !m.controls || !m.camera || !m.THREE) {
      setFollowDebugTextThrottled(nowMs, "fc waiting: URDF mesh renderer not ready");
      return;
    }
    const fixedFrame = normalizeFrameId(m.fixedFrame || defaultFixedFrame || "world");
    const strictCache = new Map();
    const strictVisiting = new Set();
    const points = Array.isArray(state.payload?.urdf?.points) ? state.payload.urdf.points : [];
    const tipF1 = computeModuleCenterFromTf(points, followCamState.tipModuleF1, fixedFrame, strictCache, strictVisiting);
    const tipF2 = computeModuleCenterFromTf(points, followCamState.tipModuleF2, fixedFrame, strictCache, strictVisiting);
    const palmCenterRaw = computeModuleCenterFromTf(points, followCamState.tipModulePalm, fixedFrame, strictCache, strictVisiting);
    if (!tipF1 || !tipF2) {
      setFollowDebugTextThrottled(nowMs, "fc waiting: tipF1/tipF2 unresolved");
      return;
    }

    const THREE = m.THREE;
    const tipF1Vec = new THREE.Vector3(tipF1.x, tipF1.y, tipF1.z);
    const tipF2Vec = new THREE.Vector3(tipF2.x, tipF2.y, tipF2.z);
    const targetBaseRaw = tipF1Vec.clone().add(tipF2Vec).multiplyScalar(0.5);
    if (
      followCamState.filteredTargetBase &&
      targetBaseRaw.distanceTo(followCamState.filteredTargetBase) > followCamState.jumpRejectDist
    ) {
      followCamState.jumpRejectCount += 1;
      if (followCamState.jumpRejectCount < followCamState.jumpRejectLimit) {
        return;
      }
      followCamState.filteredTargetBase = targetBaseRaw.clone();
      followCamState.filteredForward = null;
      followCamState.lastCameraGoal = null;
      followCamState.lastTargetGoal = null;
      followCamState.jumpRejectCount = 0;
    } else {
      followCamState.jumpRejectCount = 0;
    }
    if (!followCamState.filteredTargetBase) {
      followCamState.filteredTargetBase = targetBaseRaw.clone();
    } else {
      const targetBaseDelta = targetBaseRaw.distanceTo(followCamState.filteredTargetBase);
      if (targetBaseDelta >= followCamState.minPosDeadband) {
        followCamState.filteredTargetBase.lerp(targetBaseRaw, followCamState.targetBaseLerp);
      }
    }
    const targetBase = followCamState.filteredTargetBase.clone();
    const worldUp = new THREE.Vector3(0, 0, 1);

    let rawForward = null;
    let forwardSource = "held_last_good";
    if (palmCenterRaw) {
      const palmCenter = new THREE.Vector3(palmCenterRaw.x, palmCenterRaw.y, palmCenterRaw.z);
      const palmToTip = targetBase.clone().sub(palmCenter);
      if (palmToTip.lengthSq() >= 1e-8) {
        rawForward = palmToTip;
        forwardSource = "palm_to_tip";
      }
    }
    if (!rawForward && !followCamState.filteredForward) {
      setFollowDebugTextThrottled(nowMs, "fc waiting: no reliable palm_to_tip sample yet");
      return;
    }
    if (rawForward) {
      rawForward.normalize();
      rawForward.multiplyScalar(followCamState.forwardSign >= 0.0 ? 1.0 : -1.0);
      if (!followCamState.filteredForward) {
        followCamState.filteredForward = rawForward.clone();
      } else {
        const dot = Math.max(-1.0, Math.min(1.0, followCamState.filteredForward.dot(rawForward)));
        const angleDeg = Math.acos(dot) * 180.0 / Math.PI;
        if (angleDeg >= followCamState.minForwardDeg) {
          followCamState.filteredForward.lerp(rawForward, followCamState.forwardLerp).normalize();
        }
      }
    }
    const forward = followCamState.filteredForward;
    let rightBasis = forward.clone().cross(worldUp);
    if (rightBasis.lengthSq() < 1e-8) {
      rightBasis = new THREE.Vector3(1, 0, 0);
    }
    rightBasis.normalize();
    const upBasis = rightBasis.clone().cross(forward).normalize();
    const captureLocalOffset = (worldPos) => {
      const delta = worldPos.clone().sub(targetBase);
      return {
        right: delta.dot(rightBasis),
        up: delta.dot(upBasis),
        forward: delta.dot(forward),
      };
    };
    const applyLocalOffset = (localOffset) => targetBase
      .clone()
      .addScaledVector(rightBasis, localOffset.right)
      .addScaledVector(upBasis, localOffset.up)
      .addScaledVector(forward, localOffset.forward);

    if (followCamState.manualOrbitActive || followCamState.captureViewOnNextUpdate) {
      followCamState.userCameraOffsetLocal = captureLocalOffset(m.camera.position);
      followCamState.userTargetOffsetLocal = captureLocalOffset(m.controls.target);
      followCamState.captureViewOnNextUpdate = false;
      if (followCamState.manualOrbitActive) {
        setFollowDebugTextThrottled(nowMs, "fc manual orbit: tracking paused");
        return;
      }
    }

    let camGoal;
    let targetGoal;
    if (followCamState.userCameraOffsetLocal && followCamState.userTargetOffsetLocal) {
      camGoal = applyLocalOffset(followCamState.userCameraOffsetLocal);
      targetGoal = applyLocalOffset(followCamState.userTargetOffsetLocal);
    } else {
      camGoal = targetBase
        .clone()
        .addScaledVector(forward, followCamState.forwardDistance)
        .addScaledVector(worldUp, followCamState.heightOffset);
      targetGoal = targetBase
        .clone()
        .addScaledVector(forward, -followCamState.lookAhead)
        .addScaledVector(worldUp, followCamState.lookUp);
      if (camGoal.z <= targetGoal.z + 0.01) {
        camGoal.z = targetGoal.z + 0.01;
      }
    }

    setFollowDebugTextThrottled(
      nowMs,
      `fc frame=${fixedFrame} src=${forwardSource} tipF1=${formatVec3(tipF1Vec)} tipF2=${formatVec3(tipF2Vec)} fwd=${formatVec3(forward)}`,
    );

    if (followCamState.snapOnEnable) {
      followCamState.lastCameraGoal = camGoal.clone();
      followCamState.lastTargetGoal = targetGoal.clone();
      m.camera.position.copy(camGoal);
      m.controls.target.copy(targetGoal);
      m.controls.update();
      followCamState.snapOnEnable = false;
      return;
    }

    followCamState.lastCameraGoal = camGoal.clone();
    followCamState.lastTargetGoal = targetGoal.clone();
    const nextCam = m.camera.position.clone().lerp(
      followCamState.lastCameraGoal,
      followCamState.positionLerp,
    );
    const camDelta = nextCam.clone().sub(m.camera.position);
    const camStep = camDelta.length();
    if (camStep > followCamState.maxCamStep) {
      camDelta.setLength(followCamState.maxCamStep);
      nextCam.copy(m.camera.position).add(camDelta);
    }
    const nextTarget = m.controls.target.clone().lerp(
      followCamState.lastTargetGoal,
      followCamState.targetLerp,
    );
    const targetDelta = nextTarget.clone().sub(m.controls.target);
    const targetStep = targetDelta.length();
    if (targetStep > followCamState.maxTargetStep) {
      targetDelta.setLength(followCamState.maxTargetStep);
      nextTarget.copy(m.controls.target).add(targetDelta);
    }
    m.camera.position.copy(nextCam);
    m.controls.target.copy(nextTarget);
  }

  // Ported/parameterized from index.html's page-global `followCamState` object literal (834) --
  // DG5F's pinch-reference module ids (f1/f2/palm) and tuning constants (forwardSign,
  // forwardDistance, etc., originally read from URL params into module-scope consts) are now
  // factory inputs instead of hardcoded, so a different hand/gripper can supply its own. Defaults
  // below match DG5F's tuned values from the original file.
  function createFollowCamState(config = {}) {
    return {
      enabled: false,
      manualOrbitActive: false,
      captureViewOnNextUpdate: false,
      tipModuleF1: config.tipModuleF1 ?? "f1_dg5f_ft",
      tipModuleF2: config.tipModuleF2 ?? "f2_dg5f_ft",
      tipModulePalm: config.tipModulePalm ?? "palm_uSPa46",
      forwardSign: config.forwardSign ?? 1.0,
      forwardDistance: config.forwardDistance ?? 0.22,
      heightOffset: config.heightOffset ?? 0.10,
      lookAhead: config.lookAhead ?? 0.06,
      lookUp: 0.0,
      positionLerp: 0.24,
      targetLerp: 0.26,
      targetBaseLerp: 0.30,
      forwardLerp: 0.22,
      maxCamStep: 0.030,
      maxTargetStep: 0.024,
      tipFallbackDistance: 0.045,
      minPosDeadband: 0.0001,
      minForwardDeg: 0.8,
      jumpRejectDist: 0.16,
      jumpRejectLimit: 2,
      jumpRejectCount: 0,
      filteredTargetBase: null,
      filteredForward: null,
      lastCameraGoal: null,
      lastTargetGoal: null,
      userCameraOffsetLocal: null,
      userTargetOffsetLocal: null,
      snapOnEnable: false,
      debugText: "",
      debugLastUpdateMs: 0.0,
      debugUpdatePeriodMs: 120.0,
    };
  }

  return {
    clamp,
    normalizeFrameId,
    getUrdfSourceForUiMode,
    getActiveUrdfSource,
    bindActiveUrdfSourceToMesh,
    getFrameLeafName,
    getFrameLeafIndex,
    resolveChildFrameAlias,
    selectFixedFrame,
    selectFixedFrameUncached,
    resolveFrameToFixed,
    updateRobotLinkTransforms,
    ensureMarkerObject,
    resolveMarkerPose,
    resolveTaxelLocalOffsetViaXela,
    resolveTaxelFixedFramePoseRobust,
    updateUrdfMarkers,
    autoFrameUrdfMeshCamera,
    computeModuleCenterFromTf,
    updateFollowCamera,
    createFollowCamState,
    operatorAdjustNorm,
    collectVisibleModuleIdsFromPayload,
    computeActiveModuleIds,
    resolveLinkColorForSelection,
    refreshUrdfModuleHighlight,
    getLinkMaterial,
  };
}
