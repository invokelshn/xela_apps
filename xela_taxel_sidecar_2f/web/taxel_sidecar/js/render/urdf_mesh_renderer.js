function parseFloatTriple(text, fallback) {
  if (!text) return fallback;
  const parts = String(text).trim().split(/\s+/).map((v) => Number(v));
  if (parts.length !== 3 || parts.some((v) => Number.isNaN(v))) {
    return fallback;
  }
  return parts;
}

function urdfRpyToQuaternion(roll, pitch, yaw) {
  const hr = roll * 0.5;
  const hp = pitch * 0.5;
  const hy = yaw * 0.5;
  const cr = Math.cos(hr);
  const sr = Math.sin(hr);
  const cp = Math.cos(hp);
  const sp = Math.sin(hp);
  const cy = Math.cos(hy);
  const sy = Math.sin(hy);
  return {
    qx: (sr * cp * cy) - (cr * sp * sy),
    qy: (cr * sp * cy) + (sr * cp * sy),
    qz: (cr * cp * sy) - (sr * sp * cy),
    qw: (cr * cp * cy) + (sr * sp * sy),
  };
}

function packageUriToHttpPath(uri) {
  const text = String(uri || "");
  if (!text.startsWith("package://")) {
    return text;
  }
  const rem = text.slice("package://".length);
  const slash = rem.indexOf("/");
  if (slash <= 0) {
    return "";
  }
  const pkg = rem.slice(0, slash);
  const rel = rem
    .slice(slash + 1)
    .replace(/^\/+/, "")
    .replace(/\/{2,}/g, "/");
  return `/pkg/${pkg}/${rel}`;
}

function parseUrdfVisualSpec(xmlText) {
  const parser = new DOMParser();
  const xml = parser.parseFromString(xmlText, "application/xml");
  if (xml.getElementsByTagName("parsererror").length > 0) {
    return [];
  }
  const linkSpecs = [];
  const linkElements = Array.from(xml.getElementsByTagName("link"));
  for (const linkEl of linkElements) {
    const linkName = linkEl.getAttribute("name");
    if (!linkName) continue;
    const visuals = [];
    const visualEls = Array.from(linkEl.getElementsByTagName("visual"));
    for (const visualEl of visualEls) {
      const geometryEl = visualEl.getElementsByTagName("geometry")[0];
      if (!geometryEl) continue;
      const meshEl = geometryEl.getElementsByTagName("mesh")[0];
      if (!meshEl) continue;
      const filename = meshEl.getAttribute("filename");
      if (!filename) continue;
      const ext = (filename.match(/\.([a-zA-Z0-9]+)(?:\?|#|$)/)?.[1] || "").toLowerCase();
      if (ext !== "stl" && ext !== "dae") continue;
      const originEl = visualEl.getElementsByTagName("origin")[0];
      visuals.push({
        filename,
        ext,
        scale: parseFloatTriple(meshEl.getAttribute("scale"), [1, 1, 1]),
        xyz: parseFloatTriple(originEl ? originEl.getAttribute("xyz") : "", [0, 0, 0]),
        rpy: parseFloatTriple(originEl ? originEl.getAttribute("rpy") : "", [0, 0, 0]),
      });
    }
    if (visuals.length > 0) {
      linkSpecs.push({ linkName, visuals });
    }
  }
  return linkSpecs;
}

export function createUrdfMeshRenderer({
  state,
  urdf3dLayer,
  normalizeFrameId,
  setHintText,
  safeErrorText,
  applyOrbitProfileForMode,
  getLinkMaterial,
  onOrbitInteraction,
}) {
  function parseRootLink(xmlText) {
    const parser = new DOMParser();
    const xml = parser.parseFromString(xmlText, "application/xml");
    if (xml.getElementsByTagName("parsererror").length > 0) {
      return "";
    }
    const links = new Set();
    const childLinks = new Set();
    for (const linkEl of Array.from(xml.getElementsByTagName("link"))) {
      const name = normalizeFrameId(linkEl.getAttribute("name"));
      if (name) links.add(name);
    }
    for (const jointEl of Array.from(xml.getElementsByTagName("joint"))) {
      const childEl = jointEl.getElementsByTagName("child")[0];
      const childName = normalizeFrameId(childEl?.getAttribute("link"));
      if (childName) childLinks.add(childName);
    }
    for (const link of links) {
      if (!childLinks.has(link)) {
        return link;
      }
    }
    return links.size ? Array.from(links)[0] : "";
  }

  function resize() {
    const m = state.urdfMesh;
    if (!m.ready) return;
    const rect = urdf3dLayer.getBoundingClientRect();
    const w = Math.max(1, rect.width);
    const h = Math.max(1, rect.height);
    m.camera.aspect = w / h;
    m.camera.updateProjectionMatrix();
    m.renderer.setSize(w, h, false);
  }

  async function ensureRuntime() {
    const m = state.urdfMesh;
    if (m.ready) return true;
    if (m.loading) return false;
    if (m.failed) return false;

    m.loading = true;
    try {
      const [THREEMod, controlsMod, stlMod, colladaMod] = await Promise.all([
        import("../../vendor/three.module.js"),
        import("../../vendor/OrbitControls.js"),
        import("../../vendor/STLLoader.js"),
        import("../../vendor/ColladaLoader.js"),
      ]);
      const THREE = THREEMod;
      const { OrbitControls } = controlsMod;
      const { STLLoader } = stlMod;
      const { ColladaLoader } = colladaMod;

      const scene = new THREE.Scene();
      scene.background = new THREE.Color(0x0a1222);

      const camera = new THREE.PerspectiveCamera(48, 1, 0.001, 20.0);
      camera.up.set(0, 0, 1);
      camera.position.set(0.18, -0.22, 0.16);

      const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
      renderer.setPixelRatio(Math.max(1, window.devicePixelRatio || 1));
      renderer.domElement.style.width = "100%";
      renderer.domElement.style.height = "100%";
      renderer.domElement.style.display = "block";
      urdf3dLayer.innerHTML = "";
      urdf3dLayer.appendChild(renderer.domElement);
      renderer.domElement.addEventListener("contextmenu", (ev) => ev.preventDefault());

      const controls = new OrbitControls(camera, renderer.domElement);
      controls.enableDamping = true;
      controls.mouseButtons = {
        LEFT: THREE.MOUSE.ROTATE,
        MIDDLE: THREE.MOUSE.DOLLY,
        RIGHT: THREE.MOUSE.PAN,
      };
      controls.target.set(0.0, 0.0, 0.0);
      controls.update();
      if (typeof onOrbitInteraction === "function") {
        const notifyOrbitInteraction = (kind) => () => onOrbitInteraction(kind);
        controls.addEventListener("start", notifyOrbitInteraction("start"));
        controls.addEventListener("end", notifyOrbitInteraction("end"));
        renderer.domElement.addEventListener("pointerdown", notifyOrbitInteraction("pointerdown"));
        renderer.domElement.addEventListener("wheel", notifyOrbitInteraction("wheel"), { passive: true });
      }

      const amb = new THREE.AmbientLight(0xd0e3ff, 0.75);
      scene.add(amb);
      const dirA = new THREE.DirectionalLight(0xffffff, 0.9);
      dirA.position.set(0.7, -0.8, 1.2);
      scene.add(dirA);
      const dirB = new THREE.DirectionalLight(0x93c5fd, 0.4);
      dirB.position.set(-0.6, 0.8, 0.7);
      scene.add(dirB);

      const rootGroup = new THREE.Group();
      const robotGroup = new THREE.Group();
      const markerGroup = new THREE.Group();
      const worldGrid = new THREE.GridHelper(1.4, 28, 0x3b4f73, 0x1f3256);
      worldGrid.rotation.x = Math.PI / 2;
      worldGrid.position.set(0, 0, 0);
      worldGrid.visible = false;
      rootGroup.add(worldGrid);
      rootGroup.add(robotGroup);
      rootGroup.add(markerGroup);
      scene.add(rootGroup);

      m.THREE = THREE;
      m.scene = scene;
      m.camera = camera;
      m.renderer = renderer;
      m.controls = controls;
      m.stlLoader = new STLLoader();
      m.colladaLoader = new ColladaLoader();
      m.rootGroup = rootGroup;
      m.robotGroup = robotGroup;
      m.markerGroup = markerGroup;
      m.worldGrid = worldGrid;
      m.ready = true;
      m.loading = false;
      applyOrbitProfileForMode();
      resize();
      return true;
    } catch (err) {
      m.failed = true;
      m.loading = false;
      m.failReason = err instanceof Error ? err.message : String(err);
      setHintText(`URDF mesh renderer unavailable: ${m.failReason}`);
      return false;
    }
  }

  async function getMeshAsset(url, ext) {
    const m = state.urdfMesh;
    const kind = (ext || "").toLowerCase();
    const key = `${kind}:${url}`;
    if (m.meshCache.has(key)) {
      return m.meshCache.get(key);
    }
    const promise = new Promise((resolve, reject) => {
      if (kind === "dae") {
        m.colladaLoader.load(
          url,
          (collada) => resolve({ kind: "dae", data: collada.scene }),
          undefined,
          (error) => reject(error),
        );
        return;
      }
      m.stlLoader.load(
        url,
        (geometry) => resolve({ kind: "stl", data: geometry }),
        undefined,
        (error) => reject(error),
      );
    });
    m.meshCache.set(key, promise);
    return promise;
  }

  async function buildRobotFromDescription() {
    const m = state.urdfMesh;
    if (!m.ready || !m.robotDescriptionXml) return false;

    const specs = parseUrdfVisualSpec(m.robotDescriptionXml);
    if (!specs.length) {
      m.failReason = "No mesh visuals parsed from robot_description.";
      m.robotBuilt = false;
      return false;
    }

    let meshLoadAttemptCount = 0;
    let meshLoadSuccessCount = 0;
    let meshLoadFailCount = 0;
    let firstMeshError = "";

    m.robotBuildToken += 1;
    const token = m.robotBuildToken;
    m.linkGroups.clear();
    m.robotGroup.clear();
    m.markerMap.clear();
    m.markerGroup.clear();

    const THREE = m.THREE;
    const pending = [];
    for (const linkSpec of specs) {
      const linkGroup = new THREE.Group();
      linkGroup.name = linkSpec.linkName;
      linkGroup.visible = false;
      m.linkGroups.set(linkSpec.linkName, linkGroup);
      m.robotGroup.add(linkGroup);

      for (const visual of linkSpec.visuals) {
        const meshUrl = packageUriToHttpPath(visual.filename);
        if (!meshUrl) continue;
        meshLoadAttemptCount += 1;
        const task = getMeshAsset(meshUrl, visual.ext)
          .then((asset) => {
            if (token !== m.robotBuildToken) return;

            const visualRoot = new THREE.Group();
            visualRoot.position.set(visual.xyz[0], visual.xyz[1], visual.xyz[2]);
            const q = urdfRpyToQuaternion(visual.rpy[0], visual.rpy[1], visual.rpy[2]);
            visualRoot.quaternion.set(q.qx, q.qy, q.qz, q.qw);
            visualRoot.scale.set(visual.scale[0], visual.scale[1], visual.scale[2]);

            if (asset.kind === "dae") {
              const rootObj = asset.data.clone(true);
              if (
                Math.abs(rootObj.rotation.x + (Math.PI / 2)) < 1e-6 &&
                Math.abs(rootObj.rotation.y) < 1e-6 &&
                Math.abs(rootObj.rotation.z) < 1e-6
              ) {
                rootObj.rotation.set(0, 0, 0);
                rootObj.updateMatrixWorld(true);
              }
              rootObj.traverse((node) => {
                if (!node || !node.isMesh) return;
                node.material = getLinkMaterial(THREE, linkSpec.linkName);
              });
              visualRoot.add(rootObj);
            } else {
              const material = getLinkMaterial(THREE, linkSpec.linkName);
              const mesh = new THREE.Mesh(asset.data, material);
              visualRoot.add(mesh);
            }

            linkGroup.add(visualRoot);
            meshLoadSuccessCount += 1;
          })
          .catch((err) => {
            meshLoadFailCount += 1;
            if (!firstMeshError) {
              firstMeshError = `${meshUrl}: ${safeErrorText(err)}`;
            }
            console.warn("[xela-sidecar] mesh load failed:", meshUrl, err);
          });
        pending.push(task);
      }
    }

    await Promise.all(pending);
    if (token !== m.robotBuildToken) return false;

    if (meshLoadAttemptCount > 0 && meshLoadSuccessCount === 0) {
      m.robotBuilt = false;
      m.failReason =
        `No URDF meshes loaded (attempted=${meshLoadAttemptCount}, failed=${meshLoadFailCount}). ` +
        `${firstMeshError || "Check /pkg/<package>/... resource path."}`;
      return false;
    }

    if (meshLoadAttemptCount > 0 && meshLoadFailCount > 0) {
      console.warn(
        `[xela-sidecar] partial mesh load: success=${meshLoadSuccessCount}, failed=${meshLoadFailCount}`,
      );
    }
    m.failReason = "";
    m.robotBuilt = true;
    m.autoFrameDone = false;
    return true;
  }

  return {
    parseRootLink,
    ensureRuntime,
    resize,
    buildRobotFromDescription,
  };
}
