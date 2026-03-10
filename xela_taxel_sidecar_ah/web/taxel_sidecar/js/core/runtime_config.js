export function buildRuntimeConfig(windowRef) {
  const params = new URLSearchParams(windowRef.location.search);
  const parseNum = (value, fallback) => {
    if (value == null) return fallback;
    if (typeof value === "string" && value.trim() === "") return fallback;
    const n = Number(value);
    return Number.isFinite(n) ? n : fallback;
  };
  const wsProto = windowRef.location.protocol === "https:" ? "wss" : "ws";
  const explicitWs = params.get("ws");
  const wsCandidates = explicitWs ? [explicitWs] : [
    `${wsProto}://${windowRef.location.hostname}:9090`,
    `${wsProto}://${windowRef.location.hostname}:3201`,
    `${wsProto}://${windowRef.location.hostname}/ros`,
  ];

  const topic = params.get("topic") || "/x_taxel_ah/web_state";
  const topicNs = (params.get("ns") || "/xvizah").replace(/\/+$/, "");

  const xelaRobotDescriptionTopic =
    params.get("xela_robot_description_topic") ||
    params.get("robot_description_topic") ||
    `${topicNs}/robot_description`;
  const xelaTfTopic =
    params.get("xela_tf_topic") ||
    params.get("tf_topic") ||
    `${topicNs}/tf`;
  const xelaTfStaticTopic =
    params.get("xela_tf_static_topic") ||
    params.get("tf_static_topic") ||
    `${topicNs}/tf_static`;
  const xelaRobotDescriptionService =
    params.get("xela_robot_description_service") ||
    params.get("robot_description_service") ||
    `${topicNs}/robot_state_publisher/get_parameters`;

  const robotRobotDescriptionTopic =
    params.get("robot_model_description_topic") ||
    "/robot_description";
  const robotTfTopic =
    params.get("robot_model_tf_topic") ||
    "/tf";
  const robotTfStaticTopic =
    params.get("robot_model_tf_static_topic") ||
    "/tf_static";
  const robotRobotDescriptionService =
    params.get("robot_model_description_service") ||
    "/robot_state_publisher/get_parameters";

  const robotModelFixedFrame =
    (params.get("robot_model_fixed_frame") || "").replace(/^\/+/, "").trim();
  const defaultFixedFrame = (params.get("fixed_frame") || "world").replace(/^\//, "");
  const gridVectorModeRaw = String(
    params.get("grid_vector_mode") || params.get("vector_mode") || "xy",
  ).toLowerCase();
  const gridVectorMode = gridVectorModeRaw === "xyz" ? "xyz" : "xy";
  const vector3dProjectX = parseNum(params.get("vector3d_proj_x"), -0.46);
  const vector3dProjectY = parseNum(params.get("vector3d_proj_y"), 0.72);
  const followCamForwardDistance = parseNum(params.get("follow_cam_forward_distance"), -0.20);
  const followCamHeightOffset = parseNum(params.get("follow_cam_height_offset"), 0.10);
  const followCamLookAhead = parseNum(params.get("follow_cam_look_ahead"), 0.028);
  const followCamForwardSign = parseNum(params.get("follow_cam_forward_sign"), -1.0);

  const vizNodeName = params.get("viz_node") || "/xvizah/std_xela_taxel_viz_ahv4";
  const bridgeNodeName = params.get("bridge_node") || "/xela_taxel_web_bridge_cpp";

  return {
    wsCandidates,
    topic,
    topicNs,
    xelaRobotDescriptionTopic,
    xelaTfTopic,
    xelaTfStaticTopic,
    xelaRobotDescriptionService,
    robotRobotDescriptionTopic,
    robotTfTopic,
    robotTfStaticTopic,
    robotRobotDescriptionService,
    robotModelFixedFrame,
    defaultFixedFrame,
    gridVectorMode,
    vector3dProjectX,
    vector3dProjectY,
    followCamForwardDistance,
    followCamHeightOffset,
    followCamLookAhead,
    followCamForwardSign,
    vizNodeName,
    bridgeNodeName,
  };
}
