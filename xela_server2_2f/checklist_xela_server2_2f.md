# Checklist - xela_server2_2f

## Phase 1: Design Review Checklist
- [x] Confirm package name, node name, and topic name
- [x] Confirm parameter list and defaults
- [x] Confirm message mapping rules and ordering
- [x] Confirm YAML model mapping (aliases, gen_frame_list, frame_ids/names)
- [x] Confirm error handling and drop behavior
- [x] Confirm QoS depth and header.frame_id usage
- [x] Agree on libraries (JSON, WebSocket, YAML) for implementation
- [x] Agree on minimal test scope and sample data source

### Phase 1: Verification Procedure
- [x] Review FSD against current requirements and confirm no open items
- [x] Record any decisions on libraries and QoS as final design notes

## Phase 2: Development Checklist
- [x] Create package scaffold under `xela_apps/xela_server2_2f`
- [x] Implement WebSocket client connection and reconnection
- [x] Implement JSON parsing and validation
- [x] Implement integer parsing into Taxel[]
- [x] Implement calibrated parsing into Forces[]
- [x] Implement temp conversion into temps[]
- [x] Implement model config loading (global frame_prefix, aliases, gen_frame_list)
- [x] Implement ROS2 publisher for `/x_taxel_2f` (XTaxelSensorTArray)
- [x] Implement parameters and defaults
- [x] Add logging for connection and drop reasons
- [x] Build and resolve dependencies

### Phase 2: Verification Procedure
- [x] `colcon build` succeeds for the workspace
- [x] Node starts without runtime errors and connects to WebSocket

## Phase 3: Test Checklist
- [x] Unit test: integer parsing -> Taxel triples
- [x] Unit test: calibrated parsing -> Forces triples
- [x] Unit test: temps Kelvin->Celsius conversion
- [x] Unit test: YAML model alias + gen_frame_list rules
- [x] Integration test with `tmp/websocket.json`
- [x] Runtime check: `ros2 topic echo /x_taxel_2f`
- [x] Verify ROS time override parameter behavior

### Phase 3: Verification Procedure
- [x] Execute unit tests and confirm all pass
- [x] Feed `tmp/websocket.json` through the node and validate output counts
- [x] Confirm `/x_taxel_2f` publishes with expected content

## Phase 4: Delivery Checklist
- [x] Document build/run steps
- [x] Provide launch/run command examples
- [x] Final review against FSD acceptance criteria

### Phase 4: Verification Procedure
- [x] Run the documented steps end-to-end in a fresh shell
- [x] Validate acceptance criteria with the sample JSON
