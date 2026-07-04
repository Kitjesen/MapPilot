# LingTu 浜у搧妯″紡杩愯鍚堝悓

鏈枃璇存槑涓€灞備骇鍝佹ā寮忛渶瑕佷覆鑱旂殑鍔熻兘銆備唬鐮佸悎鍚屽湪
`src/runtime/profiles/product_mode_contracts.py`锛岄潤鎬佸浘娴嬭瘯鍦?`src/runtime/tests/test_profile_graph_snapshots.py`銆?
## 鎬诲師鍒?

鍓嶇銆丆LI銆丮CP 鍙彂璇锋眰鍜屾樉绀虹姸鎬侊紝涓嶇洿鎺ュ喅瀹氬鑸矾寰勶紝涔熶笉鐩存帴缁曡繃
`CmdVelMux` 鎺у埗鏈哄櫒鐙椼€?

鐩爣鐐瑰彧杩涘叆浠诲姟鎴栫洰鏍囨湇鍔°€傚彧鏈?`tracking`銆乣nav`銆乣inspection`銆乣tare_explore` 浼氬惎鍔?`nav.mission -> nav.local_planner -> nav.path_follower -> nav.velocity_mux`銆?
閫熷害鍛戒护蹇呴』缁熶竴缁忚繃锛?

```text
Teleop / VisualServo / PathFollower / Recovery
  -> nav.velocity_mux
  -> nav.out.cmd_vel
  -> endpoint / driver
```

## 閫熷害鍛戒护鍜岄€氫俊杈圭晫

褰撳墠 `thunder_field` 鐜板満閾捐矾涓嶆槸 Python 杩涚▼鐩存帴 gRPC 鎺х嫍锛岃€屾槸
LingTu 鍐呴儴妯″潡鍏堝畬鎴愰€熷害浠茶鍜屽畨鍏ㄧ害鏉燂紝鍐嶉€氳繃 DDS 瀵艰埅鍑哄彛浜ょ粰鏉跨
endpoint锛?
```text
Web / CLI / MCP
  -> GatewayModule / TeleopModule
  -> nav.velocity_mux
  -> nav.out.cmd_vel
  -> DDS /nav/cmd_vel
  -> board endpoint / hardware control service
```

`thunder_field` 鐨勫叧閿繍琛岄厤缃細

```text
endpoint_transport = dds
command_output_mode = endpoint_only
hardware_control_boundary = dds_endpoint_source
native_navigation_endpoint = lingtu-nav-dds
enable_nav_in = false
enable_nav_out = false
enable_robot_driver = false
```

Field DDS navigation has one writer: C++ `lingtu-nav-dds`. The old Python nav
DDS adapters were removed to prevent duplicate goal / path / cmd_vel writers in
`thunder_field`.
澶栭儴閫熷害鍏ュ彛锛?
```text
WS /ws/teleop
  {"type":"joy","lx":0.5,"ly":0.0,"az":-0.3}
  -> GatewayModule._teleop_on_joy
  -> TeleopModule.joy_input
  -> TeleopModule.cmd_vel
  -> nav.velocity_mux.teleop_cmd_vel

POST /api/v1/cmd_vel
  {"vx":0.2,"vy":0.0,"wz":0.1}
  -> GatewayModule.cmd_vel
  -> nav.velocity_mux.teleop_cmd_vel

MCPServerModule.cmd_vel
  -> nav.velocity_mux.teleop_cmd_vel
```

鍐呴儴閫熷害鍊欓€夊叏閮ㄥ垎婧愯繘鍏?`nav.velocity_mux`锛?
```text
TeleopModule / GatewayModule / MCPServerModule
  -> teleop_cmd_vel          priority 100
VisualServoModule
  -> visual_servo_cmd_vel    priority 80
nav.mission recovery
  -> recovery_cmd_vel        priority 60
nav.path_follower
  -> path_follower_cmd_vel   priority 40
```

`nav.velocity_mux` 閫夋嫨鏈€楂樹紭鍏堢骇鐨勬椿璺冮€熷害婧愬悗锛屽啀缁熶竴杩囪繎鍦洪伩闅滅洃鎺с€?鏈夊湴鍥炬爤鐨勯仴鎺?瀵艰埅/鎺㈢储鍏ュ彛浼氭帴鍏ワ細

```text
SlamAdapterModule.odometry
  -> nav.velocity_mux.collision_odometry

TraversabilityCostModule.fused_cost
  -> nav.velocity_mux.collision_costmap
```

`fused_cost` 鏄粰閫熷害瀹夊叏闂ㄧ敤鐨勪唬浠峰浘锛屼笉鏄?UI 鍥剧墖锛屼篃涓嶆槸鍘熷鐐逛簯锛?
```python
{
    "grid": ndarray,        # 0..100 cost, 99/100 means hard obstacle
    "resolution": float,
    "origin": [x, y],
    "ts": float,
    "frame_id": "map" | "odom",
    "backend": optional str,
}
```

瀹冪敱 OccupancyGrid銆丒SDF proximity銆丒levation slope 鍜?terrain
traversability 鍙栨渶澶ч闄╄瀺鍚堣€屾潵銆俙nav.velocity_mux` 瀵瑰綋鍓嶉€熷害鍋氭姇褰辨鏌ワ細

```text
clear projected path      -> pass
near high-cost cells      -> slowdown
hard obstacle / stale map -> stop
missing odometry / map    -> stop
```

瑙嗚浼烘湇鐑垏鍏ュ彛锛?
```text
POST /api/v1/visual_servo
  {"mode":"find","target":"red chair"}
  {"mode":"follow","target":"person in red"}
  {"mode":"stop"}

GatewayModule.servo_target
  -> VisualServoModule.servo_target
```

`find` 鍜?`follow` 鏄繍鍔ㄥ懡浠わ紝Safety STOP 鐢熸晥鏃朵細琚嫆缁濄€俙stop` 鍙噴鏀?VisualServo锛孲afety STOP 鐢熸晥鏃朵粛鍏佽閫氳繃銆傝鍏ュ彛鍙湪宸茬粡鍔犺浇
`VisualServoModule` 鐨?profile 鍐呯儹鍒囷紱涓嶅湪 `teleop`銆乣teleop_avoid`銆乣map`
杩欑被杞婚噺鍥鹃噷鍔ㄦ€佸垱寤鸿瑙夋ā鍧椼€?
淇濈暀鐨勭洿杩?gRPC 妯″紡鍙敤浜庤交閲?鍏煎/鏈湴鐩磋繛椹卞姩閾捐矾锛?
```text
nav.velocity_mux.driver_cmd_vel
  -> ThunderDriver.cmd_vel
  -> brainstem gRPC RobotControlStub.Walk(Vector3)
```

杩欐潯閾捐矾瀛樺湪浜?ThunderDriver锛屼絾涓嶆槸 `thunder_field` 榛樿鐜板満鍑哄彛銆?
## 浜у搧妯″紡

| Profile | 浜у搧妯″紡 | 蹇呴』涓茶仈 | 绂佹涓茶仈 | 鍒囨崲绛栫暐 |
| --- | --- | --- | --- | --- |
| `teleop` | 閬ユ帶 | Gateway/Teleop/MCP -> CmdVelMux -> Safety -> NavOut | SLAM銆佸叏灞€瑙勫垝銆佸眬閮ㄨ鍒掋€佽矾寰勮窡韪?| 鍐烽噸鍚?|
| `teleop_avoid` | 閬ユ帶閬块殰 | SLAM/鍦板浘/鍙€氳浠ｄ环 + Teleop -> CmdVelMux -> Safety -> NavOut | mission銆乴ocal planner銆乸ath follower銆佽涔夎鍒?| 鍐烽噸鍚?|
| `map` | 寤哄浘 | SLAM -> Occupancy/Voxel/Elevation/ESDF/Traversability -> Gateway/MapManager锛汿eleop -> CmdVelMux -> NavOut | mission銆乴ocal planner銆乸ath follower銆佽涔夎鍒?| 鍐烽噸鍚?|
| `tracking` | 璺熻釜 | GoalService/NavIn -> Mission -> LocalPlanner -> PathFollower -> CmdVelMux -> NavOut | 璇箟瑙勫垝 | 鍐烽噸鍚紝鏈潵鍙仛鍚屽浘鐑垏鍊欓€?|
| `nav` | 瀵艰埅 | Web/CLI/MCP/璇箟鐩爣 -> Mission -> OctoPlanner3D -> LocalPlanner -> PathFollower -> CmdVelMux -> NavOut | 鐩爣鐩存帴鍙樼數鏈哄懡浠?| 鍐烽噸鍚紝鏈潵鍙仛鍚屽浘鐑垏鍊欓€?|
| `inspection` | 宸℃ | Scheduler/Patrol/Semantic -> GoalService -> Mission -> OctoPlanner3D -> LocalPlanner -> PathFollower -> CmdVelMux -> NavOut | 宸℃浠诲姟鐩存帴鎺у埗搴曠洏 | 鍐烽噸鍚紝鏈潵鍙仛鍚屽浘鐑垏鍊欓€?|
| `tare_explore` | 鎺㈢储 | Livox/IMU 鎴?endpoint SLAM -> SlamAdapter -> Occupancy/Voxel/Elevation/ESDF/Traversability -> TARE -> Mission -> OctoPlanner3D -> LocalPlanner -> PathFollower -> CmdVelMux -> NavOut | TARE 鐩存帴鎺у埗搴曠洏銆乄avefrontFrontierExplorer 鍚屾椂鍚敤銆佹棤瀹炴椂鍦板浘杈撳叆 | 鍐烽噸鍚?|

`explore` 鍙繚鐣欎负 wavefront frontier 鍏煎/璋冭瘯鍏ュ彛锛屼笉鍐嶄綔涓洪粯璁や骇鍝佸叆鍙ｃ€傜幇鍦烘帰绱㈤粯璁や娇鐢?`tare_explore` 鎴栧埆鍚?`thunder-explore`銆?
## 鍔熻兘閾捐矾

### 閬ユ帶閾?

```text
GatewayModule.cmd_vel
MCPServerModule.cmd_vel
TeleopModule.cmd_vel
  -> nav.velocity_mux.teleop_cmd_vel
  -> nav.velocity_mux.driver_cmd_vel
  -> nav.out.cmd_vel
  -> nav.safety.cmd_vel
```

### 鍦板浘閾?

```text
SlamBridgeModule.map_cloud
  -> OccupancyGridModule.map_cloud
  -> VoxelGridModule.map_cloud
  -> ElevationMapModule.map_cloud
  -> nav.maps.map_cloud

OccupancyGridModule.costmap
ESDFModule.esdf
ElevationMapModule.elevation_map
  -> TraversabilityCostModule
  -> GatewayModule.costmap
```

### 瀵艰埅鎵ц閾?
```text
GatewayModule / MCPServerModule / SemanticPlannerModule
  -> nav.goals / nav.mission.goal_pose
  -> nav.mission.global_path
  -> nav.local_planner.global_path
  -> nav.local_planner.local_path
  -> nav.path_follower.local_path
  -> nav.path_follower.cmd_vel
  -> nav.velocity_mux.path_follower_cmd_vel
  -> nav.out.cmd_vel
```

### TARE 鎺㈢储閾?
```text
Livox MID-360 / IMU
  -> Fast-LIO2 / SLAM endpoint
  -> SlamAdapterModule.odometry + SlamAdapterModule.map_cloud
  -> OccupancyGridModule / VoxelGridModule / ElevationMapModule / ESDFModule / TraversabilityCostModule
  -> OccupancyGridModule.exploration_grid
  -> TAREExplorerModule
  -> nav.mission.goal_pose / nav.mission.patrol_goals
  -> Navigation execution chain
```

鍦?`thunder_field` endpoint 涓嬶紝闆疯揪椹卞姩鍜?Fast-LIO2 鍙互鏄閮?C++/DDS 鏈嶅姟锛?Module 鍥鹃噷鐪嬪埌鐨勬槸 `SlamAdapterModule`銆傚湪鏈満/浠跨湡/鎵樼 SLAM profile 涓嬶紝
闆疯揪鍙互浣滀负 `LidarModule` 鎴栦豢鐪熺偣浜戞簮杩涘叆鍚屼竴缁?SLAM/map 杈撳嚭鍚堝悓銆?
```text
SlamAdapterModule.map_cloud
  -> OccupancyGridModule.map_cloud
  -> VoxelGridModule.map_cloud
  -> ElevationMapModule.map_cloud
  -> nav.terrain.map_cloud

SlamAdapterModule.odometry
  -> OccupancyGridModule.odometry
  -> VoxelGridModule.odometry
  -> ElevationMapModule.odometry
  -> nav.terrain.odometry

OccupancyGridModule.exploration_grid
  -> TAREExplorerModule
  -> nav.mission.goal_pose / nav.mission.patrol_goals
  -> Navigation execution chain
```

## 褰撳墠鍒囨崲缁撹

鐜板湪鍙彁渚涘垏鎹㈤妫€锛屼笉鍋氬湪绾跨儹鍒囥€俙tracking`銆乣nav`銆乣inspection` 琚爣涓?
鍚屽浘鐑垏鍊欓€夛紝鏄负浜嗗悗缁疄鐜版椂鏈夋槑纭竟鐣岋紱褰撳墠 `switch-plan` 浠嶈繑鍥?
`required_lifecycle=cold_restart`銆?

楠屾敹鍛戒护锛?

```bash
python -m pytest src/runtime/tests/test_profile_graph_snapshots.py::test_product_modes_required_wires_are_contract_locked -q
python lingtu.py switch-plan teleop nav --json
python lingtu.py switch-plan tracking inspection --json
python lingtu.py switch-plan tare_explore nav --json
```
