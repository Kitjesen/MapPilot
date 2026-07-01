# LingTu 鍏ㄩ潰浼樺寲璁″垝 �?瑙ｈ�?�?妯″潡�?�?瑙勮寖鍖?

> **瀹¤鏃ユ�?*�?026-05-31
> **瀹¤鏂瑰�?*�? 涓苟琛?Agent锛堝鍏ヨ竟�?/ 鏂囦欢缁勭粐 / 娴嬭瘯瑕嗙洊 / 浠ｇ爜璐ㄩ噺 / 鏋舵瀯妯″紡�? 缁煎�?Synthesis Agent
> **鍓嶅簭璁″垝**锛歔2026-05-30 瑙ｈ€︽墽琛岃鍒抅(./2026-05-30-module-decoupling-execution.md)锛圱asks 1-2 宸插畬鎴愶級

---

## 涓€銆佺幇鐘舵€昏瘎

### 馃煝 鍋ュ悍閮ㄥ垎

| 缁村�?| 璇勪�?|
|------|------|
| **瀵煎叆杈圭晫** | gateway �?nav/semantic/drivers 闆惰繚瑙勶紝杈圭晫娴嬭瘯閫氳�?�?|
| **Core 妗嗘�?* | module.py / stream.py / registry.py / blueprint.py 缁撴瀯娓呮櫚锛岄浂寰幆寮曠�?�?|
| **Module-First 瑙勮�?* | 鎵€�?Module 浣跨�?In[T]/Out[T] 绫诲瀷绔彛銆丂register銆丂skill 瑁呴グ鍣紝妯″紡缁熶�?�?|
| **Shim 鍏煎灞?* | 3 �?`sys.modules` swap shim 姝ｇ‘宸ヤ綔锛屾棫瀵煎叆璺緞閫忔槑閲嶅畾�?�?|
| **璇箟灞傞殧�?* | semantic/ �?nav/drivers/gateway 闆跺鍏?�?|

### 馃敶 闇€绔嬪嵆鍏虫敞鐨勶紙鎸変弗閲嶅害鎺掑簭�?

| # | 涓ラ噸搴?| 闂�?| 浣嶇�?|
|---|--------|------|------|
| 1 | P0 | **`tests/` 鐩綍琚?pytest 鎺掗�?* �?26 �?ROS2 闆嗘垚娴嬭瘯�?`python -m pytest` 涓嶅彲瑙?| `pyproject.toml` |
| 2 | P0 | **5 / 9 妯″潡娌℃湁鐙珛 test 鐩�?* �?137 涓祴璇曞钩閾哄�?`src/runtime/tests/` | nav/gateway/drivers/memory/slam |
| 3 | P1 | **姝讳唬鐮?* �?`gateway_module.py:161-195` 鍑芥暟瀹氫箟鍚庣珛鍗宠瑕嗙洊 | `src/gateway/gateway_module.py` |
| 4 | P1 | **缂哄�?`__init__.py`** �?`src/nav/services/` 鏄殣寮忓懡鍚嶇┖闂村寘锛岃剢寮?| `src/nav/services/` |
| 5 | P1 | **瀛ゅ�?`.pyc` 鐩�?* �?`src/semantic/common/semantic_common/` 鍙�?`.pyc` �?`.py` | `src/semantic/common/` |
| 6 | P1 | **缁曡�?runtime.yaml_helpers** �?`maps.py:967` 鐩存�?`import yaml` 鏃犲洖閫�?| `src/nav/services/maps.py` |

---

## 浜屻€佸墠搴忚鍒掑墿浣欎换鍔★紙Tasks 4-7 璇勪及锛?

| 璁″垝浠诲�?| 鐘舵�?| 寤鸿�?|
|----------|------|------|
| **Task 4**: 鎻愬�?NavigationModule ROS2 鍙戝竷鍒?Bridge �?| �?鏈紑濮?| **闄嶄紭鍏堢骇**銆俙navigation_module.py` �?rclpy 瀵煎叆宸叉槸鏉′欢闂ㄦ帶锛坄enable_ros2_bridge`锛夛紝褰撳墠鏂规鍔″疄鍙銆備粎�?ROS2-free 閮ㄧ讲鎴愪负纭渶姹傛椂鎵ц�?|
| **Task 5**: 浠跨湡璇佹嵁 Freshness Gate | �?鏈紑濮?| **涓瓑浼樺厛�?*銆俙sim/scripts/server_sim_closure.py` 宸蹭慨鏀逛絾鏈畬鎴愩€備笅�?S100P 浠跨湡娲诲姩鍓嶅畬鎴愩€?|
| **Task 6**: 鐪熷�?Planner Benchmark | �?鏈紑濮?| **涓瓑浼樺厛�?*銆傚綋鍓?`benchmark_planner.sh` 鍙�?sleep 鍗犱綅銆傚０�?planner 鎬ц兘鏀硅繘鍓嶅繀椤诲畬鎴愩€?|
| **Task 7**: 瀵艰埅閾炬晥鐜囪瘉鎹?| �?鏈紑濮?| **�?涓紭鍏堢骇**銆傜粨鏋勮壇濂戒絾闈為樆濉烇紝涓嬫瀵艰埅璋冭瘯鏃堕『鎵嬪疄鐜般�?|

---

## 涓夈€佹柊鍙戠幇闂姹囨€?

### P0 �?绔嬪嵆淇

**1. 闆嗘垚娴嬭瘯涓嶅彲瑙?*
- **鏂囦�?*锛歚pyproject.toml` lines 88-98
- **闂�?*锛歚testpaths` 鍙垪浜?`src/runtime/tests` �?3 涓洰褰曪紝椤跺眰鐨?`tests/`�?6 涓泦鎴愭祴璇曪級琚帓�?
- **鏂规�?*锛堜簩閫変竴锛夛�?
  - (a) �?`tests/` �?testpaths锛孯OS2 娴嬭瘯鍔?`@pytest.mark.ros2` + skip-if-no-ros2 fixture
  - (b) 鍒涘�?`scripts/run_integration_tests.sh` 鏂囨。鍖栬皟鐢ㄦ柟寮?

**2. 姝讳唬鐮?�?gateway_module.py**
- **鏂囦�?*锛歚src/gateway/gateway_module.py` lines 161-195
- **闂�?*锛歚_apply_dynamic_filter_step1half` 鍑芥暟瀹氫箟锛?0 琛岋�? 鏁翠�?body锛岀揣鎺ョ潃 line 196 琚鐩栦负 `= _map_apply_dynamic_filter_step1half`銆傚嚱鏁颁綋姘镐笉鍙揪�?
- **淇�?*锛氬垹闄?lines 161-195锛堟瀹氫箟锛夛紝鍙繚鐣?line 196锛堟纭殑寮曠敤璧嬪€硷級

### P1 �?搴旇淇

**3. 缂哄�?`__init__.py`**
- **鏂囦�?*锛氭柊寤?`src/nav/services/__init__.py`
- **褰卞�?*锛歚nav.services` 鎴愪�?PEP 420 闅愬紡鍛藉悕绌洪棿鍖咃紝浠讳�?`services/` 鐩綍閮藉彲鑳芥剰澶栧悎�?

**4. 瀛ゅ効鍖呯洰�?*
- **鏂囦�?*锛氬垹闄?`src/semantic/common/semantic_common/`
- **鍐呭�?*锛氫粎鍚?`__pycache__/` 涓�?`robustness.pyc`銆乣sanitize.pyc`銆乣validation.pyc`锛屽搴?`.py` 婧愭枃浠朵笉瀛樺�?

**5. 缁曡�?runtime.yaml_helpers**
- **鏂囦�?*锛歚src/nav/services/maps.py:967`
- **闂�?*锛氳�?`import yaml as _yaml` 娌℃�?fallback锛岃�?`runtime.yaml_helpers` 宸叉彁渚涙棤 yaml 鏃剁�?JSON 鍥為€€
- **淇�?*锛氭浛鎹负 `from runtime.yaml_helpers import ...`

**6. 娴嬭瘯鏂囦欢浠嶇敤鏃?shim 璺�?*
- **娑夊強鏂囦欢**�? 涓級锛歚test_dynamic_filter.py`銆乣test_map_occupancy.py`銆乣test_mujoco_mid360_pattern.py`銆乣test_nav_services.py`銆乣test_saved_map_artifact_gate.py`銆乣test_services_modules.py`
- **闂�?*锛氫�?`nav.services.nav_services.X` 瀵煎叆鑰岄潪 `runtime.X`
- **淇�?*锛氭洿鏂板鍏ヤ�?`runtime.X`锛岀劧鍚庡垹�?3 �?shim 鏂囦�?

### P2 �?鏀瑰杽

**7. 8 涓仐鐣?simplification wave 娴嬭瘯鏂囦欢**
- **鏂囦�?*锛歚test_simplification_wave1.py`銆乣test_simplification_wave2_team{A,B,C,D}.py`銆乣test_simplification_wave3_team{E,F,G}.py`
- **闂�?*锛氫竴娆℃€т唬鐮佺畝鍖栭」鐩殑浜х墿锛屽鍔?test 鐩綍鍣煶
- **淇�?*锛氶€愪竴瀹¤ �?鏈夎鐩栦环鍊肩殑鍚堝苟锛岄噸澶嶇殑鍒犻�?

**8. 3 涓湭鏂囨。鍖栫殑 stack factory**
- **鏂囦�?*锛歚src/runtime/blueprints/stacks/__init__.py` 瀵煎�?12 涓紝CLAUDE.md 鍙垪浜?9 �?
- **缂哄�?*锛歚exploration`銆乣lidar`銆乣sim_lidar`
- **淇�?*锛氳ˉ�?CLAUDE.md 鏂囨�?

**9. 5 涓ā鍧楁棤鐙珛 test 鐩�?*
- **缂哄�?*锛歚src/nav/tests/`銆乣src/gateway/tests/`銆乣src/drivers/tests/`銆乣src/memory/tests/`銆乣src/localization/tests/`
- **鐜扮�?*�?37 涓祴璇曞钩閾哄�?`src/runtime/tests/`
- **淇�?*锛氬垱寤?test 鐩綍锛岃縼绉诲搴旀祴璇曪紝鏇存�?`pyproject.toml` testpaths

**10. `.gitignore` 閬楁�?*
- `build_nb_win/`銆乣*.egg-info/`銆乣*.pt` 妯″瀷鏂囦欢鏈拷鐣?
- **淇�?*锛氳ˉ�?`.gitignore`

### P3 �?閿︿笂娣昏姳

**11. 妯″瀷鏂囦欢 `yoloe-26s-seg.pt` 鍦ㄦ簮鐮佹爲�?*
**12. 3 涓祴璇曟枃浠跺湪闈炴爣鍑嗕綅缃?*
**13. Registry 鐑矾寰勫彲�?`lru_cache`**
**14. `full_stack_wiring.py` 纭紪鐮佹ā鍧楀悕瀛楃涓?*

---

## 鍥涖€佹枃浠?鏂囦欢澶硅鑼冨寲鏂规

### 4.1 鐩綍閲嶇粍

```
Phase 1锛堜綆椋庨櫓 �?绔嬪嵆鍙仛�?
鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣�?
�?鏂板�?src/nav/services/__init__.py
�?鏂板�?src/perception/tests/__init__.py
�?鍒犻�?src/semantic/common/semantic_common/ 锛堝鍎?.pyc�?
�?鍒犻�?src/gateway/gateway_module.py:161-195 锛堟浠ｇ爜�?

Phase 2锛堜腑椋庨櫓 �?瑙ｈ€︽敹灏撅級
鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣�?
�?6 涓祴璇曟枃浠跺鍏ヨ矾寰勪�?shim 鏀逛负 runtime.*
�?鍒犻�?3 �?shim 鏂囦欢锛坰ame_source_map_artifacts / dynamic_filter / yaml_helpers�?
�?淇�?maps.py:967 �?import yaml
�?鏇存�?.gitignore

Phase 3锛堥珮宸ヤ綔�?�?娴嬭瘯鐩綍閲嶇粍锛?
鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹?
�?鍒涘�?5 涓ā鍧楃嫭绔?test 鐩�?
   src/nav/tests/          �?7 �?test_nav_*.py
   src/gateway/tests/      �?12 �?test_gateway_*.py
   src/drivers/tests/      �?test_driver_spec.py
   src/memory/tests/       �?2 �?test_memory_*.py
   src/localization/tests/         �?5 �?test_slam_*.py
�?杩佺Щ 3 涓潪鏍囧噯浣嶇疆娴嬭瘯鏂囦�?
�?瀹¤骞舵竻鐞?8 �?simplification_wave 鏂囦�?
�?鏇存�?pyproject.toml testpaths

Phase 4锛堟灦鏋勪紭�?�?鍙€夛�?
鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣�?
馃數 nav/services/nav_services/ �?nav/services/ 锛堟墎骞冲寲宓屽锛?
馃數 鍚堝�?semantic_exploration.yaml / far_planner.yaml �?robot_config.yaml
馃數 full_stack_wiring.py 甯搁噺鍖栨ā鍧楀�?
馃數 Registry 鐑矾寰勫姞 lru_cache
```

### 4.2 鍛藉悕绾﹀�?

| 瑙勮�?| 褰撳�?| 鐩�?|
|------|------|------|
| 鎵€�?package �?`__init__.py` | `nav/services/` �?| 琛ュ�?|
| test 鐩綍鏈?`__init__.py` | `perception/tests/` �?| 琛ュ�?|
| 浜岃繘鍒舵枃浠朵笉鍏ユ簮鐮佹�?| `yoloe-26s-seg.pt` �?`src/` | 绉诲�?`models/` 鎴栫�?git-lfs |
| 鏋勫缓浜х墿涓嶅叆浠撳�?| `build_nb_win/`銆乣.egg-info/` 瀛樺�?| `.gitignore` 鎺掗�?|
| 娴嬭瘯浠庤鑼冭矾寰勫�?| 6 涓枃浠剁敤 shim 璺�?| 鏀逛负 `runtime.*` |
| 缁熶竴閰嶇疆鏍煎�?| `dufomap.toml` 鏄敮涓€鐨?TOML | 鏀逛负 YAML 鎴栦繚鐣欙紙鏍囨敞渚嬪鍘熷洜锛?|

### 4.3 鐩爣鐩綍缁撴�?

```
src/
鈹溾攢鈹�?core/                     # 妗嗘灦鍐呮牳 + 鍏变韩濂戠害锛堜笉鍙橈級
�?  鈹溾攢鈹�?blueprints/           # Blueprint 缁勮锛堜笉鍙橈�?
�?  �?  鈹溾攢鈹�?full_stack.py
�?  �?  鈹溾攢鈹�?full_stack_wiring.py
�?  �?  鈹斺攢鈹�?stacks/           # 12 �?factory锛堟枃妗ｈˉ榻愬埌 12�?
�?  鈹溾攢鈹�?tests/                # 鏍稿績妗嗘灦娴嬭瘯锛堜繚鐣欙紝閫愭鐦﹁韩锛?
�?  鈹溾攢鈹�?same_source_map_artifacts.py  # 鍏变韩濂戠害 �?
�?  鈹溾攢鈹�?dynamic_filter.py             # 鍏变韩濂戠害 �?
�?  鈹溾攢鈹�?yaml_helpers.py               # 鍏变韩濂戠害 �?
�?  鈹斺攢鈹�?...
鈹溾攢鈹�?nav/
�?  鈹溾攢鈹�?services/             # nav_services 鎵佸钩鍖?�?services
�?  �?  鈹溾攢鈹�?__init__.py       # 馃啎 鏄惧�?package
�?  �?  鈹溾攢鈹�?geofence.py
�?  �?  鈹溾攢鈹�?maps.py
�?  �?  鈹溾攢鈹�?patrol.py
�?  �?  鈹斺攢鈹�?scheduler.py
�?  鈹溾攢鈹�?tests/                # 馃啎 鐙�?test 鐩�?
�?  �?  鈹溾攢鈹�?__init__.py
�?  �?  鈹溾攢鈹�?test_nav_modules.py         # �?�?core/tests/
�?  �?  鈹溾攢鈹�?test_nav_services.py        # �?�?core/tests/
�?  �?  鈹斺攢鈹�?...
�?  鈹斺攢鈹�?...
鈹溾攢鈹�?gateway/
�?  鈹溾攢鈹�?tests/                # 馃啎 鐙�?test 鐩�?
�?  �?  鈹溾攢鈹�?__init__.py
�?  �?  鈹溾攢鈹�?test_gateway_runtime_status.py
�?  �?  鈹斺攢鈹�?...
�?  鈹斺攢鈹�?...
鈹溾攢鈹�?drivers/
�?  鈹溾攢鈹�?tests/                # 馃啎 鐙�?test 鐩�?
�?  鈹斺攢鈹�?...
鈹溾攢鈹�?memory/
�?  鈹溾攢鈹�?tests/                # 馃啎 鐙�?test 鐩�?
�?  鈹斺攢鈹�?...
鈹溾攢鈹�?slam/
�?  鈹溾攢鈹�?tests/                # 馃啎 鐙�?test 鐩�?
�?  鈹斺攢鈹�?...
鈹斺攢鈹�?semantic/                 # 娓呯�?orphan common/ 鐩�?
    鈹溾攢鈹�?common/               # 淇濈暀锛堝瀹為檯鏈夊唴瀹癸�?
    �?  鈹斺攢鈹�?(鍒犻櫎绌虹殑 semantic_common/)
    鈹斺攢鈹�?...
```

---

## 浜斻€佷紭鍏堢骇鎵ц娓呭崟锛堟帓搴忥�?

### 馃敶 Sprint 1锛氱珛鍗充慨澶嶏紙棰勪及 2-3 灏忔椂锛?

| # | 琛屽�?| 宸ヤ綔閲?| 鏂囦欢鏁?|
|---|------|--------|--------|
| S1-1 | 鍒犻�?`gateway_module.py` 姝讳唬鐮侊紙lines 161-195�?| S | 1 |
| S1-2 | 鏂板�?`src/nav/services/__init__.py` | S | 1 |
| S1-3 | 鍒犻�?`src/semantic/common/semantic_common/` 瀛ゅ効鐩綍 | S | 1 |
| S1-4 | 淇�?`maps.py:967` �?`import yaml` �?`runtime.yaml_helpers` | S | 1 |
| S1-5 | 琛ュ�?`.gitignore`锛坆uild/egg/pt�?| S | 1 |
| S1-6 | 鏂囨。鍖栭泦鎴愭祴璇曡繍琛屾柟寮忥紙鏂规�?a �?b�?| S | 1-2 |
| S1-7 | 杩愯鍏ㄩ儴 core tests 纭闆跺洖�?| S | - |

### 馃煛 Sprint 2锛氳В鑰︽敹灏撅紙棰勪�?4-5 灏忔椂锛?

| # | 琛屽�?| 宸ヤ綔閲?| 鏂囦欢鏁?|
|---|------|--------|--------|
| S2-1 | 6 涓祴璇曟枃浠跺鍏ヨ矾寰勶細`nav.services.nav_services.X` �?`runtime.X` | M | 6 |
| S2-2 | 鍒犻�?3 �?shim 鏂囦�?| S | 3 |
| S2-3 | 纭杈圭晫娴嬭瘯閫氳繃 + 鍏ㄩ噺娴嬭瘯閫氳�?| S | - |
| S2-4 | 鏂板�?`src/perception/tests/__init__.py` | S | 1 |
| S2-5 | CLAUDE.md 琛ュ�?3 �?factory锛坋xploration/lidar/sim_lidar�?| S | 1 |
| S2-6 | 瀹¤ 8 �?simplification_wave 鏂囦欢锛屽喅瀹氬幓鐣?| M | 8 |

### 馃煚 Sprint 3锛氭祴璇曠洰褰曢噸缁勶紙棰勪�?6-8 灏忔椂锛?

| # | 琛屽�?| 宸ヤ綔閲?| 鏂囦欢鏁?|
|---|------|--------|--------|
| S3-1 | 鍒涘�?`src/nav/tests/` + 杩佺Щ 7 涓祴璇?| M | 7+ |
| S3-2 | 鍒涘�?`src/gateway/tests/` + 杩佺Щ 12 涓祴璇?| M | 12+ |
| S3-3 | 鍒涘�?`src/drivers/tests/` + 杩佺Щ娴嬭�?| M | 3+ |
| S3-4 | 鍒涘�?`src/memory/tests/` + 杩佺Щ 2 涓祴璇?| S | 2+ |
| S3-5 | 鍒涘�?`src/localization/tests/` + 杩佺Щ 5 涓祴璇?| M | 5+ |
| S3-6 | 杩佺Щ 3 涓潪鏍囧噯浣嶇疆娴嬭瘯鏂囦�?| M | 3 |
| S3-7 | 鏇存�?`pyproject.toml` testpaths | S | 1 |
| S3-8 | 鍏ㄩ噺娴嬭瘯 + 淇瀵煎叆璺緞 | M | - |

### 馃數 Sprint 4锛氭繁搴︿紭�?+ 鍓嶅簭璁″垝鏀跺熬锛堟寜闇€锛?

| # | 琛屽�?| 宸ヤ綔閲?| 璇存�?|
|---|------|--------|------|
| S4-1 | `nav/services/nav_services/` �?`nav/services/` 鎵佸钩鍖?| L | 褰卞搷瀵煎叆璺緞锛岄渶鍏ㄥ眬鏇存�?|
| S4-2 | Planner benchmark 鐪熷疄鍖栵紙Plan Task 6�?| M | 鏇挎�?sleep �?A*/PCT 瀹炴�?|
| S4-3 | 浠跨�?Freshness Gate锛圥lan Task 5�?| M | 缁撴瀯鍖?JSON + scenario 琛ラ�?|
| S4-4 | 瀵艰埅閾炬晥鐜囪瘉鎹紙Plan Task 7�?| M | 閾剧骇閬ユ祴 JSON artifact |
| S4-5 | ROS2 鎻愬彇鍒?Bridge 灞傦紙Plan Task 4�?| L | �?ROS2-free 閮ㄧ讲闇€姹傛椂 |
| S4-6 | Registry 鐑矾寰?`lru_cache` | S | 鎬ц兘寰紭鍖?|
| S4-7 | `full_stack_wiring.py` 甯搁噺鍖栨ā鍧楀�?| M | 缁存姢鎬ф敼鍠?|
| S4-8 | 鍚堝�?config �?`robot_config.yaml` | S | 鍑忓皯閰嶇疆纰庣�?|

---

## 鍏€佷笉鍋氱殑

| 浜嬮�?| 鐞嗙�?|
|------|------|
| `nav/services/nav_services/` 閲嶅懡鍚?| 宸ヤ綔閲忓お澶э紙褰卞搷鍑犲崄涓鍏ヨ矾寰勶級锛孭hase 4 鍙€夊�?|
| ROS2 瀹屽叏浠?NavigationModule 绉婚櫎锛圱ask 4�?| 褰撳墠鏉′欢瀵煎叆鏂规宸茶冻澶熷姟瀹烇紝鏃?ROS2-free 閮ㄧ讲闇€姹?|
| 閲嶅�?agent_loop.py 宸ュ叿璋冪敤 | 宸查€氳�?test_agent_loop_tool_schema.py 娴嬭瘯锛屽綋鍓嶅疄鐜扮ǔ�?|
| 杩佺Щ�?`src/` 浠ュ鐨勬祴璇曟鏋?| 褰撳�?pytest + conftest.py 妯″紡宸ヤ綔鑹ソ锛屼笉闇€瑕佸紩鍏ユ柊宸ュ叿 |

---

## 涓冦€侀獙鏀舵爣�?

姣忎�?Sprint 瀹屾垚鍚庢墽琛岋�?

```bash
# 杈圭晫娴嬭瘯锛堝繀�?0 澶辫触锛?
python -m pytest src/runtime/tests/test_module_boundaries.py -v

# 鏍稿績妗嗘灦娴嬭瘯锛堝繀椤诲叏缁匡紝褰撳墠鍩虹嚎 ~1226 tests�?
python -m pytest src/runtime/tests/ -q

# 瀵煎叆鏃犲惊�?
python -c "from runtime.module import Module; from runtime.blueprint import Blueprint; print('OK')"

# Lint 闆舵柊澧為棶�?
ruff check src/ --statistics
```

---

## 闄勫綍锛氭枃浠剁储寮?

| 鏂囦�?| 瀹¤鍙戠�?|
|------|----------|
| `src/gateway/gateway_module.py` | lines 161-195 姝讳唬鐮?|
| `src/nav/services/` | �?`__init__.py` |
| `src/semantic/common/semantic_common/` | 瀛ゅ�?`.pyc`锛屽簲鍒犻櫎 |
| `src/nav/services/maps.py:967` | �?`import yaml` |
| `src/nav/services/same_source_map_artifacts.py` | shim锛孲print 2 鍚庡垹闄?|
| `src/nav/services/dynamic_filter.py` | shim锛孲print 2 鍚庡垹闄?|
| `src/nav/services/yaml_helpers.py` | shim锛孲print 2 鍚庡垹闄?|
| `pyproject.toml` | testpaths 闇€鎵╁�?|
| `.gitignore` | �?build_nb_win / *.egg-info / *.pt |
| `CLAUDE.md` | �?3 �?factory 鏂囨�?|
| `tests/benchmark/benchmark_planner.sh` | sleep 鍗犱綅锛岄渶鐪熷疄鍖?|
| `sim/scripts/server_sim_closure.py` | 宸蹭慨鏀逛絾鏈畬鎴?gate |
