# Plan: sim 鈫?鐢熶骇 鍏ㄥ眬瑙勫垝閾捐矾瀵归綈 (鏂瑰悜 C)

## 1. 鐩爣涓庢垚鍔熸爣鍑?

**鐩爣**锛氭秷闄?sim 涓庣敓浜?nav 鍦ㄣ€屽叏灞€瑙勫垝灞傘€嶇殑鍒嗗弶锛岃涓よ€呰蛋鍚屼竴鏉?
PCT 鍏ㄥ眬閾捐矾锛沗astar` 閫€鍖栦负銆屾棤 PCT native 鏃剁殑鏄惧紡 fallback銆嶏紝鑰屼笉鏄?
sim 榛樿瑙勫垝鍣ㄣ€?

**鎴愬姛鏍囧噯**锛?
- 浠讳綍 profile 閫?`planner="pct"` 鏃讹細鏈?native 鈫?璺?PCT锛涙棤 native锛圵indows/
  Mac/缂?.so锛夆啋 **鑷姩銆佹樉寮忛檷绾у埌 astar**锛堜笉鍐?RuntimeError 宕╂簝锛夛紝骞跺湪
  status 閲岃兘鏌ュ埌 `active_planner` 涓?`degraded_reason`銆?
- sim 瀹舵棌鍦?Linux/CI 涓婅窇鐨勫氨鏄?PCT锛堜笌鐪熸満鍚屾瀯锛夛紱鍦?Windows 寮€鍙戞満涓?
  鑷姩闄嶇骇 astar锛堣涓轰笌浠婂ぉ绛変环锛屼絾璇箟鏄惧紡锛夈€?
- `costmap` 闂ㄦ帶琛屼负涓嶅啀 sim/鐪熸満鍒嗗弶锛坰im 璧?pct 鍚?`replan_on_costmap_update`
  鑷姩涓庣湡鏈轰竴鑷达級銆?
- 濂戠害鏂囨。 + 娴嬭瘯閿佹鏂拌竟鐣屻€?

## 2. 鑼冨洿杈圭晫

**鍋?*锛?
- GlobalPlannerService 澧炲姞銆屼富瑙勫垝鍣ㄤ笉鍙敤 鈫?鏄惧紡闄嶇骇 fallback銆嶉€昏緫 + 鐘舵€佹毚闇层€?
- sim 瀹舵棌 profile 鐨?`planner` 鐢?`astar` 鍒囧埌 `pct`锛堥樁娈?2锛夈€?
- 濂戠害鏂囨。鏂板銆宻im鈫旂敓浜у榻?+ 闄嶇骇璇箟銆嶇珷鑺傘€?
- 娴嬭瘯锛氶檷绾у崟娴嬨€乸rofile 蹇収鏇存柊銆佸绾︽祴璇曘€?

**涓嶅仛**锛?
- 涓嶆敼 PCT/A* 绠楁硶鏈韩銆?
- 涓嶆敼灞€閮ㄨ鍒掗摼锛坱errain鈫抣ocal_planner鈫抪ath_follower锛屽凡瀵归綈锛夈€?
- 涓嶉噸缂栬瘧 .so锛屼笉寮曞叆鏂颁緷璧栥€?

## 3. 鐜扮姸涓庤瘉鎹?

- `cli/profiles_data.py`锛歴im/sim_gazebo/sim_industrial/sim_mujoco_live 鍧?`planner="astar"`锛?
  鐢熶骇 nav/explore `planner="pct"`銆?
- `GlobalPlannerService.setup()` (`src/nav/services/plan/global_planner/service.py:54-57`)锛?
  `_create_backend()` 鍚?*鏃?available 妫€娴?*锛孭CT .so 缂哄け鏃?backend.available=False锛?
  plan() 杩斿洖 [] 鈫?鐜扮姸鐩存帴 RuntimeError锛屾棤鑷姩闄嶇骇銆?
- `_PCTBackend.available`锛?so + tomogram 閮藉姞杞芥垚鍔熸墠 True銆?
- `NavigationModule._on_costmap`锛歚update_map` 涓庨噸瑙勫垝閮藉湪 `replan_on_costmap_update`
  瀹堝崼涓嬶紱PCT 榛樿 False锛孉* 榛樿 True 鈫?costmap 琛屼负鍒嗗弶銆?
- tomogram `building2_9.pickle` 鐗╃悊瀛樺湪锛汸CT .so 鏈?aarch64 + x86_64 + x86_64_py312銆?

## 4. 瀹炴柦姝ラ (涓ら樁娈?

### 闃舵 1 鈥?闄嶇骇鏀跺彛 (绾寮猴紝鏃犵牬鍧忥紝鍙嫭绔嬮獙璇?

1. `GlobalPlannerService.__init__`锛氭柊澧?`_active_planner_name`銆乣_degraded_reason`銆?
2. `GlobalPlannerService.setup()`锛氬垱寤轰富 backend 鍚庢娴嬪彲鐢ㄦ€?
   锛坄getattr(backend, "available", True)` 鈥斺€?闈?PCT backend 鎭掑彲鐢級銆?
   涓讳笉鍙敤涓?fallback 涓庝富涓嶅悓 鈫?鍒囧埌 fallback backend锛岃褰?degraded_reason锛?
   骞?*閲嶇畻 map_artifact_gate**锛坅ctive=astar 鏃?pct 鐨?tomogram gate 涓嶅啀 block锛夈€?
3. 鏆撮湶鐘舵€侊細`active_planner` / `degraded_reason` 杩?`last_plan_report` 涓?
   `NavigationModule` 鐨?mission/health status銆?
4. 濂戠害鏂囨。 搂4/搂9 澧炶ˉ銆孭CT 涓嶅彲鐢ㄩ檷绾?astar銆嶈涔夈€?
5. 娴嬭瘯锛歚test_planner_backends.py` 鎴栨柊寤猴紝瑕嗙洊銆孭CT 涓嶅彲鐢?鈫?闄嶇骇 astar 鍙鍒掋€?
   + 銆岄檷绾у悗 status 鏍囨敞銆嶃€?

### 闃舵 2 鈥?sim 鍒?PCT (琛屼负鍙樻洿锛岄渶 CI/鐪熸満鍥炲綊锛屽崟鐙‘璁?

6. sim/sim_gazebo/sim_industrial `planner` astar鈫抪ct锛坱omogram 宸查厤 building2_9锛夈€?
   sim_mujoco_live 鏃?tomogram锛氫繚鐣?astar 鎴栬ˉ tomogram锛堝緟瀹氾級銆?
7. 鍒犻櫎 sim 涓娿€屾樉寮?astar銆嶇殑闅愬惈鍋囪锛屼緷璧栭樁娈?1 鐨勮嚜鍔ㄩ檷绾у湪 Windows 涓婂厹搴曘€?
8. 鏇存柊 `test_profile_graph_snapshots.py` 涓?sim 鐨?planner 鏈熸湜鍊笺€?
9. CI锛圠inux锛夊洖褰掔‘璁?sim e2e 鍦?PCT 涓嬩粛閫氳繃锛涗笉閫氳繃鍒欏畾浣嶆槸鐪熼棶棰樿繕鏄?
   娴嬭瘯鍩虹嚎闇€鏇存柊銆?

## 5. 娑夊強鏂囦欢娓呭崟

- `src/nav/services/plan/global_planner/service.py`锛堥樁娈?鏍稿績锛?
- `src/nav/mission/navigation_module.py`锛坰tatus 鏆撮湶 active_planner锛?
- `docs/architecture/NAVIGATION_COMPUTE_CONTRACT.md`锛堣涔夊琛ワ級
- `cli/profiles_data.py`锛堥樁娈?锛?
- `src/runtime/tests/test_planner_backends.py` / 鏂板闄嶇骇娴嬭瘯锛堥樁娈?锛?
- `src/runtime/tests/test_profile_graph_snapshots.py`锛堥樁娈?锛?

## 6. 楠屾敹娓呭崟

- [ ] PCT 涓嶅彲鐢ㄦ椂 GlobalPlannerService 闄嶇骇 astar 鑰岄潪宕╂簝
- [ ] status 鏆撮湶 active_planner + degraded_reason
- [ ] 闄嶇骇鍚?map_artifact_gate 涓嶈 block
- [ ] 濂戠害鏂囨。鏇存柊闄嶇骇璇箟
- [ ] 闃舵1 娴嬭瘯閫氳繃 + 鐜版湁 plan_safety/profile 蹇収涓嶅洖褰?
- [ ] (闃舵2) sim planner=pct锛屽揩鐓ф祴璇曟洿鏂帮紝CI 閫氳繃

## 7. 椋庨櫓涓庡洖婊?

- **椋庨櫓**锛氶樁娈? 鍦?Linux CI 涓婃妸 sim 浠?astar 鍒?pct锛屽彲鑳借Е鍙?PCT 鐩稿叧
  鏂拌涓?娴嬭瘯鍩虹嚎鍙樺寲銆?
  **缂撹В**锛氶樁娈?鍏堢嫭绔嬪悎鍏ラ獙璇侊紱闃舵2鍗曠嫭鎻愪氦锛孋I 绾㈠垯鍗曠嫭鍥炴粴 profile 鏀瑰姩銆?
- **椋庨櫓**锛氶檷绾ч€昏緫鏀瑰彉浜嗐€孭CT 缂哄け銆嶇殑澶辫触妯″紡锛堝穿婧冣啋闄嶇骇杩愯锛夈€?
  **缂撹В**锛氱敤 status 鏄惧紡鏍囨敞 degraded锛岄伩鍏嶃€岄潤榛樹互涓哄湪璺?PCT銆嶃€?
- **鍥炴粴**锛氶樁娈?/闃舵2 鍒嗘彁浜わ紝鍚勮嚜鍙嫭绔?revert锛屼笉浜掔浉渚濊禆銆?

## 8. 渚濊禆涓庣幆澧冨彉鏇?

- 鏃犳柊澧炰緷璧栥€?
- 鏈満 `uv run` 鍥?brainstem-api 瑕佹眰 py鈮?.13 瑙ｆ瀽澶辫触锛屽洖褰掔敤 `python -m pytest`銆?
