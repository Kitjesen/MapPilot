#!/usr/bin/env python3
"""Decision module."""

import unittest

from decision.tasks.decomposition import SubGoalAction, TaskDecomposer


class TestIntentParsing(unittest.TestCase):
    """Test all intent categories."""

    def setUp(self):
        self.decomposer = TaskDecomposer()

    def _get_action(self, instruction: str):
        """Helper: decompose and return first action, or None."""
        plan = self.decomposer.decompose_with_rules(instruction)
        if plan and plan.subgoals:
            return plan.subgoals[0].action
        return None

    def _get_plan(self, instruction: str):
        """Helper: return full plan."""
        return self.decomposer.decompose_with_rules(instruction)

    def test_stop_chinese_basic(self):
        """Test stop chinese basic."""
        for cmd in ["停", "停下", "停止", "停下来", "停一下"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.STOP, f"Failed: '{cmd}'")

    def test_stop_chinese_urgent(self):
        """Test stop chinese urgent."""
        for cmd in ["紧急停止", "急停"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.STOP, f"Failed: '{cmd}'")

    def test_stop_chinese_cancel(self):
        """Test stop chinese cancel."""
        for cmd in ["取消", "取消任务", "终止", "终止任务", "中断", "中止"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.STOP, f"Failed: '{cmd}'")

    def test_stop_chinese_dont_move(self):
        """Test stop chinese dont move."""
        for cmd in ["别动", "不要动", "站住", "别走了"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.STOP, f"Failed: '{cmd}'")

    def test_stop_chinese_informal(self):
        """Test stop chinese informal."""
        for cmd in ["算了", "不用了", "不找了", "不去了", "不用去了"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.STOP, f"Failed: '{cmd}'")

    def test_stop_english(self):
        """Test stop english."""
        for cmd in ["stop", "halt", "cancel", "abort", "quit", "enough"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.STOP, f"Failed: '{cmd}'")

    def test_stop_english_with_punctuation(self):
        """Test stop english with punctuation."""
        for cmd in ["stop!", "stop.", "halt!"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.STOP, f"Failed: '{cmd}'")

    def test_stop_english_casual(self):
        """Test stop english casual."""
        for cmd in ["nevermind", "never mind", "forget it"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.STOP, f"Failed: '{cmd}'")

    #  STATUS/QUERY intent (step 1)

    def test_status_chinese_battery(self):
        """Test status chinese battery."""
        for cmd in ["电量", "电池", "电量多少", "还有多少电", "剩余电量", "电池电量"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.STATUS, f"Failed: '{cmd}'")

    def test_status_chinese_state(self):
        """Test status chinese state."""
        for cmd in ["当前状态", "系统状态", "机器人状态", "状态"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.STATUS, f"Failed: '{cmd}'")

    def test_status_chinese_location(self):
        """Test status chinese location."""
        for cmd in ["你在哪", "现在在哪", "现在位置", "当前位置"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.STATUS, f"Failed: '{cmd}'")

    def test_status_chinese_task(self):
        """Test status chinese task."""
        for cmd in ["任务进度", "当前任务", "任务状态", "完成了吗", "做完了吗"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.STATUS, f"Failed: '{cmd}'")

    def test_status_chinese_misc(self):
        """Test status chinese misc."""
        for cmd in ["温度", "温度多少", "连接状态", "网络状态", "当前模式", "报告状态", "汇报状态", "现在在做什么"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.STATUS, f"Failed: '{cmd}'")

    def test_status_chinese_speed(self):
        """Test status chinese speed."""
        self.assertEqual(self._get_action("当前速度"), SubGoalAction.STATUS)

    def test_status_english(self):
        """Test status english."""
        for cmd in [
            "current status",
            "battery level",
            "where are you",
            "system status",
            "robot status",
            "task progress",
            "battery status",
            "current position",
            "status report",
        ]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.STATUS, f"Failed: '{cmd}'")

    def test_status_english_short(self):
        """Test status english short."""
        for cmd in ["status", "battery", "temperature"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.STATUS, f"Failed: '{cmd}'")

    #  PAUSE intent (step 1.5)

    def test_pause_chinese(self):
        """Test pause chinese."""
        for cmd in ["暂停", "等一下", "等等", "先等一下", "暂停任务", "暂时停下", "等一等", "先停一下", "先暂停"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.PAUSE, f"Failed: '{cmd}'")

    def test_pause_target(self):
        """Test pause target."""
        plan = self._get_plan("等一下")
        self.assertIsNotNone(plan)
        self.assertEqual(plan.subgoals[0].target, "current_task")

    #  RESUME intent (step 1.5)

    def test_resume_chinese(self):
        """Test resume chinese."""
        for cmd in ["继续", "恢复", "继续走", "继续任务", "恢复任务", "继续执行", "继续导航", "继续巡逻"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.RESUME, f"Failed: '{cmd}'")

    def test_resume_target(self):
        """Test resume target."""
        plan = self._get_plan("继续")
        self.assertIsNotNone(plan)
        self.assertEqual(plan.subgoals[0].target, "current_task")

    #  RETURN_HOME intent (step 1.6)

    def test_return_chinese_basic(self):
        """Test return chinese basic."""
        for cmd in ["回去", "回家", "返航", "返回", "回来"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.RETURN_HOME, f"Failed: '{cmd}'")

    def test_return_chinese_base(self):
        """Test return chinese base."""
        for cmd in ["回基地", "返回基地", "返回充电桩", "去充电", "回去充电"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.RETURN_HOME, f"Failed: '{cmd}'")

    def test_return_chinese_origin(self):
        """Test return chinese origin."""
        for cmd in ["回到起点", "返回起点", "返回原点", "回出发点"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.RETURN_HOME, f"Failed: '{cmd}'")

    def test_return_chinese_done(self):
        """Test return chinese done."""
        self.assertEqual(self._get_action("收工"), SubGoalAction.RETURN_HOME)

    def test_return_english(self):
        """Test return english."""
        for cmd in [
            "go home",
            "return home",
            "go back",
            "return to base",
            "return to start",
            "head back",
            "come back",
            "rtb",
        ]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.RETURN_HOME, f"Failed: '{cmd}'")

    def test_return_target_is_home(self):
        """Test return target is home."""
        plan = self._get_plan("回去")
        self.assertIsNotNone(plan)
        self.assertEqual(plan.subgoals[0].target, "home")

    #  PATROL intent (step 1.7)

    def test_patrol_chinese_basic(self):
        """Test patrol chinese basic."""
        for cmd in ["巡逻", "巡检", "巡视", "巡查"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.PATROL, f"Failed: '{cmd}'")

    def test_patrol_chinese_start(self):
        """Test patrol chinese start."""
        for cmd in ["开始巡逻", "开始巡检", "启动巡逻", "执行巡检", "执行巡逻", "开始巡视", "定点巡逻", "例行巡检"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.PATROL, f"Failed: '{cmd}'")

    def test_patrol_chinese_loop(self):
        """Test patrol chinese loop."""
        for cmd in ["走一圈", "跑一圈", "绕一圈", "巡逻一圈"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.PATROL, f"Failed: '{cmd}'")

    def test_patrol_english(self):
        """Test patrol english."""
        for cmd in ["patrol", "start patrol", "begin patrol", "do a patrol", "run patrol"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.PATROL, f"Failed: '{cmd}'")

    def test_patrol_route_extraction(self):
        """Test patrol route extraction."""
        plan = self._get_plan("巡逻路线A区")
        self.assertIsNotNone(plan)
        self.assertEqual(plan.subgoals[0].action, SubGoalAction.PATROL)
        route = plan.subgoals[0].parameters.get("route", "")
        self.assertIn("A", route, f"Route name not extracted: {route}")

    def test_patrol_default_route(self):
        """Test patrol default route."""
        plan = self._get_plan("巡逻")
        self.assertIsNotNone(plan)
        self.assertEqual(plan.subgoals[0].parameters.get("route"), "default")

    #  SAVE_MAP intent (step 1.8)

    def test_save_map_chinese(self):
        """Test save map chinese."""
        for cmd in [
            "保存地图",
            "存地图",
            "存图",
            "保存当前地图",
            "建图完成",
            "结束建图",
            "存一下地图",
            "地图保存",
            "存个地图",
            "把地图存下来",
            "保存一下地图",
        ]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.SAVE_MAP, f"Failed: '{cmd}'")

    def test_save_map_english(self):
        """Test save map english."""
        for cmd in ["save map", "save the map", "save current map", "store map", "finish mapping"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.SAVE_MAP, f"Failed: '{cmd}'")

    def test_save_map_target(self):
        """Test save map target."""
        plan = self._get_plan("保存地图")
        self.assertIsNotNone(plan)
        self.assertEqual(plan.subgoals[0].target, "current_map")

    #  SAVE_POI intent (step 1.9)

    def test_save_poi_chinese_mark(self):
        """Test save poi chinese mark."""
        for cmd in ["标记为充电桩", "标记成仓库", "标记这里"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.SAVE_POI, f"Failed: '{cmd}'")

    def test_save_poi_chinese_remember(self):
        """Test save poi chinese remember."""
        for cmd in ["记住这里", "记住这个位置", "记住当前位置", "保存位置", "保存这个点", "保存当前位置"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.SAVE_POI, f"Failed: '{cmd}'")

    def test_save_poi_chinese_this_is(self):
        """Test save poi chinese this is."""
        for cmd in ["这里是仓库", "这里叫大门", "把这里叫做入口"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.SAVE_POI, f"Failed: '{cmd}'")

    def test_save_poi_chinese_set_as(self):
        """Test save poi chinese set as."""
        for cmd in ["设为休息区", "设置为仓库", "命名为大门", "保存为休息区", "记为充电桩", "定义为入口"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.SAVE_POI, f"Failed: '{cmd}'")

    def test_save_poi_name_extraction_mark(self):
        """Test save poi name extraction mark."""
        plan = self._get_plan("标记为充电桩")
        self.assertIsNotNone(plan)
        self.assertEqual(plan.subgoals[0].action, SubGoalAction.SAVE_POI)
        self.assertEqual(plan.subgoals[0].parameters.get("name"), "充电桩")

    def test_save_poi_name_extraction_this_is(self):
        """Test save poi name extraction this is."""
        test_cases = [
            ("这里叫仓库", "仓库"),
            ("这里是大门", "大门"),
            ("保存为休息区", "休息区"),
        ]
        for cmd, expected_name in test_cases:
            plan = self._get_plan(cmd)
            self.assertIsNotNone(plan, f"No plan for '{cmd}'")
            self.assertEqual(plan.subgoals[0].action, SubGoalAction.SAVE_POI)
            actual_name = plan.subgoals[0].parameters.get("name", "")
            self.assertEqual(
                actual_name,
                expected_name,
                f"Name extraction failed for '{cmd}': expected '{expected_name}', got '{actual_name}'",
            )

    def test_save_poi_english(self):
        """Test save poi english."""
        for cmd in [
            "mark here as charging",
            "mark this as entrance",
            "save this location as entrance",
            "remember here as lobby",
            "save poi",
            "mark here",
            "save this location",
        ]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.SAVE_POI, f"Failed: '{cmd}'")

    def test_save_poi_english_name_extraction(self):
        """Test save poi english name extraction."""
        plan = self._get_plan("mark here as charging")
        self.assertIsNotNone(plan)
        self.assertEqual(plan.subgoals[0].parameters.get("name"), "charging")

        plan2 = self._get_plan("save this location as entrance")
        self.assertIsNotNone(plan2)
        self.assertEqual(plan2.subgoals[0].parameters.get("name"), "entrance")

    def test_save_poi_target_equals_name(self):
        """Test save poi target equals name."""
        plan = self._get_plan("标记为充电桩")
        self.assertIsNotNone(plan)
        sg = plan.subgoals[0]
        self.assertEqual(sg.target, sg.parameters.get("name"))

    #  SET_SPEED intent (step 1.10)

    def test_speed_chinese_faster(self):
        """Test speed chinese faster."""
        for cmd in ["快点", "加速", "调快", "快一点"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.SET_SPEED, f"Failed: '{cmd}'")

    def test_speed_chinese_slower(self):
        """Test speed chinese slower."""
        for cmd in ["慢点", "减速", "调慢", "慢一点"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.SET_SPEED, f"Failed: '{cmd}'")

    def test_speed_chinese_absolute(self):
        """Test speed chinese absolute."""
        for cmd in ["速度调到0.5", "速度设为1.0", "速度设置为2.0"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.SET_SPEED, f"Failed: '{cmd}'")

    def test_speed_chinese_extremes(self):
        """Test speed chinese extremes."""
        for cmd in ["全速", "最大速度", "最快", "低速", "最低速度", "最慢"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.SET_SPEED, f"Failed: '{cmd}'")

    def test_speed_chinese_normal(self):
        """Test speed chinese normal."""
        for cmd in ["正常速度", "默认速度"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.SET_SPEED, f"Failed: '{cmd}'")

    def test_speed_semantic_fast_value(self):
        """Test speed semantic fast value."""
        plan = self._get_plan("快点")
        self.assertEqual(plan.subgoals[0].action, SubGoalAction.SET_SPEED)
        val = plan.subgoals[0].parameters.get("value")
        self.assertIsNotNone(val)
        self.assertGreater(val, 1.0)

    def test_speed_semantic_slow_value(self):
        """Test speed semantic slow value."""
        plan = self._get_plan("慢点")
        self.assertEqual(plan.subgoals[0].action, SubGoalAction.SET_SPEED)
        val = plan.subgoals[0].parameters.get("value")
        self.assertIsNotNone(val)
        self.assertLess(val, 1.0)

    def test_speed_numeric_extraction(self):
        """Test speed numeric extraction."""
        plan = self._get_plan("速度调到0.5")
        self.assertIsNotNone(plan)
        self.assertEqual(plan.subgoals[0].action, SubGoalAction.SET_SPEED)
        val = plan.subgoals[0].parameters.get("value")
        self.assertAlmostEqual(val, 0.5, places=1)

    def test_speed_semantic_max_value(self):
        """Test speed semantic max value."""
        plan = self._get_plan("全速")
        self.assertIsNotNone(plan)
        self.assertAlmostEqual(plan.subgoals[0].parameters.get("value"), 2.0, places=1)

    def test_speed_semantic_normal_value(self):
        """Test speed semantic normal value."""
        plan = self._get_plan("正常速度")
        self.assertIsNotNone(plan)
        self.assertAlmostEqual(plan.subgoals[0].parameters.get("value"), 1.0, places=1)

    def test_speed_english(self):
        """Test speed english."""
        for cmd in [
            "speed up",
            "slow down",
            "go faster",
            "go slower",
            "max speed",
            "full speed",
            "normal speed",
            "default speed",
        ]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.SET_SPEED, f"Failed: '{cmd}'")

    def test_speed_english_set(self):
        """Test speed english set."""
        for cmd in ["set speed to", "set speed"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.SET_SPEED, f"Failed: '{cmd}'")

    #  EXPLORE intent (step 2)

    def test_explore_chinese(self):
        """Test explore chinese."""
        for cmd in [
            "探索",
            "探索一下",
            "四处看看",
            "到处看看",
            "逛逛",
            "逛一逛",
            "随便走走",
            "随便逛逛",
            "自由探索",
            "环顾四周",
        ]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.EXPLORE, f"Failed: '{cmd}'")

    def test_explore_strategy_frontier(self):
        """Test explore strategy frontier."""
        plan = self._get_plan("探索")
        self.assertIsNotNone(plan)
        self.assertEqual(plan.subgoals[0].parameters.get("strategy"), "frontier")

    def test_explore_english(self):
        """Test explore english."""
        for cmd in ["explore", "look around"]:
            self.assertEqual(self._get_action(cmd), SubGoalAction.EXPLORE, f"Failed: '{cmd}'")

    #  NAVIGATE intent (step 7)

    def test_navigate_chinese(self):
        """Test navigate chinese."""
        for cmd in ["去大门", "导航到仓库", "前往会议室", "走到出口", "到椅子旁边", "移动到门口"]:
            plan = self._get_plan(cmd)
            self.assertIsNotNone(plan, f"No plan for '{cmd}'")
            self.assertEqual(plan.subgoals[0].action, SubGoalAction.NAVIGATE, f"Failed: '{cmd}'")

    def test_navigate_chinese_polite(self):
        """Test navigate chinese polite."""
        for cmd in ["请去大门", "帮我去仓库", "带我去出口", "带路到办公室"]:
            plan = self._get_plan(cmd)
            self.assertIsNotNone(plan, f"No plan for '{cmd}'")
            self.assertEqual(plan.subgoals[0].action, SubGoalAction.NAVIGATE, f"Failed: '{cmd}'")

    def test_navigate_english(self):
        """Test navigate english."""
        for cmd in [
            "go to the door",
            "navigate to warehouse",
            "move to the exit",
            "head to the kitchen",
            "proceed to the office",
        ]:
            plan = self._get_plan(cmd)
            self.assertIsNotNone(plan, f"No plan for '{cmd}'")
            self.assertEqual(plan.subgoals[0].action, SubGoalAction.NAVIGATE, f"Failed: '{cmd}'")

    def test_navigate_produces_approach_verify(self):
        """Test navigate produces approach verify."""
        plan = self._get_plan("去大门")
        self.assertIsNotNone(plan)
        actions = [sg.action for sg in plan.subgoals]
        self.assertEqual(
            actions,
            [
                SubGoalAction.NAVIGATE,
                SubGoalAction.APPROACH,
                SubGoalAction.VERIFY,
            ],
        )

    def test_navigate_target_extraction(self):
        """Test navigate target extraction."""
        plan = self._get_plan("去大门")
        self.assertIsNotNone(plan)
        target = plan.subgoals[0].target
        self.assertTrue(len(target) > 0, "Target should not be empty")

    #  FIND intent (step 7, via simple_find or conversational regex)

    def test_find_chinese(self):
        """Test find chinese."""
        for cmd in ["找红色灭火器", "寻找椅子", "搜索杯子", "定位出口"]:
            plan = self._get_plan(cmd)
            self.assertIsNotNone(plan, f"No plan for '{cmd}'")
            self.assertEqual(plan.subgoals[0].action, SubGoalAction.FIND, f"Failed: '{cmd}'")

    def test_find_english(self):
        """Test find english."""
        for cmd in ["find the red chair", "search for the cup", "locate the exit"]:
            plan = self._get_plan(cmd)
            self.assertIsNotNone(plan, f"No plan for '{cmd}'")
            self.assertEqual(plan.subgoals[0].action, SubGoalAction.FIND, f"Failed: '{cmd}'")

    def test_find_produces_full_sequence(self):
        """Test find produces full sequence."""
        plan = self._get_plan("找红色灭火器")
        self.assertIsNotNone(plan)
        actions = [sg.action for sg in plan.subgoals]
        self.assertEqual(
            actions,
            [
                SubGoalAction.FIND,
                SubGoalAction.LOOK_AROUND,
                SubGoalAction.NAVIGATE,
                SubGoalAction.APPROACH,
                SubGoalAction.VERIFY,
            ],
        )

    def test_find_conversational_zh(self):
        """Test find conversational zh."""
        plan = self._get_plan("灭火器在哪")
        self.assertIsNotNone(plan)
        self.assertEqual(plan.subgoals[0].action, SubGoalAction.FIND)

    def test_find_conversational_en(self):
        """Test find conversational en."""
        plan = self._get_plan("where is the door")
        self.assertIsNotNone(plan)
        self.assertEqual(plan.subgoals[0].action, SubGoalAction.FIND)

    def test_find_desire_zh(self):
        """Test find desire zh."""
        plan = self._get_plan("我想去厨房")
        self.assertIsNotNone(plan)
        actions = [sg.action for sg in plan.subgoals]
        self.assertIn(SubGoalAction.FIND, actions)

    #  FOLLOW intent

    def test_follow_chinese(self):
        """Test follow chinese."""
        for cmd in ["跟着那个人", "跟随他", "追踪目标"]:
            plan = self._get_plan(cmd)
            self.assertIsNotNone(plan, f"No plan for '{cmd}'")
            actions = [sg.action for sg in plan.subgoals]
            self.assertIn(SubGoalAction.FOLLOW, actions, f"No FOLLOW in '{cmd}'")

    def test_follow_english(self):
        """Test follow english."""
        for cmd in ["follow the person", "track him"]:
            plan = self._get_plan(cmd)
            self.assertIsNotNone(plan, f"No plan for '{cmd}'")
            actions = [sg.action for sg in plan.subgoals]
            self.assertIn(SubGoalAction.FOLLOW, actions, f"No FOLLOW in '{cmd}'")

    def test_follow_produces_find_then_follow(self):
        """Test follow produces find then follow."""
        plan = self._get_plan("跟着那个人")
        self.assertIsNotNone(plan)
        actions = [sg.action for sg in plan.subgoals]
        self.assertEqual(actions[0], SubGoalAction.FIND)
        self.assertEqual(actions[1], SubGoalAction.FOLLOW)

    #  PICK intent

    def test_pick_chinese(self):
        """Test pick chinese."""
        for cmd in ["拿杯子", "取一下工具", "帮我拿水杯"]:
            plan = self._get_plan(cmd)
            self.assertIsNotNone(plan, f"No plan for '{cmd}'")
            actions = [sg.action for sg in plan.subgoals]
            self.assertIn(SubGoalAction.PICK, actions, f"No PICK in '{cmd}'")

    def test_pick_english(self):
        """Test pick english."""
        for cmd in ["pick up the cup", "grab the tool", "fetch me a pen"]:
            plan = self._get_plan(cmd)
            self.assertIsNotNone(plan, f"No plan for '{cmd}'")
            actions = [sg.action for sg in plan.subgoals]
            self.assertIn(SubGoalAction.PICK, actions, f"No PICK in '{cmd}'")

    #  PLACE intent

    def test_place_chinese(self):
        """Test place chinese."""
        for cmd in ["放到桌子上", "放在地上"]:
            plan = self._get_plan(cmd)
            self.assertIsNotNone(plan, f"No plan for '{cmd}'")
            actions = [sg.action for sg in plan.subgoals]
            self.assertIn(SubGoalAction.PLACE, actions, f"No PLACE in '{cmd}'")

    def test_place_english(self):
        """Test place english."""
        for cmd in ["put it on the desk", "place it on the table"]:
            plan = self._get_plan(cmd)
            self.assertIsNotNone(plan, f"No plan for '{cmd}'")
            actions = [sg.action for sg in plan.subgoals]
            self.assertIn(SubGoalAction.PLACE, actions, f"No PLACE in '{cmd}'")

    #  INSPECT intent (FIND + LOOK_AROUND + APPROACH + VERIFY)

    def test_inspect_chinese(self):
        """Test inspect chinese."""
        for cmd in ["检查灭火器", "查看门锁"]:
            plan = self._get_plan(cmd)
            self.assertIsNotNone(plan, f"No plan for '{cmd}'")
            actions = [sg.action for sg in plan.subgoals]
            self.assertEqual(len(actions), 4, f"Expected 4 subgoals for '{cmd}'")
            self.assertIn(SubGoalAction.FIND, actions)
            self.assertIn(SubGoalAction.VERIFY, actions)

    def test_inspect_english(self):
        """Test inspect english."""
        for cmd in ["inspect the fire extinguisher", "check the door"]:
            plan = self._get_plan(cmd)
            self.assertIsNotNone(plan, f"No plan for '{cmd}'")
            actions = [sg.action for sg in plan.subgoals]
            self.assertIn(SubGoalAction.FIND, actions)
            self.assertIn(SubGoalAction.VERIFY, actions)

    def test_priority_stop_is_highest(self):
        """Test priority stop is highest."""
        self.assertEqual(self._get_action("停"), SubGoalAction.STOP)
        self.assertEqual(self._get_action("停止"), SubGoalAction.STOP)
        self.assertEqual(self._get_action("取消"), SubGoalAction.STOP)

    def test_priority_return_before_navigate(self):
        """Test priority return before navigate."""
        self.assertEqual(self._get_action("回去"), SubGoalAction.RETURN_HOME)
        self.assertEqual(self._get_action("回家"), SubGoalAction.RETURN_HOME)

    def test_priority_patrol_before_explore(self):
        """Test priority patrol before explore."""
        self.assertEqual(self._get_action("巡逻"), SubGoalAction.PATROL)
        self.assertEqual(self._get_action("巡视"), SubGoalAction.PATROL)

    def test_priority_save_map_before_navigate(self):
        """Test priority save map before navigate."""
        self.assertEqual(self._get_action("保存地图"), SubGoalAction.SAVE_MAP)

    def test_priority_save_poi_before_navigate(self):
        """Test priority save poi before navigate."""
        self.assertEqual(self._get_action("标记为充电桩"), SubGoalAction.SAVE_POI)

    def test_priority_return_before_patrol(self):
        """Test priority return before patrol."""
        self.assertEqual(self._get_action("回家"), SubGoalAction.RETURN_HOME)
        self.assertEqual(self._get_action("巡逻"), SubGoalAction.PATROL)

    def test_compound_with_then(self):
        """Test compound with then."""
        self.assertIsNone(self._get_plan("去仓库然后再去大门"))

    def test_compound_with_first(self):
        """Test compound with first."""
        self.assertIsNone(self._get_plan("先保存地图再去大门"))

    def test_compound_with_after(self):
        """Test compound with after."""
        self.assertIsNone(self._get_plan("巡逻完成后回基地"))

    def test_compound_with_multiple_commas(self):
        """Test compound with multiple commas."""
        self.assertIsNone(self._get_plan("先去大门，再去仓库，最后回家"))

    def test_compound_with_condition(self):
        """Test compound with condition."""
        self.assertIsNone(self._get_plan("如果电量低就回家"))

    def test_compound_english(self):
        """Test compound english."""
        self.assertIsNone(self._get_plan("go to door and then come back"))

    def test_compound_english_if(self):
        """Test compound english if."""
        self.assertIsNone(self._get_plan("go to the door if it is open"))

    #  Edge cases

    def test_empty_instruction(self):
        """Test empty instruction."""
        plan = self._get_plan("")
        self.assertIsNone(plan)

    def test_whitespace_only(self):
        """Test whitespace only."""
        plan = self._get_plan("   ")
        self.assertIsNone(plan)

    def test_gibberish(self):
        """Test gibberish."""
        plan = self._get_plan("asdfghjkl")
        self.assertIsNone(plan)

    def test_single_character_stop(self):
        """Test single character stop."""
        plan = self._get_plan("停")
        self.assertIsNotNone(plan)
        self.assertEqual(plan.subgoals[0].action, SubGoalAction.STOP)

    def test_unrecognized_returns_none(self):
        """Test unrecognized returns none."""
        self.assertIsNone(self._get_plan("打开空调"))
        self.assertIsNone(self._get_plan("播放音乐"))

    def test_extract_speed_value_numeric(self):
        """Test extract speed value numeric."""
        self.assertAlmostEqual(self.decomposer._extract_speed_value("速度调到0.5"), 0.5, places=1)
        self.assertAlmostEqual(self.decomposer._extract_speed_value("速度1.0"), 1.0, places=1)

    def test_extract_speed_value_percentage(self):
        """Test extract speed value percentage."""
        self.assertAlmostEqual(self.decomposer._extract_speed_value("速度50"), 0.5, places=1)

    def test_extract_speed_value_semantic(self):
        """Test extract speed value semantic."""
        self.assertAlmostEqual(self.decomposer._extract_speed_value("全速前进"), 2.0, places=1)
        self.assertAlmostEqual(self.decomposer._extract_speed_value("快一点"), 1.5, places=1)
        self.assertAlmostEqual(self.decomposer._extract_speed_value("慢一些"), 0.3, places=1)
        self.assertAlmostEqual(self.decomposer._extract_speed_value("正常速度"), 1.0, places=1)

    def test_extract_speed_value_none(self):
        """Test extract speed value none."""
        self.assertIsNone(self.decomposer._extract_speed_value("你好"))

    def test_extract_poi_name_patterns(self):
        """Test extract poi name patterns."""
        test_cases = [
            ("标记为充电桩", "充电桩"),
            ("标记成仓库", "仓库"),
            ("这里是大门", "大门"),
            ("这里叫仓库", "仓库"),
            ("保存为休息区", "休息区"),
            ("设为入口", "入口"),
            ("命名为出口", "出口"),
            ("定义为停车场", "停车场"),
        ]
        for inst, expected in test_cases:
            name = self.decomposer._extract_poi_name(inst)
            self.assertEqual(name, expected, f"POI name for '{inst}': expected '{expected}', got '{name}'")

    def test_extract_poi_name_english(self):
        """Test extract poi name english."""
        self.assertEqual(self.decomposer._extract_poi_name("mark here as charging"), "charging")
        self.assertEqual(self.decomposer._extract_poi_name("save this location as entrance"), "entrance")
        self.assertEqual(self.decomposer._extract_poi_name("remember here as lobby"), "lobby")

    def test_extract_route_name_with_route(self):
        """Test extract route name with route."""
        self.assertEqual(self.decomposer._extract_route_name("巡逻路线A区"), "A区")

    def test_extract_route_name_default(self):
        """Test extract route name default."""
        self.assertEqual(self.decomposer._extract_route_name("巡逻"), "default")

    def test_intent_classification_prompt(self):
        """Test intent classification prompt."""
        from decision.llm.prompts import (
            build_intent_classification_prompt,
        )

        prompt = build_intent_classification_prompt(
            instruction="巡逻完了去充电桩",
            robot_state="IDLE",
            available_routes="路线A, 路线B",
            available_pois="充电桩, 大门",
        )
        self.assertIsInstance(prompt, str)
        self.assertIn("巡逻完了去充电桩", prompt)
        self.assertIn("NAVIGATE", prompt)
        self.assertIn("PATROL", prompt)

    def test_compound_decomposition_prompt(self):
        """Test compound decomposition prompt."""
        from decision.llm.prompts import (
            build_compound_decomposition_prompt,
        )

        prompt = build_compound_decomposition_prompt(
            instruction="先保存地图再去大门",
            robot_state="NAVIGATING",
        )
        self.assertIsInstance(prompt, str)
        self.assertIn("先保存地图再去大门", prompt)
        self.assertIn("SAVE_MAP", prompt)


class TestIntentCoverage(unittest.TestCase):
    """Verify that all SubGoalAction types are testable."""

    def test_all_new_actions_exist_in_enum(self):
        """Every new action type should exist in SubGoalAction enum."""
        new_actions = [
            SubGoalAction.PATROL,
            SubGoalAction.SAVE_MAP,
            SubGoalAction.SAVE_POI,
            SubGoalAction.SET_SPEED,
            SubGoalAction.SET_GEOFENCE,
            SubGoalAction.RETURN_HOME,
            SubGoalAction.PAUSE,
            SubGoalAction.RESUME,
        ]
        all_values = [a.value for a in SubGoalAction]
        for action in new_actions:
            self.assertIn(action.value, all_values, f"{action.value} missing from SubGoalAction enum")

    def test_full_enum_values(self):
        """Test full enum values."""
        expected = {
            "navigate",
            "find",
            "approach",
            "verify",
            "look_around",
            "explore",
            "backtrack",
            "wait",
            "follow",
            "stop",
            "pick",
            "place",
            "status",
            "patrol",
            "save_map",
            "save_poi",
            "set_speed",
            "set_geofence",
            "return_home",
            "pause",
            "resume",
        }
        actual = {a.value for a in SubGoalAction}
        self.assertEqual(expected, actual, f"Missing: {expected - actual}, Extra: {actual - expected}")

    def test_new_actions_reachable_via_rules(self):
        """Test new actions reachable via rules."""
        decomposer = TaskDecomposer()
        action_examples = {
            SubGoalAction.PATROL: "巡逻",
            SubGoalAction.SAVE_MAP: "保存地图",
            SubGoalAction.SAVE_POI: "标记为充电桩",
            SubGoalAction.SET_SPEED: "快点",
            SubGoalAction.RETURN_HOME: "回去",
            SubGoalAction.PAUSE: "暂停",
            SubGoalAction.RESUME: "继续",
        }
        for action, instruction in action_examples.items():
            plan = decomposer.decompose_with_rules(instruction)
            self.assertIsNotNone(plan, f"No plan for '{instruction}' (expected {action.value})")
            self.assertEqual(
                plan.subgoals[0].action,
                action,
                f"'{instruction}' expected {action.value}, got {plan.subgoals[0].action.value}",
            )


class TestSubGoalPlanStructure(unittest.TestCase):
    """Test Sub Goal Plan Structure."""

    def setUp(self):
        self.decomposer = TaskDecomposer()

    def test_stop_plan_single_subgoal(self):
        """Test stop plan single subgoal."""
        plan = self.decomposer.decompose_with_rules("停止")
        self.assertEqual(len(plan.subgoals), 1)
        self.assertEqual(plan.subgoals[0].action, SubGoalAction.STOP)
        self.assertEqual(plan.subgoals[0].target, "current_task")

    def test_status_plan_single_subgoal(self):
        """Test status plan single subgoal."""
        plan = self.decomposer.decompose_with_rules("当前状态")
        self.assertEqual(len(plan.subgoals), 1)
        self.assertEqual(plan.subgoals[0].action, SubGoalAction.STATUS)

    def test_patrol_plan_with_route(self):
        """Test patrol plan with route."""
        plan = self.decomposer.decompose_with_rules("巡逻")
        self.assertEqual(len(plan.subgoals), 1)
        self.assertEqual(plan.subgoals[0].action, SubGoalAction.PATROL)
        self.assertIn("route", plan.subgoals[0].parameters)

    def test_save_map_plan(self):
        """Test save map plan."""
        plan = self.decomposer.decompose_with_rules("保存地图")
        self.assertEqual(len(plan.subgoals), 1)
        self.assertEqual(plan.subgoals[0].action, SubGoalAction.SAVE_MAP)
        self.assertEqual(plan.subgoals[0].target, "current_map")

    def test_save_poi_plan_with_name(self):
        """Test save poi plan with name."""
        plan = self.decomposer.decompose_with_rules("标记为充电桩")
        self.assertEqual(len(plan.subgoals), 1)
        self.assertEqual(plan.subgoals[0].action, SubGoalAction.SAVE_POI)
        self.assertIn("name", plan.subgoals[0].parameters)
        self.assertEqual(plan.subgoals[0].parameters["name"], "充电桩")

    def test_speed_plan_with_value(self):
        """Test speed plan with value."""
        plan = self.decomposer.decompose_with_rules("快点")
        self.assertEqual(len(plan.subgoals), 1)
        self.assertEqual(plan.subgoals[0].action, SubGoalAction.SET_SPEED)
        self.assertIn("value", plan.subgoals[0].parameters)
        self.assertIsInstance(plan.subgoals[0].parameters["value"], float)

    def test_return_plan(self):
        """Test return plan."""
        plan = self.decomposer.decompose_with_rules("回去")
        self.assertEqual(len(plan.subgoals), 1)
        self.assertEqual(plan.subgoals[0].action, SubGoalAction.RETURN_HOME)
        self.assertEqual(plan.subgoals[0].target, "home")

    def test_pause_plan(self):
        """Test pause plan."""
        plan = self.decomposer.decompose_with_rules("等一下")
        self.assertEqual(len(plan.subgoals), 1)
        self.assertEqual(plan.subgoals[0].action, SubGoalAction.PAUSE)
        self.assertEqual(plan.subgoals[0].target, "current_task")

    def test_resume_plan(self):
        """Test resume plan."""
        plan = self.decomposer.decompose_with_rules("继续")
        self.assertEqual(len(plan.subgoals), 1)
        self.assertEqual(plan.subgoals[0].action, SubGoalAction.RESUME)
        self.assertEqual(plan.subgoals[0].target, "current_task")

    def test_explore_plan_with_strategy(self):
        """Test explore plan with strategy."""
        plan = self.decomposer.decompose_with_rules("探索")
        self.assertEqual(len(plan.subgoals), 1)
        self.assertEqual(plan.subgoals[0].action, SubGoalAction.EXPLORE)
        self.assertEqual(plan.subgoals[0].parameters.get("strategy"), "frontier")

    def test_navigate_plan_three_steps(self):
        """Test navigate plan three steps."""
        plan = self.decomposer.decompose_with_rules("去大门")
        self.assertEqual(len(plan.subgoals), 3)
        self.assertEqual(plan.subgoals[0].action, SubGoalAction.NAVIGATE)
        self.assertEqual(plan.subgoals[1].action, SubGoalAction.APPROACH)
        self.assertEqual(plan.subgoals[2].action, SubGoalAction.VERIFY)

    def test_find_plan_five_steps(self):
        """Test find plan five steps."""
        plan = self.decomposer.decompose_with_rules("找灭火器")
        self.assertEqual(len(plan.subgoals), 5)
        self.assertEqual(plan.subgoals[0].action, SubGoalAction.FIND)
        self.assertEqual(plan.subgoals[1].action, SubGoalAction.LOOK_AROUND)
        self.assertEqual(plan.subgoals[2].action, SubGoalAction.NAVIGATE)
        self.assertEqual(plan.subgoals[3].action, SubGoalAction.APPROACH)
        self.assertEqual(plan.subgoals[4].action, SubGoalAction.VERIFY)

    def test_pick_plan_three_steps(self):
        """Test pick plan three steps."""
        plan = self.decomposer.decompose_with_rules("拿杯子")
        self.assertIsNotNone(plan)
        actions = [sg.action for sg in plan.subgoals]
        self.assertEqual(
            actions,
            [
                SubGoalAction.FIND,
                SubGoalAction.APPROACH,
                SubGoalAction.PICK,
            ],
        )

    def test_place_plan_two_steps(self):
        """Test place plan two steps."""
        plan = self.decomposer.decompose_with_rules("放到桌子上")
        self.assertIsNotNone(plan)
        actions = [sg.action for sg in plan.subgoals]
        self.assertEqual(
            actions,
            [
                SubGoalAction.NAVIGATE,
                SubGoalAction.PLACE,
            ],
        )

    def test_follow_plan_two_steps(self):
        """Test follow plan two steps."""
        plan = self.decomposer.decompose_with_rules("跟着那个人")
        self.assertIsNotNone(plan)
        actions = [sg.action for sg in plan.subgoals]
        self.assertEqual(actions[0], SubGoalAction.FIND)
        self.assertEqual(actions[1], SubGoalAction.FOLLOW)

    def test_inspect_plan_four_steps(self):
        """Test inspect plan four steps."""
        plan = self.decomposer.decompose_with_rules("检查灭火器")
        self.assertIsNotNone(plan)
        actions = [sg.action for sg in plan.subgoals]
        self.assertEqual(
            actions,
            [
                SubGoalAction.FIND,
                SubGoalAction.LOOK_AROUND,
                SubGoalAction.APPROACH,
                SubGoalAction.VERIFY,
            ],
        )

    def test_step_ids_sequential(self):
        """Test step ids sequential."""
        plan = self.decomposer.decompose_with_rules("找红色灭火器")
        for i, sg in enumerate(plan.subgoals):
            self.assertEqual(sg.step_id, i, f"step_id mismatch at index {i}")

    def test_plan_to_dict_serializable(self):
        """Test plan to dict serializable."""
        import json

        plan = self.decomposer.decompose_with_rules("去大门")
        d = plan.to_dict()
        serialized = json.dumps(d, ensure_ascii=False)
        self.assertIn("navigate", serialized)
        self.assertEqual(d["total_steps"], 3)


if __name__ == "__main__":
    unittest.main(verbosity=2)
