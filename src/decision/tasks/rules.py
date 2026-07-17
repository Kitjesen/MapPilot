"""Rule-based task parsing helpers."""

import re as _re


class TaskRulesMixin:
    """Taskrulesmixin."""

    SIMPLE_NAV_PATTERNS_ZH = sorted(
        [
            "去",
            "到",
            "走到",
            "前往",
            "导航到",
            "导航去",
            "走去",
            "过去",
            "走向",
            "前进到",
            "移动到",
            "回到",
            "返回到",
            "回去",
            "返回",
            "去找",
            "去看",
            "去看看",
            "去查看",
            "去检查",
            "去往",
            "赶到",
            "赶去",
            "赶往",
            "赶快去",
            "出发去",
            "出发到",
            "开到",
            "开去",
            "开往",
            "带路到",
            "带我到",
            "带我去",
            "领我到",
            "领我去",
            "引导到",
            "引导去",
            "引导我到",
            "引导我去",
            "陪我去",
            "陪我到",
            "请去",
            "请到",
            "请前往",
            "请导航到",
            "麻烦去",
            "帮我去",
            "帮我到",
            "帮忙去",
            "立刻去",
            "马上去",
            "快去",
            "赶紧去",
            "立即前往",
            "速去",
            "快到",
            "马上到",
            "立刻到",
            "移动至",
            "运动到",
            "行进到",
            "行走到",
            "巡航到",
            "自主前往",
            "自动前往",
            "规划路径到",
            "路径规划到",
        ],
        key=len,
        reverse=True,
    )

    SIMPLE_FIND_PATTERNS_ZH = sorted(
        [
            "找",
            "找到",
            "寻找",
            "搜索",
            "定位",
            "查找",
            "找一下",
            "找找",
            "找找看",
            "找一找",
            "搜一下",
            "搜一搜",
            "搜搜",
            "搜搜看",
            "定位到",
            "定位一下",
            "锁定",
            "锁定目标",
            "确认",
            "确定",
            "确认一下",
            "发现",
            "看到",
            "识别",
            "检测",
            "探测",
            "辨认",
            "辨识",
            "认出",
            "帮我找",
            "帮忙找",
            "帮我找到",
            "帮忙找到",
            "帮我搜",
            "帮忙搜",
            "帮我搜索",
            "帮我定位",
            "帮忙定位",
            "请找",
            "请寻找",
            "请搜索",
            "请定位",
            "麻烦找",
            "麻烦帮我找",
            "劳驾找",
            "快找",
            "赶紧找",
            "立刻找",
            "马上找",
            "快搜",
            "赶紧搜",
        ],
        key=len,
        reverse=True,
    )

    SIMPLE_INSPECT_PATTERNS_ZH = sorted(
        [
            "检查",
            "检查一下",
            "巡检",
            "巡查",
            "查看",
            "查看一下",
            "查验",
            "核查",
            "检测",
            "检测一下",
            "排查",
            "排查一下",
            "检视",
            "审查",
            "查勘",
            "帮我检查",
            "帮忙检查",
            "请检查",
            "去检查",
            "去查看",
            "去巡检",
        ],
        key=len,
        reverse=True,
    )

    SIMPLE_FOLLOW_PATTERNS_ZH = sorted(
        [
            "跟着",
            "跟随",
            "跟踪",
            "追踪",
            "追着",
            "跟住",
            "盯着",
            "盯住",
            "尾随",
            "跟上",
            "追上",
            "跟这个人",
            "跟那个人",
            "跟他",
            "跟她",
            "跟他们",
            "跟上他",
            "跟上她",
            "追上他",
            "追上她",
            "跟着他",
            "跟着她",
            "跟着他走",
            "跟着她走",
            "帮我跟着",
            "帮忙跟着",
            "请跟着",
            "帮我跟踪",
            "请跟踪",
            "紧跟",
            "紧紧跟着",
            "一直跟着",
            "持续跟随",
            "不要跟丢",
            "别跟丢",
        ],
        key=len,
        reverse=True,
    )

    SIMPLE_EXPLORE_PATTERNS_ZH = sorted(
        [
            "探索",
            "探索一下",
            "逛逛",
            "逛一逛",
            "四处看看",
            "到处看看",
            "环顾四周",
            "看看周围",
            "看看附近",
            "巡视",
            "扫描",
            "扫描一下",
            "扫描周围",
            "勘察",
            "侦察",
            "侦查",
            "帮我探索",
            "自由探索",
            "随便走走",
            "随便逛逛",
        ],
        key=len,
        reverse=True,
    )

    SIMPLE_STOP_PATTERNS_ZH = sorted(
        [
            "停",
            "停下",
            "停止",
            "停下来",
            "停一下",
            "别走了",
            "别动",
            "不要动",
            "站住",
            "取消",
            "取消任务",
            "终止",
            "终止任务",
            "中断",
            "中止",
            "算了",
            "不用了",
            "不找了",
            "不去了",
            "不用去了",
            "紧急停止",
            "急停",
            "停止任务",
            "停掉",
            "关掉",
        ],
        key=len,
        reverse=True,
    )

    SIMPLE_PICK_PATTERNS_ZH = sorted(
        [
            "拿",
            "取",
            "拿一下",
            "取一下",
            "拿个",
            "取个",
            "拿一个",
            "取一个",
            "拿过来",
            "取过来",
            "拿来",
            "取来",
            "帮我拿",
            "帮我取",
            "帮忙拿",
            "帮忙取",
            "帮我拿一下",
            "帮我取一下",
            "帮我拿来",
            "帮我取来",
            "帮我带来",
            "帮忙带来",
            "给我拿",
            "给我取",
            "给我拿个",
            "给我取个",
            "给我拿一个",
            "给我取一个",
            "递给我",
            "递一下",
            "递过来",
            "请拿",
            "请取",
            "麻烦拿",
            "麻烦取",
            "快拿",
            "赶紧拿",
            "马上拿",
            "抓",
            "抓住",
            "抓取",
            "夹取",
            "夹住",
            "捡",
            "捡起",
            "捡起来",
            "捡一下",
            "把X拿来",
            "把X给我",
        ],
        key=len,
        reverse=True,
    )

    SIMPLE_PLACE_PATTERNS_ZH = sorted(
        [
            "放",
            "放下",
            "放到",
            "放在",
            "放回",
            "放一下",
            "放好",
            "放回去",
            "摆到",
            "摆在",
            "摆好",
            "摆放",
            "放回原处",
            "归位",
            "还回去",
            "搁到",
            "搁在",
            "搁下",
            "帮我放",
            "帮忙放",
            "帮我放到",
            "帮我放在",
            "请放到",
            "请放在",
            "放置",
            "放置到",
            "放置在",
            "丢到",
            "丢在",
            "扔到",
            "扔在",
            "把X放到",
            "把X放在",
            "把X放回",
        ],
        key=len,
        reverse=True,
    )

    SIMPLE_STATUS_PATTERNS_ZH = sorted(
        [
            "电量",
            "电池",
            "电池电量",
            "电量多少",
            "还有多少电",
            "状态",
            "系统状态",
            "当前状态",
            "机器人状态",
            "模式",
            "当前模式",
            "什么模式",
            "当前任务",
            "任务状态",
            "任务进度",
            "完成了吗",
            "做完了吗",
            "好了吗",
            "还剩多少",
            "剩余电量",
            "剩余时间",
            "现在在哪",
            "现在位置",
            "当前位置",
            "你在哪",
            "现在在做什么",
            "温度",
            "温度多少",
            "当前温度",
            "是否在线",
            "连接状态",
            "网络状态",
            "当前速度",
            "报告状态",
            "汇报状态",
        ],
        key=len,
        reverse=True,
    )

    CONVERSATIONAL_FIND_RE_ZH = [
        _re.compile(
            r"(?:帮我|请|能|能不能|可以|麻烦)?(?:看看|看一下|看一看|查一下|查一查|查看|查看一下|瞧瞧|瞧一下|瞅瞅|望望|看下|查下|瞅一眼|瞧一瞧)(.+?)(?:在哪|在哪里|在哪儿|在什么地方|的位置|在什么位置|在不在)?$"
        ),
        _re.compile(r"(.+?)在哪(?:里|儿)?(?:啊|呢|吗|呀|嘛|哦|嗯)?$"),
        _re.compile(r"(.+?)在什么(?:地方|位置|方向|方位)(?:啊|呢|吗)?$"),
        _re.compile(r"(.+?)(?:的位置|的方位|的方向)(?:在哪|是什么)?(?:啊|呢|吗)?$"),
        _re.compile(
            r"(?:帮我|请|能不能|可以|麻烦|劳驾|烦请)?(?:找一下|找一找|帮忙找|帮忙找一下|帮忙搜一下|帮找)(.+?)(?:吧|好吗|可以吗|行吗)?$"
        ),
        _re.compile(
            r"(?:带我去|带我到|领我去|领我到|带路去|带路到|引导我去|引导我到|陪我去|引我去|送我去|送我到)(.+?)(?:吧|好吗)?$"
        ),
        _re.compile(
            r"(?:我想|我要|我想去|我要去|我得去|我得找|我需要找|我需要去|我想要|我希望|我打算去|我打算找|我准备去|我准备找|咱们去|我们去)(?:找|看|去|找到|看看|看一下|去看)?(.+?)$"
        ),
        _re.compile(
            r"(?:哪里有|哪儿有|什么地方有|哪边有|哪有|有没有|有没有看到|附近有没有|周围有没有|能看到|看得到)(.+?)(?:吗|呢|啊)?$"
        ),
        _re.compile(
            r"(.+?)(?:怎么走|怎么去|怎么找|怎么到|如何去|如何找|如何到达|怎么过去|咋走|咋去|咋找)(?:啊|呢|吗)?$"
        ),
        _re.compile(
            r"(?:告诉我|告诉一下|指给我看|指一下|给我看看|给我指|给我指一下|帮我指|报告|汇报|说一下|说说)(.+?)(?:在哪|的位置|的方向)?(?:吧|好吗)?$"
        ),
        _re.compile(
            r"(.+?)(?:是在|在|是|到底在)(?:哪个位置|哪个地方|什么地方|什么位置|哪边|哪个方向|哪个方位)(?:啊|呢|吗)?$"
        ),
        _re.compile(r"(?:这里|这边|这儿|附近|周围|旁边|楼里|室内|室外|前面|后面)?有(.+?)(?:吗|没|没有|嘛)$"),
        _re.compile(
            r"(?:能|能不能|可以|能否|可否|行不行|能不|可不可以)(?:找到|找|看到|看|定位|发现|搜到|搜索到|检测到|识别)(.+?)(?:吗|嘛|呢|啊)?$"
        ),
        _re.compile(r"(?:最近的|离我最近的|距离最近的|附近的|身边的|旁边的|面前的|眼前的)(.+?)(?:在哪|的位置)?$"),
        _re.compile(r"(?:有几个|有多少|有多少个|一共有多少)(.+?)(?:啊|呢|吗)?$"),
        _re.compile(
            r"(?:你知道|你看到|你看见|你发现|你检测到|你有没有看到|你有没有发现)(.+?)(?:吗|了吗|没|没有|在哪)?$"
        ),
        _re.compile(r"(?:快|赶紧|赶快|马上|立刻|立马|速速|抓紧)(?:找|去找|帮我找|搜|去搜|定位|去看|去查)(.+?)$"),
        _re.compile(r"(?:给我找|给我找个|给我找一个|给我搜|帮我找个|帮我找一个)(.+?)$"),
        _re.compile(r"(?:不是|不要|别找)(?:.+?)(?:是|要|找|去找)(.+?)$"),
        _re.compile(r"(?:整个|搞个|弄个|来个|搞一个|弄一个)(.+?)(?:来|过来|出来)?$"),
        _re.compile(r"(?:这里|这边|附近|周围|前面|后面)(?:都)?有(?:些|哪些|什么)(.+?)$"),
    ]

    CONVERSATIONAL_FIND_RE_EN = [
        # --- Command / Request ---
        _re.compile(
            r"(?:can you |could you |would you |please |hey |ok |okay )?(?:show me|take me to|bring me to|lead me to|walk me to|escort me to) (?:the |a |an )?(.+?)(?:\?|\.)?$",
            _re.IGNORECASE,
        ),
        _re.compile(
            r"(?:can you |could you |would you |please )?(?:point (?:me )?(?:to|at|towards)|direct me to|guide me to|navigate (?:me )?to) (?:the |a |an )?(.+?)(?:\?|\.)?$",
            _re.IGNORECASE,
        ),
        # --- Question ---
        _re.compile(
            r"where (?:is|are|can i find|do i find|would i find|might i find|shall i find) (?:the |a |an )?(.+?)(?:\?)?$",
            _re.IGNORECASE,
        ),
        _re.compile(r"(?:where\'s|where are) (?:the |a |an )?(.+?)(?:\?)?$", _re.IGNORECASE),
        _re.compile(
            r"(?:do you know where|do you see|can you see|have you seen|did you see|did you spot) (?:the |a |an )?(.+?)(?:\?| is)?$",
            _re.IGNORECASE,
        ),
        # --- Desire / Need ---
        _re.compile(
            r"(?:i need|i want|i\'d like|i would like|i\'m looking for|i am looking for|i gotta find|i must find|i have to find) (?:to find |to see |to go to |to get to |to reach |to locate )?(?:the |a |an )?(.+?)(?:\.|!)?$",
            _re.IGNORECASE,
        ),
        # --- Existence ---
        _re.compile(
            r"(?:is there|are there|do we have|does this (?:place|building|floor) have) (?:a |an |any |some )?(.+?)(?:\s+)?(?:here|nearby|around|close by|in (?:this|the) (?:building|room|area|floor))?(?:\?)?$",
            _re.IGNORECASE,
        ),
        # --- How to ---
        _re.compile(
            r"(?:how do i (?:get to|find|reach|locate)|how can i (?:find|get to|reach)|how to (?:find|get to|reach|locate)|what\'s the way to) (?:the |a |an )?(.+?)(?:\?)?$",
            _re.IGNORECASE,
        ),
        # --- Help ---
        _re.compile(
            r"(?:help me (?:find|locate|get to|reach|look for)|help (?:find|locate)) (?:the |a |an )?(.+?)(?:\.|!)?$",
            _re.IGNORECASE,
        ),
        # --- Informal / Collaborative ---
        _re.compile(
            r"(?:let me see|let\'s find|let\'s go (?:to|find)|let\'s look for|let\'s check(?: out)?|let\'s head to|lemme see|lemme find) (?:the |a |an )?(.+?)$",
            _re.IGNORECASE,
        ),
        # --- Imperative ---
        _re.compile(
            r"(?:go (?:find|get|fetch|grab)|fetch me|get me|bring me|grab|gimme) (?:the |a |an )?(.+?)$", _re.IGNORECASE
        ),
        # --- Closest / Nearest ---
        _re.compile(
            r"(?:(?:find|locate|show me|where is|where\'s) )?(?:the )?(?:nearest|closest|closest available|nearest available) (.+?)(?:\?)?$",
            _re.IGNORECASE,
        ),
        # --- Confirmation ---
        _re.compile(
            r"(?:is|are) (?:the |that |this )?(.+?) (?:around here|over there|near(?:by)?|still there|in this (?:room|area|building))(?:\?)?$",
            _re.IGNORECASE,
        ),
        # --- Robot-specific ---
        _re.compile(
            r"(?:scan for|search for|detect|identify|locate|look for) (?:the |a |an |any )?(.+?)$", _re.IGNORECASE
        ),
        _re.compile(
            r"(?:report|check) (?:the )?(?:location|position|status) (?:of )?(?:the |a |an )?(.+?)$", _re.IGNORECASE
        ),
        # --- Informal slang ---
        _re.compile(r"(?:swing by|drop by|pop over to|head over to) (?:the |a |an )?(.+?)$", _re.IGNORECASE),
        # --- Scene awareness ---
        _re.compile(
            r"what(?:\'s| is| do you see) (?:around here|nearby|over there|in (?:this|the) (?:room|area))(?:\?)?$",
            _re.IGNORECASE,
        ),
        _re.compile(r"(?:anything|something) (?:interesting |useful )?(?:nearby|around|here)(?:\?)?$", _re.IGNORECASE),
        _re.compile(r"what (?:can you|do you) (?:see|detect|spot)(?:\?)?$", _re.IGNORECASE),
    ]

    SIMPLE_NAV_PATTERNS_EN = [
        "go to ",
        "navigate to ",
        "move to ",
        "head to ",
        "walk to ",
        "proceed to ",
        "drive to ",
        "travel to ",
        "advance to ",
        "return to ",
        "go back to ",
        "head back to ",
        "rush to ",
        "hurry to ",
        "quickly go to ",
        "head over to ",
        "swing by ",
        "drop by ",
        "pop over to ",
    ]

    SIMPLE_FOLLOW_PATTERNS_EN = [
        "follow ",
        "track ",
        "chase ",
        "tail ",
        "keep following ",
        "keep tracking ",
        "stay with ",
        "stick with ",
        "pursue ",
        "shadow ",
    ]

    SIMPLE_PICK_PATTERNS_EN = [
        "pick up ",
        "grab ",
        "fetch ",
        "fetch me ",
        "bring me ",
        "get me ",
        "hand me ",
        "gimme ",
        "go grab ",
        "go get ",
        "go fetch ",
        "please grab ",
        "please fetch ",
    ]

    SIMPLE_PLACE_PATTERNS_EN = [
        "put ",
        "place ",
        "drop ",
        "set down ",
        "put down ",
        "leave ",
        "return ",
        "put it on ",
        "place it on ",
        "set it on ",
    ]

    SIMPLE_STATUS_PATTERNS_EN = [
        "battery level",
        "battery status",
        "how much battery",
        "current status",
        "system status",
        "robot status",
        "where are you",
        "your location",
        "current position",
        "what mode",
        "current mode",
        "task status",
        "task progress",
        "are you done",
        "remaining battery",
        "remaining time",
        "temperature",
        "current speed",
        "connection status",
        "network status",
        "report status",
        "status report",
        "status",
        "battery",
    ]

    SIMPLE_INSPECT_PATTERNS_EN = [
        "inspect ",
        "examine ",
        "audit ",
    ]

    SIMPLE_PATROL_PATTERNS_ZH = sorted(
        [
            "巡逻",
            "巡检",
            "开始巡逻",
            "开始巡检",
            "巡视",
            "巡查",
            "跑一圈",
            "走一圈",
            "绕一圈",
            "巡逻一圈",
            "启动巡逻",
            "执行巡检",
            "执行巡逻",
            "巡逻路线",
            "巡检路线",
            "按路线巡逻",
            "开始巡视",
            "定点巡逻",
            "例行巡检",
        ],
        key=len,
        reverse=True,
    )

    SIMPLE_PATROL_PATTERNS_EN = [
        "patrol",
        "start patrol",
        "begin patrol",
        "do a patrol",
        "run patrol",
        "patrol route",
        "start inspection",
        "inspection round",
    ]

    SIMPLE_SAVE_MAP_PATTERNS_ZH = sorted(
        [
            "保存地图",
            "存地图",
            "存图",
            "保存当前地图",
            "存一下地图",
            "地图保存",
            "建图完成",
            "结束建图",
            "存个地图",
            "把地图存下来",
            "保存一下地图",
        ],
        key=len,
        reverse=True,
    )

    SIMPLE_SAVE_MAP_PATTERNS_EN = [
        "save map",
        "save the map",
        "save current map",
        "store map",
        "finish mapping",
    ]

    SIMPLE_SAVE_POI_PATTERNS_ZH = sorted(
        [
            "标记",
            "标记为",
            "标记成",
            "标记这里",
            "记住这里",
            "记住这个位置",
            "记住当前位置",
            "保存位置",
            "保存这个位置",
            "保存这个点",
            "保存当前位置",
            "设为",
            "设置为",
            "命名为",
            "记录位置",
            "存个点",
            "存个位置",
            "这里是",
            "这里叫",
            "把这里叫做",
            "添加兴趣点",
            "新建兴趣点",
            "添加POI",
            "保存为",
            "记为",
            "定义为",
        ],
        key=len,
        reverse=True,
    )

    SIMPLE_SAVE_POI_PATTERNS_EN = [
        "mark here as",
        "mark this as",
        "mark here",
        "mark this",
        "save this location as",
        "save this location",
        "remember here as",
        "remember this as",
        "save poi",
        "add poi",
        "mark location",
        "set waypoint",
        "name this place",
        "this is ",
        "call this place",
    ]

    SIMPLE_SPEED_PATTERNS_ZH = sorted(
        [
            "快点",
            "慢点",
            "加速",
            "减速",
            "调速",
            "速度调到",
            "速度设为",
            "速度设置为",
            "调快",
            "调慢",
            "快一点",
            "慢一点",
            "全速",
            "最大速度",
            "最快",
            "低速",
            "最低速度",
            "最慢",
            "正常速度",
            "恢复速度",
            "默认速度",
        ],
        key=len,
        reverse=True,
    )

    SIMPLE_SPEED_PATTERNS_EN = [
        "speed up",
        "slow down",
        "go faster",
        "go slower",
        "set speed to",
        "set speed",
        "speed ",
        "max speed",
        "full speed",
        "normal speed",
        "default speed",
    ]

    SIMPLE_RETURN_PATTERNS_ZH = sorted(
        [
            "回去",
            "回家",
            "返航",
            "回基地",
            "回出发点",
            "回到起点",
            "返回起点",
            "返回基地",
            "返回充电桩",
            "回充电桩",
            "去充电",
            "回去充电",
            "收工",
            "任务完成回去",
            "回来",
            "返回原点",
            "返回",
            "回到原来的位置",
        ],
        key=len,
        reverse=True,
    )

    SIMPLE_RETURN_PATTERNS_EN = [
        "go home",
        "return home",
        "go back",
        "return to base",
        "return to start",
        "go to charging",
        "head back",
        "come back",
        "rtb",
    ]

    SIMPLE_PAUSE_PATTERNS_ZH = sorted(
        [
            "暂停",
            "等一下",
            "等等",
            "先等一下",
            "暂停任务",
            "暂时停下",
            "等一等",
            "先停一下",
            "先暂停",
        ],
        key=len,
        reverse=True,
    )

    SIMPLE_RESUME_PATTERNS_ZH = sorted(
        [
            "继续",
            "恢复",
            "继续走",
            "继续任务",
            "继续导航",
            "继续巡逻",
            "恢复任务",
            "接着走",
            "接着",
            "继续执行",
        ],
        key=len,
        reverse=True,
    )

    # ================================================================
    # ================================================================

    def _extract_poi_name(self, inst: str) -> str:
        """Extract poi name."""
        import re

        for pat in [
            r"标记为(.+?)$",
            r"标记成(.+?)$",
            r"设为(.+?)$",
            r"设置为(.+?)$",
            r"命名为(.+?)$",
            r"保存为(.+?)$",
            r"记为(.+?)$",
            r"定义为(.+?)$",
            r"这里叫(.+?)$",
            r"这里是(.+?)$",
            r"把这里叫做(.+?)$",
            r"叫做(.+?)$",
            r"叫(.+?)$",
            r"mark (?:here |this )?as (.+?)$",
            r"save (?:this )?(?:location |poi )?as (.+?)$",
            r"remember (?:here |this )?as (.+?)$",
            r"(?:this is |call this place )(.+?)$",
            r"name this place (.+?)$",
        ]:
            m = re.search(pat, inst, re.IGNORECASE)
            if m:
                name = m.group(1).strip().rstrip("。.!！")
                if name:
                    return name
        # Fallback: extract last noun phrase
        try:
            from decision.goals.tokenizer import get_tokenizer

            tok = get_tokenizer()
            nouns = tok.extract_noun_phrases(inst)
            if nouns:
                return nouns[-1]
            kws = tok.extract_keywords(inst, min_length=2)
            if kws:
                return kws[-1]
        except Exception:
            pass
        return "未命名"

    def _extract_speed_value(self, inst: str) -> float | None:
        """Extract speed value."""
        import re

        m = re.search(r"(\d+\.?\d*)\s*(?:m/?s|米每秒)?", inst)
        if m:
            val = float(m.group(1))
            if val > 10:  # Likely percentage
                val = val / 100.0
            return min(max(val, 0.1), 3.0)  # Clamp to [0.1, 3.0]
        # Semantic speed keywords
        if any(w in inst for w in ["全速", "最大", "最快", "full", "max"]):
            return 2.0
        if any(w in inst for w in ["快", "faster", "speed up", "加速"]):
            return 1.5
        if any(w in inst for w in ["慢", "slower", "slow down", "减速", "低速"]):
            return 0.3
        if any(w in inst for w in ["正常", "默认", "normal", "default", "恢复速度"]):
            return 1.0
        return None

    def _extract_route_name(self, inst: str) -> str:
        """Extract route name."""
        import re

        for pat in [r"路线(.+?)$", r"按(.+?)巡", r"route\s+(.+?)$", r"patrol\s+(.+?)$"]:
            m = re.search(pat, inst, re.IGNORECASE)
            if m:
                name = m.group(1).strip()
                if name:
                    return name
        return "default"

    # ================================================================
    # ================================================================

    def decompose_with_rules(self, instruction: str):
        """Decompose with rules."""
        from .decomposition import SubGoal, SubGoalAction, TaskPlan

        inst = instruction.strip()

        _COMPLEXITY_MARKERS_ZH = (
            "如果",
            "否则",
            "然后再",
            "接着再",
            "之后再",
            "并且",
            "而且",
            "同时",
            "以及",
            "先去",
            "先找",
            "先到",
            "每个",
            "每一个",
            "所有",
            "全部",
            "依次",
            "巡视所有",
            "逐一",
            "完成后",
            "完成之后",
            "完成以后",
            "做完再",
            "然后",
            "接着",
            "再去",
            "再找",
            "每隔",
            "定期",
            "循环",
            "反复",
        )
        _COMPLEXITY_MARKERS_EN = (
            " if ",
            " then ",
            " else ",
            " otherwise ",
            " and then ",
            " after that ",
            " followed by ",
            " every ",
            " each ",
            " all ",
            " one by one ",
            " after ",
            " before ",
            " once done ",
            " repeat ",
            " periodically ",
            " continuously ",
        )
        if any(m in inst for m in _COMPLEXITY_MARKERS_ZH):
            return None
        if any(m in f" {inst.lower()} " for m in _COMPLEXITY_MARKERS_EN):
            return None
        if inst.count("，") >= 2 or inst.count(",") >= 2:
            return None

        inst_lower = inst.lower()

        is_stop = any(inst.startswith(p) for p in self.SIMPLE_STOP_PATTERNS_ZH)
        if inst_lower.rstrip("!. ") in (
            "stop",
            "halt",
            "cancel",
            "abort",
            "quit",
            "enough",
            "nevermind",
            "never mind",
            "forget it",
        ):
            is_stop = True
        if is_stop:
            return TaskPlan(
                instruction=instruction,
                subgoals=[SubGoal(step_id=0, action=SubGoalAction.STOP, target="current_task")],
            )

        is_status = any(inst.startswith(p) for p in self.SIMPLE_STATUS_PATTERNS_ZH)
        if any(inst_lower.startswith(p) or inst_lower == p for p in self.SIMPLE_STATUS_PATTERNS_EN):
            is_status = True
        if is_status:
            query = inst
            for p in self.SIMPLE_STATUS_PATTERNS_ZH:
                if query.startswith(p):
                    query = p
                    break
            return TaskPlan(
                instruction=instruction,
                subgoals=[SubGoal(step_id=0, action=SubGoalAction.STATUS, target=query)],
            )

        if any(inst.startswith(p) for p in self.SIMPLE_PAUSE_PATTERNS_ZH):
            return TaskPlan(
                instruction=instruction,
                subgoals=[SubGoal(step_id=0, action=SubGoalAction.PAUSE, target="current_task")],
            )
        if any(inst.startswith(p) for p in self.SIMPLE_RESUME_PATTERNS_ZH):
            return TaskPlan(
                instruction=instruction,
                subgoals=[SubGoal(step_id=0, action=SubGoalAction.RESUME, target="current_task")],
            )

        is_return = any(inst.startswith(p) for p in self.SIMPLE_RETURN_PATTERNS_ZH)
        if any(inst_lower.startswith(p) for p in self.SIMPLE_RETURN_PATTERNS_EN):
            is_return = True
        if is_return:
            return TaskPlan(
                instruction=instruction,
                subgoals=[SubGoal(step_id=0, action=SubGoalAction.RETURN_HOME, target="home")],
            )

        is_patrol = any(inst.startswith(p) for p in self.SIMPLE_PATROL_PATTERNS_ZH)
        if any(inst_lower.startswith(p) for p in self.SIMPLE_PATROL_PATTERNS_EN):
            is_patrol = True
        if is_patrol:
            route = self._extract_route_name(inst)
            return TaskPlan(
                instruction=instruction,
                subgoals=[
                    SubGoal(step_id=0, action=SubGoalAction.PATROL, target="patrol", parameters={"route": route})
                ],
            )

        is_save_map = any(inst.startswith(p) for p in self.SIMPLE_SAVE_MAP_PATTERNS_ZH)
        if any(inst_lower.startswith(p) for p in self.SIMPLE_SAVE_MAP_PATTERNS_EN):
            is_save_map = True
        if is_save_map:
            return TaskPlan(
                instruction=instruction,
                subgoals=[SubGoal(step_id=0, action=SubGoalAction.SAVE_MAP, target="current_map")],
            )

        is_save_poi = any(inst.startswith(p) for p in self.SIMPLE_SAVE_POI_PATTERNS_ZH)
        if any(inst_lower.startswith(p) for p in self.SIMPLE_SAVE_POI_PATTERNS_EN):
            is_save_poi = True
        if is_save_poi:
            name = self._extract_poi_name(inst)
            return TaskPlan(
                instruction=instruction,
                subgoals=[SubGoal(step_id=0, action=SubGoalAction.SAVE_POI, target=name, parameters={"name": name})],
            )

        is_speed = any(inst.startswith(p) for p in self.SIMPLE_SPEED_PATTERNS_ZH)
        if any(inst_lower.startswith(p) for p in self.SIMPLE_SPEED_PATTERNS_EN):
            is_speed = True
        if is_speed:
            speed = self._extract_speed_value(inst)
            return TaskPlan(
                instruction=instruction,
                subgoals=[
                    SubGoal(
                        step_id=0, action=SubGoalAction.SET_SPEED, target="speed", parameters={"value": speed or 1.0}
                    )
                ],
            )

        is_explore = any(inst.startswith(p) for p in self.SIMPLE_EXPLORE_PATTERNS_ZH)
        if inst_lower.startswith(("explore", "look around", "scan the", "survey", "wander")):
            is_explore = True
        if is_explore:
            return TaskPlan(
                instruction=instruction,
                subgoals=[
                    SubGoal(
                        step_id=0,
                        action=SubGoalAction.EXPLORE,
                        target="environment",
                        parameters={"strategy": "frontier"},
                    ),
                ],
            )

        is_place = any(inst.startswith(p) for p in self.SIMPLE_PLACE_PATTERNS_ZH)
        if any(inst_lower.startswith(p) for p in self.SIMPLE_PLACE_PATTERNS_EN):
            is_place = True
        if is_place:
            target = inst
            for p in self.SIMPLE_PLACE_PATTERNS_ZH:
                if target.startswith(p):
                    target = target[len(p) :].strip()
                    break
            for p in self.SIMPLE_PLACE_PATTERNS_EN:
                if inst_lower.startswith(p):
                    target = inst[len(p) :].strip()
                    break
            if not target or target == inst:
                target = "current_location"
            return TaskPlan(
                instruction=instruction,
                subgoals=[
                    SubGoal(step_id=0, action=SubGoalAction.NAVIGATE, target=target),
                    SubGoal(step_id=1, action=SubGoalAction.PLACE, target=target),
                ],
            )

        is_pick = any(inst.startswith(p) for p in self.SIMPLE_PICK_PATTERNS_ZH)
        if any(inst_lower.startswith(p) for p in self.SIMPLE_PICK_PATTERNS_EN):
            is_pick = True
        if is_pick:
            target = inst
            for p in self.SIMPLE_PICK_PATTERNS_ZH:
                if target.startswith(p):
                    target = target[len(p) :].strip()
                    break
            for p in self.SIMPLE_PICK_PATTERNS_EN:
                if inst_lower.startswith(p):
                    target = inst[len(p) :].strip()
                    break
            if not target or target == inst:
                target = "object"

            kg_params = {}
            if self._knowledge_graph is not None:
                manip_info = self._knowledge_graph.get_manipulation_info(target, "pick")
                if not manip_info.get("feasible", True):
                    reason = manip_info.get("reason", "unknown")
                    notes = manip_info.get("notes", [])
                    import logging

                    logging.getLogger(__name__).warning(
                        "KG safety gate BLOCKED pick '%s': %s — %s",
                        target,
                        reason,
                        notes,
                    )
                    return TaskPlan(
                        instruction=instruction,
                        subgoals=[
                            SubGoal(
                                step_id=0,
                                action=SubGoalAction.STATUS,
                                target=target,
                                parameters={
                                    "kg_blocked": True,
                                    "reason": reason,
                                    "notes": notes,
                                    "safety": manip_info.get("safety", "unknown"),
                                },
                            ),
                        ],
                    )
                kg_params = {
                    "kg_safety": manip_info.get("safety", "safe"),
                    "kg_weight": manip_info.get("weight_range", []),
                    "kg_size": manip_info.get("size_class", "unknown"),
                }

            return TaskPlan(
                instruction=instruction,
                subgoals=[
                    SubGoal(step_id=0, action=SubGoalAction.FIND, target=target),
                    SubGoal(
                        step_id=1,
                        action=SubGoalAction.APPROACH,
                        target=target,
                        parameters={"approach_distance": 0.3, **kg_params},
                    ),
                    SubGoal(step_id=2, action=SubGoalAction.PICK, target=target, parameters=kg_params),
                ],
            )

        is_inspect = any(inst.startswith(p) for p in self.SIMPLE_INSPECT_PATTERNS_ZH)
        if any(inst_lower.startswith(p) for p in self.SIMPLE_INSPECT_PATTERNS_EN):
            is_inspect = True
        if inst_lower.startswith("check ") and not inst_lower.startswith(("check if ", "check whether ")):
            is_inspect = True
        if is_inspect:
            target = inst
            for p in self.SIMPLE_INSPECT_PATTERNS_ZH:
                if target.startswith(p):
                    target = target[len(p) :].strip()
                    break
            for p in [*list(self.SIMPLE_INSPECT_PATTERNS_EN), "check "]:
                if inst_lower.startswith(p):
                    target = inst[len(p) :].strip()
                    break
            if not target or target == inst:
                target = "environment"
            return TaskPlan(
                instruction=instruction,
                subgoals=[
                    SubGoal(step_id=0, action=SubGoalAction.FIND, target=target),
                    SubGoal(
                        step_id=1,
                        action=SubGoalAction.LOOK_AROUND,
                        target=target,
                        parameters={"reason": "inspect target"},
                    ),
                    SubGoal(step_id=2, action=SubGoalAction.APPROACH, target=target),
                    SubGoal(step_id=3, action=SubGoalAction.VERIFY, target=target),
                ],
            )

        is_follow = any(inst.startswith(p) for p in self.SIMPLE_FOLLOW_PATTERNS_ZH)
        if any(inst_lower.startswith(p) for p in self.SIMPLE_FOLLOW_PATTERNS_EN):
            is_follow = True
        if "follow" in inst_lower and (
            "person" in inst_lower or "human" in inst_lower or "him" in inst_lower or "her" in inst_lower
        ):
            is_follow = True

        if is_follow:
            target = inst
            for p in self.SIMPLE_FOLLOW_PATTERNS_ZH:
                if target.startswith(p):
                    target = target[len(p) :].strip()
                    break
            for p in self.SIMPLE_FOLLOW_PATTERNS_EN:
                if inst_lower.startswith(p):
                    target = inst[len(p) :].strip()
                    break
            if not target or target == inst:
                target = "person"
            return TaskPlan(
                instruction=instruction,
                subgoals=[
                    SubGoal(step_id=0, action=SubGoalAction.FIND, target=target),
                    SubGoal(
                        step_id=1,
                        action=SubGoalAction.FOLLOW,
                        target=target,
                        parameters={"follow_distance": 1.5, "timeout": 300},
                        max_retries=5,
                    ),
                ],
            )

        is_simple_nav = any(inst.startswith(p) for p in self.SIMPLE_NAV_PATTERNS_ZH)
        is_simple_find = any(inst.startswith(p) for p in self.SIMPLE_FIND_PATTERNS_ZH)

        if any(inst_lower.startswith(p) for p in self.SIMPLE_NAV_PATTERNS_EN):
            is_simple_nav = True
        if inst_lower.startswith(
            (
                "find ",
                "search for ",
                "locate ",
                "look for ",
                "where is ",
                "where are ",
                "show me ",
                "seek ",
                "spot ",
                "identify ",
            )
        ):
            is_simple_find = True

        conversational_target = None
        if not is_simple_nav and not is_simple_find:
            for pat in self.CONVERSATIONAL_FIND_RE_ZH:
                m = pat.search(inst)
                if m:
                    conversational_target = m.group(1).strip()
                    if conversational_target:
                        is_simple_find = True
                        break
            if not is_simple_find:
                for pat in self.CONVERSATIONAL_FIND_RE_EN:
                    m = pat.search(inst)
                    if m:
                        conversational_target = m.group(1).strip()
                        if conversational_target:
                            is_simple_find = True
                            break

        if not is_simple_nav and not is_simple_find:
            return None

        if conversational_target:
            target = conversational_target
        else:
            target = inst
            for p in self.SIMPLE_NAV_PATTERNS_ZH + self.SIMPLE_FIND_PATTERNS_ZH:
                if target.startswith(p):
                    target = target[len(p) :].strip()
                    break
            for p in [
                *self.SIMPLE_NAV_PATTERNS_EN,
                "find ",
                "search for ",
                "locate ",
                "look for ",
                "where is ",
                "where are ",
                "show me ",
                "seek ",
                "spot ",
                "identify ",
            ]:
                if inst_lower.startswith(p):
                    target = inst[len(p) :].strip()
                    break

        subgoals = []
        step = 0

        kg_approach_params: dict = {"approach_distance": 0.5}
        kg_find_params: dict = {}
        if self._knowledge_graph is not None:
            constraint = self._knowledge_graph.check_safety(target, "approach")
            if constraint and constraint.max_approach_distance > 0:
                kg_approach_params["approach_distance"] = max(
                    kg_approach_params["approach_distance"],
                    constraint.max_approach_distance,
                )
                kg_approach_params["kg_safety_note"] = constraint.message_en

            typical_locs = self._knowledge_graph.get_typical_locations(target)
            if typical_locs:
                kg_find_params["typical_locations"] = typical_locs[:5]

            safety = self._knowledge_graph.get_safety_level(target)
            kg_approach_params["kg_safety"] = safety.value

        if is_simple_find:
            subgoals.append(
                SubGoal(
                    step_id=step,
                    action=SubGoalAction.FIND,
                    target=target,
                    parameters=kg_find_params,
                )
            )
            step += 1
            subgoals.append(
                SubGoal(
                    step_id=step,
                    action=SubGoalAction.LOOK_AROUND,
                    target=target,
                    parameters={"reason": "scan for target"},
                )
            )
            step += 1

        subgoals.append(
            SubGoal(
                step_id=step,
                action=SubGoalAction.NAVIGATE,
                target=target,
            )
        )
        step += 1
        subgoals.append(
            SubGoal(
                step_id=step,
                action=SubGoalAction.APPROACH,
                target=target,
                parameters=kg_approach_params,
            )
        )
        step += 1
        subgoals.append(
            SubGoal(
                step_id=step,
                action=SubGoalAction.VERIFY,
                target=target,
            )
        )

        return TaskPlan(instruction=instruction, subgoals=subgoals)
