"""
中文分词工具模块 - 使用jieba进行精确分词

参考: 报告第11.1节 P0任务
目标: 替换简单regex分词，提升中文指令解析准确率

扩展: 双语标签映射 (中↔英)
  YOLO 检测器输出英文标签, 用户指令常为中文 → Fast Path 需要跨语言匹配。
  映射表从 IndustrialKnowledgeGraph 概念提取, 覆盖室内/工业/户外场景常见物体。
"""

import logging
from typing import Dict, List, Set, Tuple

logger = logging.getLogger(__name__)

# 停用词表 — 工业级覆盖 (中英文 + 口语 + 方言 + 语气)
STOP_WORDS: Set[str] = {
    # ── 英文停用词 ──
    # 注意: 不删除意图动词 (go/find/get/show/take/bring/lead/see)
    #       这些词在 intent detection 阶段仍有语义价值
    #       仅在 noun extraction 阶段才应过滤
    "the", "a", "an", "to", "me", "my", "i",
    "is", "at", "in", "on", "near", "next", "by", "of", "for",
    "and", "or", "but", "with", "from", "this", "that", "it",
    "please", "can", "could", "would", "should", "will", "shall",
    "you", "your", "there", "here", "where", "how", "what", "which",
    "do", "does", "did", "have", "has", "had", "be", "am", "are", "was",
    "not", "no", "yes", "ok", "okay", "hey", "hi", "hello",
    "help", "let",
    "just", "also", "too", "very", "really", "actually", "basically",
    "some", "any", "each", "every",
    # ── 中文基础停用词 ──
    "去", "到", "找", "拿", "的", "在", "旁边", "附近", "那个",
    "请", "帮", "我", "你", "他", "她", "它", "我们", "他们",
    "一个", "把", "了", "着", "过", "给", "被", "让", "将",
    "和", "与", "或", "但是", "然后", "接着", "之后",
    "这个", "那个", "这些", "那些", "某个",
    # ── 动词填充 ──
    "看", "看看", "看一下", "看一看", "看下", "查", "查看",
    "一下", "一看", "一找", "一搜",
    "帮我", "帮忙", "麻烦", "劳驾", "烦请",
    # ── 疑问 / 位置 ──
    "在哪", "在哪里", "在哪儿", "在什么地方", "在什么位置",
    "怎么走", "怎么去", "怎么找", "如何", "如何去",
    "是什么", "什么", "几个", "多少",
    # ── 情态 / 意愿 ──
    "告诉我", "带我去", "领我去", "引导",
    "能不能", "可以", "是不是", "是否", "能否", "可否",
    "有没有", "知道", "知不知道",
    "想", "要", "想要", "需要", "希望", "打算", "准备",
    "得", "得去", "得找",
    # ── 语气词 ──
    "吗", "呢", "啊", "吧", "嘛", "呀", "哦", "嗯", "哈",
    "哎", "唉", "诶", "喂", "嘿", "哟",
    # ── 程度 / 时间 ──
    "快", "赶紧", "赶快", "马上", "立刻", "立即", "立马",
    "先", "再", "也", "都", "就", "才", "还", "又",
    "很", "非常", "特别", "最", "比较", "稍微",
    # ── 方言 / 口头禅 ──
    "整", "搞", "弄", "来", "来个", "整个", "搞个", "弄个",
    "咋", "啥", "咋整", "咋办", "咋走",
    # ── 连词 / 介词 ──
    "从", "往", "向", "朝", "对", "跟", "比",
    "因为", "所以", "如果", "虽然", "不过",
    # ── 否定 ──
    "不", "没", "没有", "别", "不要", "不用", "无",
}


# ── 双语标签映射 (中↔英) ──
# 格式: concept_id → (names_zh, names_en)
# 来源: IndustrialKnowledgeGraph 概念 (轻量子集, 避免跨包依赖)
# 每个 concept 取主要别名, 不需要全部 (全量在 KG 中)
_BILINGUAL_CONCEPTS: List[Tuple[List[str], List[str]]] = [
    # ── 家具 ──
    (["椅子", "办公椅", "座椅", "凳子"], ["chair", "office chair", "seat", "stool"]),
    (["桌子", "办公桌", "书桌", "电脑桌"], ["desk", "table", "workstation"]),
    (["柜子", "文件柜", "储物柜", "橱柜"], ["cabinet", "file cabinet", "storage cabinet", "locker"]),
    (["架子", "书架", "货架", "置物架"], ["shelf", "bookshelf", "rack", "shelving"]),
    (["沙发", "长椅"], ["sofa", "couch", "bench"]),
    (["白板"], ["whiteboard", "board"]),
    (["床", "床铺"], ["bed"]),
    # ── 电子设备 ──
    (["显示器", "屏幕", "电脑屏幕"], ["monitor", "screen", "display"]),
    (["电脑", "计算机", "笔记本"], ["computer", "PC", "laptop", "desktop"]),
    (["投影仪", "投影机"], ["projector"]),
    (["打印机", "复印机"], ["printer", "copier"]),
    (["电视", "电视机"], ["TV", "television"]),
    (["微波炉"], ["microwave"]),
    (["冰箱"], ["refrigerator", "fridge"]),
    (["洗衣机"], ["washing machine"]),
    # ── 安全设备 ──
    (["灭火器"], ["fire extinguisher", "extinguisher"]),
    (["消防栓", "消火栓"], ["fire hydrant", "fire hose cabinet"]),
    (["火灾报警器", "烟感", "烟雾报警器"], ["fire alarm", "smoke detector"]),
    (["安全出口", "紧急出口"], ["emergency exit", "fire exit", "exit sign"]),
    (["急救箱", "急救包"], ["first aid kit", "medical kit"]),
    (["安全标识", "警示牌"], ["safety sign", "warning sign"]),
    (["AED", "除颤仪"], ["AED", "defibrillator"]),
    # ── 建筑结构 ──
    (["门", "房门", "大门", "玻璃门"], ["door", "gate", "glass door"]),
    (["窗户", "窗", "玻璃窗"], ["window"]),
    (["楼梯", "台阶"], ["stairs", "staircase", "steps"]),
    (["电梯"], ["elevator", "lift"]),
    (["栏杆", "扶手", "护栏"], ["railing", "handrail", "guardrail"]),
    (["柱子", "立柱"], ["pillar", "column"]),
    (["墙", "墙壁"], ["wall"]),
    # ── 生活用品 ──
    (["垃圾桶", "垃圾箱"], ["trash can", "trash bin", "garbage bin", "waste bin"]),
    (["饮水机", "净水器"], ["water dispenser", "water cooler"]),
    (["水瓶", "瓶子", "矿泉水"], ["bottle", "water bottle"]),
    (["杯子", "茶杯", "咖啡杯", "水杯"], ["cup", "mug", "glass"]),
    (["箱子", "纸箱", "盒子"], ["box", "carton", "package"]),
    (["灯", "台灯", "落地灯"], ["lamp", "light", "desk lamp"]),
    (["背包", "书包", "双肩包"], ["backpack", "bag", "rucksack"]),
    (["植物", "盆栽", "绿植"], ["plant", "potted plant"]),
    (["伞架", "雨伞架"], ["umbrella stand"]),
    (["镜子"], ["mirror"]),
    (["水槽", "洗手台"], ["sink", "basin"]),
    (["马桶", "坐便器"], ["toilet"]),
    # ── 工业 ──
    (["配电箱", "电箱"], ["electrical panel", "distribution box"]),
    (["充电桩", "充电站"], ["charging station", "charger"]),
    (["叉车", "铲车"], ["forklift"]),
    (["托盘", "栈板"], ["pallet"]),
    (["传送带", "输送带"], ["conveyor", "conveyor belt"]),
    (["阀门"], ["valve"]),
    (["管道", "水管"], ["pipe", "pipeline"]),
    (["控制面板", "操作台"], ["control panel"]),
    # ── 户外 ──
    (["树", "树木"], ["tree"]),
    (["路锥", "锥桶"], ["traffic cone", "cone"]),
    (["围栏", "栅栏"], ["fence", "barrier"]),
    (["路灯", "灯柱"], ["street light", "lamp post"]),
    (["井盖"], ["manhole cover"]),
    (["公园长椅", "户外长椅"], ["park bench"]),
    # ── 人 ──
    (["人", "行人", "员工", "访客"], ["person", "people", "pedestrian"]),
    # ── 房间 / 区域 (场景图 room label) ──
    (["厨房"], ["kitchen"]),
    (["卧室"], ["bedroom"]),
    (["客厅", "起居室"], ["living room"]),
    (["卫生间", "洗手间", "厕所", "浴室"], ["bathroom", "restroom", "toilet", "washroom"]),
    (["办公室"], ["office"]),
    (["会议室"], ["meeting room", "conference room"]),
    (["走廊", "过道"], ["corridor", "hallway"]),
    (["大厅", "大堂", "门厅"], ["lobby", "hall"]),
    (["仓库"], ["warehouse", "storage room"]),
    (["车库"], ["garage"]),
]

# 预构建查找索引: word(lower) → set of cross-language equivalents
_ZH_TO_EN: Dict[str, Set[str]] = {}
_EN_TO_ZH: Dict[str, Set[str]] = {}

for _zh_names, _en_names in _BILINGUAL_CONCEPTS:
    _en_set = {e.lower() for e in _en_names}
    _zh_set = set(_zh_names)
    for zh in _zh_names:
        _ZH_TO_EN.setdefault(zh, set()).update(_en_set)
    for en in _en_names:
        _EN_TO_ZH.setdefault(en.lower(), set()).update(_zh_set)


def expand_bilingual(keywords: List[str]) -> List[str]:
    """
    将关键词列表扩展为中英双语。

    "桌子" → ["桌子", "desk", "table", "workstation"]
    "chair" → ["chair", "椅子", "办公椅", "座椅", "凳子"]

    只扩展匹配到的词, 其余原样保留。返回去重后的列表。
    """
    expanded: List[str] = list(keywords)
    for kw in keywords:
        kw_lower = kw.lower()
        # zh → en
        if kw in _ZH_TO_EN:
            expanded.extend(_ZH_TO_EN[kw])
        # en → zh
        if kw_lower in _EN_TO_ZH:
            expanded.extend(_EN_TO_ZH[kw_lower])
    # 去重, 保序
    seen: set = set()
    result: List[str] = []
    for w in expanded:
        w_key = w.lower()
        if w_key not in seen:
            seen.add(w_key)
            result.append(w)
    return result


def translate_label(label: str) -> List[str]:
    """
    将单个标签翻译为另一种语言的所有别名。

    "desk" → ["桌子", "办公桌", "书桌", "电脑桌"]
    "椅子" → ["chair", "office chair", "seat", "stool"]

    未找到时返回空列表。
    """
    label_lower = label.lower()
    if label in _ZH_TO_EN:
        return sorted(_ZH_TO_EN[label])
    if label_lower in _EN_TO_ZH:
        return sorted(_EN_TO_ZH[label_lower])
    # 子串匹配: "red chair" → 匹配 "chair" → 返回中文
    for en_key, zh_set in _EN_TO_ZH.items():
        if en_key in label_lower or label_lower in en_key:
            return sorted(zh_set)
    for zh_key, en_set in _ZH_TO_EN.items():
        if zh_key in label or label in zh_key:
            return sorted(en_set)
    return []


class ChineseTokenizer:
    """
    中文分词器 - 使用jieba进行精确分词

    特性:
    - 支持中英文混合分词
    - 自定义词典支持（机器人领域词汇）
    - 停用词过滤
    - 关键词提取
    """

    def __init__(self, use_jieba: bool = True, custom_dict_path: str = None):
        """
        初始化分词器

        Args:
            use_jieba: 是否使用jieba（False则回退到简单分词）
            custom_dict_path: 自定义词典路径
        """
        self.use_jieba = use_jieba
        self._jieba = None

        if use_jieba:
            try:
                import jieba
                import jieba.posseg as pseg
                self._jieba = jieba
                self._pseg = pseg

                # 加载自定义词典
                if custom_dict_path:
                    jieba.load_userdict(custom_dict_path)

                # 添加机器人领域词汇
                self._add_robot_vocabulary()

                logger.info("Jieba分词器初始化成功")
            except ImportError:
                logger.warning("jieba未安装，回退到简单分词。请运行: pip install jieba")
                self.use_jieba = False

    def _add_robot_vocabulary(self):
        """添加机器人领域的自定义词汇"""
        robot_words = [
            # 物体类别
            ("灭火器", 10, "n"),
            ("充电桩", 10, "n"),
            ("垃圾桶", 10, "n"),
            ("饮水机", 10, "n"),
            ("打印机", 10, "n"),
            ("投影仪", 10, "n"),
            ("白板", 10, "n"),
            ("会议桌", 10, "n"),
            ("办公桌", 10, "n"),
            ("书架", 10, "n"),
            # 颜色+物体组合
            ("红色灭火器", 15, "n"),
            ("蓝色椅子", 15, "n"),
            ("白色门", 15, "n"),
            # 空间关系
            ("左边", 10, "f"),
            ("右边", 10, "f"),
            ("前面", 10, "f"),
            ("后面", 10, "f"),
            ("上面", 10, "f"),
            ("下面", 10, "f"),
            # 动作
            ("导航", 10, "v"),
            ("接近", 10, "v"),
            ("探索", 10, "v"),
            ("回退", 10, "v"),
            ("扫描", 10, "v"),
        ]

        for word, freq, tag in robot_words:
            self._jieba.add_word(word, freq, tag)

    def tokenize(self, text: str, keep_pos: bool = False) -> List[str]:
        """
        分词

        Args:
            text: 输入文本
            keep_pos: 是否保留词性标注

        Returns:
            分词结果列表
        """
        if not text:
            return []

        if self.use_jieba and self._jieba:
            if keep_pos:
                # 带词性标注的分词
                words = self._pseg.cut(text)
                return [(w.word, w.flag) for w in words]
            else:
                # 精确模式分词
                return list(self._jieba.cut(text, cut_all=False))
        else:
            # 回退到简单分词
            return self._simple_tokenize(text)

    def _simple_tokenize(self, text: str) -> List[str]:
        """简单分词（回退方案）"""
        import re
        # 混合分词: 英文按空格, 中文按字符组
        tokens = re.findall(r"[a-zA-Z]+|[\u4e00-\u9fff]+", text.lower())
        return tokens

    def extract_keywords(
        self,
        text: str,
        min_length: int = 2,
        filter_stopwords: bool = True,
        keep_colors: bool = True,
        keep_spatial: bool = True
    ) -> List[str]:
        """
        提取关键词

        Args:
            text: 输入文本
            min_length: 最小词长
            filter_stopwords: 是否过滤停用词
            keep_colors: 是否保留颜色词
            keep_spatial: 是否保留空间关系词

        Returns:
            关键词列表
        """
        # 分词
        if self.use_jieba and self._jieba:
            words_with_pos = self._pseg.cut(text)
            tokens = []

            for word, pos in words_with_pos:
                word_lower = word.lower()

                # 过滤停用词
                if filter_stopwords and word_lower in STOP_WORDS:
                    continue

                # 长度过滤
                if len(word) < min_length:
                    continue

                # 词性过滤（保留名词、动词、形容词、方位词）
                if pos in ['n', 'v', 'a', 'f', 'ns', 'nr', 'nz', 'vn']:
                    tokens.append(word_lower)
                # 颜色词（形容词）
                elif keep_colors and self._is_color_word(word):
                    tokens.append(word_lower)
                # 空间关系词
                elif keep_spatial and self._is_spatial_word(word):
                    tokens.append(word_lower)
        else:
            # 简单分词
            tokens = self._simple_tokenize(text)
            if filter_stopwords:
                tokens = [t for t in tokens if t not in STOP_WORDS]
            tokens = [t for t in tokens if len(t) >= min_length]

        return tokens

    def _is_color_word(self, word: str) -> bool:
        """判断是否为颜色词"""
        colors = {
            "红", "红色", "蓝", "蓝色", "绿", "绿色", "黄", "黄色",
            "白", "白色", "黑", "黑色", "灰", "灰色", "紫", "紫色",
            "橙", "橙色", "粉", "粉色", "棕", "棕色", "银", "银色",
            "red", "blue", "green", "yellow", "white", "black",
            "gray", "grey", "purple", "orange", "pink", "brown", "silver"
        }
        return word.lower() in colors

    def _is_spatial_word(self, word: str) -> bool:
        """判断是否为空间关系词"""
        spatial = {
            "左", "右", "前", "后", "上", "下", "里", "外",
            "左边", "右边", "前面", "后面", "上面", "下面", "里面", "外面",
            "旁边", "附近", "对面", "中间", "角落",
            "left", "right", "front", "back", "above", "below",
            "inside", "outside", "near", "beside", "opposite", "middle", "corner"
        }
        return word.lower() in spatial

    def extract_noun_phrases(self, text: str) -> List[str]:
        """
        提取名词短语（如"红色灭火器"、"会议室的门"）

        Args:
            text: 输入文本

        Returns:
            名词短语列表
        """
        if not self.use_jieba or not self._jieba:
            return self.extract_keywords(text)

        words_with_pos = list(self._pseg.cut(text))
        noun_phrases = []
        current_phrase = []

        for i, (word, pos) in enumerate(words_with_pos):
            # 形容词 + 名词 → 名词短语
            if pos in ['a', 'b'] and i + 1 < len(words_with_pos):
                next_word, next_pos = words_with_pos[i + 1]
                if next_pos in ['n', 'ns', 'nr', 'nz']:
                    noun_phrases.append(word + next_word)

            # 名词 + 的 + 名词 → 名词短语
            if pos in ['n', 'ns', 'nr', 'nz']:
                if i + 2 < len(words_with_pos):
                    mid_word, mid_pos = words_with_pos[i + 1]
                    next_word, next_pos = words_with_pos[i + 2]
                    if mid_word == "的" and next_pos in ['n', 'ns', 'nr', 'nz']:
                        noun_phrases.append(word + "的" + next_word)

        return noun_phrases


# 全局单例
_global_tokenizer: ChineseTokenizer = None


def get_tokenizer(use_jieba: bool = True, custom_dict_path: str = None) -> ChineseTokenizer:
    """
    获取全局分词器单例

    Args:
        use_jieba: 是否使用jieba
        custom_dict_path: 自定义词典路径

    Returns:
        ChineseTokenizer实例
    """
    global _global_tokenizer
    if _global_tokenizer is None:
        _global_tokenizer = ChineseTokenizer(use_jieba, custom_dict_path)
    return _global_tokenizer


def extract_keywords(text: str, **kwargs) -> List[str]:
    """
    便捷函数：提取关键词

    Args:
        text: 输入文本
        **kwargs: 传递给ChineseTokenizer.extract_keywords的参数

    Returns:
        关键词列表
    """
    tokenizer = get_tokenizer()
    return tokenizer.extract_keywords(text, **kwargs)


# 向后兼容：保留原有的简单分词函数
def simple_extract_keywords(instruction: str) -> List[str]:
    """
    简单分词（向后兼容）

    这是原有的regex分词实现，保留用于回退
    """
    import re
    stop_words = STOP_WORDS
    tokens = re.findall(r"[a-zA-Z]+|[\u4e00-\u9fff]+", instruction.lower())
    return [t for t in tokens if t not in stop_words and len(t) > 1]
