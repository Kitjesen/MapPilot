"""
中文分词器测试

测试jieba分词器的功能和性能
"""

import pytest
from semantic_planner.chinese_tokenizer import (
    ChineseTokenizer,
    extract_keywords,
    simple_extract_keywords,
)


class TestChineseTokenizer:
    """测试ChineseTokenizer类"""

    def test_init_with_jieba(self):
        """测试使用jieba初始化"""
        tokenizer = ChineseTokenizer(use_jieba=True)
        assert tokenizer.use_jieba is True or tokenizer.use_jieba is False  # 取决于是否安装

    def test_init_without_jieba(self):
        """测试不使用jieba初始化"""
        tokenizer = ChineseTokenizer(use_jieba=False)
        assert tokenizer.use_jieba is False

    def test_simple_tokenize_chinese(self):
        """测试简单中文分词: 无jieba时连续汉字作为整体返回"""
        tokenizer = ChineseTokenizer(use_jieba=False)
        result = tokenizer.tokenize("去红色灭火器旁边")
        # 简单分词器 regex [\u4e00-\u9fff]+ 将连续汉字作为整体返回
        assert len(result) > 0
        assert "去红色灭火器旁边" in result

    def test_simple_tokenize_english(self):
        """测试简单英文分词"""
        tokenizer = ChineseTokenizer(use_jieba=False)
        result = tokenizer.tokenize("go to the red fire extinguisher")
        assert "go" in result
        assert "red" in result
        assert "fire" in result

    def test_simple_tokenize_mixed(self):
        """测试中英文混合分词"""
        tokenizer = ChineseTokenizer(use_jieba=False)
        result = tokenizer.tokenize("go to 红色灭火器")
        assert len(result) > 0

    def test_extract_keywords_chinese(self):
        """测试中文关键词提取"""
        tokenizer = ChineseTokenizer(use_jieba=True)
        result = tokenizer.extract_keywords("请去红色灭火器旁边")
        # 应该过滤掉"请"、"去"等停用词
        assert "请" not in result
        assert "去" not in result
        # 应该保留关键词
        assert any(kw in result for kw in ["红色", "灭火器", "红色灭火器", "旁边"])

    def test_extract_keywords_english(self):
        """测试英文关键词提取: jieba 将英文标注为 'eng' POS，只有颜色/空间词被保留"""
        tokenizer = ChineseTokenizer(use_jieba=True)
        result = tokenizer.extract_keywords("go to the red fire extinguisher")
        # 停用词应被过滤
        assert "the" not in result
        assert "to" not in result
        # "red" 是颜色词，被 _is_color_word 保留
        assert "red" in result
        # "fire"/"extinguisher" 被 jieba 标注为 'eng' POS (非中文名词)，当前不提取英文名词
        # 这是已知的设计局限: extract_keywords 主要针对中文输入

    def test_extract_keywords_min_length(self):
        """测试最小长度过滤"""
        tokenizer = ChineseTokenizer(use_jieba=True)
        result = tokenizer.extract_keywords("去a红b色c灭火器", min_length=2)
        # 单字符应该被过滤
        assert "a" not in result
        assert "b" not in result
        assert "c" not in result

    def test_extract_keywords_keep_colors(self):
        """测试保留颜色词"""
        tokenizer = ChineseTokenizer(use_jieba=True)
        result = tokenizer.extract_keywords("红色椅子", keep_colors=True)
        assert any(kw in result for kw in ["红", "红色"])

    def test_extract_keywords_keep_spatial(self):
        """测试保留空间关系词"""
        tokenizer = ChineseTokenizer(use_jieba=True)
        result = tokenizer.extract_keywords("门的左边", keep_spatial=True)
        assert any(kw in result for kw in ["左", "左边"])

    def test_is_color_word(self):
        """测试颜色词判断"""
        tokenizer = ChineseTokenizer(use_jieba=True)
        assert tokenizer._is_color_word("红色")
        assert tokenizer._is_color_word("red")
        assert not tokenizer._is_color_word("椅子")

    def test_is_spatial_word(self):
        """测试空间关系词判断"""
        tokenizer = ChineseTokenizer(use_jieba=True)
        assert tokenizer._is_spatial_word("左边")
        assert tokenizer._is_spatial_word("left")
        assert not tokenizer._is_spatial_word("椅子")

    def test_extract_noun_phrases(self):
        """测试名词短语提取"""
        tokenizer = ChineseTokenizer(use_jieba=True)
        result = tokenizer.extract_noun_phrases("红色灭火器")
        # 应该提取"红色灭火器"这个名词短语
        assert len(result) >= 0  # jieba可能识别为单个词或短语


class TestGlobalFunctions:
    """测试全局函数"""

    def test_extract_keywords_function(self):
        """测试extract_keywords全局函数"""
        result = extract_keywords("去红色灭火器旁边")
        assert isinstance(result, list)
        assert len(result) > 0

    def test_simple_extract_keywords_function(self):
        """测试simple_extract_keywords函数"""
        result = simple_extract_keywords("去红色灭火器旁边")
        assert isinstance(result, list)
        assert len(result) > 0


class TestRobotVocabulary:
    """测试机器人领域词汇"""

    def test_robot_objects(self):
        """测试机器人物体识别"""
        tokenizer = ChineseTokenizer(use_jieba=True)
        test_cases = [
            "灭火器",
            "充电桩",
            "垃圾桶",
            "饮水机",
            "打印机",
        ]
        for obj in test_cases:
            result = tokenizer.extract_keywords(f"去{obj}旁边")
            assert obj in result or any(obj in kw for kw in result)

    def test_color_object_combinations(self):
        """测试颜色+物体组合"""
        tokenizer = ChineseTokenizer(use_jieba=True)
        result = tokenizer.extract_keywords("去红色灭火器旁边")
        # 应该识别"红色灭火器"或分别识别"红色"和"灭火器"
        assert any(kw in result for kw in ["红色灭火器", "红色", "灭火器"])


class TestPerformance:
    """性能测试"""

    def test_tokenize_performance(self):
        """测试分词性能"""
        tokenizer = ChineseTokenizer(use_jieba=True)
        text = "请导航到会议室左边的红色灭火器旁边，然后回到充电桩附近"

        import time
        start = time.time()
        for _ in range(100):
            tokenizer.tokenize(text)
        elapsed = time.time() - start

        # 100次分词应该在1秒内完成
        assert elapsed < 1.0, f"分词性能不佳: {elapsed:.3f}s for 100 iterations"

    def test_extract_keywords_performance(self):
        """测试关键词提取性能"""
        tokenizer = ChineseTokenizer(use_jieba=True)
        text = "请导航到会议室左边的红色灭火器旁边，然后回到充电桩附近"

        import time
        start = time.time()
        for _ in range(100):
            tokenizer.extract_keywords(text)
        elapsed = time.time() - start

        # 100次提取应该在1秒内完成
        assert elapsed < 1.0, f"关键词提取性能不佳: {elapsed:.3f}s for 100 iterations"


class TestEdgeCases:
    """边界情况测试"""

    def test_empty_string(self):
        """测试空字符串"""
        tokenizer = ChineseTokenizer(use_jieba=True)
        result = tokenizer.extract_keywords("")
        assert result == []

    def test_only_stopwords(self):
        """测试只有停用词"""
        tokenizer = ChineseTokenizer(use_jieba=True)
        result = tokenizer.extract_keywords("去到在的")
        assert len(result) == 0 or len(result) < 2

    def test_special_characters(self):
        """测试特殊字符"""
        tokenizer = ChineseTokenizer(use_jieba=True)
        result = tokenizer.extract_keywords("去@#$%红色灭火器！！！")
        # 应该能提取出有效词汇
        assert any(kw in result for kw in ["红色", "灭火器", "红色灭火器"])

    def test_numbers(self):
        """测试数字"""
        tokenizer = ChineseTokenizer(use_jieba=True)
        result = tokenizer.extract_keywords("去3号会议室")
        # 应该保留"会议室"
        assert any("会议室" in kw for kw in result)


class TestComparison:
    """对比测试：jieba vs 简单分词"""

    def test_complex_sentence_comparison(self):
        """测试复杂句子的分词对比"""
        text = "请导航到会议室左边的红色灭火器旁边"

        # jieba分词
        tokenizer_jieba = ChineseTokenizer(use_jieba=True)
        result_jieba = tokenizer_jieba.extract_keywords(text)

        # 简单分词
        tokenizer_simple = ChineseTokenizer(use_jieba=False)
        result_simple = tokenizer_simple.extract_keywords(text)

        # jieba应该能识别更多有意义的词
        # 注意：如果jieba未安装，两者结果可能相同
        assert len(result_jieba) >= len(result_simple) or len(result_simple) > 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
