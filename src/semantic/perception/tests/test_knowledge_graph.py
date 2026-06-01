"""
test_knowledge_graph.py — 工业知识图谱单元测试

覆盖:
  - lookup() — 精确/子串/concept_id 匹配
  - get_safety_level() / check_safety() / is_dangerous()
  - get_affordances() / is_graspable() / query_by_affordance()
  - enrich_object_properties() — known vs unknown
  - get_typical_locations() / get_category_members()
  - to_json() / register_concept() / register_constraint()
"""

import json
import unittest

from memory.knowledge.knowledge_graph import (
    AffordanceType,
    IndustrialKnowledgeGraph,
    ObjectConcept,
    RelationType,
    SafetyConstraint,
    SafetyLevel,
)


class TestKGInit(unittest.TestCase):
    """KG 初始化测试。"""

    def setUp(self):
        self.kg = IndustrialKnowledgeGraph()

    def test_builds_with_concepts(self):
        """初始化后应包含至少30个概念。"""
        concepts = self.kg.get_all_concepts()
        self.assertGreater(len(concepts), 30)

    def test_has_safety_category(self):
        """应包含 safety 类别。"""
        members = self.kg.get_category_members("safety")
        self.assertGreater(len(members), 0)

    def test_has_furniture_category(self):
        members = self.kg.get_category_members("furniture")
        self.assertGreater(len(members), 0)

    def test_clip_vocabulary_nonempty(self):
        vocab = self.kg.get_clip_vocabulary()
        self.assertIsInstance(vocab, list)
        self.assertGreater(len(vocab), 50)


class TestKGLookup(unittest.TestCase):
    """lookup() 匹配逻辑测试。"""

    def setUp(self):
        self.kg = IndustrialKnowledgeGraph()

    def test_exact_english(self):
        concept = self.kg.lookup("fire extinguisher")
        self.assertIsNotNone(concept)
        self.assertEqual(concept.concept_id, "fire_extinguisher")

    def test_exact_chinese(self):
        concept = self.kg.lookup("灭火器")
        self.assertIsNotNone(concept)
        self.assertEqual(concept.concept_id, "fire_extinguisher")

    def test_case_insensitive(self):
        concept = self.kg.lookup("FIRE EXTINGUISHER")
        self.assertIsNotNone(concept)
        self.assertEqual(concept.concept_id, "fire_extinguisher")

    def test_concept_id_direct(self):
        concept = self.kg.lookup("fire_extinguisher")
        self.assertIsNotNone(concept)
        self.assertEqual(concept.concept_id, "fire_extinguisher")

    def test_substring_chinese(self):
        """子串匹配: '干粉灭火器' 包含 '灭火器'。"""
        concept = self.kg.lookup("干粉灭火器")
        self.assertIsNotNone(concept)

    def test_unknown_returns_none(self):
        concept = self.kg.lookup("completely_unknown_xyz_123")
        self.assertIsNone(concept)

    def test_empty_string_returns_none(self):
        concept = self.kg.lookup("")
        self.assertIsNone(concept)

    def test_door_lookup(self):
        concept = self.kg.lookup("door")
        self.assertIsNotNone(concept)

    def test_chair_lookup(self):
        concept = self.kg.lookup("chair")
        self.assertIsNotNone(concept)


class TestKGSafetyLevel(unittest.TestCase):
    """安全等级查询测试。"""

    def setUp(self):
        self.kg = IndustrialKnowledgeGraph()

    def test_fire_extinguisher_is_caution(self):
        level = self.kg.get_safety_level("fire extinguisher")
        self.assertEqual(level, SafetyLevel.CAUTION)

    def test_unknown_defaults_to_safe(self):
        level = self.kg.get_safety_level("totally_unknown_object_xyz")
        self.assertEqual(level, SafetyLevel.SAFE)

    def test_is_dangerous_for_electrical_panel(self):
        self.assertTrue(self.kg.is_dangerous("electrical_panel"))

    def test_is_dangerous_for_fire_extinguisher_false(self):
        """灭火器是 CAUTION 级别, 不是 DANGEROUS/FORBIDDEN。"""
        self.assertFalse(self.kg.is_dangerous("fire extinguisher"))

    def test_is_dangerous_for_unknown_false(self):
        self.assertFalse(self.kg.is_dangerous("xyz_unknown"))


class TestKGAffordances(unittest.TestCase):
    """可供性查询测试。"""

    def setUp(self):
        self.kg = IndustrialKnowledgeGraph()

    def test_fire_extinguisher_is_graspable(self):
        self.assertTrue(self.kg.is_graspable("fire extinguisher"))

    def test_fire_extinguisher_affordances(self):
        affordances = self.kg.get_affordances("fire extinguisher")
        self.assertIn(AffordanceType.GRASPABLE, affordances)
        self.assertIn(AffordanceType.INSPECTABLE, affordances)

    def test_unknown_affordances_empty(self):
        affordances = self.kg.get_affordances("unknown_xyz")
        self.assertEqual(affordances, set())

    def test_door_is_openable(self):
        affordances = self.kg.get_affordances("door")
        self.assertIn(AffordanceType.OPENABLE, affordances)
        self.assertIn(AffordanceType.PASSABLE, affordances)

    def test_chair_is_sittable(self):
        affordances = self.kg.get_affordances("chair")
        self.assertIn(AffordanceType.SITTABLE, affordances)

    def test_query_by_affordance_graspable(self):
        graspables = self.kg.query_by_affordance(AffordanceType.GRASPABLE)
        self.assertGreater(len(graspables), 0)
        for c in graspables:
            self.assertIn(AffordanceType.GRASPABLE, c.affordances)

    def test_query_by_affordance_passable(self):
        passables = self.kg.query_by_affordance(AffordanceType.PASSABLE)
        labels = [c.concept_id for c in passables]
        self.assertIn("door", labels)


class TestKGSafetyConstraints(unittest.TestCase):
    """安全约束检查测试。"""

    def setUp(self):
        self.kg = IndustrialKnowledgeGraph()

    def test_electrical_panel_blocks_all(self):
        constraint = self.kg.check_safety("electrical_panel", "pick")
        self.assertIsNotNone(constraint)
        self.assertEqual(constraint.response, "block")

    def test_electrical_panel_blocks_approach(self):
        constraint = self.kg.check_safety("electrical_panel", "approach")
        self.assertIsNotNone(constraint)

    def test_gas_cylinder_pick_blocked(self):
        constraint = self.kg.check_safety("gas_cylinder", "pick")
        self.assertIsNotNone(constraint)
        self.assertEqual(constraint.response, "block")

    def test_gas_cylinder_approach_warns(self):
        constraint = self.kg.check_safety("gas_cylinder", "approach")
        self.assertIsNotNone(constraint)
        # approach の response は limit_speed or warn
        self.assertIn(constraint.response, ("warn", "limit_speed", "block"))

    def test_unknown_object_no_constraint(self):
        constraint = self.kg.check_safety("banana_xyz_unknown", "pick")
        self.assertIsNone(constraint)

    def test_safe_object_no_constraint(self):
        """椅子 (safe) 应该没有安全约束。"""
        constraint = self.kg.check_safety("chair", "pick")
        self.assertIsNone(constraint)


class TestKGLocationsAndCategories(unittest.TestCase):
    """典型位置 + 类别查询测试。"""

    def setUp(self):
        self.kg = IndustrialKnowledgeGraph()

    def test_fire_extinguisher_locations(self):
        locs = self.kg.get_typical_locations("fire extinguisher")
        self.assertGreater(len(locs), 0)
        # 应包含走廊相关
        locs_lower = [l.lower() for l in locs]
        self.assertTrue(any("corridor" in l or "走廊" in l for l in locs_lower))

    def test_unknown_locations_empty(self):
        locs = self.kg.get_typical_locations("xyz_unknown")
        self.assertEqual(locs, [])

    def test_safety_category(self):
        members = self.kg.get_category_members("safety")
        ids = [c.concept_id for c in members]
        self.assertIn("fire_extinguisher", ids)

    def test_unknown_category_empty(self):
        members = self.kg.get_category_members("nonexistent_category")
        self.assertEqual(members, [])


class TestKGEnrichment(unittest.TestCase):
    """enrich_object_properties() 测试。"""

    def setUp(self):
        self.kg = IndustrialKnowledgeGraph()

    def test_known_object_enriched(self):
        props = self.kg.enrich_object_properties("fire extinguisher")
        self.assertTrue(props["kg_matched"])
        self.assertEqual(props["concept_id"], "fire_extinguisher")
        self.assertIn("safety_level", props)
        self.assertIn("affordances", props)
        self.assertIn("category", props)

    def test_unknown_object_not_matched(self):
        props = self.kg.enrich_object_properties("xyz_unknown_object")
        self.assertFalse(props["kg_matched"])
        self.assertEqual(props["category"], "unknown")

    def test_enriched_affordances_list(self):
        props = self.kg.enrich_object_properties("fire extinguisher")
        self.assertIsInstance(props["affordances"], list)
        self.assertIn("graspable", props["affordances"])


class TestKGJSON(unittest.TestCase):
    """JSON 导出测试。"""

    def setUp(self):
        self.kg = IndustrialKnowledgeGraph()

    def test_to_json_valid(self):
        json_str = self.kg.to_json()
        data = json.loads(json_str)  # 不抛异常则通过
        self.assertIn("concepts", data)
        self.assertIn("relations", data)
        self.assertIn("safety_constraints", data)

    def test_to_json_contains_fire_extinguisher(self):
        data = json.loads(self.kg.to_json())
        self.assertIn("fire_extinguisher", data["concepts"])

    def test_to_json_has_safety_constraints(self):
        data = json.loads(self.kg.to_json())
        self.assertGreater(len(data["safety_constraints"]), 0)


class TestKGDynamic(unittest.TestCase):
    """动态注册测试。"""

    def setUp(self):
        self.kg = IndustrialKnowledgeGraph()

    def test_register_new_concept(self):
        new_concept = ObjectConcept(
            concept_id="test_robot_arm",
            names_zh=["机械臂"],
            names_en=["robot arm", "robotic arm"],
            category="industrial",
            safety_level=SafetyLevel.CAUTION,
            affordances={AffordanceType.GRASPABLE},
        )
        self.kg.register_concept(new_concept)
        found = self.kg.lookup("robot arm")
        self.assertIsNotNone(found)
        self.assertEqual(found.concept_id, "test_robot_arm")

    def test_register_concept_by_chinese(self):
        new_concept = ObjectConcept(
            concept_id="test_sensor",
            names_zh=["测试传感器"],
            names_en=["test sensor"],
            category="electronics",
        )
        self.kg.register_concept(new_concept)
        found = self.kg.lookup("测试传感器")
        self.assertIsNotNone(found)

    def test_register_constraint(self):
        constraint = SafetyConstraint(
            constraint_id="SC_TEST_001",
            constraint_type="factual",
            concept_id="fire_extinguisher",
            action="throw",
            condition="投掷",
            response="block",
            message_zh="禁止投掷灭火器",
            message_en="Do not throw fire extinguisher",
        )
        self.kg.register_constraint(constraint)
        result = self.kg.check_safety("fire extinguisher", "throw")
        self.assertIsNotNone(result)
        self.assertEqual(result.constraint_id, "SC_TEST_001")

    def test_get_relations(self):
        relations = self.kg.get_relations("fire_extinguisher")
        self.assertGreater(len(relations), 0)
        for r in relations:
            self.assertEqual(r.source, "fire_extinguisher")


class TestEnumValues(unittest.TestCase):
    """枚举值测试。"""

    def test_safety_levels(self):
        self.assertEqual(SafetyLevel.SAFE.value, "safe")
        self.assertEqual(SafetyLevel.CAUTION.value, "caution")
        self.assertEqual(SafetyLevel.DANGEROUS.value, "dangerous")
        self.assertEqual(SafetyLevel.FORBIDDEN.value, "forbidden")

    def test_affordance_values(self):
        self.assertEqual(AffordanceType.GRASPABLE.value, "graspable")
        self.assertEqual(AffordanceType.OPENABLE.value, "openable")
        self.assertEqual(AffordanceType.PASSABLE.value, "passable")

    def test_relation_types(self):
        self.assertEqual(RelationType.IS_A.value, "is_a")
        self.assertEqual(RelationType.DANGEROUS_IF.value, "dangerous_if")


if __name__ == "__main__":
    unittest.main()
