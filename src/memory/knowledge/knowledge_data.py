"""
kg_data.py — 工业知识图谱静态数据定义。

纯数据文件: 所有硬编码的概念 (ObjectConcept)、关系 (KGRelation)、
安全约束 (SafetyConstraint) 在此定义，不含任何业务逻辑。

由 knowledge_graph.IndustrialKnowledgeGraph._build_industrial_knowledge() 调用。
"""


from .knowledge_graph import (
    AffordanceType,
    KGRelation,
    ObjectConcept,
    RelationType,
    SafetyConstraint,
    SafetyLevel,
)

# ════════════════════════════════════════════════════════════════
#  公共入口
# ════════════════════════════════════════════════════════════════

def build_all_knowledge(
    concepts: dict[str, ObjectConcept],
    relations: list[KGRelation],
    safety_constraints: list[SafetyConstraint],
) -> None:
    """
    填充工业/室内/户外/医疗/居住全场景知识库。

    Args:
        concepts: concept_id → ObjectConcept 字典 (原地修改)
        relations: KGRelation 列表 (原地追加)
        safety_constraints: SafetyConstraint 列表 (原地追加)
    """
    _build_safety_equipment(concepts)
    _build_furniture(concepts)
    _build_electronics(concepts)
    _build_structure(concepts)
    _build_utility(concepts)
    _build_industrial_specific(concepts)
    _build_medical(concepts)
    _build_outdoor(concepts)
    _build_residential(concepts)
    _build_industrial_extended(concepts)
    _build_relations(relations)
    _build_safety_constraints(safety_constraints)


# ════════════════════════════════════════════════════════════════
#  安全设备
# ════════════════════════════════════════════════════════════════

def _build_safety_equipment(concepts: dict[str, ObjectConcept]) -> None:
    concepts["fire_extinguisher"] = ObjectConcept(
        concept_id="fire_extinguisher",
        names_zh=["灭火器", "干粉灭火器", "消防灭火器", "灭火瓶", "CO2灭火器"],
        names_en=["fire extinguisher", "extinguisher", "ABC extinguisher"],
        category="safety",
        safety_level=SafetyLevel.CAUTION,
        affordances={AffordanceType.GRASPABLE, AffordanceType.INSPECTABLE},
        typical_locations=["走廊", "楼梯间", "corridor", "hallway", "stairwell", "lobby"],
        weight_kg=(2.0, 8.0),
        size_class="medium",
        properties={
            "color": "red", "material": "steel_cylinder",
            "shape": "cylindrical", "mounting": "wall_bracket/floor_stand",
            "height_cm": "35-60", "diameter_cm": "10-18",
            "inspection_period": "monthly",
            "grasp_hint": "grip_handle_top, lift_from_bracket",
            "has_pressure_gauge": "yes",
            "typical_count_per_floor": "2-6",
        },
        safety_notes=["操作前检查压力表", "拔保险销后才能使用", "指向火源根部喷射"],
        clip_aliases=["red cylinder on wall", "fire safety equipment",
                      "red metal bottle with nozzle", "wall-mounted red cylinder"],
    )
    concepts["fire_alarm"] = ObjectConcept(
        concept_id="fire_alarm",
        names_zh=["火灾报警器", "烟感", "烟雾报警器", "消防报警", "烟感探测器", "感烟探头"],
        names_en=["fire alarm", "smoke detector", "smoke alarm", "heat detector"],
        category="safety",
        safety_level=SafetyLevel.CAUTION,
        affordances={AffordanceType.INSPECTABLE, AffordanceType.SWITCHABLE},
        typical_locations=["天花板", "走廊", "ceiling", "corridor"],
        weight_kg=(0.1, 0.5),
        size_class="small",
        properties={
            "color": "white", "material": "plastic",
            "shape": "circular_disc", "mounting": "ceiling_flush",
            "diameter_cm": "10-15", "height_cm": "3-5",
            "power": "battery/wired", "has_led_indicator": "yes",
            "typical_spacing_m": "6-10",
        },
        clip_aliases=["white device on ceiling", "round detector on ceiling",
                      "small white disc on ceiling"],
    )
    concepts["emergency_exit"] = ObjectConcept(
        concept_id="emergency_exit",
        names_zh=["安全出口", "紧急出口", "逃生出口", "消防通道"],
        names_en=["emergency exit", "fire exit", "safety exit", "exit sign"],
        category="safety",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.PASSABLE, AffordanceType.INSPECTABLE, AffordanceType.READABLE},
        typical_locations=["走廊尽头", "楼梯间", "end of corridor", "stairwell"],
        size_class="fixed",
        properties={
            "color": "green", "material": "plastic_acrylic/LED",
            "shape": "rectangular_sign", "illuminated": "yes",
            "status": "always_open", "mounting": "above_door/wall",
            "height_cm": "15-30", "width_cm": "30-60",
            "topology_role": "egress_point",
            "navigation_hint": "escape_route_marker",
        },
        clip_aliases=["green exit sign", "glowing exit sign above door",
                      "illuminated emergency exit sign"],
    )
    concepts["first_aid_kit"] = ObjectConcept(
        concept_id="first_aid_kit",
        names_zh=["急救箱", "急救包", "医药箱"],
        names_en=["first aid kit", "medical kit", "first aid box"],
        category="safety",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.GRASPABLE, AffordanceType.OPENABLE, AffordanceType.INSPECTABLE},
        typical_locations=["走廊", "办公室", "corridor", "office"],
        weight_kg=(1.0, 5.0),
        size_class="small",
        properties={
            "color": "white/red", "material": "plastic/metal",
            "shape": "rectangular_box", "mounting": "wall_bracket/shelf",
            "height_cm": "15-30", "width_cm": "20-40", "depth_cm": "10-20",
            "marking": "red_cross_symbol", "inspection_period": "quarterly",
            "grasp_hint": "grip_handle_top_or_side",
        },
        clip_aliases=["white box with red cross", "first aid cabinet on wall"],
    )
    concepts["fire_hydrant"] = ObjectConcept(
        concept_id="fire_hydrant",
        names_zh=["消防栓", "消火栓", "消防水龙头"],
        names_en=["fire hydrant", "fire hose cabinet"],
        category="safety",
        safety_level=SafetyLevel.CAUTION,
        affordances={AffordanceType.OPENABLE, AffordanceType.INSPECTABLE},
        typical_locations=["走廊", "楼梯间", "corridor", "stairwell"],
        weight_kg=(5.0, 20.0),
        size_class="large",
        properties={
            "color": "red", "material": "metal/glass_door",
            "shape": "rectangular_cabinet", "mounting": "wall_recessed",
            "height_cm": "60-80", "width_cm": "50-70", "depth_cm": "20-30",
            "has_hose": "yes", "has_valve": "yes",
            "marking": "消火栓 / FIRE HOSE",
            "inspection_period": "monthly",
        },
        safety_notes=["需专业培训后操作", "检查水压和软管完好"],
        clip_aliases=["red cabinet on wall", "fire hose box",
                      "red glass-door cabinet with hose"],
    )
    concepts["safety_sign"] = ObjectConcept(
        concept_id="safety_sign",
        names_zh=["安全标识", "警示牌", "安全标志", "警告标识", "禁止标识"],
        names_en=["safety sign", "warning sign", "caution sign", "prohibition sign"],
        category="safety",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.READABLE, AffordanceType.INSPECTABLE},
        typical_locations=["走廊", "工厂", "corridor", "factory"],
        size_class="small",
        properties={
            "color": "yellow/red/blue/green", "material": "plastic/metal/sticker",
            "shape": "triangular/circular/rectangular", "mounting": "wall/post",
            "height_cm": "20-40", "width_cm": "20-40",
            "sign_types": "warning/prohibition/mandatory/information",
            "navigation_hint": "hazard_zone_boundary_indicator",
        },
        clip_aliases=["yellow warning sign", "safety notice on wall",
                      "red prohibition sign", "blue mandatory sign"],
    )


# ════════════════════════════════════════════════════════════════
#  家具
# ════════════════════════════════════════════════════════════════

def _build_furniture(concepts: dict[str, ObjectConcept]) -> None:
    concepts["chair"] = ObjectConcept(
        concept_id="chair",
        names_zh=["椅子", "办公椅", "座椅", "凳子", "转椅", "工位椅"],
        names_en=["chair", "office chair", "seat", "stool", "swivel chair", "desk chair"],
        category="furniture",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.SITTABLE, AffordanceType.PUSHABLE, AffordanceType.GRASPABLE},
        typical_locations=["办公室", "会议室", "office", "meeting_room"],
        weight_kg=(3.0, 15.0),
        size_class="medium",
        properties={
            "color": "black/grey/blue", "material": "fabric/mesh/leather+metal",
            "shape": "seated_with_backrest", "has_wheels": "usually",
            "height_cm": "80-120", "width_cm": "45-65", "depth_cm": "45-60",
            "grasp_hint": "push_backrest_or_armrest",
            "movable": "yes",
        },
        clip_aliases=["office chair with wheels", "black swivel chair",
                      "mesh back office chair"],
    )
    concepts["desk"] = ObjectConcept(
        concept_id="desk",
        names_zh=["桌子", "办公桌", "工作台", "书桌", "台面", "电脑桌"],
        names_en=["desk", "table", "workstation", "work table", "computer desk"],
        category="furniture",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.SUPPORTIVE, AffordanceType.INSPECTABLE},
        typical_locations=["办公室", "office"],
        weight_kg=(10.0, 50.0),
        size_class="large",
        properties={
            "color": "white/brown/grey", "material": "wood/MDF/metal_frame",
            "shape": "rectangular_flat_surface", "has_drawers": "sometimes",
            "height_cm": "70-80", "width_cm": "100-180", "depth_cm": "60-80",
            "surface_objects": "monitor, keyboard, mouse, cup, phone",
            "movable": "difficult",
        },
        clip_aliases=["office desk with monitor", "wooden table",
                      "white desk with computer", "standing desk"],
    )
    concepts["cabinet"] = ObjectConcept(
        concept_id="cabinet",
        names_zh=["柜子", "文件柜", "储物柜", "橱柜", "衣柜"],
        names_en=["cabinet", "file cabinet", "storage cabinet", "locker", "cupboard"],
        category="furniture",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.OPENABLE, AffordanceType.CLOSABLE, AffordanceType.CONTAINABLE},
        typical_locations=["办公室", "储物间", "office", "storage"],
        weight_kg=(15.0, 80.0),
        size_class="large",
        properties={
            "color": "grey/white/wood", "material": "metal/wood/MDF",
            "shape": "rectangular_tall_box", "has_lock": "sometimes",
            "height_cm": "80-200", "width_cm": "40-120", "depth_cm": "40-60",
            "has_doors": "yes", "has_drawers": "sometimes",
            "movable": "difficult",
        },
        clip_aliases=["metal file cabinet", "tall storage cabinet",
                      "wooden cupboard with doors"],
    )
    concepts["shelf"] = ObjectConcept(
        concept_id="shelf",
        names_zh=["架子", "书架", "货架", "置物架", "展示架"],
        names_en=["shelf", "bookshelf", "rack", "shelving", "display shelf"],
        category="furniture",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.SUPPORTIVE, AffordanceType.INSPECTABLE},
        typical_locations=["储物间", "办公室", "storage", "office"],
        weight_kg=(5.0, 40.0),
        size_class="large",
        properties={
            "color": "grey/brown/black", "material": "metal/wood",
            "shape": "open_frame_with_shelves", "has_back_panel": "sometimes",
            "height_cm": "80-200", "width_cm": "60-120", "depth_cm": "30-50",
            "shelf_count": "3-6", "tip_over_risk": "if_not_anchored",
            "movable": "sometimes",
        },
        clip_aliases=["metal shelf with items", "bookshelf against wall",
                      "industrial shelving unit"],
    )
    concepts["sofa"] = ObjectConcept(
        concept_id="sofa",
        names_zh=["沙发", "长椅", "休息椅", "双人沙发"],
        names_en=["sofa", "couch", "bench", "lounge", "loveseat"],
        category="furniture",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.SITTABLE},
        typical_locations=["大厅", "休息区", "lobby", "lounge"],
        weight_kg=(20.0, 80.0),
        size_class="large",
        properties={
            "color": "black/brown/grey/blue", "material": "leather/fabric/faux_leather",
            "shape": "low_wide_cushioned", "has_armrest": "yes",
            "height_cm": "70-90", "width_cm": "120-240", "depth_cm": "70-100",
            "seat_count": "2-4", "movable": "difficult",
        },
        clip_aliases=["leather sofa", "cushioned couch",
                      "office lounge sofa"],
    )
    concepts["whiteboard"] = ObjectConcept(
        concept_id="whiteboard",
        names_zh=["白板", "写字板", "展示板", "磁性白板"],
        names_en=["whiteboard", "board", "display board", "dry-erase board"],
        category="furniture",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.READABLE, AffordanceType.INSPECTABLE},
        typical_locations=["会议室", "办公室", "meeting_room", "office"],
        size_class="large",
        properties={
            "color": "white", "material": "enamel/melamine_on_board",
            "shape": "rectangular_flat_panel", "mounting": "wall/stand",
            "height_cm": "90-120", "width_cm": "120-240",
            "has_tray": "usually", "has_magnets": "sometimes",
        },
        clip_aliases=["white board on wall", "dry erase board",
                      "whiteboard with markers"],
    )


# ════════════════════════════════════════════════════════════════
#  电子设备
# ════════════════════════════════════════════════════════════════

def _build_electronics(concepts: dict[str, ObjectConcept]) -> None:
    concepts["monitor"] = ObjectConcept(
        concept_id="monitor",
        names_zh=["显示器", "屏幕", "电脑屏幕", "监视器"],
        names_en=["monitor", "screen", "display", "computer screen"],
        category="electronics",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.READABLE, AffordanceType.SWITCHABLE, AffordanceType.INSPECTABLE},
        typical_locations=["办公室", "监控室", "office", "control_room"],
        weight_kg=(2.0, 10.0),
        size_class="medium",
        properties={
            "color": "black/silver", "material": "plastic/metal_frame+LCD",
            "shape": "thin_rectangular_screen", "mounting": "desk_stand/VESA_arm",
            "height_cm": "30-50", "width_cm": "50-70", "depth_cm": "5-15",
            "diagonal_inch": "21-34", "has_power_led": "yes",
            "fragile": "screen_breakable",
        },
        clip_aliases=["computer monitor on desk", "flat screen display",
                      "dual monitor setup"],
    )
    concepts["computer"] = ObjectConcept(
        concept_id="computer",
        names_zh=["电脑", "计算机", "主机", "台式机", "笔记本"],
        names_en=["computer", "PC", "laptop", "desktop", "workstation"],
        category="electronics",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.SWITCHABLE, AffordanceType.READABLE},
        typical_locations=["办公室", "office"],
        weight_kg=(1.5, 15.0),
        size_class="medium",
        properties={
            "color": "black/silver/grey", "material": "metal/plastic",
            "shape": "tower_or_laptop", "has_cables": "yes",
            "height_cm": "3-45", "width_cm": "15-50",
        },
        clip_aliases=["desktop computer tower", "laptop on desk",
                      "PC workstation"],
    )
    concepts["projector"] = ObjectConcept(
        concept_id="projector",
        names_zh=["投影仪", "投影机", "幕布投影"],
        names_en=["projector", "beamer"],
        category="electronics",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.SWITCHABLE, AffordanceType.INSPECTABLE},
        typical_locations=["会议室", "meeting_room"],
        weight_kg=(2.0, 8.0),
        size_class="medium",
        properties={
            "color": "white/black", "material": "plastic/metal",
            "shape": "compact_box", "mounting": "ceiling/table",
            "height_cm": "10-15", "width_cm": "25-35", "depth_cm": "20-30",
            "has_lens": "yes", "has_fan_noise": "yes",
        },
        clip_aliases=["ceiling mounted projector", "white projector in meeting room"],
    )
    concepts["printer"] = ObjectConcept(
        concept_id="printer",
        names_zh=["打印机", "复印机", "一体机"],
        names_en=["printer", "copier", "multifunction printer"],
        category="electronics",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.SWITCHABLE, AffordanceType.INSPECTABLE},
        typical_locations=["办公室", "打印间", "office"],
        weight_kg=(5.0, 30.0),
        size_class="medium",
        properties={
            "color": "white/grey/black", "material": "plastic",
            "shape": "boxy_with_tray", "has_paper_tray": "yes",
            "height_cm": "20-50", "width_cm": "40-60", "depth_cm": "35-55",
            "has_display": "sometimes",
        },
        clip_aliases=["office printer on table", "multifunction copier",
                      "laser printer"],
    )
    concepts["electrical_panel"] = ObjectConcept(
        concept_id="electrical_panel",
        names_zh=["配电箱", "配电柜", "电箱", "电力开关箱", "强电箱", "弱电箱"],
        names_en=["electrical panel", "power panel", "breaker box", "fuse box",
                   "distribution board", "switchboard"],
        category="electronics",
        safety_level=SafetyLevel.FORBIDDEN,
        affordances={AffordanceType.INSPECTABLE},
        typical_locations=["走廊", "设备间", "corridor", "utility_room"],
        weight_kg=(10.0, 100.0),
        size_class="large",
        properties={
            "color": "grey/beige", "material": "sheet_metal",
            "shape": "rectangular_box", "mounting": "wall_recessed/wall_surface",
            "voltage": "220V/380V", "has_door": "yes",
            "height_cm": "40-120", "width_cm": "30-80", "depth_cm": "15-30",
            "danger_zone_m": "1.0",
            "marking": "⚡ high voltage warning sign",
            "inspection_period": "annual",
        },
        safety_notes=["高压危险, 严禁非专业人员触碰", "保持箱前1m无障碍物",
                      "检查: 外观完好/无烧焦/指示灯正常"],
        clip_aliases=["grey metal box on wall", "electrical distribution box",
                      "panel with warning sign", "metal cabinet with lock on wall"],
    )
    concepts["server_rack"] = ObjectConcept(
        concept_id="server_rack",
        names_zh=["服务器机柜", "机架", "网络机柜", "数据柜"],
        names_en=["server rack", "network rack", "IT cabinet", "data cabinet"],
        category="electronics",
        safety_level=SafetyLevel.DANGEROUS,
        affordances={AffordanceType.INSPECTABLE},
        typical_locations=["机房", "server_room"],
        weight_kg=(50.0, 500.0),
        size_class="large",
        properties={
            "color": "black/grey", "material": "steel",
            "shape": "tall_rectangular_cabinet", "has_glass_door": "sometimes",
            "height_cm": "180-210", "width_cm": "60-80", "depth_cm": "80-120",
            "has_led_indicators": "yes", "has_fans": "yes",
            "thermal_output": "high", "noise_level": "moderate_to_high",
            "danger_zone_m": "0.5",
        },
        safety_notes=["内部线缆复杂, 勿触碰", "注意散热环境温度",
                      "UPS 供电: 断电后仍有电流"],
        clip_aliases=["tall black server cabinet", "rack with blinking lights",
                      "data center server rack"],
    )
    concepts["television"] = ObjectConcept(
        concept_id="television",
        names_zh=["电视", "电视机", "大屏", "显示屏"],
        names_en=["television", "TV", "flat screen TV", "display screen"],
        category="electronics",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.SWITCHABLE, AffordanceType.READABLE},
        typical_locations=["客厅", "会议室", "living_room", "meeting_room"],
        weight_kg=(5.0, 30.0),
        size_class="large",
        properties={
            "color": "black_screen/silver_bezel", "material": "plastic+glass_panel",
            "shape": "flat_rectangular_screen",
            "diagonal_inch": "32-85",
            "height_cm": "45-110", "width_cm": "75-190",
            "mounting": "wall/stand",
            "lidar_behavior": "flat_reflective_surface",
        },
        clip_aliases=["flat screen TV on wall", "large television screen",
                      "wall mounted flat TV"],
    )


# ════════════════════════════════════════════════════════════════
#  建筑结构
# ════════════════════════════════════════════════════════════════

def _build_structure(concepts: dict[str, ObjectConcept]) -> None:
    concepts["door"] = ObjectConcept(
        concept_id="door",
        names_zh=["门", "房门", "大门", "玻璃门", "防火门", "推拉门", "卷帘门"],
        names_en=["door", "gate", "entrance", "glass door", "fire door",
                   "sliding door", "roller shutter"],
        category="structure",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.OPENABLE, AffordanceType.CLOSABLE, AffordanceType.PASSABLE},
        typical_locations=["走廊", "办公室", "corridor", "office"],
        size_class="fixed",
        properties={
            "color": "white/brown/grey/transparent",
            "material": "wood/glass/metal/composite",
            "shape": "rectangular_panel",
            "type": "hinged/sliding/revolving/automatic",
            "height_cm": "200-220", "width_cm": "80-120",
            "has_handle": "usually", "has_window": "sometimes",
            "passable_width_cm": "80-100",
            "navigation_hint": "room_boundary_indicator",
            "topology_role": "connects_rooms",
        },
        clip_aliases=["wooden door", "glass door with handle",
                      "white door in hallway", "automatic sliding door"],
    )
    concepts["window"] = ObjectConcept(
        concept_id="window",
        names_zh=["窗户", "窗", "玻璃窗", "落地窗", "百叶窗"],
        names_en=["window", "glass window", "floor-to-ceiling window", "blind"],
        category="structure",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.OPENABLE, AffordanceType.CLOSABLE},
        typical_locations=["办公室", "走廊", "office", "corridor"],
        size_class="fixed",
        properties={
            "color": "transparent/white_frame", "material": "glass/aluminum_frame",
            "shape": "rectangular", "type": "sliding/casement/fixed",
            "height_cm": "100-250", "width_cm": "60-200",
            "navigation_hint": "wall_boundary_indicator",
            "lidar_behavior": "transparent_to_lidar_if_clean_glass",
        },
        clip_aliases=["glass window in wall", "office window with blinds"],
    )
    concepts["stairs"] = ObjectConcept(
        concept_id="stairs",
        names_zh=["楼梯", "台阶", "阶梯", "楼梯间"],
        names_en=["stairs", "staircase", "stairwell", "steps"],
        category="structure",
        safety_level=SafetyLevel.CAUTION,
        affordances={AffordanceType.CLIMBABLE, AffordanceType.PASSABLE},
        typical_locations=["楼梯间", "stairwell"],
        size_class="fixed",
        properties={
            "material": "concrete/metal/wood",
            "shape": "ascending_steps", "has_railing": "usually",
            "step_height_cm": "15-20", "step_depth_cm": "25-30",
            "width_cm": "90-150",
            "gait_mode": "stair_climb",
            "topology_role": "floor_transition",
            "navigation_hint": "switch_gait_before_traversal",
        },
        safety_notes=["四足机器人上下楼梯需切换步态模式",
                      "下行比上行更危险, 降低速度"],
        clip_aliases=["concrete staircase", "indoor stairs with railing",
                      "metal staircase with handrail"],
    )
    concepts["elevator"] = ObjectConcept(
        concept_id="elevator",
        names_zh=["电梯", "升降梯", "货梯", "客梯"],
        names_en=["elevator", "lift", "freight elevator", "passenger elevator"],
        category="structure",
        safety_level=SafetyLevel.CAUTION,
        affordances={AffordanceType.PASSABLE},
        typical_locations=["大厅", "走廊", "lobby", "corridor"],
        size_class="fixed",
        properties={
            "color": "silver/grey", "material": "stainless_steel/glass",
            "shape": "enclosed_cabin",
            "door_type": "sliding_double",
            "door_width_cm": "80-120", "cabin_depth_cm": "120-200",
            "has_buttons": "yes", "has_indicator": "floor_display",
            "topology_role": "floor_transition",
            "navigation_hint": "requires_human_assist_or_IoT_control",
            "load_capacity_kg": "630-2000",
        },
        safety_notes=["需人工协助开门", "注意载重限制",
                      "缝隙可能卡住腿部关节"],
        clip_aliases=["elevator door", "metal elevator entrance",
                      "stainless steel elevator doors"],
    )
    concepts["railing"] = ObjectConcept(
        concept_id="railing",
        names_zh=["栏杆", "扶手", "护栏", "楼梯扶手"],
        names_en=["railing", "handrail", "guardrail", "banister"],
        category="structure",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.INSPECTABLE},
        typical_locations=["楼梯间", "阳台", "stairwell", "balcony"],
        size_class="fixed",
        properties={
            "material": "metal/wood/glass", "shape": "horizontal_bar+vertical_posts",
            "height_cm": "90-110", "navigation_hint": "edge_boundary",
        },
        clip_aliases=["metal handrail on stairs", "glass railing on balcony"],
    )
    concepts["pillar"] = ObjectConcept(
        concept_id="pillar",
        names_zh=["柱子", "立柱", "支柱", "承重柱"],
        names_en=["pillar", "column", "post", "structural column"],
        category="structure",
        safety_level=SafetyLevel.SAFE,
        affordances=set(),
        typical_locations=["大厅", "走廊", "lobby", "corridor"],
        size_class="fixed",
        properties={
            "material": "concrete/steel/stone",
            "shape": "cylindrical/rectangular_vertical",
            "diameter_cm": "30-80",
            "navigation_hint": "static_obstacle_landmark",
            "lidar_behavior": "strong_reflector_useful_for_localization",
        },
        clip_aliases=["concrete column in hallway", "round pillar in lobby"],
    )
    concepts["sink"] = ObjectConcept(
        concept_id="sink",
        names_zh=["水槽", "洗手台", "水池", "洗碗池", "洗手盆"],
        names_en=["sink", "washbasin", "wash basin", "kitchen sink", "hand wash basin"],
        category="structure",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.INSPECTABLE},
        typical_locations=["厨房", "卫生间", "kitchen", "bathroom"],
        size_class="fixed",
        properties={
            "color": "white/stainless_steel", "material": "ceramic/stainless_steel",
            "shape": "basin_with_faucet",
            "width_cm": "40-80", "depth_cm": "35-55", "height_cm": "80-90",
            "has_faucet": "yes", "has_mirror_above": "if_bathroom",
        },
        clip_aliases=["stainless steel sink", "bathroom sink with faucet",
                      "white ceramic wash basin"],
    )
    concepts["toilet"] = ObjectConcept(
        concept_id="toilet",
        names_zh=["马桶", "坐便器", "厕所", "智能马桶"],
        names_en=["toilet", "lavatory", "WC", "smart toilet"],
        category="structure",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.INSPECTABLE},
        typical_locations=["卫生间", "bathroom", "restroom"],
        size_class="medium",
        properties={
            "color": "white", "material": "porcelain/ceramic",
            "shape": "seated_bowl",
            "height_cm": "35-45_seat", "width_cm": "35-40", "depth_cm": "60-75",
            "has_tank": "usually", "has_lid": "yes",
        },
        clip_aliases=["white porcelain toilet", "bathroom toilet",
                      "modern toilet with tank"],
    )
    concepts["mirror"] = ObjectConcept(
        concept_id="mirror",
        names_zh=["镜子", "穿衣镜", "浴室镜", "落地镜"],
        names_en=["mirror", "wall mirror", "bathroom mirror", "full-length mirror"],
        category="structure",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.INSPECTABLE, AffordanceType.READABLE},
        typical_locations=["卫生间", "走廊", "bathroom", "corridor"],
        size_class="medium",
        properties={
            "color": "reflective_silver", "material": "glass+metal_frame",
            "shape": "flat_rectangular/oval",
            "height_cm": "40-180", "width_cm": "30-100",
            "mounting": "wall/stand",
            "lidar_behavior": "specular_reflector_may_cause_ghost_points",
            "navigation_hint": "may_confuse_lidar_with_reflections",
        },
        clip_aliases=["rectangular mirror on wall", "bathroom mirror",
                      "full length mirror on stand"],
    )
    concepts["fire_door"] = ObjectConcept(
        concept_id="fire_door",
        names_zh=["防火门", "消防门", "甲级防火门"],
        names_en=["fire door", "fire-rated door", "emergency door"],
        category="structure",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.OPENABLE, AffordanceType.CLOSABLE, AffordanceType.PASSABLE},
        typical_locations=["楼梯间", "走廊", "stairwell", "corridor"],
        size_class="fixed",
        properties={
            "color": "grey/red", "material": "steel_core",
            "shape": "heavy_rectangular_door",
            "height_cm": "200-220", "width_cm": "90-120",
            "has_closer": "yes_auto_close",
            "has_push_bar": "usually",
            "fire_rating": "A_class_1.5h",
            "should_be_closed": "yes",
            "topology_role": "fire_compartment_boundary",
            "navigation_hint": "heavy_door_may_need_force",
        },
        safety_notes=["防火门须保持关闭", "推杆开启, 不可锁闭"],
        clip_aliases=["grey fire door with closer", "heavy metal door in corridor",
                      "fire door with push bar"],
    )


# ════════════════════════════════════════════════════════════════
#  日用品
# ════════════════════════════════════════════════════════════════

def _build_utility(concepts: dict[str, ObjectConcept]) -> None:
    concepts["trash_bin"] = ObjectConcept(
        concept_id="trash_bin",
        names_zh=["垃圾桶", "垃圾箱", "废纸篓", "分类垃圾桶"],
        names_en=["trash bin", "trash can", "garbage bin", "waste bin", "dustbin",
                   "recycling bin"],
        category="utility",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.CONTAINABLE, AffordanceType.PUSHABLE},
        typical_locations=["办公室", "走廊", "office", "corridor"],
        weight_kg=(0.5, 5.0),
        size_class="medium",
        properties={
            "color": "grey/green/blue/black", "material": "plastic/metal",
            "shape": "cylindrical_or_rectangular",
            "height_cm": "30-80", "diameter_cm": "25-50",
            "has_lid": "sometimes", "has_pedal": "sometimes",
            "navigation_hint": "minor_obstacle_pushable",
        },
        clip_aliases=["grey trash can", "plastic waste bin",
                      "stainless steel garbage bin", "pedal bin"],
    )
    concepts["bottle"] = ObjectConcept(
        concept_id="bottle",
        names_zh=["水瓶", "瓶子", "矿泉水", "饮料瓶", "保温杯"],
        names_en=["bottle", "water bottle", "drink", "thermos"],
        category="utility",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.GRASPABLE},
        typical_locations=["桌子上", "办公室", "desk", "office"],
        weight_kg=(0.1, 1.5),
        size_class="small",
        properties={
            "color": "transparent/blue/white", "material": "plastic/glass/stainless_steel",
            "shape": "cylindrical", "height_cm": "15-30", "diameter_cm": "5-8",
            "grasp_hint": "wrap_grip_body, pinch_cap",
            "grasp_aperture_cm": "5-8",
            "fragile": "if_glass", "may_contain_liquid": "yes",
        },
        clip_aliases=["plastic water bottle", "transparent bottle on table",
                      "stainless steel thermos"],
    )
    concepts["cup"] = ObjectConcept(
        concept_id="cup",
        names_zh=["杯子", "茶杯", "咖啡杯", "马克杯", "水杯", "纸杯"],
        names_en=["cup", "mug", "coffee cup", "glass", "paper cup"],
        category="utility",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.GRASPABLE},
        typical_locations=["桌子上", "厨房", "desk", "kitchen"],
        weight_kg=(0.1, 0.5),
        size_class="small",
        properties={
            "color": "white/various", "material": "ceramic/glass/paper/plastic",
            "shape": "cylindrical_with_handle",
            "height_cm": "8-12", "diameter_cm": "7-10",
            "grasp_hint": "hook_handle or wrap_grip_body",
            "grasp_aperture_cm": "7-10",
            "fragile": "if_ceramic_or_glass", "may_contain_liquid": "yes",
            "orientation_sensitive": "yes_upright_only",
        },
        clip_aliases=["ceramic mug", "coffee cup on desk",
                      "white mug with handle", "disposable paper cup"],
    )
    concepts["box"] = ObjectConcept(
        concept_id="box",
        names_zh=["箱子", "纸箱", "包裹", "盒子", "快递"],
        names_en=["box", "carton", "package", "cardboard box", "parcel"],
        category="utility",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.GRASPABLE, AffordanceType.CONTAINABLE, AffordanceType.PUSHABLE},
        typical_locations=["储物间", "走廊", "storage", "corridor"],
        weight_kg=(0.5, 30.0),
        size_class="medium",
        properties={
            "color": "brown/white/grey", "material": "cardboard/plastic/wood",
            "shape": "rectangular_cube",
            "height_cm": "10-60", "width_cm": "15-60", "depth_cm": "15-60",
            "grasp_hint": "grip_sides_or_bottom_edges",
            "stackable": "yes", "may_be_sealed": "yes",
            "contents_unknown": "yes",
        },
        clip_aliases=["cardboard box on floor", "brown package",
                      "sealed cardboard carton", "delivery box"],
    )
    concepts["lamp"] = ObjectConcept(
        concept_id="lamp",
        names_zh=["灯", "台灯", "落地灯", "吊灯", "顶灯"],
        names_en=["lamp", "desk lamp", "floor lamp", "light", "ceiling light",
                   "pendant light"],
        category="utility",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.SWITCHABLE},
        typical_locations=["桌子上", "办公室", "desk", "office"],
        weight_kg=(0.5, 5.0),
        size_class="small",
        properties={
            "color": "white/black/silver", "material": "metal/plastic",
            "shape": "arm_with_shade/pendant",
            "type": "desk/floor/ceiling/pendant",
            "height_cm": "30-180", "base_diameter_cm": "10-30",
            "power": "LED/fluorescent/incandescent",
            "navigation_hint": "lighting_landmark",
        },
        clip_aliases=["desk lamp on table", "floor standing lamp",
                      "modern ceiling pendant light"],
    )
    concepts["refrigerator"] = ObjectConcept(
        concept_id="refrigerator",
        names_zh=["冰箱", "冷藏柜", "小冰箱", "冷冻柜"],
        names_en=["refrigerator", "fridge", "mini fridge", "freezer"],
        category="utility",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.OPENABLE, AffordanceType.CLOSABLE, AffordanceType.CONTAINABLE},
        typical_locations=["厨房", "休息区", "kitchen", "lounge"],
        weight_kg=(30.0, 80.0),
        size_class="large",
        properties={
            "color": "white/silver/black", "material": "sheet_metal_painted",
            "shape": "tall_rectangular_cabinet",
            "height_cm": "80-190", "width_cm": "50-70", "depth_cm": "55-70",
            "has_door_handle": "yes", "door_swing": "left_or_right",
            "noise_level": "low_humming",
            "navigation_hint": "kitchen_landmark",
        },
        clip_aliases=["white refrigerator in kitchen", "stainless steel fridge",
                      "mini fridge under desk"],
    )
    concepts["plant"] = ObjectConcept(
        concept_id="plant",
        names_zh=["植物", "盆栽", "绿植", "花盆", "仙人掌"],
        names_en=["plant", "potted plant", "flower pot", "greenery", "succulent"],
        category="utility",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.INSPECTABLE},
        typical_locations=["走廊", "大厅", "办公室", "corridor", "lobby"],
        weight_kg=(0.5, 20.0),
        size_class="medium",
        properties={
            "color": "green_foliage/various_pot", "material": "organic+ceramic/plastic_pot",
            "shape": "organic_foliage_on_pot",
            "height_cm": "15-150", "pot_diameter_cm": "10-50",
            "fragile": "pot_may_break", "movable": "if_small",
            "navigation_hint": "decoration_landmark",
        },
        clip_aliases=["green potted plant", "indoor plant in pot",
                      "large floor plant", "small succulent on desk"],
    )
    concepts["backpack"] = ObjectConcept(
        concept_id="backpack",
        names_zh=["背包", "书包", "双肩包", "旅行包"],
        names_en=["backpack", "bag", "rucksack", "school bag"],
        category="utility",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.GRASPABLE, AffordanceType.CONTAINABLE},
        typical_locations=["桌子旁", "椅子旁", "地面", "desk", "chair", "floor"],
        weight_kg=(0.5, 10.0),
        size_class="small",
        properties={
            "color": "black/grey/blue/red", "material": "nylon/polyester/canvas",
            "shape": "soft_rectangular",
            "height_cm": "35-55", "width_cm": "25-35", "depth_cm": "15-25",
            "grasp_hint": "grip_top_handle_or_shoulder_strap",
            "contents_unknown": "yes",
            "navigation_hint": "temporary_floor_obstacle",
        },
        clip_aliases=["black backpack on floor", "school bag next to chair",
                      "travel backpack"],
    )
    concepts["person"] = ObjectConcept(
        concept_id="person",
        names_zh=["人", "行人", "工人", "员工", "访客"],
        names_en=["person", "human", "pedestrian", "worker", "visitor"],
        category="dynamic",
        safety_level=SafetyLevel.CAUTION,
        affordances={AffordanceType.INSPECTABLE},
        typical_locations=["任何位置", "anywhere"],
        weight_kg=(30.0, 120.0),
        size_class="medium",
        properties={
            "shape": "upright_bipedal",
            "height_cm": "100-200", "width_cm": "40-60",
            "dynamic": "yes_unpredictable_motion",
            "velocity_mps": "0-2.0",
            "may_change_direction": "yes",
            "social_distance_m": "0.5-1.5",
            "navigation_hint": "dynamic_obstacle_highest_priority_yield",
        },
        safety_notes=["最高优先级避让", "保持社交距离 ≥ 0.5m",
                      "尾随跟踪需用户授权"],
        clip_aliases=["person walking", "standing human", "worker in uniform",
                      "person sitting"],
    )
    concepts["water_dispenser"] = ObjectConcept(
        concept_id="water_dispenser",
        names_zh=["饮水机", "净水器", "开水器", "热水机"],
        names_en=["water dispenser", "water cooler", "hot water dispenser"],
        category="utility",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.SWITCHABLE, AffordanceType.INSPECTABLE},
        typical_locations=["茶水间", "走廊", "break_room", "corridor"],
        weight_kg=(10.0, 30.0),
        size_class="medium",
        properties={
            "color": "white/grey", "material": "plastic+stainless_steel",
            "shape": "tall_unit_with_taps",
            "height_cm": "90-130", "width_cm": "30-40", "depth_cm": "30-40",
            "has_hot_water": "yes_caution",
            "has_cold_water": "usually",
        },
        clip_aliases=["water dispenser in break room", "tall white water cooler",
                      "office water dispenser"],
    )
    concepts["umbrella_stand"] = ObjectConcept(
        concept_id="umbrella_stand",
        names_zh=["伞架", "雨伞架", "伞桶"],
        names_en=["umbrella stand", "umbrella holder"],
        category="utility",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.CONTAINABLE, AffordanceType.INSPECTABLE},
        typical_locations=["入口", "大厅", "entrance", "lobby"],
        weight_kg=(1.0, 5.0),
        size_class="small",
        properties={
            "color": "grey/black", "material": "metal/plastic",
            "shape": "cylindrical_bucket",
            "height_cm": "40-60", "diameter_cm": "20-30",
            "navigation_hint": "entrance_landmark",
        },
        clip_aliases=["umbrella stand at entrance", "metal umbrella holder"],
    )
    concepts["vending_machine"] = ObjectConcept(
        concept_id="vending_machine",
        names_zh=["自动售货机", "售卖机", "贩卖机", "自助售货"],
        names_en=["vending machine", "snack machine", "beverage machine"],
        category="utility",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.INSPECTABLE, AffordanceType.SWITCHABLE},
        typical_locations=["走廊", "大厅", "休息区", "corridor", "lobby"],
        weight_kg=(100.0, 300.0),
        size_class="large",
        properties={
            "color": "varies_with_branding", "material": "sheet_metal+glass_front",
            "shape": "tall_rectangular_cabinet",
            "height_cm": "170-190", "width_cm": "70-100", "depth_cm": "60-85",
            "has_glass_front": "yes", "has_coin_slot": "yes",
            "has_display": "sometimes",
            "navigation_hint": "corridor_landmark",
        },
        clip_aliases=["vending machine in hallway", "snack vending machine",
                      "beverage machine with glass front"],
    )
    concepts["washing_machine"] = ObjectConcept(
        concept_id="washing_machine",
        names_zh=["洗衣机", "滚筒洗衣机", "波轮洗衣机"],
        names_en=["washing machine", "washer", "laundry machine",
                   "front-loading washer", "top-loading washer"],
        category="utility",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.SWITCHABLE, AffordanceType.OPENABLE, AffordanceType.INSPECTABLE},
        typical_locations=["洗衣房", "浴室", "laundry", "bathroom"],
        weight_kg=(30.0, 80.0),
        size_class="large",
        properties={
            "color": "white", "material": "sheet_metal+plastic",
            "shape": "cubic_box",
            "height_cm": "85-90", "width_cm": "55-65", "depth_cm": "55-65",
            "has_door": "yes_front_or_top",
            "vibration_when_running": "yes",
        },
        clip_aliases=["white washing machine", "front-loading washer",
                      "washing machine in laundry room"],
    )
    concepts["microwave"] = ObjectConcept(
        concept_id="microwave",
        names_zh=["微波炉", "微波", "微波加热器"],
        names_en=["microwave", "microwave oven"],
        category="utility",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.SWITCHABLE, AffordanceType.OPENABLE, AffordanceType.INSPECTABLE},
        typical_locations=["厨房", "茶水间", "kitchen", "break_room"],
        weight_kg=(8.0, 20.0),
        size_class="medium",
        properties={
            "color": "white/black/silver", "material": "metal+plastic",
            "shape": "compact_box_with_door",
            "height_cm": "25-35", "width_cm": "40-55", "depth_cm": "30-40",
            "has_turntable": "yes", "has_timer_display": "yes",
            "placement": "countertop",
        },
        clip_aliases=["microwave on counter", "kitchen microwave",
                      "white microwave oven"],
    )


# ════════════════════════════════════════════════════════════════
#  工业特有
# ════════════════════════════════════════════════════════════════

def _build_industrial_specific(concepts: dict[str, ObjectConcept]) -> None:
    concepts["forklift"] = ObjectConcept(
        concept_id="forklift",
        names_zh=["叉车", "铲车", "堆高车", "电动叉车"],
        names_en=["forklift", "pallet jack", "lift truck", "electric forklift"],
        category="industrial",
        safety_level=SafetyLevel.DANGEROUS,
        affordances={AffordanceType.INSPECTABLE},
        typical_locations=["仓库", "工厂", "warehouse", "factory"],
        weight_kg=(1000.0, 5000.0),
        size_class="large",
        properties={
            "color": "yellow/orange/blue", "material": "steel_body",
            "shape": "vehicle_with_forks",
            "length_cm": "200-350", "width_cm": "100-150", "height_cm": "180-250",
            "has_flashing_light": "yes", "has_horn": "yes",
            "danger_zone_m": "3.0", "max_speed_kmh": "15-25",
            "dynamic": "yes_may_be_moving",
            "navigation_hint": "dynamic_large_obstacle_avoid",
        },
        safety_notes=["机器人需保持至少3m安全距离", "运行中勿接近",
                      "注意盲区: 叉车后方视野受限"],
        clip_aliases=["yellow forklift", "industrial lift truck",
                      "forklift carrying pallet"],
    )
    concepts["gas_cylinder"] = ObjectConcept(
        concept_id="gas_cylinder",
        names_zh=["气瓶", "氧气瓶", "乙炔瓶", "钢瓶", "气罐", "液化气罐"],
        names_en=["gas cylinder", "oxygen tank", "compressed gas", "gas bottle",
                   "propane tank", "acetylene cylinder"],
        category="industrial",
        safety_level=SafetyLevel.FORBIDDEN,
        affordances={AffordanceType.INSPECTABLE},
        typical_locations=["仓库", "实验室", "warehouse", "lab"],
        weight_kg=(5.0, 80.0),
        size_class="medium",
        properties={
            "color": "green/blue/grey/red_by_gas_type", "material": "steel_seamless",
            "shape": "tall_cylinder_with_valve_top",
            "height_cm": "60-150", "diameter_cm": "15-35",
            "pressure_bar": "150-300",
            "has_cap": "should_be_on", "has_chain": "should_be_chained_to_wall",
            "danger_zone_m": "1.5",
            "color_coding": "green=oxygen, red=acetylene, grey=CO2, blue=argon",
        },
        safety_notes=["高压易爆, 严禁碰撞或倾倒", "远离火源和高温",
                      "检查: 瓶帽/链条/阀门/压力表"],
        clip_aliases=["tall metal gas cylinder", "green/blue pressurized tank",
                      "chained gas bottle against wall"],
    )
    concepts["conveyor"] = ObjectConcept(
        concept_id="conveyor",
        names_zh=["传送带", "输送带", "皮带线", "辊道"],
        names_en=["conveyor", "conveyor belt", "belt line", "roller conveyor"],
        category="industrial",
        safety_level=SafetyLevel.DANGEROUS,
        affordances={AffordanceType.INSPECTABLE},
        typical_locations=["工厂", "仓库", "factory", "warehouse"],
        size_class="large",
        properties={
            "color": "grey/black_belt", "material": "steel_frame+rubber_belt",
            "shape": "long_horizontal_belt",
            "height_cm": "70-100", "width_cm": "40-120",
            "speed_mps": "0.5-2.0", "dynamic": "yes_when_running",
            "danger_zone_m": "2.0",
            "pinch_points": "belt_rollers_and_edges",
        },
        safety_notes=["运行中禁止靠近, 有卷入风险",
                      "夹点在滚筒两端"],
        clip_aliases=["moving belt with items", "industrial conveyor line",
                      "roller conveyor with packages"],
    )
    concepts["pallet"] = ObjectConcept(
        concept_id="pallet",
        names_zh=["托盘", "栈板", "卡板", "塑料托盘"],
        names_en=["pallet", "wooden pallet", "plastic pallet", "euro pallet"],
        category="industrial",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.SUPPORTIVE, AffordanceType.INSPECTABLE},
        typical_locations=["仓库", "warehouse"],
        weight_kg=(10.0, 30.0),
        size_class="large",
        properties={
            "color": "brown/blue", "material": "wood/plastic",
            "shape": "flat_platform_with_slots",
            "height_cm": "12-15", "width_cm": "80-100", "depth_cm": "100-120",
            "stackable": "yes", "may_have_goods_on_top": "yes",
            "standard": "EUR_1200x800 / US_1219x1016",
            "navigation_hint": "floor_obstacle_often_stacked",
        },
        clip_aliases=["wooden pallet on floor", "stacked pallets",
                      "blue plastic pallet"],
    )
    concepts["fire_blanket"] = ObjectConcept(
        concept_id="fire_blanket",
        names_zh=["灭火毯", "防火毯", "阻燃毯"],
        names_en=["fire blanket", "fire suppression blanket"],
        category="safety",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.GRASPABLE, AffordanceType.INSPECTABLE},
        typical_locations=["厨房", "实验室", "kitchen", "lab"],
        weight_kg=(0.5, 2.0),
        size_class="small",
        properties={
            "color": "red_container/white_blanket", "material": "fiberglass",
            "shape": "folded_in_wall_pouch",
            "height_cm": "30-40", "width_cm": "20-30",
            "mounting": "wall_pouch", "deployment": "pull_tabs_down",
            "grasp_hint": "pull_tab_downward_to_deploy",
        },
        clip_aliases=["red fire blanket container on wall",
                      "white fire blanket pouch"],
    )
    concepts["aed"] = ObjectConcept(
        concept_id="aed",
        names_zh=["AED", "自动体外除颤器", "除颤仪", "心脏除颤器"],
        names_en=["AED", "automated external defibrillator", "defibrillator"],
        category="safety",
        safety_level=SafetyLevel.CAUTION,
        affordances={AffordanceType.GRASPABLE, AffordanceType.INSPECTABLE},
        typical_locations=["大厅", "走廊", "lobby", "corridor"],
        weight_kg=(1.5, 4.0),
        size_class="small",
        properties={
            "color": "green/yellow_with_heart_symbol",
            "material": "plastic_case",
            "shape": "small_briefcase",
            "height_cm": "25-35", "width_cm": "20-30", "depth_cm": "8-12",
            "mounting": "wall_cabinet/shelf",
            "has_voice_instruction": "yes",
            "grasp_hint": "open_cabinet_then_grab_handle",
        },
        safety_notes=["仅紧急情况下使用", "检查电池和电极片有效期",
                      "设备有语音引导, 非专业人员也可操作"],
        clip_aliases=["green AED box on wall", "heart defibrillator cabinet",
                      "yellow AED case with heart logo"],
    )
    concepts["robot_charging_station"] = ObjectConcept(
        concept_id="robot_charging_station",
        names_zh=["充电桩", "充电站", "机器人充电座", "对接充电桩"],
        names_en=["charging station", "charging dock", "robot charger",
                   "auto-dock station"],
        category="industrial",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.INSPECTABLE},
        typical_locations=["走廊", "大厅", "corridor", "lobby"],
        size_class="medium",
        properties={
            "color": "grey/black", "material": "metal/plastic",
            "shape": "flat_base_with_contacts",
            "height_cm": "10-30", "width_cm": "40-60", "depth_cm": "40-60",
            "connector": "auto-dock_magnetic/pin",
            "has_led_indicator": "yes",
            "navigation_hint": "home_base_for_autonomous_return",
        },
        clip_aliases=["robot docking station", "charging platform on floor",
                      "flat charging base with LED"],
    )


# ════════════════════════════════════════════════════════════════
#  医疗设备
# ════════════════════════════════════════════════════════════════

def _build_medical(concepts: dict[str, ObjectConcept]) -> None:
    concepts["wheelchair"] = ObjectConcept(
        concept_id="wheelchair",
        names_zh=["轮椅", "手动轮椅", "电动轮椅"],
        names_en=["wheelchair", "manual wheelchair", "electric wheelchair"],
        category="medical",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.PUSHABLE, AffordanceType.INSPECTABLE},
        typical_locations=["走廊", "大厅", "corridor", "lobby", "hospital"],
        weight_kg=(10.0, 30.0),
        size_class="medium",
        properties={
            "color": "black/blue_frame", "material": "aluminum_frame+fabric_seat",
            "shape": "seated_with_large_wheels",
            "width_cm": "60-75", "depth_cm": "80-110", "height_cm": "80-100",
            "wheel_diameter_cm": "55-60_rear, 15-20_front",
            "foldable": "yes_most_models", "has_brakes": "yes",
            "may_be_occupied": "yes",
            "navigation_hint": "yield_right_of_way_if_occupied",
        },
        clip_aliases=["wheelchair in hallway", "folding wheelchair",
                      "blue manual wheelchair"],
    )
    concepts["stretcher"] = ObjectConcept(
        concept_id="stretcher",
        names_zh=["担架", "病床", "推床", "急救床"],
        names_en=["stretcher", "gurney", "hospital bed", "medical bed"],
        category="medical",
        safety_level=SafetyLevel.CAUTION,
        affordances={AffordanceType.PUSHABLE, AffordanceType.INSPECTABLE},
        typical_locations=["走廊", "急诊室", "corridor", "emergency_room"],
        weight_kg=(20.0, 80.0),
        size_class="large",
        properties={
            "color": "white/grey/silver", "material": "stainless_steel_frame+mattress",
            "shape": "flat_bed_on_wheels",
            "length_cm": "180-210", "width_cm": "60-80", "height_cm": "60-100",
            "has_side_rails": "yes", "has_iv_pole": "sometimes",
            "has_wheels": "yes_lockable",
            "may_be_occupied": "yes_high_probability",
            "navigation_hint": "high_priority_yield_and_slow_approach",
        },
        safety_notes=["可能有病人在上面, 轻缓靠近",
                      "勿碰触 IV 管线和监护设备"],
        clip_aliases=["hospital gurney", "wheeled medical bed",
                      "hospital bed on wheels in corridor"],
    )
    concepts["medicine_cabinet"] = ObjectConcept(
        concept_id="medicine_cabinet",
        names_zh=["药柜", "药品柜", "医药柜"],
        names_en=["medicine cabinet", "pharmacy cabinet", "drug cabinet"],
        category="medical",
        safety_level=SafetyLevel.CAUTION,
        affordances={AffordanceType.OPENABLE, AffordanceType.INSPECTABLE},
        typical_locations=["诊室", "药房", "clinic", "pharmacy"],
        weight_kg=(15.0, 60.0),
        size_class="large",
        properties={
            "color": "white/light_grey", "material": "metal_with_glass_door",
            "shape": "tall_cabinet_with_shelves",
            "height_cm": "150-200", "width_cm": "60-100", "depth_cm": "30-50",
            "has_lock": "yes_always", "has_glass_door": "usually",
            "requires_authorization": "yes",
        },
        safety_notes=["药品管控, 需授权访问",
                      "未经许可禁止打开"],
        clip_aliases=["glass medicine cabinet", "pharmacy shelf",
                      "locked white cabinet with medicines"],
    )
    concepts["oxygen_supply"] = ObjectConcept(
        concept_id="oxygen_supply",
        names_zh=["制氧机", "氧气供应器", "医用氧气瓶"],
        names_en=["oxygen concentrator", "oxygen supply", "medical oxygen"],
        category="medical",
        safety_level=SafetyLevel.CAUTION,
        affordances={AffordanceType.INSPECTABLE},
        typical_locations=["病房", "急诊室", "ward", "icu"],
        weight_kg=(5.0, 25.0),
        size_class="medium",
        properties={
            "color": "white/green", "material": "plastic_casing/steel_cylinder",
            "shape": "box_or_cylinder_on_cart",
            "height_cm": "40-80", "width_cm": "30-50",
            "has_flow_meter": "yes", "has_tubing": "yes",
            "flammable_environment": "oxygen_enriched",
        },
        safety_notes=["易燃环境远离明火",
                      "高浓度氧气: 禁止火源/火花"],
        clip_aliases=["medical oxygen equipment", "portable oxygen concentrator",
                      "green oxygen cylinder on cart"],
    )


# ════════════════════════════════════════════════════════════════
#  户外 / 园区
# ════════════════════════════════════════════════════════════════

def _build_outdoor(concepts: dict[str, ObjectConcept]) -> None:
    concepts["tree"] = ObjectConcept(
        concept_id="tree",
        names_zh=["树", "树木", "行道树", "绿化树"],
        names_en=["tree", "street tree", "garden tree"],
        category="outdoor",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.INSPECTABLE},
        typical_locations=["室外", "园区", "outdoor", "garden", "parking"],
        size_class="fixed",
        properties={
            "color": "green_canopy/brown_trunk", "material": "organic_wood",
            "shape": "trunk_with_canopy", "height_cm": "200-1500",
            "trunk_diameter_cm": "10-100",
            "lidar_behavior": "strong_reflector_at_trunk, scattered_at_canopy",
            "navigation_hint": "static_obstacle_landmark_outdoor",
            "seasonal_change": "deciduous_may_lose_leaves",
        },
        clip_aliases=["green tree outdoor", "tree with leaves",
                      "large tree with trunk", "street tree along road"],
    )
    concepts["bench_outdoor"] = ObjectConcept(
        concept_id="bench_outdoor",
        names_zh=["公园长椅", "户外长椅", "石凳", "园区座椅"],
        names_en=["park bench", "outdoor bench", "garden bench", "stone seat"],
        category="outdoor",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.SITTABLE, AffordanceType.INSPECTABLE},
        typical_locations=["园区", "公园", "outdoor", "garden"],
        size_class="large",
        properties={
            "color": "brown/grey/green", "material": "wood+metal/stone/concrete",
            "shape": "long_seat_with_backrest",
            "length_cm": "120-200", "depth_cm": "40-55", "height_cm": "70-90",
            "seat_height_cm": "40-45",
            "navigation_hint": "rest_area_landmark",
        },
        clip_aliases=["wooden park bench", "stone bench outdoor",
                      "metal framed garden bench"],
    )
    concepts["traffic_cone"] = ObjectConcept(
        concept_id="traffic_cone",
        names_zh=["路锥", "锥桶", "反光锥", "警示锥"],
        names_en=["traffic cone", "safety cone", "pylon", "road cone"],
        category="outdoor",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.GRASPABLE, AffordanceType.PUSHABLE, AffordanceType.INSPECTABLE},
        typical_locations=["停车场", "道路", "parking", "road", "construction"],
        weight_kg=(1.0, 5.0),
        size_class="small",
        properties={
            "color": "orange+white_reflective", "material": "PVC/rubber",
            "shape": "conical",
            "height_cm": "45-75", "base_diameter_cm": "25-40",
            "has_reflective_stripe": "yes",
            "grasp_hint": "grip_body_middle",
            "navigation_hint": "temporary_obstacle_boundary_marker",
            "semantic_cue": "construction_or_restricted_zone",
        },
        clip_aliases=["orange traffic cone", "reflective safety cone",
                      "road cone with reflective stripe"],
    )
    concepts["fence"] = ObjectConcept(
        concept_id="fence",
        names_zh=["围栏", "栅栏", "护栏", "围墙", "铁丝网"],
        names_en=["fence", "barrier", "railing", "wall fence", "chain link fence"],
        category="outdoor",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.INSPECTABLE},
        typical_locations=["园区", "停车场", "outdoor", "parking"],
        size_class="fixed",
        properties={
            "color": "silver/green/black", "material": "metal/wire/wood",
            "shape": "vertical_posts_with_infill",
            "height_cm": "90-200",
            "gap_size_cm": "varies_5-15",
            "lidar_behavior": "thin_wires_may_be_invisible_to_lidar",
            "navigation_hint": "boundary_impassable",
        },
        clip_aliases=["metal fence", "chain link fence",
                      "green fence around area"],
    )
    concepts["street_light"] = ObjectConcept(
        concept_id="street_light",
        names_zh=["路灯", "灯柱", "照明灯", "园区灯"],
        names_en=["street light", "lamp post", "outdoor light", "pole light"],
        category="outdoor",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.INSPECTABLE},
        typical_locations=["室外", "道路", "停车场", "outdoor", "road"],
        size_class="fixed",
        properties={
            "color": "grey/black_pole", "material": "steel/aluminum_pole",
            "shape": "tall_pole_with_light_head",
            "height_cm": "300-1000", "pole_diameter_cm": "10-25",
            "has_sensor": "sometimes_photocell",
            "lidar_behavior": "strong_vertical_reflector",
            "navigation_hint": "outdoor_landmark_and_obstacle",
        },
        clip_aliases=["tall street lamp", "outdoor light pole",
                      "modern LED street light"],
    )
    concepts["fire_hydrant_outdoor"] = ObjectConcept(
        concept_id="fire_hydrant_outdoor",
        names_zh=["室外消防栓", "地上消防栓", "马路消防栓"],
        names_en=["outdoor fire hydrant", "street hydrant"],
        category="outdoor",
        safety_level=SafetyLevel.CAUTION,
        affordances={AffordanceType.INSPECTABLE},
        typical_locations=["室外", "道路旁", "outdoor", "roadside"],
        size_class="medium",
        properties={
            "color": "red/yellow", "material": "cast_iron",
            "shape": "upright_post_with_outlets",
            "height_cm": "50-80", "width_cm": "20-30",
            "has_coupling": "yes", "clearance_required_m": "3.0",
        },
        safety_notes=["消防设施, 禁止遮挡和停放",
                      "保持 3m 通道畅通"],
        clip_aliases=["red fire hydrant on street", "yellow fire hydrant",
                      "red outdoor hydrant on sidewalk"],
    )
    concepts["manhole_cover"] = ObjectConcept(
        concept_id="manhole_cover",
        names_zh=["井盖", "下水道盖", "检查井", "窨井盖"],
        names_en=["manhole cover", "utility cover", "sewer cover", "access cover"],
        category="outdoor",
        safety_level=SafetyLevel.CAUTION,
        affordances={AffordanceType.INSPECTABLE},
        typical_locations=["道路", "人行道", "road", "sidewalk"],
        size_class="medium",
        properties={
            "color": "dark_grey/rusty", "material": "cast_iron/ductile_iron",
            "shape": "circular_disc_flush_with_ground",
            "diameter_cm": "60-80", "thickness_cm": "3-8",
            "weight_kg_approx": "40-80",
            "flush_with_ground": "should_be_yes",
            "danger_if_missing": "fall_hazard",
            "navigation_hint": "ground_texture_change_detectable_by_lidar",
        },
        safety_notes=["缺失或松动的井盖有跌落风险, 标记并绕行",
                      "异响/移动的井盖 → 高风险, 报告"],
        clip_aliases=["round metal cover on ground", "manhole on road",
                      "circular iron cover on pavement"],
    )


# ════════════════════════════════════════════════════════════════
#  居住环境
# ════════════════════════════════════════════════════════════════

def _build_residential(concepts: dict[str, ObjectConcept]) -> None:
    concepts["bed"] = ObjectConcept(
        concept_id="bed",
        names_zh=["床", "单人床", "双人床", "床铺", "上下铺"],
        names_en=["bed", "single bed", "double bed", "bunk bed"],
        category="furniture",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.SUPPORTIVE, AffordanceType.INSPECTABLE},
        typical_locations=["卧室", "宿舍", "bedroom", "dormitory"],
        weight_kg=(30.0, 100.0),
        size_class="large",
        properties={
            "color": "varies_frame+white_sheets", "material": "wood/metal_frame+mattress",
            "shape": "rectangular_flat_elevated",
            "length_cm": "190-210", "width_cm": "90-180", "height_cm": "40-60",
            "may_be_occupied": "yes",
            "navigation_hint": "bedroom_landmark",
        },
        clip_aliases=["bed with pillow and blanket", "mattress on frame",
                      "wooden bed frame with sheets"],
    )


# ════════════════════════════════════════════════════════════════
#  更多工业设备
# ════════════════════════════════════════════════════════════════

def _build_industrial_extended(concepts: dict[str, ObjectConcept]) -> None:
    concepts["valve"] = ObjectConcept(
        concept_id="valve",
        names_zh=["阀门", "截止阀", "球阀", "蝶阀", "手动阀", "安全阀"],
        names_en=["valve", "gate valve", "ball valve", "butterfly valve",
                   "safety valve", "shut-off valve"],
        category="industrial",
        safety_level=SafetyLevel.CAUTION,
        affordances={AffordanceType.SWITCHABLE, AffordanceType.INSPECTABLE},
        typical_locations=["管道间", "设备间", "pipe_room", "utility_room"],
        size_class="small",
        properties={
            "color": "red_handwheel/grey_body", "material": "brass/stainless_steel/cast_iron",
            "shape": "disc_body_with_handwheel_or_lever",
            "diameter_inch": "0.5-12",
            "type": "gate/ball/butterfly/globe/check",
            "state": "open_or_closed",
            "has_handwheel": "usually", "has_lever": "sometimes",
            "turn_direction": "CW_to_close",
        },
        safety_notes=["关闭/开启可能影响系统运行, 需确认",
                      "标签应标明介质和流向"],
        clip_aliases=["industrial valve on pipe", "red handwheel valve",
                      "lever handle ball valve"],
    )
    concepts["pipe"] = ObjectConcept(
        concept_id="pipe",
        names_zh=["管道", "水管", "气管", "暖气管", "排水管"],
        names_en=["pipe", "piping", "water pipe", "gas pipe", "conduit", "drain pipe"],
        category="industrial",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.INSPECTABLE},
        typical_locations=["天花板", "墙壁", "设备间", "ceiling", "wall", "utility_room"],
        size_class="fixed",
        properties={
            "color": "silver/grey/red/blue_by_content",
            "material": "steel/copper/PVC/cast_iron",
            "shape": "cylindrical_long",
            "diameter_cm": "1-30", "mounting": "ceiling/wall/underground",
            "color_coding": "red=fire, blue=water, yellow=gas, green=drain",
            "insulated": "sometimes",
            "lidar_behavior": "linear_feature_useful_for_localization",
        },
        clip_aliases=["metal pipe on ceiling", "industrial piping system",
                      "exposed pipes along wall"],
    )
    concepts["control_panel"] = ObjectConcept(
        concept_id="control_panel",
        names_zh=["控制面板", "操作台", "控制台", "仪表盘", "PLC 面板"],
        names_en=["control panel", "control console", "instrument panel",
                   "dashboard", "PLC panel"],
        category="industrial",
        safety_level=SafetyLevel.DANGEROUS,
        affordances={AffordanceType.SWITCHABLE, AffordanceType.READABLE, AffordanceType.INSPECTABLE},
        typical_locations=["控制室", "工厂", "control_room", "factory"],
        size_class="large",
        properties={
            "color": "grey/beige", "material": "sheet_metal+electronics",
            "shape": "desk_or_wall_panel_with_displays",
            "height_cm": "80-180", "width_cm": "60-200",
            "has_buttons": "yes", "has_displays": "yes",
            "has_emergency_stop": "usually_red_mushroom_button",
            "requires_authorization": "yes",
        },
        safety_notes=["未经授权禁止操作",
                      "红色急停按钮: 紧急情况可按下"],
        clip_aliases=["industrial control panel with buttons", "operator console",
                      "control room dashboard with screens"],
    )
    concepts["crane"] = ObjectConcept(
        concept_id="crane",
        names_zh=["吊车", "起重机", "天车", "行车", "龙门吊"],
        names_en=["crane", "overhead crane", "gantry crane", "hoist",
                   "bridge crane"],
        category="industrial",
        safety_level=SafetyLevel.DANGEROUS,
        affordances={AffordanceType.INSPECTABLE},
        typical_locations=["工厂", "仓库", "factory", "warehouse"],
        size_class="large",
        properties={
            "color": "yellow/blue", "material": "steel_structure",
            "shape": "overhead_beam_with_hoist",
            "span_m": "5-30", "height_m": "5-15",
            "load_capacity_ton": "1-50",
            "danger_zone_m": "5.0",
            "dynamic": "yes_hoist_and_bridge_move",
            "has_warning_light": "yes", "has_horn": "yes",
        },
        safety_notes=["运行中保持5m以上安全距离, 注意吊物下方",
                      "绝不站在吊物正下方"],
        clip_aliases=["overhead crane in factory", "yellow industrial crane",
                      "gantry crane with hook"],
    )
    concepts["generator"] = ObjectConcept(
        concept_id="generator",
        names_zh=["发电机", "柴油发电机", "备用电源", "UPS"],
        names_en=["generator", "diesel generator", "backup power",
                   "standby generator"],
        category="industrial",
        safety_level=SafetyLevel.DANGEROUS,
        affordances={AffordanceType.INSPECTABLE},
        typical_locations=["设备间", "地下室", "utility_room", "basement"],
        weight_kg=(100.0, 5000.0),
        size_class="large",
        properties={
            "color": "green/yellow/grey", "material": "steel_casing+engine",
            "shape": "large_boxy_machine",
            "length_cm": "100-400", "width_cm": "60-180", "height_cm": "80-200",
            "fuel": "diesel/natural_gas",
            "noise_dB": "75-100",
            "danger_zone_m": "3.0",
            "exhaust": "yes_hot_and_toxic",
            "vibration": "significant_when_running",
        },
        safety_notes=["高温/高压/噪声, 运行时禁止靠近",
                      "排气管高温, 勿触碰"],
        clip_aliases=["large diesel generator", "industrial power generator",
                      "green backup generator in room"],
    )
    concepts["hvac_unit"] = ObjectConcept(
        concept_id="hvac_unit",
        names_zh=["空调外机", "新风机组", "暖通设备", "空调机组", "风管"],
        names_en=["HVAC unit", "air conditioning unit", "air handler",
                   "AC outdoor unit", "ductwork"],
        category="industrial",
        safety_level=SafetyLevel.CAUTION,
        affordances={AffordanceType.INSPECTABLE},
        typical_locations=["屋顶", "设备间", "rooftop", "mechanical_room"],
        size_class="large",
        properties={
            "color": "white/grey", "material": "sheet_metal",
            "shape": "boxy_with_fan_grille",
            "height_cm": "50-150", "width_cm": "60-120", "depth_cm": "30-80",
            "has_fan": "yes_rotating", "has_refrigerant": "yes",
            "noise_level": "moderate",
        },
        safety_notes=["旋转部件, 维护时断电",
                      "制冷剂泄漏有毒"],
        clip_aliases=["rooftop HVAC unit", "air conditioning outdoor unit",
                      "white AC unit on wall"],
    )
    concepts["safety_helmet"] = ObjectConcept(
        concept_id="safety_helmet",
        names_zh=["安全帽", "防护帽", "头盔", "工地帽"],
        names_en=["safety helmet", "hard hat", "protective helmet",
                   "construction helmet"],
        category="safety",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.GRASPABLE, AffordanceType.INSPECTABLE},
        typical_locations=["工厂入口", "仓库", "factory_entrance", "warehouse"],
        weight_kg=(0.2, 0.5),
        size_class="small",
        properties={
            "color": "yellow/white/red/blue_by_role",
            "material": "ABS/HDPE_plastic",
            "shape": "dome_with_brim",
            "has_chin_strap": "yes", "has_liner": "yes",
            "color_coding": "white=manager, yellow=worker, red=safety, blue=visitor",
            "grasp_hint": "grip_brim_edge",
        },
        clip_aliases=["yellow hard hat", "white safety helmet",
                      "construction hard hat"],
    )
    concepts["safety_vest"] = ObjectConcept(
        concept_id="safety_vest",
        names_zh=["安全背心", "反光背心", "防护马甲", "高可见度背心"],
        names_en=["safety vest", "high-visibility vest", "reflective vest",
                   "hi-vis vest"],
        category="safety",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.GRASPABLE, AffordanceType.INSPECTABLE},
        typical_locations=["工厂入口", "仓库", "factory_entrance", "warehouse"],
        weight_kg=(0.1, 0.3),
        size_class="small",
        properties={
            "color": "fluorescent_yellow/orange",
            "material": "polyester_mesh+reflective_tape",
            "shape": "vest",
            "has_reflective_strips": "yes",
            "grasp_hint": "pick_from_hook_or_pile",
        },
        clip_aliases=["orange safety vest", "yellow reflective vest",
                      "hi-vis vest hanging on hook"],
    )
    concepts["spill_kit"] = ObjectConcept(
        concept_id="spill_kit",
        names_zh=["泄漏处理包", "应急处理箱", "化学品应急包"],
        names_en=["spill kit", "spill response kit", "chemical spill kit"],
        category="safety",
        safety_level=SafetyLevel.CAUTION,
        affordances={AffordanceType.GRASPABLE, AffordanceType.INSPECTABLE},
        typical_locations=["仓库", "实验室", "工厂", "warehouse", "lab", "factory"],
        weight_kg=(2.0, 15.0),
        size_class="medium",
        properties={
            "color": "yellow/black", "material": "plastic_drum/bag",
            "shape": "barrel_or_bag",
            "height_cm": "40-80", "diameter_cm": "30-50",
            "contents": "absorbent_pads, nitrile_gloves, disposal_bags",
            "grasp_hint": "carry_by_handle_on_top",
        },
        safety_notes=["化学品泄漏时使用, 需培训",
                      "使用后按危废处理"],
        clip_aliases=["yellow spill kit container", "spill response bin",
                      "yellow drum with spill kit label"],
    )
    concepts["eye_wash_station"] = ObjectConcept(
        concept_id="eye_wash_station",
        names_zh=["洗眼器", "冲淋洗眼器", "紧急喷淋", "紧急冲淋站"],
        names_en=["eye wash station", "emergency shower", "safety shower",
                   "emergency eyewash"],
        category="safety",
        safety_level=SafetyLevel.SAFE,
        affordances={AffordanceType.INSPECTABLE, AffordanceType.SWITCHABLE},
        typical_locations=["实验室", "工厂", "化学储存区", "lab", "factory"],
        size_class="medium",
        properties={
            "color": "green/yellow", "material": "stainless_steel/plastic",
            "shape": "pedestal_with_dual_nozzles_or_shower_head",
            "height_cm": "80-200", "width_cm": "30-50",
            "has_foot_pedal": "sometimes", "has_pull_handle": "yes",
            "water_flow_time_min": "15",
            "inspection_period": "weekly",
            "navigation_hint": "emergency_equipment_landmark",
        },
        clip_aliases=["green eye wash station", "emergency shower on wall",
                      "yellow emergency eyewash pedestal"],
    )


# ════════════════════════════════════════════════════════════════
#  关系
# ════════════════════════════════════════════════════════════════

def _build_relations(relations: list[KGRelation]) -> None:
    R = RelationType
    relations.extend([
        # IS_A — 分类层次
        KGRelation("fire_extinguisher", R.IS_A, "safety_equipment"),
        KGRelation("fire_alarm", R.IS_A, "safety_equipment"),
        KGRelation("emergency_exit", R.IS_A, "safety_equipment"),
        KGRelation("first_aid_kit", R.IS_A, "safety_equipment"),
        KGRelation("fire_hydrant", R.IS_A, "safety_equipment"),
        KGRelation("aed", R.IS_A, "safety_equipment"),
        KGRelation("fire_blanket", R.IS_A, "safety_equipment"),
        KGRelation("safety_sign", R.IS_A, "safety_equipment"),
        KGRelation("safety_helmet", R.IS_A, "personal_protective_equipment"),
        KGRelation("safety_vest", R.IS_A, "personal_protective_equipment"),
        KGRelation("spill_kit", R.IS_A, "safety_equipment"),
        KGRelation("eye_wash_station", R.IS_A, "safety_equipment"),
        KGRelation("chair", R.IS_A, "furniture"),
        KGRelation("desk", R.IS_A, "furniture"),
        KGRelation("shelf", R.IS_A, "furniture"),
        KGRelation("cabinet", R.IS_A, "furniture"),
        KGRelation("sofa", R.IS_A, "furniture"),
        KGRelation("bed", R.IS_A, "furniture"),
        KGRelation("door", R.IS_A, "structure"),
        KGRelation("stairs", R.IS_A, "structure"),
        KGRelation("elevator", R.IS_A, "structure"),
        KGRelation("window", R.IS_A, "structure"),
        KGRelation("sink", R.IS_A, "structure"),
        KGRelation("toilet", R.IS_A, "structure"),
        KGRelation("wheelchair", R.IS_A, "medical_equipment"),
        KGRelation("stretcher", R.IS_A, "medical_equipment"),
        KGRelation("medicine_cabinet", R.IS_A, "medical_equipment"),
        KGRelation("forklift", R.IS_A, "industrial_vehicle"),
        KGRelation("crane", R.IS_A, "industrial_machine"),
        KGRelation("conveyor", R.IS_A, "industrial_machine"),
        KGRelation("generator", R.IS_A, "industrial_machine"),
        KGRelation("person", R.IS_A, "dynamic_entity"),
        KGRelation("backpack", R.IS_A, "personal_item"),
        KGRelation("water_dispenser", R.IS_A, "appliance"),
        KGRelation("vending_machine", R.IS_A, "appliance"),
        KGRelation("fire_door", R.IS_A, "structure"),
        KGRelation("umbrella_stand", R.IS_A, "container"),

        # USED_FOR — 功能
        KGRelation("fire_extinguisher", R.USED_FOR, "灭火"),
        KGRelation("fire_alarm", R.USED_FOR, "火灾预警"),
        KGRelation("emergency_exit", R.USED_FOR, "紧急逃生"),
        KGRelation("first_aid_kit", R.USED_FOR, "急救处理"),
        KGRelation("aed", R.USED_FOR, "心脏急救"),
        KGRelation("fire_blanket", R.USED_FOR, "扑灭衣物火焰"),
        KGRelation("spill_kit", R.USED_FOR, "化学品泄漏处理"),
        KGRelation("eye_wash_station", R.USED_FOR, "眼睛冲洗急救"),
        KGRelation("door", R.USED_FOR, "通行"),
        KGRelation("elevator", R.USED_FOR, "垂直运输"),
        KGRelation("chair", R.USED_FOR, "坐"),
        KGRelation("desk", R.USED_FOR, "办公/放置物品"),
        KGRelation("shelf", R.USED_FOR, "存储/展示"),
        KGRelation("wheelchair", R.USED_FOR, "辅助移动"),
        KGRelation("stretcher", R.USED_FOR, "转运病人"),
        KGRelation("traffic_cone", R.USED_FOR, "标记/隔离区域"),
        KGRelation("valve", R.USED_FOR, "控制流体/气体"),
        KGRelation("generator", R.USED_FOR, "备用供电"),
        KGRelation("robot_charging_station", R.USED_FOR, "机器人充电"),

        # LOCATED_IN — 典型位置
        KGRelation("fire_extinguisher", R.LOCATED_IN, "corridor"),
        KGRelation("fire_alarm", R.LOCATED_IN, "ceiling"),
        KGRelation("printer", R.LOCATED_IN, "office"),
        KGRelation("monitor", R.LOCATED_IN, "office"),
        KGRelation("refrigerator", R.LOCATED_IN, "kitchen"),
        KGRelation("microwave", R.LOCATED_IN, "kitchen"),
        KGRelation("toilet", R.LOCATED_IN, "bathroom"),
        KGRelation("sink", R.LOCATED_IN, "bathroom"),
        KGRelation("bed", R.LOCATED_IN, "bedroom"),
        KGRelation("server_rack", R.LOCATED_IN, "server_room"),
        KGRelation("generator", R.LOCATED_IN, "utility_room"),

        # DANGEROUS_IF — 危险条件
        KGRelation("electrical_panel", R.DANGEROUS_IF, "接触/打开"),
        KGRelation("gas_cylinder", R.DANGEROUS_IF, "碰撞/倾倒/靠近火源"),
        KGRelation("conveyor", R.DANGEROUS_IF, "靠近运行中的传送带"),
        KGRelation("forklift", R.DANGEROUS_IF, "运行中靠近"),
        KGRelation("stairs", R.DANGEROUS_IF, "快速移动/无扶手辅助"),
        KGRelation("crane", R.DANGEROUS_IF, "吊物下方/运行中靠近"),
        KGRelation("generator", R.DANGEROUS_IF, "运行中靠近/接触排气口"),
        KGRelation("control_panel", R.DANGEROUS_IF, "未授权操作"),
        KGRelation("valve", R.DANGEROUS_IF, "未经确认的开关操作"),
        KGRelation("manhole_cover", R.DANGEROUS_IF, "缺失或松动时踩踏"),

        # REQUIRES — 操作前置条件
        KGRelation("fire_extinguisher", R.REQUIRES, "拔保险销"),
        KGRelation("elevator", R.REQUIRES, "按键/人工协助"),
        KGRelation("aed", R.REQUIRES, "培训认证"),
        KGRelation("medicine_cabinet", R.REQUIRES, "授权访问"),
        KGRelation("control_panel", R.REQUIRES, "操作许可"),
        KGRelation("crane", R.REQUIRES, "操作资质"),
        KGRelation("forklift", R.REQUIRES, "驾驶资质"),

        # RELATED_TO — 空间/功能关联
        KGRelation("fire_extinguisher", R.RELATED_TO, "emergency_exit"),
        KGRelation("fire_alarm", R.RELATED_TO, "fire_extinguisher"),
        KGRelation("fire_hydrant", R.RELATED_TO, "fire_extinguisher"),
        KGRelation("fire_blanket", R.RELATED_TO, "first_aid_kit"),
        KGRelation("eye_wash_station", R.RELATED_TO, "first_aid_kit"),
        KGRelation("spill_kit", R.RELATED_TO, "eye_wash_station"),
        KGRelation("stairs", R.RELATED_TO, "railing"),
        KGRelation("desk", R.RELATED_TO, "chair"),
        KGRelation("monitor", R.RELATED_TO, "desk"),
        KGRelation("computer", R.RELATED_TO, "monitor"),
        KGRelation("projector", R.RELATED_TO, "whiteboard"),
        KGRelation("sink", R.RELATED_TO, "mirror"),
        KGRelation("toilet", R.RELATED_TO, "sink"),
        KGRelation("safety_helmet", R.RELATED_TO, "safety_vest"),
        KGRelation("valve", R.RELATED_TO, "pipe"),
        KGRelation("robot_charging_station", R.RELATED_TO, "corridor"),

        # PART_OF — 组成关系
        KGRelation("railing", R.PART_OF, "stairs"),
        KGRelation("valve", R.PART_OF, "pipe"),
        KGRelation("mirror", R.PART_OF, "bathroom"),
    ])


# ════════════════════════════════════════════════════════════════
#  安全约束
# ════════════════════════════════════════════════════════════════

def _build_safety_constraints(safety_constraints: list[SafetyConstraint]) -> None:
    safety_constraints.extend([
        # 配电箱 — 禁止所有操作
        SafetyConstraint(
            constraint_id="SC001",
            constraint_type="factual",
            concept_id="electrical_panel",
            action="*",
            condition="任何接触或操作",
            response="block",
            message_zh="⚠️ 配电箱 (220V/380V) 高压危险, 机器人禁止触碰。需专业电工操作。",
            message_en=(
                "⚠️ Electrical panel (220V/380V) - HIGH VOLTAGE."
                " Robot interaction forbidden. Requires licensed electrician."
            ),
            max_approach_distance=1.0,
        ),
        # 气瓶 — 禁止抓取
        SafetyConstraint(
            constraint_id="SC002",
            constraint_type="factual",
            concept_id="gas_cylinder",
            action="pick",
            condition="抓取或移动",
            response="block",
            message_zh="⚠️ 高压气瓶 (可爆炸), 机器人禁止抓取或移动。需人工固定搬运。",
            message_en=(
                "Compressed gas cylinder (explosive risk)."
                " Robot MUST NOT grasp or move. Requires manual handling."
            ),
            max_approach_distance=0.5,
        ),
        SafetyConstraint(
            constraint_id="SC003",
            constraint_type="causal",
            concept_id="gas_cylinder",
            action="approach",
            condition="靠近",
            response="limit_speed",
            message_zh="⚠️ 靠近气瓶, 已限制速度至 0.3 m/s。避免碰撞。",
            message_en="⚠️ Approaching gas cylinder. Speed limited to 0.3 m/s. Avoid collision.",
            max_approach_distance=1.5,
        ),
        # 传送带 — 禁止靠近运行中
        SafetyConstraint(
            constraint_id="SC004",
            constraint_type="causal",
            concept_id="conveyor",
            action="approach",
            condition="传送带运行中",
            response="warn",
            message_zh="⚠️ 传送带运行中, 有卷入风险。保持 2m 安全距离。",
            message_en="⚠️ Conveyor in operation. Entanglement risk. Maintain 2m safety distance.",
            max_approach_distance=2.0,
        ),
        # 叉车
        SafetyConstraint(
            constraint_id="SC005",
            constraint_type="causal",
            concept_id="forklift",
            action="approach",
            condition="叉车运行中",
            response="warn",
            message_zh="⚠️ 叉车作业区, 保持 3m 安全距离。",
            message_en="⚠️ Forklift operating zone. Maintain 3m safety distance.",
            max_approach_distance=3.0,
        ),
        # 楼梯
        SafetyConstraint(
            constraint_id="SC006",
            constraint_type="factual",
            concept_id="stairs",
            action="approach",
            condition="四足机器人接近楼梯",
            response="confirm",
            message_zh="⚠️ 检测到楼梯, 需切换上下楼梯步态模式。是否继续?",
            message_en="⚠️ Stairs detected. Requires stair-climbing gait mode. Continue?",
        ),
        # 服务器机柜
        SafetyConstraint(
            constraint_id="SC007",
            constraint_type="factual",
            concept_id="server_rack",
            action="pick",
            condition="抓取服务器机柜内部线缆或设备",
            response="block",
            message_zh="⚠️ 服务器机柜, 禁止抓取内部任何物品。需 IT 人员操作。",
            message_en="⚠️ Server rack. Robot MUST NOT interact with internal components. IT personnel required.",
        ),
        # 灭火器检查
        SafetyConstraint(
            constraint_id="SC008",
            constraint_type="temporal",
            concept_id="fire_extinguisher",
            action="pick",
            condition="抓取灭火器",
            response="confirm",
            message_zh="⚠️ 即将抓取灭火器 (安全设备)。确认这是计划任务?",
            message_en="⚠️ About to grasp fire extinguisher (safety equipment). Confirm this is a planned task?",
        ),
        # 起重机 — 运行区域
        SafetyConstraint(
            constraint_id="SC009",
            constraint_type="causal",
            concept_id="crane",
            action="approach",
            condition="起重机运行中",
            response="warn",
            message_zh="⚠️ 起重机/天车作业区, 保持 5m 安全距离, 勿在吊物下方停留。",
            message_en="⚠️ Crane operating zone. Maintain 5m distance. Never stand under suspended loads.",
            max_approach_distance=5.0,
        ),
        # 发电机
        SafetyConstraint(
            constraint_id="SC010",
            constraint_type="causal",
            concept_id="generator",
            action="approach",
            condition="发电机运行中",
            response="warn",
            message_zh="⚠️ 发电机运行中, 高温/噪声/排气, 保持 3m 安全距离。",
            message_en="⚠️ Generator running. High temperature/noise/exhaust. Maintain 3m distance.",
            max_approach_distance=3.0,
        ),
        # 控制面板 — 禁止未授权操作
        SafetyConstraint(
            constraint_id="SC011",
            constraint_type="factual",
            concept_id="control_panel",
            action="pick",
            condition="操作控制面板",
            response="block",
            message_zh="⚠️ 控制面板, 禁止未授权操作, 可能导致设备失控。",
            message_en="⚠️ Control panel. Unauthorized operation forbidden. May cause equipment malfunction.",
        ),
        # 药品柜 — 需授权
        SafetyConstraint(
            constraint_id="SC012",
            constraint_type="factual",
            concept_id="medicine_cabinet",
            action="pick",
            condition="从药品柜取物",
            response="confirm",
            message_zh="⚠️ 药品柜受管控, 确认已获得授权?",
            message_en="⚠️ Medicine cabinet is access-controlled. Confirm authorization?",
        ),
        # 井盖 — 绕行
        SafetyConstraint(
            constraint_id="SC013",
            constraint_type="causal",
            concept_id="manhole_cover",
            action="approach",
            condition="检测到井盖缺失/松动",
            response="warn",
            message_zh="⚠️ 检测到井盖, 可能缺失或松动, 注意绕行。",
            message_en="⚠️ Manhole cover detected. May be missing or loose. Navigate around.",
            max_approach_distance=1.0,
        ),
        # 担架 — 可能有人
        SafetyConstraint(
            constraint_id="SC014",
            constraint_type="causal",
            concept_id="stretcher",
            action="approach",
            condition="接近担架/病床",
            response="limit_speed",
            message_zh="⚠️ 靠近担架/病床, 限速 0.3 m/s, 可能有人。",
            message_en="⚠️ Approaching stretcher/bed. Speed limited to 0.3 m/s. Patient may be present.",
            max_approach_distance=1.5,
        ),
        # 阀门 — 需确认
        SafetyConstraint(
            constraint_id="SC015",
            constraint_type="causal",
            concept_id="valve",
            action="pick",
            condition="操作阀门",
            response="confirm",
            message_zh="⚠️ 阀门操作可能影响管道系统运行, 确认是否继续?",
            message_en="⚠️ Valve operation may affect piping system. Confirm to proceed?",
        ),
    ])
