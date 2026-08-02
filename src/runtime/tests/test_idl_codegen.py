import importlib.util
import sys
import types as py_types
import unittest
from dataclasses import fields, is_dataclass
from pathlib import Path
from typing import get_type_hints

# Load the code generator as a standalone module; it lives outside src/.
_CODEGEN_PATH = Path(__file__).resolve().parents[3] / "scripts" / "codegen" / "idl_to_python.py"
_spec = importlib.util.spec_from_file_location("idl_to_python", _CODEGEN_PATH)
idl_to_python = importlib.util.module_from_spec(_spec)
sys.modules["idl_to_python"] = idl_to_python
_spec.loader.exec_module(idl_to_python)

IdlParser = idl_to_python.IdlParser
CodeGenerator = idl_to_python.CodeGenerator


class TestIdlParser(unittest.TestCase):
    """Tests for the IDL parser."""

    def test_basic_types(self):
        idl = """
        struct BasicTypes {
            short s;
            long l;
            unsigned long ul;
            float f;
            double d;
            boolean b;
            string str;
            octet o;
        };
        """
        structs = IdlParser().parse(idl)
        self.assertEqual(len(structs), 1)
        struct = structs[0]
        self.assertEqual(struct.name, "BasicTypes")
        self.assertEqual(len(struct.fields), 8)
        self.assertEqual({f.name for f in struct.fields}, {"s", "l", "ul", "f", "d", "b", "str", "o"})

    def test_sequence_field(self):
        idl = """
        struct SequenceHolder {
            sequence<octet> bytes;
            sequence<float> values;
        };
        """
        structs = IdlParser().parse(idl)
        self.assertEqual(len(structs), 1)
        struct = structs[0]
        self.assertEqual(len(struct.fields), 2)
        bytes_field = struct.fields[0]
        self.assertTrue(bytes_field.is_sequence)
        self.assertEqual(bytes_field.element_type, "octet")
        values_field = struct.fields[1]
        self.assertTrue(values_field.is_sequence)
        self.assertEqual(values_field.element_type, "float")

    def test_fixed_array_field(self):
        idl = """
        struct ArrayHolder {
            double covariance[9];
            octet rsvd[3];
        };
        """
        structs = IdlParser().parse(idl)
        self.assertEqual(len(structs), 1)
        struct = structs[0]
        self.assertEqual(len(struct.fields), 2)
        cov = struct.fields[0]
        self.assertEqual(cov.array_size, 9)
        self.assertEqual(cov.raw_type, "double")
        rsvd = struct.fields[1]
        self.assertEqual(rsvd.array_size, 3)
        self.assertEqual(rsvd.raw_type, "octet")

    def test_nested_struct(self):
        idl = """
        struct Point {
            double x;
            double y;
        };

        struct Pose {
            Point position;
        };
        """
        structs = IdlParser().parse(idl)
        self.assertEqual(len(structs), 2)
        pose = next(s for s in structs if s.name == "Pose")
        self.assertEqual(len(pose.fields), 1)
        self.assertEqual(pose.fields[0].raw_type, "Point")

    def test_module_nesting(self):
        idl = """
        module outer {
            module inner {
                struct T {
                    long value;
                };
            };
        };
        """
        structs = IdlParser().parse(idl)
        self.assertEqual(len(structs), 1)
        self.assertEqual(structs[0].module_path, ("outer", "inner"))
        self.assertEqual(structs[0].fully_qualified, "outer::inner::T")

    def test_keylist(self):
        idl = """
        struct KeyedStruct {
            long id;
            string name;
        };
        #pragma keylist KeyedStruct id
        """
        structs = IdlParser().parse(idl)
        self.assertEqual(structs[0].keys, ["id"])


class TestCodeGenerator(unittest.TestCase):
    """Tests for generated Python dataclasses."""

    def _generate_module(self, idl: str) -> py_types.ModuleType:
        """Parse *idl*, generate code, and return a compiled module object."""
        structs = IdlParser().parse(idl)
        self.assertTrue(structs, "No structs found in test IDL")
        generated = CodeGenerator(source_idl="<test-idl>").generate(structs)
        types_source = generated["types.py"]
        # Verify no syntax errors via compile().
        compiled = compile(types_source, "<generated>", "exec")
        module = py_types.ModuleType("generated_types")
        # Register the dynamic module so dataclass machinery can resolve
        # string annotations and the kw_only sentinel correctly.
        sys.modules[module.__name__] = module
        exec(compiled, module.__dict__)  # noqa: S102
        return module

    def test_generated_basic_types(self):
        module = self._generate_module("""
        struct Basic {
            short s;
            unsigned long ul;
            float f;
            double d;
            boolean b;
            string str;
            octet o;
        };
        """)
        self.assertTrue(is_dataclass(module.Basic))
        hints = get_type_hints(module.Basic)
        self.assertEqual(hints["s"], int)
        self.assertEqual(hints["ul"], int)
        self.assertEqual(hints["f"], float)
        self.assertEqual(hints["d"], float)
        self.assertEqual(hints["b"], bool)
        self.assertEqual(hints["str"], str)
        self.assertEqual(hints["o"], int)

    def test_generated_sequence_and_array_fields(self):
        module = self._generate_module("""
        struct Holder {
            sequence<octet> bytes;
            sequence<float> values;
            double matrix[4];
        };
        """)
        self.assertTrue(is_dataclass(module.Holder))
        dataclass_fields = {f.name: f for f in fields(module.Holder)}
        hints = get_type_hints(module.Holder)
        self.assertEqual(hints["bytes"], list[int])
        self.assertEqual(hints["values"], list[float])
        self.assertEqual(hints["matrix"], list[float])
        # Sequence and array fields should default to empty lists.
        self.assertEqual(dataclass_fields["bytes"].default_factory(), [])
        self.assertEqual(dataclass_fields["values"].default_factory(), [])
        self.assertEqual(dataclass_fields["matrix"].default_factory(), [])

    def test_generated_nested_struct(self):
        module = self._generate_module("""
        struct Point {
            double x;
            double y;
        };

        struct Pose {
            Point position;
        };
        """)
        self.assertTrue(is_dataclass(module.Point))
        self.assertTrue(is_dataclass(module.Pose))
        pose_hints = get_type_hints(module.Pose)
        self.assertEqual(pose_hints["position"], module.Point)
        # Topological sort should place Point before Pose in __all__.
        init_source = CodeGenerator(source_idl="<test-idl>").generate(
            IdlParser().parse("""
            struct Point { double x; };
            struct Pose { Point position; };
            """)
        )["__init__.py"]
        self.assertIn("Point", init_source)
        self.assertIn("Pose", init_source)

    def test_generated_kw_only_dataclass(self):
        """Non-default fields may follow default fields in generated classes."""
        module = self._generate_module("""
        struct A { long value; };
        struct B {
            A a;
            double arr[3];
            A b;
        };
        """)
        # Instantiation must use keyword arguments.
        instance = module.B(a=module.A(value=1), b=module.A(value=2))
        self.assertIsInstance(instance.a, module.A)
        self.assertIsInstance(instance.b, module.A)
        self.assertEqual(instance.arr, [])

    def test_real_idl_generates_valid_python(self):
        """The committed SLAM IDL produces syntactically valid Python."""
        idl_path = Path(__file__).resolve().parents[3] / "src" / "message" / "idl" / "lingtu_slam.idl"
        structs = IdlParser().parse(idl_path.read_text(encoding="utf-8"))
        generated = CodeGenerator(source_idl=str(idl_path)).generate(structs)
        # All generated source must compile.
        compile(generated["types.py"], "<types.py>", "exec")
        compile(generated["__init__.py"], "<__init__.py>", "exec")

    def test_real_idl_declares_final_extensibility_for_every_struct(self):
        """CycloneDDS upgrades must not change the wire layout implicitly."""
        idl_path = Path(__file__).resolve().parents[3] / "src" / "message" / "idl" / "lingtu_slam.idl"
        lines = idl_path.read_text(encoding="utf-8").splitlines()
        struct_lines = [
            index
            for index, line in enumerate(lines)
            if line.startswith("struct ")
        ]

        self.assertTrue(struct_lines)
        for index in struct_lines:
            self.assertGreater(index, 0)
            self.assertEqual(lines[index - 1].strip(), "@final", lines[index])


if __name__ == "__main__":
    unittest.main()
