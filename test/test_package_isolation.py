"""Check the package boundaries without a ROS installation."""

import importlib.util
import json
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = sorted(path.parent for path in ROOT.glob("src/*_behaviors/package.xml"))


def index_data(path):
    text = path.read_text()
    return json.loads(text[text.index("{") : text.rindex("}") + 1])


class PackageIsolation(unittest.TestCase):
    def test_colcon_can_discover_packages(self):
        self.assertTrue(PACKAGES)
        self.assertFalse((ROOT / "package.xml").exists())
        self.assertFalse((ROOT / "CMakeLists.txt").exists())
        self.assertFalse((ROOT / "behaviors").exists())
        self.assertFalse(list(ROOT.glob("*_behaviors")), "packages belong under src/")

    def test_each_package_is_complete(self):
        for package in PACKAGES:
            with self.subTest(package=package.name):
                for name in (
                    "CMakeLists.txt",
                    "README.md",
                    "LICENSE",
                    "NOTICE",
                    ".clang-format",
                    "behavior_plugin.yaml",
                    "src/register_behaviors.cpp",
                    "test/CMakeLists.txt",
                    "test/test_behavior_plugins.cpp",
                    "test/test_behavior_index.cpp",
                    "docs/AGENT_COMMENTS.md",
                    "docs/behavior_index/index.html",
                    "docs/behavior_index/behavior_index_data.js",
                ):
                    self.assertTrue((package / name).is_file(), name)
                manifest = ET.parse(package / "package.xml").getroot()
                self.assertEqual(manifest.findtext("name"), package.name)
                depends = {entry.text for entry in manifest.findall("depend")}
                self.assertIn("behaviortree_cpp_picknik", depends)
                self.assertNotIn("behaviortree_cpp", depends)
                self.assertNotIn("experimental_behaviors", depends)
                self.assertFalse(depends.intersection(p.name for p in PACKAGES))
                cmake = (package / "CMakeLists.txt").read_text()
                self.assertIn(f"project({package.name} CXX)", cmake)
                self.assertIn("ament_export_targets", cmake)
                self.assertIn("install(DIRECTORY include/ DESTINATION include)", cmake)
                self.assertNotIn("BEHAVIOR_CATEGORY_TARGET", cmake)
                plugin = ET.parse(
                    package / f"{package.name}_plugin_description.xml"
                ).getroot()
                self.assertEqual(plugin.get("path"), package.name)
                loader = plugin.find("class").get("type")
                self.assertTrue(loader.startswith(package.name + "::"))
                self.assertIn(loader, (package / "behavior_plugin.yaml").read_text())
                self.assertIn(
                    loader, (package / "test/test_behavior_plugins.cpp").read_text()
                )

    def test_sources_do_not_depend_on_sibling_packages(self):
        for package in PACKAGES:
            for path in package.rglob("*"):
                if path.suffix not in (".cpp", ".hpp", ".h"):
                    continue
                text = path.read_text()
                with self.subTest(path=path.relative_to(ROOT)):
                    self.assertNotIn("experimental_behaviors", text)
                    self.assertNotIn("BEHAVIOR_CATEGORY_TARGET", text)
                    for include in re.findall(r'#include\s+[<"]([^">]+)', text):
                        prefix = include.split("/")[0]
                        if prefix.endswith("_behaviors"):
                            self.assertEqual(prefix, package.name)
                            self.assertTrue((package / "include" / include).is_file())

    def test_index_has_no_duplicate_ids_and_covers_every_package(self):
        combined = index_data(ROOT / "docs/behavior_index/behavior_index_data.js")
        categories = combined["categories"]
        self.assertEqual([c["folder"] for c in categories], [p.name for p in PACKAGES])
        ids = [b["id"] for category in categories for b in category["behaviors"]]
        self.assertEqual(len(ids), len(set(ids)))
        for package, category in zip(PACKAGES, categories):
            own = index_data(package / "docs/behavior_index/behavior_index_data.js")
            self.assertEqual(own["categories"], [category])

    def test_repository_index_is_current(self):
        script = ROOT / "tools/update_behavior_index.py"
        spec = importlib.util.spec_from_file_location("update_behavior_index", script)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.assertEqual(
            (ROOT / "docs/behavior_index/behavior_index_data.js").read_text(),
            module.render_index(ROOT),
        )


if __name__ == "__main__":
    unittest.main()
