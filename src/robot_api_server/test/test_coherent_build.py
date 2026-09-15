"""Candidate CLI must reject source/output mistakes before invoking CMake."""
import subprocess
import sys
import tempfile
from pathlib import Path
import unittest

BUILDER = Path(__file__).resolve().parents[3]/'scripts/jetson/build_api_coherent_candidate.py'


class CoherentBuildAdmission(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.source = self.root/'source'
        (self.source/'src').mkdir(parents=True)
        (self.source/'src/robot_api_server_node.cpp').write_text('// fixture only\n')

    def invoke(self, output, *extra):
        return subprocess.run([sys.executable, str(BUILDER), '--source', str(self.source),
                               '--output', str(output), *extra], text=True, capture_output=True)

    def test_existing_build_is_not_reused_or_removed(self):
        output = self.root/'previous'
        output.mkdir()
        artifact = output/'old.o'
        artifact.write_bytes(b'old object stays untouched')
        result = self.invoke(output)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('FileExistsError', result.stderr)
        self.assertEqual(artifact.read_bytes(), b'old object stays untouched')

    def test_output_cannot_pollute_approved_source(self):
        output = self.source/'build'
        result = self.invoke(output)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('output must be outside source', result.stderr)
        self.assertFalse(output.exists())

    def test_incomplete_package_is_rejected_before_output_creation(self):
        (self.source/'src/robot_api_server_node.cpp').unlink()
        output = self.root/'new'
        result = self.invoke(output)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('complete robot_api_server package', result.stderr)
        self.assertFalse(output.exists())

    def test_unbounded_parallel_build_is_rejected(self):
        result = self.invoke(self.root/'new', '--jobs', '99')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('invalid choice', result.stderr)


if __name__ == '__main__':
    unittest.main()
