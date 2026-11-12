import tempfile
import unittest
from pathlib import Path

from modularity_metrics import analyze


class MetricsTest(unittest.TestCase):
    def test_graph_and_scopes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifests = {
                'a': '<depend>b</depend><exec_depend>b</exec_depend><test_depend>c</test_depend>',
                'b': '<run_depend>c</run_depend><buildtool_depend>tool</buildtool_depend>',
                'c': '<depend>b</depend><depend>external</depend>',
                'isolated': '',
            }
            for name, dependencies in manifests.items():
                folder = root / name
                folder.mkdir()
                (folder / 'package.xml').write_text(
                    f'<package><name>{name}</name>{dependencies}</package>')
            report = analyze(root, ['production'])
            rows = {r['package']: r for r in report['metrics']}
            self.assertEqual(report['cycles'], [['b', 'c']])
            self.assertEqual(rows['a']['Ce'], 1)  # Duplicate declarations count once.
            self.assertEqual(rows['b']['Ca'], 2)
            self.assertEqual(rows['c']['transitive_dependents'], 2)
            self.assertEqual(rows['c']['external_dependencies'], ['external'])
            self.assertIsNone(rows['isolated']['instability'])
            self.assertEqual(rows['b']['instability'], 1 / 3)
            expanded = analyze(root, ['production', 'test', 'buildtool'])
            expanded_rows = {r['package']: r for r in expanded['metrics']}
            self.assertEqual(expanded_rows['a']['Ce'], 2)
            self.assertEqual(expanded_rows['b']['external_dependencies'], ['tool'])


if __name__ == '__main__':
    unittest.main()
