"""Runner logic tests only; actual Dymola acceptance evidence is kept separately."""
import json
from pathlib import Path
import tempfile
import unittest

import run_dymola_tests as runner


class RunnerTests(unittest.TestCase):
    def test_check_language_rejects_arbitrary_python(self):
        for expression in ["__import__('os')", "v('x').real > 0", "v('x')[0] > 0", "sum([1, 2]) > 0"]:
            with self.subTest(expression=expression), self.assertRaises(ValueError):
                runner.expression_signals(expression)

    def test_check_language_collects_real_signals(self):
        _, names = runner.expression_signals("0 < v('P_out') < v('P_in') and v('Q') > 0")
        self.assertEqual(names, {"P_out", "P_in", "Q"})

    def test_input_inventory_required(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'package.mo').write_text('package Test end Test;')
            manifest = {'libraries': [{'file': 'package.mo'}], 'models': [
                {'class_name': 'Test.Model', 'checks': [{'name': 'physical', 'expression': "v('Q')>0"}]}]}
            path = root / 'manifest.json'
            path.write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ValueError, 'inventory'):
                runner.load_manifest(path)

    def test_model_with_only_an_ordinary_input_can_be_validated(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'package.mo').write_text('package Test end Test;')
            manifest = {'libraries': [{'file': 'package.mo'}], 'models': [
                {'class_name': 'Test.Model', 'scalar_input_starts': {'h': 2300000},
                 'checks': [{'name': 'positive', 'expression': "v('h')>0"}]}]}
            path = root / 'manifest.json'
            path.write_text(json.dumps(manifest))
            self.assertEqual(runner.load_manifest(path)['models'][0]['scalar_input_starts']['h'], 2300000)

    def test_failed_check_is_recorded_and_simulation_is_not_called(self):
        class FailedCheckSession:
            simulated = False
            def cd(self, path): return True
            def checkModel(self, name): return False
            def getLastErrorLog(self): return 'Structurally singular'
            def simulateModel(self, *args, **kwargs):
                self.simulated = True
                raise AssertionError('Simulation must not run after a failed Check Model')
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            dymola = FailedCheckSession()
            report = runner.validate_model(dymola, {'class_name': 'Test.Model'}, root/'manifest.json', root/'run')
            self.assertEqual(report['status'], 'check_failed')
            self.assertFalse(dymola.simulated)
            self.assertEqual(json.loads((root/'run/report.json').read_text())['status'], 'check_failed')
            self.assertIn('singular', (root/'run/check_model.log').read_text())


if __name__ == '__main__':
    unittest.main()
