import argparse
import copy
import pathlib
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import contracts as C
import run as runner


def declaration(directory):
    problem = directory / 'problem.json'
    C.write(problem, {'lower': [-2], 'upper': [2],
                      'tasks': [{'A': [[1]], 'b': [0.5], 'priority': 0}]})
    descriptor = directory / 'input.json'
    C.write(descriptor, {'schema_version': 'input_descriptor.v1',
                         'canonical': C.artifact(problem)})
    return {'schema_version': 'experiment.v2', 'experiment_id': 'E06',
            'execution_contract': 'execution_request.v1',
            'question': 'Native explicit matrix execution regression',
            'failure_policy': 'continue-preserve-all', 'metrics': [],
            'evaluation_windows': [], 'controlled_factors': [],
            'units': [{'case_id': 'native', 'arm_id': 'mcc', 'method_id': 'native-matrix',
                       'app': 'mcl_optimization_problem', 'execution_structure': 'explicit_matrix',
                       'config': {'solver': 'mcc', 'mode': 'weighted', 'backend': 'eiquadprog'},
                       'execution': {}, 'observation': {}, 'input': str(descriptor),
                       'required': True}]}


class NativeDeclaration(unittest.TestCase):
    def test_invalid_native_configuration_is_not_a_migration_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp); (root/'bin').mkdir()
            (root/'bin/mcl_optimization_problem').touch()
            d = declaration(root); source = root/'definition.json'
            caps = {'sha256': 'test', 'document': {'execution_structures': []}}
            for migrated, expected in [(False, 'configuration_invalid'), (True, 'migration_defect')]:
                if migrated:
                    old = root/'old.json'; C.write(old, {})
                    d['legacy_definition'] = C.artifact(old)
                    d['units'][0]['migration'] = {'status': 'mapped', 'legacy_definition': d['legacy_definition']}
                C.write(source, d)
                with mock.patch.object(runner, 'capabilities', return_value=caps):
                    check = runner.preflight([(source, d, d['units'][0], 0)], root, probe=False)
                self.assertEqual(check['units'][0]['status'], expected)
                self.assertEqual(check['status'], 'failed')

    def test_prepare_preserves_bytes_and_needs_no_legacy(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp)
            source, output = root/'source.json', root/'copy.json'
            d = declaration(root)
            C.write(source, d)
            C.prepare_definition(source, output)
            self.assertEqual(source.read_bytes(), output.read_bytes())
            with self.assertRaises(FileExistsError):
                C.prepare_definition(source, output)
            request, _, _ = runner.request_for(d, d['units'][0], 0, root/'out')
            self.assertNotIn('legacy_definition_sha256', request['tracking'])
            self.assertFalse((root/'out').exists())

    def test_partial_binding_and_missing_legacy_are_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp)
            d = declaration(root)
            old = root/'old.json'; C.write(old, {})
            ref = C.artifact(old)
            d['units'][0]['migration'] = {'status': 'mapped', 'legacy_definition': ref}
            with self.assertRaisesRegex(ValueError, 'together'):
                C.validate_definition(d)
            d['legacy_definition'] = ref
            C.validate_definition(d)
            d['units'][0]['migration']['legacy_definition'] = dict(ref, path=str(root/'other.json'))
            with self.assertRaisesRegex(ValueError, 'cross-declaration'):
                C.validate_definition(d)
            d['units'][0]['migration']['legacy_definition'] = ref
            old.unlink()
            with self.assertRaisesRegex(ValueError, 'missing or corrupt'):
                C.validate_definition(d)

    def test_unavailable_and_retired_units_never_probe_or_execute(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp); (root/'bin').mkdir()
            (root/'bin/mcl_optimization_problem').touch()
            d = declaration(root); d['units'][0].update(available=False, unavailable_reason='explicit absence')
            path = root/'definition.json'; C.write(path, d)
            with mock.patch.object(runner, 'capabilities', side_effect=AssertionError('must not probe')):
                check = runner.preflight([(path, d, d['units'][0], 0)], root)
            self.assertEqual(check['units'][0]['status'], 'declared_unavailable')
            old = root/'old.json'; C.write(old, {})
            d['legacy_definition'] = C.artifact(old)
            d['units'][0]['migration'] = {'status': 'retired_research_method', 'reason': 'old method retired',
                                         'legacy_definition': d['legacy_definition']}
            C.write(path, d); C.validate_definition(d)
            with mock.patch.object(runner, 'capabilities', side_effect=AssertionError('must not probe')):
                check = runner.preflight([(path, d, d['units'][0], 0)], root)
            self.assertEqual(check['units'][0]['status'], 'retired_research_method')


def native_execution(binary):
    """Exercise the real app through preparation, preflight, execution, and verification."""
    with tempfile.TemporaryDirectory(prefix='mcl-native-declaration-') as tmp:
        root = pathlib.Path(tmp); (root/'experiments/E06_native').mkdir(parents=True)
        (root/'bin').mkdir(); (root/'bin/mcl_optimization_problem').symlink_to(binary.resolve())
        source, output = root/'definition.json', root/'prepared.json'
        d = declaration(root)
        impossible = root/'infeasible.json'
        C.write(impossible, {'lower': [-2], 'upper': [2],
                             'tasks': [{'A': [[1]], 'b': [3], 'hard': True}]})
        descriptor = root/'infeasible-input.json'
        C.write(descriptor, {'schema_version': 'input_descriptor.v1',
                             'canonical': C.artifact(impossible)})
        rejected = copy.deepcopy(d['units'][0])
        rejected.update(case_id='infeasible', input=str(descriptor))
        d['units'].append(rejected)
        C.write(source, d); C.prepare_definition(source, output)
        args = argparse.Namespace(experiment=None, case=None, app=None, repeat=[0],
                                  install_prefix=root, smoke=True, timeout=10)
        selection = runner.select([output], args)
        before = {p.relative_to(root) for p in root.rglob('*')}
        check = runner.preflight(selection, root, smoke=True)
        assert check['counts'] == {'ready': 2}, check
        assert before == {p.relative_to(root) for p in root.rglob('*')}
        with mock.patch.object(runner, 'ROOT', root):
            assert runner.execute(selection, check, args) == 1
        manifest = C.read(next(root.glob('experiments/E06_native/runs/*/E06.manifest.json')))
        assert manifest['status'] == 'failed'
        assert [u['status'] for u in manifest['units']] == ['completed', 'failed']
        assert manifest['units'][1]['exit_code'] != 0
        assert manifest['units'][0]['validation'] == 'passed'
        request = C.read(next(root.glob('experiments/E06_native/runs/*/units/*/*/*/request.json')))
        assert 'legacy_definition_sha256' not in request['tracking']


if __name__ == '__main__':
    if len(sys.argv) == 2 and pathlib.Path(sys.argv[1]).is_file():
        native_execution(pathlib.Path(sys.argv[1]))
    else:
        unittest.main()
