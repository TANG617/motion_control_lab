"""Progress counts input records without changing the validation outcome."""
import importlib.util
import json
import pathlib

import numpy as np
import pytest


EXPERIMENTS = pathlib.Path(__file__).parents[2]


@pytest.fixture(params=[
    'E06_solver_semantic_parity',
    'E07_hqp_priority_and_redundancy',
    'E08_constraints_scaling_and_degeneracy',
])
def verifier(request):
    spec = importlib.util.spec_from_file_location(
        request.param + '_progress_test', EXPERIMENTS / request.param / 'verify.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('candidate,expected', [([1, 1], 'completed'), ([3, 3], 'failed')])
def test_processed_records_and_final_artifact_order(verifier, tmp_path, monkeypatch, capsys,
                                                   candidate, expected):
    model = tmp_path / 'model.urdf'
    model.write_text('<robot name="analytic"><link name="base_link"/></robot>')
    records = [
        {'record_type': 'attempt_started'},
        {'record_type': 'analytic', 'status': 'solved', 'candidate': candidate,
         'problem': {'lower': [-2, -2], 'upper': [2, 2],
                     'tasks': [{'A': [[1, 0], [0, 1]], 'b': [1, 1]}]}},
        {'record_type': 'run_finished'},
    ]
    (tmp_path / 'raw.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in records))
    data = {'model': {'locator': str(model)}, 'joint_names': [], 'active_joint_names': [],
            'root_frame': 'base_link', 'frames': {}, 'tcp_offsets': {}, 'limits': {}, 'units': {}}
    request = tmp_path / 'request.json'
    request.write_text(json.dumps({'output_dir': str(tmp_path), 'input': data,
                                  'config': {'primary_only': False, 'regularization': 0,
                                             'hard_tolerance': 1e-7},
                                  'identity': {'method_id': 'mcc-hqp'}}))
    events = []
    original_progress = verifier.SampleProgress

    class TracedProgress(original_progress):
        def update(self, current):
            events.append((current, (tmp_path / 'app_validation.json').exists()))
            super().update(current)

    monkeypatch.setattr(verifier, 'SampleProgress', TracedProgress)
    assert verifier.verify(request) == int(expected == 'failed')
    assert events == [(0, False), (0, False), (1, False), (2, False), (3, True)]
    progress = json.loads((tmp_path / 'sample_progress.json').read_text())
    assert progress['stage'] == 'app-validation'
    assert progress['current'] == progress['total'] == len(records)
    assert json.loads((tmp_path / 'app_validation.json').read_text())['status'] == expected
    assert capsys.readouterr().out == ''


def test_begin_only_trace_is_processed_but_unavailable(verifier, tmp_path):
    (tmp_path / 'raw.jsonl').write_text(json.dumps({'record_type': 'attempt_started'}) + '\n')
    request = tmp_path / 'request.json'
    request.write_text(json.dumps({'output_dir': str(tmp_path), 'input': {}, 'config': {}}))
    assert verifier.verify(request) == 2
    progress = json.loads((tmp_path / 'sample_progress.json').read_text())
    assert progress['current'] == progress['total'] == 1
    assert json.loads((tmp_path / 'app_validation.json').read_text())['status'] == 'unavailable'


def test_secondary_effects_use_each_samples_own_reference(verifier, tmp_path, monkeypatch):
    class FixedKinematics:
        def __init__(self, locator):
            pass

        def pose(self, *args, **kwargs):
            return {'position': [0., 0., 0.], 'rotation': np.eye(3).tolist()}

        def jacobian(self, *args, **kwargs):
            return np.zeros((3, 2))

    monkeypatch.setattr(verifier, 'UrdfFk', FixedKinematics)
    pose = {'position': [0., 0., 0.], 'rotation': np.eye(3).tolist()}
    rows = []
    for index, velocity in enumerate([.25, .5]):
        rows.append({'record_type': 'attempt', 'attempt_sequence': index,
                     'source_time_s': float(index), 'q': [0., 0.],
                     'v': [velocity, velocity], 'posture_target': [1., 1.],
                     'targets': {'left': pose, 'right': pose},
                     'tasks': [{'name': 'posture', 'enabled': True, 'state': 2,
                                'residual_at_level': [velocity - 1., velocity - 1.]}]})
    # A skipped row between attempts must not mix their task references.
    rows.insert(1, {'record_type': 'attempt_started'})
    (tmp_path / 'raw.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in rows))
    data = {'model': {'locator': 'fixture'}, 'joint_names': ['a', 'b'],
            'active_joint_names': ['a', 'b'], 'root_frame': 'base_link',
            'frames': {'left': 'left', 'right': 'right'},
            'tcp_offsets': {'left': [0., 0., 0.], 'right': [0., 0., 0.]},
            'limits': {'lower': [-5., -5.], 'upper': [5., 5.], 'velocity': [10., 10.]},
            'units': {}}
    config = {'gain_per_s': 1., 'period_s': 1., 'regularization': 1e-8,
              'preservation_tolerance': 1e-7, 'hard_tolerance': 1e-7,
              'enforcement': 'soft', 'mode': 'ServoStep', 'state_mode': 'evolving',
              'secondary_weight': 1., 'orientation_weight': 1.}
    request = tmp_path / 'request.json'
    request.write_text(json.dumps({'output_dir': str(tmp_path), 'input': data,
                                  'config': config, 'identity': {'method_id': 'mcc-hqp'}}))
    assert verifier.verify(request) == 0
    effects = json.loads((tmp_path / 'secondary_effects.json').read_text())
    assert [row['source_line'] for row in effects] == [1, 3]
    assert [row['task'] for row in effects] == ['posture', 'posture']
    assert [row['primary_only_secondary_cost'] for row in effects] == [1., 1.]
    assert [row['candidate_secondary_cost'] for row in effects] == [.5625, .25]
    assert [row['absolute'] for row in effects] == [.4375, .75]
    assert [row['relative'] for row in effects] == [.4375, .75]
    assert [row['primary_semantic_objective_gap'] for row in effects] == [0., 0.]
