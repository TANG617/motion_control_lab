"""Forward verifier operand regressions; no native app execution."""
import importlib.util
from pathlib import Path
import sys
import pytest
HERE=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('e08_verifier_v2',HERE/'verify_evidence_v2.py')
V=importlib.util.module_from_spec(spec);spec.loader.exec_module(V)
sys.path.insert(0,str(HERE.parents[1]/'analyses/A01_priority_and_robustness/tests'))
from test_e08_final_scale import fixture


def test_final_operand_and_real_exceedance():
    for candidate,exceeds in [(.9+9.951406023528397e-8,False),(.9+1.0397834428882424e-7,True)]:
        raw,g=fixture(candidate=candidate);task=raw['evidence_linearization']['tasks'][0]
        scale=V.selected_scale(raw,task,'mcc-hqp-3')
        assert scale['value']==.9 and scale['primary_optimum']==1.
        assert (V.residual_record([[1.]],[1.],[0.],[candidate],scale['value'],1e-7)['within_original_tolerance'] is False)==exceeds


@pytest.mark.parametrize('change',['missing_scale','missing_pass','duplicate_scale','missing_candidate','nonfinite'])
def test_missing_final_evidence_never_inferred(change):
    raw,g=fixture();task=raw['evidence_linearization']['tasks'][0]
    if change=='missing_scale':raw['native_constraints']=[]
    elif change=='missing_pass':raw['selected_priority']=None
    elif change=='duplicate_scale':raw['native_constraints'].append(raw['native_constraints'][0])
    elif change=='missing_candidate':raw['evidence_linearization']['candidate_available']=False
    else:raw['native_constraints'][0]['value']=float('nan')
    raw['scales'][0]['drift']=.1
    assert V.selected_scale(raw,task,'mcc-hqp-3')['value'] is None


def test_weighted_successful_final_scale_only():
    raw,g=fixture();task=raw['evidence_linearization']['tasks'][0];raw['completed_passes']=1
    assert V.selected_scale(raw,task,'mcc-weighted')['value']==1.
    raw['completed_passes']=0;assert V.selected_scale(raw,task,'mcc-weighted')['value'] is None


def test_unscaled_task_unchanged():
    assert V.selected_scale({},dict(enforcement='hard'),'placo-controlled')['value']==1.


def test_offline_recheck_refuses_existing_destination(tmp_path):
    request=tmp_path/'request.json';request.write_text('{"output_dir":"/unused","input":{},"config":{}}')
    with pytest.raises(FileExistsError):V.audit(request,tmp_path/'raw.jsonl',tmp_path)



def test_v2_identity():
    assert V.VERIFIER_VERSION=='e08-verifier.v2'
