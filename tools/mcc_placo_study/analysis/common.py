"""Shared analysis contracts, bounded readers, immutable runs and figure provenance."""
from __future__ import annotations
import argparse
import collections
import csv
import datetime
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
IDENTITY = ('experiment_id', 'case_id', 'method_id', 'arm_id', 'repeat_id',
            'session_id', 'split_id', 'window_id', 'component_id', 'input_hash', 'model_hash')
PAIR_KEYS = ('experiment_id', 'case_id', 'input_hash', 'model_hash', 'window_id',
             'repeat_id', 'session_id', 'split_id', 'component_id')
VERSION = 'exploratory-analysis.v1'


def read_json(path):
    with Path(path).open() as stream:
        return json.load(stream)


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, sort_keys=True,
                                    indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def stable_hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def artifact(path):
    path = Path(path).resolve()
    return dict(locator=str(path), sha256=digest(path), size_bytes=path.stat().st_size)


def signature(path):
    s = Path(path).stat()
    return (s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns, s.st_ctime_ns)


def verify(record):
    path = Path(record['locator'])
    before = signature(path)
    if digest(path) != record['sha256'] or (record.get('size_bytes', before[2]) != before[2]):
        raise ValueError('source hash mismatch: ' + str(path))
    if signature(path) != before:
        raise ValueError('source changed while reading: ' + str(path))
    return before


def csv_rows(path):
    with Path(path).open(newline='') as stream:
        yield from csv.DictReader(stream)


def jsonl_rows(path):
    with Path(path).open() as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def array_rows(path):
    """Stream a JSON array with bounded buffering and strict delimiter validation."""
    decoder = json.JSONDecoder()
    with Path(path).open() as stream:
        buffer = ''; position = 0; eof = False

        def more():
            nonlocal buffer, position, eof
            chunk = stream.read(1024 * 1024)
            buffer = buffer[position:] + chunk
            position = 0
            eof = not chunk

        def whitespace():
            nonlocal position
            while True:
                while position < len(buffer) and buffer[position].isspace():
                    position += 1
                if position < len(buffer) or eof:
                    return
                more()

        more(); whitespace()
        if position >= len(buffer) or buffer[position] != '[':
            raise ValueError('expected JSON array: ' + str(path))
        position += 1; whitespace()
        first = True
        while True:
            if position >= len(buffer):
                raise ValueError('truncated JSON array: ' + str(path))
            if buffer[position] == ']':
                position += 1; whitespace()
                if position != len(buffer):
                    raise ValueError('trailing JSON array data: ' + str(path))
                return
            if not first:
                if buffer[position] != ',':
                    raise ValueError('missing JSON array delimiter: ' + str(path))
                position += 1; whitespace()
                if position < len(buffer) and buffer[position] == ']':
                    raise ValueError('trailing JSON array comma: ' + str(path))
            while True:
                try:
                    row, end = decoder.raw_decode(buffer, position)
                    # A scalar number can end at a chunk boundary; do not truncate it.
                    if end == len(buffer) and not eof:
                        more(); continue
                    break
                except json.JSONDecodeError:
                    if eof:
                        raise ValueError('truncated or malformed JSON array: ' + str(path))
                    more()
            yield row
            position = end; first = False; whitespace()


def write_csv(path, rows, fields=None):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    if fields is None:
        rows = list(rows)
        fields = sorted({k for row in rows for k in row}) or ['status']
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='raise')
        writer.writeheader()
        for row in rows:
            writer.writerow({k: json.dumps(v, sort_keys=True, ensure_ascii=False,
                                            allow_nan=False) if isinstance(v, (dict, list, tuple)) else v
                             for k, v in row.items()})


def unit_id(unit):
    return '/'.join(str(unit.get(k, '')) for k in ('experiment_id', 'case_id', 'arm_id', 'repeat_id'))


def unit_path(unit, name):
    """Only declared artifacts may be consumed; no broad directory scans."""
    expected = Path(unit['directory']) / name
    for a in unit.get('artifacts', []):
        if Path(a['locator']) == expected:
            return expected
    return None


def unit_json(unit, name, default=None):
    p = unit_path(unit, name)
    return read_json(p) if p else default


def base_row(unit):
    result = {k: unit.get(k) for k in IDENTITY}
    result.update(unit_id=unit_id(unit), source_run_id=unit['run_id'],
                  native_status=unit['status'], validation_status=unit['validation']['status'],
                  source_directory=unit['directory'])
    return result


def validate_definition(value):
    import jsonschema
    if value.get('schema_version') not in ('analysis.v1','analysis.v2'): raise ValueError('unsupported analysis version')
    jsonschema.validate(value, read_json(ROOT / ('contracts/definitions/'+value['schema_version']+'.schema.json')))
    if value['analysis_id'] not in ('A01', 'A02'):
        raise ValueError('only A01/A02 implemented')
    directory = {'A01': 'A01_priority_and_robustness', 'A02': 'A02_runtime_and_scheduling'}
    if value['analysis_directory'] != directory[value['analysis_id']]:
        raise ValueError('analysis output ownership mismatch')
    if any('latest' in Path(a['locator']).parts for a in value['campaign'].values()):
        raise ValueError('implicit latest is forbidden')
    expected = ['E06', 'E07', 'E08'] if value['analysis_id'] == 'A01' else ['E09', 'E10']
    if [s['experiment_id'] for s in value['sources']] != expected:
        raise ValueError('analysis source selection differs from declared responsibility')
    if value['phase'] != 'development' or value['statistics']['bootstrap'] or value['statistics']['significance_tests']:
        raise ValueError('this implementation is descriptive non-RT only')
    return value


class Context:
    def __init__(self, definition, output=None):
        self.definition = validate_definition(definition)
        self.output = Path(output) if output else None
        self.units = []
        self.supplemental_units = []
        self.checked = {}
        self.source_records = {}
        self.figures = []
        self.started = time.monotonic()

    def check_artifact(self, record):
        name = record['locator']
        old = self.source_records.get(name)
        if old and old != record:
            raise ValueError('conflicting artifact identities: ' + name)
        if name not in self.checked:
            self.checked[name] = verify(record)
            self.source_records[name] = record

    def preflight(self):
        d = self.definition
        if d['analysis_id'] == 'A02' and not d.get('admission_source'):
            raise ValueError('A02 requires a pinned completed A01 admission source')
        for record in d['campaign'].values():
            self.check_artifact(record)
        index = read_json(d['campaign']['execution_index']['locator'])
        status = read_json(d['campaign']['status']['locator'])
        if not status.get('all_experiment_waves_processed') or status.get('remaining_experiments'):
            raise ValueError('campaign not closed')
        if status.get('mixed_native_runtime_versions'):
            raise ValueError('fresh campaign cannot mix native runtime versions')
        seen = set()
        for source in d['sources']:
            match = [s for s in index['runs'] if s['experiment_id'] == source['experiment_id']]
            if len(match) != 1 or any(match[0][k] != source[k] for k in ('run_id', 'run_dir', 'manifest', 'inventory')):
                raise ValueError('source does not match pinned execution index')
            self.check_artifact(source['manifest']); self.check_artifact(source['inventory'])
            manifest = read_json(source['manifest']['locator'])
            if manifest.get('schema_version') != 'run_manifest.v2' or manifest.get('experiment_id') != source['experiment_id']:
                raise ValueError('experiment manifest identity/version mismatch')
            if manifest['status'] not in d['source_policy']['allowed_statuses']:
                raise ValueError('source status rejected')
            inventory = read_json(source['inventory']['locator'])
            if inventory.get('phase') != 'development' or inventory.get('smoke'):
                raise ValueError('source phase/smoke mismatch')
            expected = {unit_id(u) for u in inventory['required_units']}
            actual = inventory['actual_units']
            if len(actual) != len(expected) or {unit_id(u) for u in actual} != expected:
                raise ValueError('required matrix coverage mismatch')
            manifest_units = {unit_id(u): u for u in manifest['units']}
            for u in actual:
                key = unit_id(u)
                if key in seen or manifest_units.get(key) != u:
                    raise ValueError('duplicate unit or manifest/inventory disagreement: ' + key)
                seen.add(key)
                u = dict(u)
                u['directory'] = str(Path(source['run_dir']) / 'units' / u['experiment_id'] / u['case_id'] / u['arm_id'] / str(u['repeat_id']))
                for a in u.get('artifacts', []):
                    if not Path(a['locator']).resolve().is_relative_to(Path(source['run_dir']).resolve()):
                        raise ValueError('unit artifact outside source run')
                    self.check_artifact(a)
                u['config'] = unit_json(u, 'request.json', {}).get('config', {})
                self.units.append(u)
                if len(self.units) % 100 == 0:
                    self.progress('hashes', len(self.units), d['expected_units'])
            for a in inventory.get('artifacts', []): self.check_artifact(a)
            for key in ('source_inventory', 'build_inventory'):
                if isinstance(inventory.get(key), dict) and 'locator' in inventory[key]:
                    self.check_artifact(inventory[key])
            print('VERIFIED ' + source['experiment_id'] + ' units=' + str(len(actual)), flush=True)
        if len(self.units) != d['expected_units']:
            raise ValueError('declared total mismatch')
        if d.get('admission_source'):
            for a in d['admission_source'].values(): self.check_artifact(a)
            previous = read_json(d['admission_source']['manifest']['locator'])
            if previous.get('run_kind') != 'analysis' or previous.get('analysis_id') != 'A01' or previous['status'] != 'completed':
                raise ValueError('A01 admission analysis not completed')
            definition_record = previous['outputs'].get('definition.json')
            if not definition_record:
                raise ValueError('A01 admission definition missing')
            definition_path = Path(d['admission_source']['manifest']['locator']).parent/'definition.json'
            self.check_artifact(dict(locator=str(definition_path), **definition_record))
            admission_definition = read_json(definition_path)
            if (stable_hash(admission_definition) != previous['definition_hash'] or
                    admission_definition['campaign'] != d['campaign']):
                raise ValueError('A01 admission source belongs to a different campaign or definition')
            if d['schema_version']=='analysis.v2' and (admission_definition.get('schema_version')!='analysis.v2' or
                    admission_definition.get('supplement')!=d['supplement']):
                raise ValueError('A01 admission source belongs to a different supplement')
            for key in ('parity_table', 'pair_admission'):
                a = d['admission_source'][key]
                relative = str(Path(a['locator']).relative_to(Path(d['admission_source']['manifest']['locator']).parent))
                if previous['outputs'].get(relative) != {k:v for k,v in a.items() if k != 'locator'}:
                    raise ValueError('A01 admission table not owned by frozen manifest')
        if d['schema_version']=='analysis.v2':
            from repair.common import load_supplement
            supplement_stages=load_supplement(self)
        counts = collections.Counter((u['status'], u['validation']['status']) for u in self.units)
        result=dict(status='passed', units=len(self.units), checked_artifacts=len(self.checked),
                    status_counts={a+'/'+b:n for (a,b),n in sorted(counts.items())})
        if d['schema_version']=='analysis.v2':
            supplemental=collections.Counter((u['status'],u['validation']['status']) for u in self.supplemental_units)
            result.update(supplemental_units=len(self.supplemental_units),repair_stage_counts=supplement_stages,
                supplemental_status_counts={a+'/'+b:n for (a,b),n in sorted(supplemental.items())})
        return result

    def progress(self, stage, current, total):
        print(f'ANALYSIS {self.definition["analysis_id"]} {stage} {current}/{total}', flush=True)

    def table(self, name, rows, fields=None):
        path = self.output / 'evaluation' / (name if name.endswith('.csv') else name + '.csv')
        write_csv(path, rows, fields)
        return path

    def unchanged(self):
        for path, saved in self.checked.items():
            if signature(path) != saved:
                raise ValueError('source changed during analysis: ' + path)

    def save_sources(self):
        write_json(self.output / 'sources/inventory.json', {
            'sources': self.definition['sources'], 'artifacts': list(self.source_records.values()),
            'source_policy': self.definition['source_policy'], 'phase': 'development',
            'supplement':self.definition.get('supplement'),
            'source_signatures': self.checked})
        rows = []
        for u in self.units:
            r = base_row(u); r.update(reason=u.get('reason'), required=u.get('required'),
                config=u['config'], quality_interpretation='unit status is not per-attempt solve success')
            rows.append(r)
        write_csv(self.output/'combined/units.csv', rows)
        self.table('coverage', rows)
        if self.definition['schema_version']=='analysis.v2':
            from repair.common import evidence_class
            self.table('evidence_classification', [dict(base_row(u), evidence_class=evidence_class(u)) for u in self.units])
            self.table('supplemental_coverage', [dict(base_row(u), repair_stage=u['repair_stage'], evidence_class=u['evidence_class'], reason=u.get('reason')) for u in self.supplemental_units])


def save_figure(ctx, fig, figure_id, name, sources, description, fields=None, selection=None):
    """Renderer consumes frozen evaluation tables; every plot gets a source sidecar."""
    directory = ctx.output / 'figures'; directory.mkdir(parents=True, exist_ok=True)
    stem = f'{figure_id}-{name}'
    for extension in ('svg', 'pdf', 'png'):
        fig.savefig(directory / (stem + '.' + extension), bbox_inches='tight', dpi=160,
                    metadata={'Creator': VERSION} if extension != 'png' else {'Software': VERSION})
    source_records = [artifact(ctx.output / s) for s in sources]
    record = dict(figure_id=figure_id, name=name, description=description,
                  sources=source_records, fields=fields or [], selection=selection or {},
                  analysis_id=ctx.definition['analysis_id'], analysis_run_id=ctx.output.name,
                  definition_hash=stable_hash(ctx.definition), metric_version='study-metrics.v1',
                  phase='development', interpretation='exploratory non-RT, no confidence intervals',
                  display_policy=ctx.definition['display'], renderer_version=VERSION)
    write_json(directory / (stem + '.json'), record)
    ctx.figures.append(record)
    import matplotlib.pyplot as plt
    plt.close(fig)


def plotting():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    matplotlib.rcParams.update({'svg.hashsalt': VERSION, 'pdf.fonttype': 42,
                                'font.size': 9, 'axes.grid': True, 'grid.alpha': .2})
    return plt


def method_policy_reasons(left, right):
    """A named common method needs concrete hash-bound native row audit evidence."""
    lm, rm = left['method_id'], right['method_id']
    if 'production-static' in (lm, rm): return ['historical-production-static']
    if ('placo' in lm) == ('placo' in rm): return []
    from repair.common import COMMON_ACCEPTANCE_HASH
    for unit in (left, right):
        config=unit.get('config', {})
        if config.get('acceptance_contract_hash',config.get('common_acceptance_contract_hash')) != COMMON_ACCEPTANCE_HASH:
            return ['E06-native-acceptance-mismatch']
        record = next((a for a in unit.get('artifacts', []) if Path(a['locator']).name == 'common_servo_audit.json'), None)
        if record is None: return ['common-native-audit-missing']
        verify(record)
        audit = read_json(record['locator'])
        if (audit.get('acceptance_contract_hash') != COMMON_ACCEPTANCE_HASH or
                audit.get('design_comparable') is not True or audit.get('observation_complete') is not True):
            return ['common-native-audit-blocked']
    return []


def compatible(left, right, allowed=()):
    reasons = []
    if (left.get('run_id') is not None and right.get('run_id') is not None and
            left['run_id']!=right['run_id']):
        reasons.append('cross-run-controlled-comparison-not-declared')
    for k in PAIR_KEYS:
        if left.get(k) is None or right.get(k) is None or left.get(k) != right.get(k):
            reasons.append('identity:' + k)
    for name, u in [('baseline', left), ('candidate', right)]:
        if u['status'] != 'completed' or u['validation']['status'] != 'passed':
            reasons.append(name + ':failed-or-unavailable')
    lm, rm = left['method_id'], right['method_id']
    reasons.extend(method_policy_reasons(left, right))
    for k in sorted(set(left['config']) | set(right['config'])):
        if k not in allowed and left['config'].get(k) != right['config'].get(k):
            reasons.append('controlled-factor:' + k)
    return sorted(set(reasons))


def render_existing(module, directory):
    source = Path(directory).resolve()
    manifest = read_json(source/'manifest.json')
    if manifest['status'] != 'completed': raise ValueError('render source incomplete')
    source_signatures = {}
    for rel, record in manifest['outputs'].items():
        child = (source/rel).resolve()
        if not child.is_relative_to(source): raise ValueError('render artifact escapes source')
        source_signatures[str(child)] = verify(dict(locator=str(child), **record))
    definition = read_json(source/'definition.json')
    ctx = Context(definition)
    ctx.checked = source_signatures
    ctx.output = create_output(definition)
    write_json(ctx.output/'definition.json', definition)
    running = manifest_base(ctx)
    running['render_source'] = artifact(source/'manifest.json')
    write_json(ctx.output/'manifest.json', running)
    try:
        for name in ('combined', 'evaluation', 'sources'):
            if (source/name).exists(): shutil.copytree(source/name, ctx.output/name)
        for relative in manifest['outputs']:
            if Path(relative).parent == Path('.') and Path(relative).suffix == '.md':
                shutil.copyfile(source/relative, ctx.output/relative)
        module.render(ctx)
        finish(ctx, running)
    except BaseException as error:
        fail(ctx, running, error); raise
    return ctx.output


def create_output(definition):
    parent = ROOT/'analyses'/definition['analysis_directory']/'runs'
    if shutil.disk_usage(ROOT).free < 16 * 1024**3:
        raise ValueError('analysis storage guard: less than 16 GiB available; no automatic deletion')
    parent.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    path = parent/(stamp+'-'+stable_hash(definition)[:10]); path.mkdir(exist_ok=False)
    return path


def code_inventory(definition):
    paths = list((ROOT/'tools/mcc_placo_study/analysis').rglob('*.py'))
    paths += list((ROOT/'analyses'/definition['analysis_directory']).glob('*.py'))
    paths += [ROOT/'tools/mcc_placo_study/metrics.py']
    if definition['schema_version']=='analysis.v2':
        paths+=list((ROOT/'tools/mcc_placo_study/repair').rglob('*.py'))
        for folder in (ROOT/'experiments').glob('E*'):
            if folder.name[:3] in ('E06','E07','E08','E09','E10'):paths+=list(folder.glob('*.py'))
    return [artifact(p) for p in sorted(paths) if '__pycache__' not in p.parts]


def manifest_base(ctx):
    def git(*args):
        return subprocess.check_output(['git', '-C', str(ROOT), *args], text=True).strip()
    return dict(schema_version='run_manifest.v3', run_kind='analysis',
                analysis_id=ctx.definition['analysis_id'], run_id=ctx.output.name, status='running',
                phase='development', definition_hash=stable_hash(ctx.definition),
                metric_version='study-metrics.v1', analysis_version=VERSION,
                supplemental_metric_version='evidence-repair.v1' if ctx.definition['schema_version']=='analysis.v2' else None,
                supplemental_metric_versions=ctx.definition.get('repair_metrics',{}),
                code=code_inventory(ctx.definition), command=sys.argv,
                source_control={'lab_revision':git('rev-parse','HEAD'),
                                'dirty':bool(git('status','--porcelain')),
                                'reproduction':'exact analysis code hashes above; original source fingerprints remain in pinned experiment inventories'},
                environment={'python':sys.version,'platform':platform.platform(),
                             'dependencies':{n:importlib.metadata.version(n) for n in ('numpy','matplotlib','jsonschema')}},
                source_policy=ctx.definition['source_policy'], outputs={})


def finish(ctx, manifest):
    ctx.unchanged()
    for code in manifest['code']:
        verify(code)
    if ctx.definition['schema_version']=='analysis.v2':
        snapshots=[]
        for code in manifest['code']:
            target=ctx.output/'sources/analysis_code'/Path(code['locator']).relative_to('/')
            target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(code['locator'],target)
            snapshot=artifact(target)
            if snapshot['sha256']!=code['sha256']:raise ValueError('analysis source changed while archiving')
            snapshots.append({'original':code,'snapshot':snapshot})
        write_json(ctx.output/'sources/code_inventory.json',snapshots)
        # Analysis modules can introduce explicitly checked source-policy artifacts
        # after preflight. Retain those too, without replacing evaluation tables.
        source_index=ctx.output/'sources/inventory.json'
        if ctx.source_records and source_index.exists():
            inventory=read_json(source_index);inventory.update(artifacts=list(ctx.source_records.values()),source_signatures=ctx.checked)
            write_json(source_index,inventory)
    if ctx.figures: write_json(ctx.output/'figures/index.json', ctx.figures)
    manifest.update(status='completed', sources_unchanged=True,
                    scientific_status='exploratory-only; formal claims deferred',
                    outputs={str(p.relative_to(ctx.output)):{k:v for k,v in artifact(p).items() if k!='locator'}
                             for p in sorted(ctx.output.rglob('*')) if p.is_file() and p.name!='manifest.json'})
    import jsonschema
    jsonschema.validate(manifest, read_json(ROOT/'contracts/manifests/run_manifest.v3.schema.json'))
    write_json(ctx.output/'manifest.json', manifest)


def fail(ctx, manifest, error):
    write_json(ctx.output/'failure.json', {'error':str(error),'type':type(error).__name__, 'partial_evidence_preserved':True})
    manifest.update(status='interrupted' if isinstance(error, KeyboardInterrupt) else 'failed')
    if ctx.definition['schema_version']=='analysis.v2':
        manifest['outputs']={str(p.relative_to(ctx.output)):{k:v for k,v in artifact(p).items() if k!='locator'}
                            for p in sorted(ctx.output.rglob('*')) if p.is_file() and p.name!='manifest.json'}
    write_json(ctx.output/'manifest.json', manifest)


def cli(module, default_definition):
    parser = argparse.ArgumentParser(description=module.__doc__)
    parser.add_argument('--definition', default=str(default_definition))
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--render-only', metavar='ANALYSIS_RUN')
    args = parser.parse_args()
    if args.render_only:
        if args.dry_run: parser.error('--dry-run and --render-only are exclusive')
        print('ANALYSIS_RUN='+str(render_existing(module,args.render_only)),flush=True); return 0
    d = validate_definition(read_json(args.definition)); ctx = Context(d)
    if args.dry_run:
        result=ctx.preflight();ctx.unchanged(); print(json.dumps(result),flush=True);return 0
    ctx.output=create_output(d);write_json(ctx.output/'definition.json',d)
    manifest=manifest_base(ctx);write_json(ctx.output/'manifest.json',manifest)
    try:
        result=ctx.preflight();ctx.save_sources()
        module.analyze(ctx)
        write_json(ctx.output/'evaluation/source_validation.json',result)
        module.render(ctx)
        finish(ctx,manifest)
    except BaseException as error:
        fail(ctx,manifest,error);raise
    print('ANALYSIS_RUN='+str(ctx.output),flush=True);return 0
