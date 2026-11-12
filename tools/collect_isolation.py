"""Run local Semgrep rules, preserve native outputs and record the scan scope."""
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'reports/isolation'
SEMGREP = ROOT / '.venv-semgrep/bin/semgrep'
PACKAGES = ['adaptation', 'central_hub', 'patient', 'sensor', 'shared_components', 'system_monitor']


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    files = sorted(str(p.relative_to(ROOT)) for package in PACKAGES
                   for p in (ROOT / package / package).rglob('*.py')
                   if 'test_components' not in p.parts and not p.name.endswith('_tests.py')
                   and not p.name.startswith('test_'))
    (OUT / 'files.txt').write_text('\n'.join(files) + '\n')
    rules = ['tools/semgrep/isolation-python.yml', 'tools/semgrep/isolation-cpp.yml']
    counts = {rule: len(re.findall(r'^  - id:', (ROOT / rule).read_text(), re.M)) for rule in rules}
    commands = []
    with tempfile.TemporaryDirectory(prefix='bsn-semgrep-') as tmp:
        env = {**os.environ, 'SEMGREP_SETTINGS_FILE': str(Path(tmp) / 'settings.yml'),
               'SEMGREP_LOG_FILE': str(Path(tmp) / 'semgrep.log'),
               'XDG_CACHE_HOME': str(Path(tmp) / 'cache')}

        def run(args, name):
            command = [str(SEMGREP), 'scan', '--metrics', 'off', '--disable-version-check', *map(str, args)]
            result = subprocess.run(command, cwd=ROOT, env=env, text=True, capture_output=True)
            commands.append({'command': command, 'exit_code': result.returncode})
            (OUT / (name + '.stdout')).write_text(result.stdout)
            (OUT / (name + '.stderr')).write_text(result.stderr)
            if result.returncode:
                raise RuntimeError(f'Semgrep failed; see {name}.stderr')
            return result.stdout + result.stderr

        for language, extension, rule in [('python', 'py', rules[0]), ('cpp', 'cpp', rules[1])]:
            target = f'tools/semgrep/tests/isolation-{language}.{extension}'
            text = run(['--test', '--config', rule, target], language + '-rule-tests')
            (OUT / (language + '-rule-tests.txt')).write_text(text)
            match = re.search(r'(\d+)/(\d+):.*All tests passed', text)
            if not match or tuple(map(int, match.groups())) != (counts[rule], counts[rule]):
                raise RuntimeError(f'Incomplete rule validation: {language}')
            run(['--config', rule, '--no-git-ignore', '--strict',
                 '--json-output', OUT / f'validation-{language}-native.json', target],
                'validation-' + language)
        command = ['--config', rules[0], '--no-git-ignore', '--disable-nosem', '--strict',
                   '--jobs', '2', '--json-output', OUT / 'semgrep-native.json',
                   '--sarif-output', OUT / 'semgrep-native.sarif',
                   '--text-output', OUT / 'semgrep-native.txt', *files]
        run(command, 'scan')
    report = json.loads((OUT / 'semgrep-native.json').read_text())
    if report['errors'] or report.get('skipped_rules'):
        raise RuntimeError('Semgrep reported errors or skipped rules')
    if set(report['paths']['scanned']) != set(files):
        raise RuntimeError('Scanned file set does not match requested production scope')
    metadata = {'semgrep_version': report['version'], 'engine': report.get('engine_requested'),
                'source_files': len(files), 'rules_count': counts,
                'source_sha256': {f: hashlib.sha256((ROOT / f).read_bytes()).hexdigest() for f in files},
                'rules_sha256': {f: hashlib.sha256((ROOT / f).read_bytes()).hexdigest() for f in rules},
                'fixtures_sha256': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                                    for p in sorted((ROOT / 'tools/semgrep/tests').glob('isolation-*'))},
                'rule_authorship': 'Project-specific audit rules authored by the assistant; not a standard Semgrep testability metric',
                'scan': 'ROS 2 production Python only; C++ rules executed on fixtures only',
                'runtime_execution': False,
                'native_outputs': ['semgrep-native.json', 'semgrep-native.sarif', 'semgrep-native.txt'],
                'deduplication': 'Native rule results preserved; a location can match multiple rules'}
    (OUT / 'metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
    (OUT / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
    print(f"Semgrep {report['version']}: {len(files)} files, {len(report['results'])} findings, no analysis errors")


if __name__ == '__main__':
    main()
