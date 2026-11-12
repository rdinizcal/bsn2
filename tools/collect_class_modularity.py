"""Run existing analyzers and preserve their native outputs; no metric implementation."""
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'reports/class_modularity'
BIN = ROOT / '.venv-metrics/bin'
PACKAGES = ['adaptation', 'central_hub', 'patient', 'sensor',
            'shared_components', 'system_monitor']


def run(args, log, cwd=ROOT, output=None, env=None):
    result = subprocess.run(list(map(str, args)), cwd=cwd, env=env,
                            text=True, capture_output=True)
    log.write_text(result.stderr if output else result.stdout + result.stderr)
    if output:
        output.write_text(result.stdout)
    if result.returncode:
        raise RuntimeError(f'Command failed ({result.returncode}): {args}; see {log}')


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    files = sorted(str(p.relative_to(ROOT)) for name in PACKAGES
                   for p in (ROOT / name / name).rglob('*.py')
                   if 'test_components' not in p.parts
                   and not p.name.endswith('_tests.py')
                   and not p.name.startswith('test_'))
    (OUT / 'files.txt').write_text('\n'.join(files) + '\n')
    versions = {name: importlib.metadata.version(name)
                for name in ['pyscn', 'pylint', 'astroid', 'lizard', 'networkx', 'pydot', 'jinja2']}
    config = OUT / 'pyscn.toml'
    config.write_text('project_root = "../.."\n[cbo]\nshow_zeros = true\n')
    for fmt, filename in [('json', 'pyscn.json'), ('html', 'pyscn-native.html')]:
        run([BIN / 'pyscn', 'analyze', '--select', 'cbo,complexity',
             '--min-cbo', '0', '--config', config, '--' + fmt, '--no-open',
             '--output', OUT / filename, *files], OUT / (filename + '.log'))
    for fmt, filename in [(['--csv', '-V'], 'lizard-native.csv'),
                          (['--html'], 'lizard-native.html'), ([], 'lizard-native.txt')]:
        run([BIN / 'lizard', '-l', 'python', '-i', '-1', *fmt, *files],
            OUT / (filename + '.stderr'), output=OUT / filename)
    mapping = {}
    with tempfile.TemporaryDirectory(prefix='bsn-class-metrics-') as tmp:
        stage = Path(tmp)
        staged = []
        for file in files:
            relative = Path(*Path(file).parts[1:])
            destination = stage / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / file, destination)
            staged.append(str(relative))
            mapping[str(relative)] = file
        env = {**os.environ, 'PYTHONPATH': str(stage), 'PYTHONDONTWRITEBYTECODE': '1'}
        run([BIN / 'pyreverse', '--source-roots', stage, '-k', '-m', 'y',
             '-p', 'BSN', '-o', 'dot', '-d', OUT, *staged],
            OUT / 'pyreverse.log', cwd=stage, env=env)
        (stage / 'pyscn.toml').write_text('project_root = "."\n')
        for fmt, filename in [('json', 'deps-native.json'), ('html', 'deps-native.html')]:
            run([BIN / 'pyscn', 'analyze', '--select', 'deps', '--config', 'pyscn.toml',
                 '--' + fmt, '--no-open', '--output', OUT / filename, *staged],
                OUT / (filename + '.log'), cwd=stage)
        run([BIN / 'omen', '-p', stage, '-f', 'json', 'cohesion', '--top', '0'],
            OUT / 'omen-native.stderr', cwd=stage, output=OUT / 'omen-native.json')
    run(['dot', '-Tsvg', OUT / 'classes_BSN.dot', '-o', OUT / 'classes-native.svg'],
        OUT / 'graphviz.log')
    validation = OUT / 'validation'
    validation_config = validation / 'pyscn.toml'
    validation_config.write_text('project_root = "."\n[cbo]\nshow_zeros = true\n')
    validation_files = sorted(str(p.relative_to(validation)) for p in (validation / 'cases').rglob('*.py'))
    for fmt, name in [('json', 'pyscn-native.json'), ('html', 'pyscn-native.html')]:
        run([BIN / 'pyscn', 'analyze', '--select', 'cbo,complexity,deps', '--min-cbo', '0',
             '--config', validation_config,
             '--' + fmt, '--no-open', '--output', validation / name, *validation_files],
            validation / (name + '.log'), cwd=validation)
    run([BIN / 'pyreverse', '--source-roots', validation, '-k', '-m', 'y', '-p', 'Cases',
         '-o', 'dot', '-d', validation, *validation_files], validation / 'pyreverse.log',
        cwd=validation, env={**os.environ, 'PYTHONPATH': str(validation)})
    for source, language, name in [('cases/complexity_example.py', 'python', 'lizard-python.csv'),
                                   ('complexity_example.cpp', 'cpp', 'lizard-cpp.csv')]:
        run([BIN / 'lizard', '-l', language, '-i', '-1', '--csv', '-V', source],
            validation / (name + '.stderr'), cwd=validation, output=validation / name)
    run([BIN / 'omen', '-p', validation, '-f', 'json', 'cohesion', '--top', '0'],
        validation / 'omen-native.stderr', output=validation / 'omen-native.json')
    metadata = {'versions': versions,
                'omen_version': subprocess.check_output([BIN / 'omen', '--version'], text=True).strip(),
                'graphviz_version': subprocess.run(['dot', '-V'], capture_output=True, text=True).stderr.strip(),
                'sources_sha256': {f: hashlib.sha256((ROOT / f).read_bytes()).hexdigest() for f in files},
                'normalized_path_to_source': mapping,
                'source_file_count': len(files), 'scope': 'production Python files; no ROS runtime or tests',
                'class_graph': 'Pyreverse inference of inheritance and member associations, not all uses',
                'complexity': 'Lizard default CCN; constructors/destructors included; -i -1 retains warnings but disables failure on thresholds',
                'cbo': 'pyscn defaults with show_zeros=true; excludes same-file peers and annotation-only dependencies; includes recognized external class uses',
                'module_graph': 'Separate native analysis on byte-identical temporary copy with ROS source roots normalized'}
    (OUT / 'metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print('Native outputs saved in ' + str(OUT))


if __name__ == '__main__':
    main()
