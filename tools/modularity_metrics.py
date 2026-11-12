#!/usr/bin/env python3
"""Measure declared ROS package coupling without ROS or third-party libraries."""
import argparse
import csv
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET


TAGS = {
    'production': {'depend', 'build_depend', 'build_export_depend',
                   'exec_depend', 'run_depend'},
    'buildtool': {'buildtool_depend', 'buildtool_export_depend'},
    'test': {'test_depend'},
    'documentation': {'doc_depend'},
}
SKIP = {'build', 'install', 'log', '__pycache__', 'node_modules'}


def reachable(graph, start):
    seen, pending = set(), list(graph[start])
    while pending:
        node = pending.pop()
        if node not in seen:
            seen.add(node)
            pending.extend(graph[node] - seen)
    return seen - {start}


def analyze(root, scope):
    packages, warnings = {}, []
    for directory, dirs, files in os.walk(root):
        dirs[:] = sorted(d for d in dirs if d not in SKIP and not d.startswith('.'))
        if 'package.xml' not in files:
            continue
        path = Path(directory) / 'package.xml'
        xml = ET.parse(path).getroot()
        name = (xml.findtext('name') or '').strip()
        if not name:
            raise ValueError('Package without name: ' + str(path))
        if name in packages:
            raise ValueError('Duplicate package name: ' + name)
        dependencies = []
        for element in xml:
            if element.tag in set().union(*TAGS.values()):
                dependency = (element.text or '').strip()
                if not dependency:
                    raise ValueError('Empty dependency: ' + str(path))
                dependencies.append({'name': dependency, 'tag': element.tag,
                                     'condition': element.get('condition')})
                if element.get('condition'):
                    warnings.append(f'{name}: condition not evaluated for {dependency}; included conservatively')
            if element.tag in {'group_depend', 'member_of_group'}:
                warnings.append(f'{name}: {element.tag} not expanded into dependency edges')
        packages[name] = {'manifest': str(path.relative_to(root)),
                          'dependencies': dependencies}
    if not packages:
        raise ValueError('No package.xml files found under ' + str(root))
    selected = set().union(*(TAGS[s] for s in scope))
    graph = {name: {d['name'] for d in p['dependencies']
                    if d['tag'] in selected and d['name'] in packages}
             for name, p in packages.items()}
    reverse = {name: set() for name in graph}
    for name, dependencies in graph.items():
        for dependency in dependencies:
            reverse[dependency].add(name)
    closures = {name: reachable(graph, name) for name in graph}
    # Mutual reachability partitions the graph into strongly connected components.
    remaining, cycles = set(graph), []
    while remaining:
        name = min(remaining)
        component = {name} | {other for other in closures[name]
                              if name in closures[other]}
        remaining -= component
        if len(component) > 1 or name in graph[name]:
            cycles.append(sorted(component))
    cyclic = {name for component in cycles for name in component}
    rows = []
    for name in sorted(graph):
        ca, ce = len(reverse[name]), len(graph[name])
        external = {d['name'] for d in packages[name]['dependencies']
                    if d['tag'] in selected and d['name'] not in packages}
        rows.append({'package': name, 'Ca': ca, 'Ce': ce,
                     'instability': ce / (ca + ce) if ca + ce else None,
                     'in_cycle': name in cyclic,
                     'transitive_dependents': len(reachable(reverse, name)),
                     'internal_dependencies': sorted(graph[name]),
                     'external_dependencies': sorted(external)})
    return {'schema_version': 1, 'root': str(root), 'scope': scope,
            'definition': 'Distinct declared package dependencies; source -> dependency. External means outside scanned root.',
            'summary': {'packages': len(graph),
                        'internal_edges': sum(map(len, graph.values())),
                        'cyclic_components': len(cycles),
                        'packages_in_cycles': len(cyclic)},
            'cycles': cycles, 'metrics': rows, 'manifests': packages,
            'warnings': warnings}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path, help='Workspace or source directory')
    parser.add_argument('--output', type=Path, default=Path('reports/modularity'))
    parser.add_argument('--include-buildtool', action='store_true')
    parser.add_argument('--include-test', action='store_true')
    args = parser.parse_args()
    root = args.root.resolve()
    if not root.is_dir():
        parser.error('Root must be an existing directory')
    scope = ['production']
    if args.include_buildtool:
        scope.append('buildtool')
    if args.include_test:
        scope.append('test')
    try:
        report = analyze(root, scope)
    except (ValueError, ET.ParseError, OSError) as error:
        parser.error(str(error))
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'metrics.json').write_text(json.dumps(report, indent=2) + '\n')
    with (args.output / 'metrics.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(report['metrics'][0]))
        writer.writeheader()
        for row in report['metrics']:
            writer.writerow({key: ';'.join(value) if isinstance(value, list) else value
                             for key, value in row.items()})
    lines = ['# Declared package modularity', '',
             'Scope: ' + ', '.join(scope), '',
             '| Package | Ca | Ce | Instability | In cycle | Transitive dependents |',
             '|---|---:|---:|---:|---|---:|']
    for row in report['metrics']:
        instability = 'N/A' if row['instability'] is None else f"{row['instability']:.3f}"
        lines.append(f"| {row['package']} | {row['Ca']} | {row['Ce']} | {instability} | {row['in_cycle']} | {row['transitive_dependents']} |")
    lines += ['', 'Cycles are strongly connected groups, not an enumeration of all paths.',
              'Internal = within scanned root. Counts describe manifests, not verified source or ROS communication.',
              'Transitive dependents exclude the package itself and include direct dependents.',
              '', '## Cycle groups', '']
    lines += [', '.join(component) for component in report['cycles']] or ['None.']
    lines += ['', '## Warnings', '']
    lines += report['warnings'] or ['None.']
    (args.output / 'report.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))
    print('\nReports: ' + str(args.output.resolve()))


if __name__ == '__main__':
    main()
