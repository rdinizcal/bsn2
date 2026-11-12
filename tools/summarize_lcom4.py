"""Format results from external tools; does not calculate class cohesion."""
import ast
import csv
import hashlib
import json
from collections import Counter
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'reports/lcom4'


def parse_potfur(text):
    scores = {}
    for line in text.splitlines():
        fields = [field.strip() for field in line.split('|')]
        if len(fields) == 4 and fields[2].isdigit():
            scores[fields[1]] = int(fields[2])
    return scores


def main():
    raw = json.loads((OUT / 'pyscn.json').read_text())
    report = raw['lcom']
    if report.get('errors') or report.get('failures'):
        raise RuntimeError('External analyzer reported errors; inspect pyscn.json')
    primary = {(c['file_path'], c['name']): c for c in report['classes']}
    secondary = parse_potfur((OUT / 'potfur.txt').read_text())
    files = (OUT / 'files.txt').read_text().splitlines()
    rows, hashes = [], {}
    for file in files:
        path = ROOT / file
        hashes[file] = hashlib.sha256(path.read_bytes()).hexdigest()
        for node in ast.walk(ast.parse(path.read_text())):
            if not isinstance(node, ast.ClassDef):
                continue
            c = primary.get((file, node.name))
            metric = c['metrics'] if c else {}
            methods = metric.get('total_methods')
            excluded = metric.get('excluded_methods')
            participating = methods - excluded if c else None
            key = file[:-3].replace('/', '.') + '.' + node.name
            if key not in secondary:
                raise RuntimeError('Class missing from potfur output: ' + key)
            rows.append({'package': Path(file).parts[0], 'file': file,
                         'class': node.name, 'line': node.lineno,
                         'pyscn_lcom4': metric.get('lcom4'),
                         'potfur_lcom4': secondary[key],
                         'pyscn_total_methods': methods,
                         'pyscn_excluded_methods': excluded,
                         'pyscn_participating_methods': participating,
                         'pyscn_method_groups': metric.get('method_groups', []),
                         'interpretation': 'omitida' if c is None else
                         'sem metodos participantes' if participating == 0 else
                         'um metodo participante' if participating == 1 else
                         'dois ou mais metodos participantes'})
    with (OUT / 'classes.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        for row in rows:
            writer.writerow({k: json.dumps(v) if isinstance(v, list) else v
                             for k, v in row.items()})
    validation = json.loads((OUT / 'validation/pyscn.json').read_text())['lcom']
    pv = {c['name']: c['metrics']['lcom4'] for c in validation['classes']}
    sv = {key.rsplit('.', 1)[-1]: value for key, value in parse_potfur(
        (OUT / 'validation/potfur.stdout').read_text()).items()}
    for name, expected in [('SharedState', 1), ('SplitState', 2), ('CallBridge', 1)]:
        assert pv[name] == sv[name] == expected, (name, pv[name], sv[name])
    eligible = [r for r in rows if (r['pyscn_participating_methods'] or 0) >= 2]
    nonparticipating = [r for r in rows if r['pyscn_participating_methods'] == 0]
    trivial = [r for r in rows if r['pyscn_participating_methods'] == 1]
    metadata = {'pyscn_version': raw['version'],
                'potfur_commit': 'b8b7d75c149864ab8c2cab7296a06da01a4a3359',
                'omen_version': '4.30.0', 'omen_metric': 'LCOM3, excluded from LCOM4 conclusions',
                'source_sha256': hashes,
                'population': {'source_files': len(files), 'class_inventory': len(rows),
                               'pyscn_reported_classes': len(primary),
                               'pyscn_zero_participating_methods': len(nonparticipating),
                               'pyscn_one_participating_method': len(trivial),
                               'pyscn_two_or_more_participating_methods': len(eligible)}}
    (OUT / 'metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
    lines = ['# LCOM4 das classes da BSN ROS 2', '',
             '**Interpretação e consolidação produzidas pelo assistente.** '
             'Valores calculados pelas ferramentas externas, sem implementação própria de LCOM4.', '',
             '## Relatórios originais das ferramentas', '',
             '- [HTML nativo do pyscn](pyscn-native.html): resumo e tabela por classe.',
             '- [JSON nativo do pyscn](pyscn.json): valores e grupos de métodos completos.',
             '- [Tabela nativa do potfur/lcom](potfur.txt): segunda medição por classe.',
             '- [CSV consolidado pelo assistente](classes.csv): organização das saídas, não relatório nativo.',
             '- [Equivalência com C++](../../tools/METRICS_COMPARISON.md): ferramentas e limitações verificadas.', '',
             'Medição estática do código de produção; nenhum nó ROS foi executado ou alterado.', '',
             f"Escopo: {len(files)} arquivos Python; {len(rows)} classes inventariadas; "
             f"{len(primary)} classes reportadas pelo pyscn e {len(secondary)} pelo potfur/lcom.", '',
             '## Ferramentas e definição', '',
             '- **pyscn 1.33.2:** grupos conectados por atributos compartilhados ou chamadas internas. '
             'Exclui construtores, métodos herdados, estáticos/de classe, abstratos/vazios, '
             'métodos sem estado e sem conexão de chamada e certos acessores triviais isolados. '
             'Pode reportar 1 mesmo quando não resta nenhum método participante. Enums são omitidos.',
             '- **potfur/lcom 0.1.0**, commit `b8b7d75c149864ab8c2cab7296a06da01a4a3359`: '
             'segunda medição. Exclui __init__, métodos sem acesso a self/cls e classmethods. '
             'Referências self.metodo() não definido localmente podem conectar métodos pelo mesmo nome, '
             'inclusive chamadas herdadas de logging. O resultado não é intercambiável com pyscn.',
             '- **Omen 4.30.0:** executado no mesmo conjunto de arquivos. A implementação instalada '
             'calcula **LCOM3**, sem arestas de chamada, apesar de a página consultada anunciar LCOM4. '
             'Resultado preservado em omen.json apenas como diagnóstico; não usado como LCOM4.', '',
             'LCOM4 = 1 significa um grupo conectado; >1 sugere grupos independentes a investigar. '
             'Não demonstra responsabilidade única nem testabilidade. Nenhum método participante '
             'não constitui evidência de boa coesão.', '',
             '## Resultados', '',
             f"pyscn, resultado bruto: média {report['summary']['average_lcom']:.4f}; "
             f"máximo {report['summary']['max_lcom']}; distribuição "
             f"{report['summary']['lcom_distribution']}.", '',
             f"Há {len(nonparticipating)} classes sem métodos participantes e {len(trivial)} com apenas um. "
             'Esses casos estão separados no CSV e não devem inflar a conclusão de coesão.', '',
             f"Subconjunto com ao menos dois métodos participantes no pyscn: {len(eligible)} classes; "
             f"distribuição LCOM4 {dict(sorted(Counter(r['pyscn_lcom4'] for r in eligible).items()))}.", '',
             '| Pacote | Classes com ≥2 métodos participantes | LCOM4=1 | LCOM4>1 |',
             '|---|---:|---:|---:|']
    for package in sorted({r['package'] for r in rows}):
        group = [r for r in eligible if r['package'] == package]
        lines.append(f"| {package} | {len(group)} | {sum(r['pyscn_lcom4'] == 1 for r in group)} | {sum(r['pyscn_lcom4'] > 1 for r in group)} |")
    lines += ['', '## Valores por classe', '',
              '“—” = omitida pelo pyscn; “0 métodos” e “1 método” referem-se aos participantes após suas exclusões.', '',
              '| Classe (arquivo) | pyscn LCOM4 | potfur LCOM4 | Métodos participantes pyscn |',
              '|---|---:|---:|---:|']
    for row in rows:
        relative = '../../' + row['file']
        score = row['pyscn_lcom4'] if row['pyscn_lcom4'] is not None else '—'
        participating = row['pyscn_participating_methods']
        lines.append(f"| [{row['class']}]({relative}#L{row['line']}) (`{row['file']}`) | {score} | {row['potfur_lcom4']} | {participating if participating is not None else '—'} |")
    lines += ['', '## Grupos desconectados encontrados pelo pyscn', '']
    for c in report['classes']:
        if c['metrics']['lcom4'] > 1:
            lines += [f"### {c['name']} — {c['file_path']}", '']
            for i, group in enumerate(c['metrics']['method_groups'], 1):
                lines.append(f"{i}. " + ', '.join('`' + method + '`' for method in group))
            lines.append('')
    lines += ['## Divergências relevantes', '',
              '- **SystemMonitor:** pyscn=2, potfur=1. energy_callback utiliza um publicador próprio; '
              'chamadas comuns a get_logger() herdado podem unir os métodos no potfur. '
              'O fixture InheritedLogging reproduz a divergência.',
              '- **ReliabilityEngine:** pyscn=1, potfur=2. get_prefix é um acessor isolado do atributo '
              'prefix e é excluído pelo pyscn. O fixture SeparateGetters evidencia a política.',
              '- Casos sem métodos elegíveis têm valores convencionais diferentes: 1 no pyscn e '
              '0 no potfur. Não comparar médias brutas das duas ferramentas.', '',
              '## Validação de ferramentas', '',
              '| Exemplo | pyscn | potfur | Observação |', '|---|---:|---:|---|']
    notes = {'SharedState': 'Atributo compartilhado: esperado 1',
             'SplitState': 'Dois estados desconectados, construtor excluído: esperado 2',
             'CallBridge': 'Conexão por chamada interna: esperado 1',
             'OnlyConstructor': 'Sem métodos participantes: convenções distintas',
             'Stateless': 'Métodos sem estado excluídos: convenções distintas',
             'SeparateGetters': 'pyscn exclui os acessores triviais isolados',
             'InheritedLogging': 'potfur conecta pela chamada herdada get_logger'}
    for name, note in notes.items():
        lines.append(f'| {name} | {pv[name]} | {sv[name]} | {note} |')
    lines += ['', '## Escopo, reprodução e limites', '',
              '- Arquivos de entrada: files.txt. Hashes SHA-256 e versões: metadata.json.',
              '- Testes, auxiliares *_tests.py, test_*.py e test_components foram excluídos. '
              'build/install/log, launch e features não entram nos diretórios selecionados.',
              '- bsn_interfaces não contém classes Python de produção escritas manualmente neste escopo; '
              'código gerado não foi incluído.',
              '- Reexecutar a medição: `bash tools/run_lcom4.sh`. Instalação fixada em tools/requirements-lcom4.txt.',
              '- Resultados brutos, sem arredondamento: pyscn.json e potfur.txt; tabela consolidada: classes.csv.',
              '- Atributos amplos como self.node podem conectar métodos com responsabilidades distintas. '
              'LCOM4=1 é evidência de conectividade estrutural, não garantia de bom projeto.',
              '- Apenas ROS 2 foi medido. Melhora exige resultados ROS 1 com definição e exclusões equivalentes. '
              'Não usar diretamente a média desta variante do pyscn contra outra implementação em C++.', '',
              '## Fontes', '',
              '- https://github.com/ludo-technologies/pyscn/tree/v1.33.2/internal/analyzer',
              '- https://github.com/potfur/lcom/tree/b8b7d75c149864ab8c2cab7296a06da01a4a3359/src',
              '- https://github.com/panbanda/omen/blob/omen-v4.30.0/src/analyzers/cohesion.rs', '']
    (OUT / 'report.md').write_text('\n'.join(lines))
    print(json.dumps(metadata['population'], indent=2))
    print('Relatório e CSV: ' + str(OUT))


if __name__ == '__main__':
    main()
