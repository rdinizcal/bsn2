"""Interpret native outputs. CC/CBO come from tools; WMC is a sum of native CC values."""
import ast
import csv
import json
from pathlib import Path
from statistics import mean

import networkx as nx
import pydot

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'reports/class_modularity'


def class_graph(path):
    """Read Pyreverse UML edges, orienting each as owner/user -> dependency."""
    dot = pydot.graph_from_dot_file(str(path))[0]
    graph = nx.DiGraph()
    for node in dot.get_nodes():
        name = node.get_name().strip('"')
        if '.' in name:
            graph.add_node(name)
    for edge in dot.get_edges():
        source = edge.get_source().strip('"')
        target = edge.get_destination().strip('"')
        arrow = edge.get_attributes().get('arrowhead', '').strip('"')
        # UML diamonds point at the owner. Inheritance and vee point at dependency.
        if arrow in {'diamond', 'odiamond'}:
            source, target = target, source
        graph.add_edge(source, target)
    return graph


def cycle_groups(graph):
    return sorted(sorted(c) for c in nx.strongly_connected_components(graph)
                  if len(c) > 1 or any(graph.has_edge(n, n) for n in c))


def read_csv(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


def main():
    native = json.loads((OUT / 'pyscn.json').read_text())
    assert not native['cbo']['errors'] and not native['complexity']['errors']
    cbo = {(c['file_path'], c['name']): c for c in native['cbo']['classes']}
    cc = read_csv(OUT / 'lizard-native.csv')
    cc_index = {(r['file'], int(r['start'])): r for r in cc}
    graph = class_graph(OUT / 'classes_BSN.dot')
    cycles = cycle_groups(graph)
    rows, method_rows = [], []
    for file in (OUT / 'files.txt').read_text().splitlines():
        for node in ast.walk(ast.parse((ROOT / file).read_text())):
            if not isinstance(node, ast.ClassDef):
                continue
            methods = [m for m in node.body if isinstance(m, (ast.FunctionDef, ast.AsyncFunctionDef))]
            scores = []
            for method in methods:
                if (file, method.lineno) not in cc_index:
                    raise RuntimeError(f'Method missing from native Lizard output: {file}:{method.lineno}')
                source = cc_index[file, method.lineno]
                value = int(source['CCN'])
                scores.append(value)
                method_rows.append({'file': file, 'class': node.name, 'method': method.name,
                                    'line': method.lineno, 'cc_lizard': value})
            c = cbo[file, node.name]
            qualified = '.'.join(Path(file).parts[1:])[:-3] + '.' + node.name
            rows.append({'file': file, 'class': node.name, 'line': node.lineno,
                         'cbo_pyscn': c['metrics']['coupling_count'],
                         'cbo_dependencies': c['metrics']['dependent_classes'],
                         'nom_declared': len(methods), 'wmc_sum_lizard_cc': sum(scores),
                         'max_method_cc_lizard': max(scores, default=0),
                         'pyreverse_outgoing_relations': graph.out_degree(qualified),
                         'pyreverse_incoming_relations': graph.in_degree(qualified),
                         'in_detected_class_cycle': any(qualified in group for group in cycles)})
    for filename, values in [('classes.csv', rows), ('methods.csv', method_rows)]:
        with (OUT / filename).open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(values[0]))
            writer.writeheader()
            for row in values:
                writer.writerow({k: json.dumps(v) if isinstance(v, list) else v for k, v in row.items()})
    vroot = OUT / 'validation'
    validation = json.loads((vroot / 'pyscn-native.json').read_text())
    vcbo = {c['name']: c['metrics']['coupling_count'] for c in validation['cbo']['classes']}
    assert vcbo['ConstructedConsumer'] == 1
    assert vcbo['LocalConsumer'] == vcbo['TypedConsumer'] == vcbo['UntypedConsumer'] == 0
    fixture_graph = class_graph(vroot / 'classes_Cases.dot')
    assert fixture_graph.has_edge('cases.consumer.TypedConsumer', 'cases.dependency.Dependency')
    assert fixture_graph.has_edge('cases.consumer.LocalConsumer', 'cases.consumer.LocalDependency')
    known_cycles = cycle_groups(fixture_graph)
    assert ['cases.loop_a.A', 'cases.loop_b.B'] in known_cycles
    for language in ['python', 'cpp']:
        assert sorted(int(r['CCN']) for r in read_csv(vroot / f'lizard-{language}.csv')) == [1, 2]
    module_native = json.loads((OUT / 'deps-native.json').read_text())['system']
    assert not module_native['errors']
    assert len(graph) == len(rows) == len(cbo)
    cycle_analysis = {'engine': 'networkx ' + nx.__version__,
                      'algorithm': 'strongly_connected_components',
                      'input_native_pyreverse': 'classes_BSN.dot',
                      'scope': 'inferred inheritance and member associations only; UML diamonds reversed to owner -> member',
                      'classes': len(graph), 'distinct_edges': graph.number_of_edges(),
                      'cyclic_components': cycles,
                      'classes_in_cycles': sorted({name for group in cycles for name in group}),
                      'validation_known_cycle_detected': known_cycles}
    (OUT / 'class-cycles-analysis.json').write_text(json.dumps(cycle_analysis, indent=2) + '\n')
    (OUT / 'class-graph-networkx.json').write_text(json.dumps(nx.node_link_data(graph), indent=2) + '\n')
    summary = {'classes': len(rows), 'direct_methods': len(method_rows),
               'native_lizard_functions_including_globals': len(cc),
               'cbo_pyscn_mean': mean(r['cbo_pyscn'] for r in rows),
               'cbo_pyscn_max': max(r['cbo_pyscn'] for r in rows),
               'wmc_lizard_mean': mean(r['wmc_sum_lizard_cc'] for r in rows),
               'wmc_lizard_max': max(r['wmc_sum_lizard_cc'] for r in rows),
               'max_method_cc_lizard': max(r['cc_lizard'] for r in method_rows),
               'methods_cc_over_10': sum(r['cc_lizard'] > 10 for r in method_rows),
               'class_cycle_components_detected': len(cycles),
               'module_cycles_native': module_native['summary']['cyclic_dependencies']}
    (OUT / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    lines = ['# Modularidade por classes — BSN ROS 2', '',
             '**Interpretação do assistente, baseada nas saídas nativas abaixo.** '
             'CBO e CC são calculados pelas ferramentas. WMC_Lizard é a soma dos CC dos métodos '
             'declarados diretamente em cada classe; NOM conta esses métodos. '
             'A identificação da classe proprietária usa a AST Python e as linhas reportadas pelo Lizard.', '',
             '## Relatórios nativos', '',
             '- [pyscn HTML](pyscn-native.html): CBO por classe e complexidade pela definição do pyscn.',
             '- [pyscn JSON](pyscn.json): dependências reconhecidas por classe e configurações.',
             '- [Lizard HTML](lizard-native.html), [CSV](lizard-native.csv) e [texto](lizard-native.txt): CC por função/método.',
             '- [Pyreverse DOT](classes_BSN.dot) e [SVG renderizado por Graphviz](classes-native.svg): relações de classes.',
             '- [pyscn dependências HTML](deps-native.html) e [JSON](deps-native.json): grafo de módulos, separado do grafo de classes.',
             '- [Omen JSON](omen-native.json): diagnóstico adicional de métricas OO; não usado como referência comparativa.', '',
             'As tabelas classes.csv, methods.csv, summary.json e a análise de ciclos são derivados explicitamente '
             'produzidos pelo assistente. O algoritmo de componentes fortemente conexos é o da biblioteca NetworkX.', '',
             '## Escopo e valores', '',
             f"54 arquivos de produção, seis pacotes, {len(rows)} classes e {len(method_rows)} métodos diretamente declarados. "
             'Construtores, destrutores e métodos estáticos/de classe entram no NOM/WMC. '
             'Métodos herdados não entram novamente; classes sem métodos têm NOM=0 e WMC=0. '
             'Funções globais e locais aninhadas não são atribuídas ao WMC da classe.', '',
             '| Medida | Resultado |', '|---|---:|',
             f"| CBO médio (variante pyscn) | {summary['cbo_pyscn_mean']:.4f} |",
             f"| CBO máximo | {summary['cbo_pyscn_max']} |",
             f"| WMC máximo, soma do CC Lizard | {summary['wmc_lizard_max']} |",
             f"| CC máxima de método, Lizard | {summary['max_method_cc_lizard']} |",
             f"| Métodos com CC > 10 | {summary['methods_cc_over_10']} de {len(method_rows)} |",
             f"| Classes em ciclos inferidos pelo Pyreverse | {len(cycle_analysis['classes_in_cycles'])} |",
             f"| Ciclos entre módulos detectados pelo pyscn | {summary['module_cycles_native']} |", '',
             '## Classes prioritárias para inspeção', '',
             '| Classe | CBO pyscn | WMC Lizard | CC máxima | NOM |', '|---|---:|---:|---:|---:|']
    for row in sorted(rows, key=lambda r: r['wmc_sum_lizard_cc'], reverse=True)[:10]:
        lines.append(f"| [{row['class']}](../../{row['file']}#L{row['line']}) | {row['cbo_pyscn']} | {row['wmc_sum_lizard_cc']} | {row['max_method_cc_lizard']} | {row['nom_declared']} |")
    lines += ['', 'DataAccess concentra o maior WMC e está entre as classes de maior CBO. '
              'ReliabilityEngine também concentra complexidade. Esses são candidatos à revisão de '
              'responsabilidades e dependências, não uma conclusão automática de projeto ruim.', '',
              '## CBO: definição operacional e limites', '',
              '- O pyscn inclui usos reconhecidos de classes externas ao arquivo, inclusive bibliotecas/ROS. '
              'Exclui classes pares no mesmo arquivo e referências apenas em anotações de tipo. '
              'Builtin types são excluídos; não confundir builtin types com todas as classes da biblioteca padrão.',
              '- Formula tem CBO=15, com dependências de tipos ast.* da biblioteca padrão. '
              'Esse valor não representa 15 dependências entre classes da BSN.',
              '- DataProcessor tem dependências via self.node que não são completamente resolvidas. '
              'O CBO informado não cobre toda a colaboração dinâmica.',
              '- O fixture ConstructedConsumer mede 1. TypedConsumer e UntypedConsumer medem 0. '
              'LocalConsumer mede 0 embora instancie outra classe no mesmo arquivo. '
              'Esses resultados foram preservados no relatório nativo de validação.',
              '- Não tratar esse CBO como equivalente automático a CBO/TypeCe de uma ferramenta C++.', '',
              '## Ciclos de classes e módulos', '',
              f"O grafo nativo Pyreverse contém {len(graph)} classes e {graph.number_of_edges()} relações distintas "
              'após orientar associação/composição do proprietário para o membro. '
              'NetworkX não encontrou componentes cíclicos nesse grafo. '
              'O fixture A↔B foi detectado pelo mesmo procedimento.', '',
              '**Ausência de ciclos detectados não comprova ausência de ciclos reais.** '
              'O Pyreverse infere herança e atributos, mas referências recebidas sem tipo e chamadas '
              'dinâmicas podem não gerar arestas. Por exemplo, componentes que guardam self.node '
              'podem referenciar o pai sem essa relação aparecer no diagrama.', '',
              'O relatório de imports do pyscn é separado, em nível de módulos. Os pacotes ROS foram '
              'copiados sem alterar bytes para uma pasta temporária com os nomes importáveis corretos '
              '(sensor/sensor.py, adaptation/..., etc.). O mapa de caminhos e hashes está em metadata.json. '
              'Os caminhos temporários nos relatórios nativos não persistem, mas os arquivos correspondentes '
              'continuam identificáveis pelo mapa. Não se usou package.xml como substituto de classes.', '',
              '## Complexidade e comparabilidade com C++', '',
              'O Lizard executou no mesmo código de produção e em exemplos Python/C++ equivalentes. '
              'Nos dois exemplos, straight tem CC=1 e branch tem CC=2. '
              'Assim, a soma é WMC=3 em ambos. O Lizard é a referência proposta para CC nas duas BSNs; '
              'a soma por classe permanece identificada como derivada.', '',
              'Não usar a métrica class_scopes.complexity do pyscn como WMC: ela descreve o corpo '
              'executável da classe, não a soma da complexidade dos métodos.', '',
              'Omen foi conferido como candidato de WMC/NOM, mas no exemplo de validação retornou '
              'WMC=4/NOM=2 em Python e WMC=0/NOM=0 no C++ com métodos inline. Por isso, '
              'seus valores nativos foram mantidos como diagnóstico e rejeitados para essa comparação. '
              'Não comparar WMC Omen em uma versão contra WMC Lizard na outra.', '',
              'Para acoplamento e ciclos C++, CppDepend e Doxygen são candidatos existentes com '
              'análise/saídas nativas; a equivalência de relações com pyscn/Pyreverse ainda precisa '
              'ser validada. Doxygen produz diagramas de classe/colaboração, não automaticamente '
              'o mesmo CBO. Nenhuma BSN C++ foi analisada aqui.', '',
              '## Valores por classe', '',
              '| Classe e arquivo | CBO pyscn | WMC Lizard | CC máxima | NOM |',
              '|---|---:|---:|---:|---:|']
    for row in rows:
        lines.append(f"| [{row['class']}](../../{row['file']}#L{row['line']}) (`{row['file']}`) | {row['cbo_pyscn']} | {row['wmc_sum_lizard_cc']} | {row['max_method_cc_lizard']} | {row['nom_declared']} |")
    lines += ['', '## Validação e reprodução', '',
              '- Exemplos e saídas originais: validation/. A tipagem, a dependência no mesmo arquivo '
              'e o ciclo conhecido documentam o alcance da análise.',
              '- Relatório nativo dos exemplos: [pyscn HTML](validation/pyscn-native.html); '
              '[Pyreverse DOT](validation/classes_Cases.dot); '
              '[Lizard Python](validation/lizard-python.csv) e [C++](validation/lizard-cpp.csv).',
              '- Executar `.venv-metrics/bin/python tools/collect_class_modularity.py` e depois '
              '`.venv-metrics/bin/python tools/summarize_class_modularity.py`.',
              '- Versões e hashes estão em metadata.json. Testes, launch, código gerado e auxiliares '
              'de teste não fazem parte da população.', '',
              '## Fontes primárias', '',
              '- https://github.com/ludo-technologies/pyscn/blob/v1.33.2/internal/analyzer/cbo.go',
              '- https://github.com/terryyin/lizard',
              '- https://pylint.readthedocs.io/en/latest/additional_tools/pyreverse/index.html',
              '- https://networkx.org/documentation/stable/reference/algorithms/generated/networkx.algorithms.components.strongly_connected_components.html',
              '- https://cppdepend.com/documentation/code-metrics',
              '- https://doxygen.nl/manual/features.html', '']
    (OUT / 'report.md').write_text('\n'.join(lines))
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
