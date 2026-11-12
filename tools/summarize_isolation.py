"""Interpret Semgrep's native findings; no testability score is computed."""
import ast
from collections import Counter
import csv
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'reports/isolation'


def main():
    native = json.loads((OUT / 'semgrep-native.json').read_text())
    assert not native['errors'] and not native.get('skipped_rules')
    classes, findings, totals = [], [], Counter()
    for file in (OUT / 'files.txt').read_text().splitlines():
        tree = ast.parse((ROOT / file).read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef):
                classes.append({'file': file, 'class': node.name, 'line': node.lineno,
                                'end_line': node.end_lineno,
                                'bases': [ast.unparse(base) for base in node.bases],
                                'findings': []})
    for finding in native['results']:
        metadata = finding['extra']['metadata']
        category, kind = metadata['category'], metadata['kind']
        totals[category] += 1
        owners = [c for c in classes if c['file'] == finding['path']
                  and c['line'] <= finding['start']['line'] <= c['end_line']]
        owner = min(owners, key=lambda c: c['end_line'] - c['line']) if owners else None
        row = {'file': finding['path'], 'line': finding['start']['line'],
               'end_line': finding['end']['line'], 'class': owner['class'] if owner else '',
               'rule_id': finding['check_id'], 'category': category, 'kind': kind,
               'message': finding['extra']['message']}
        if owner:
            owner['findings'].append(row)
        findings.append(row)
    class_rows = []
    for c in classes:
        infra = [f for f in c['findings'] if f['kind'] == 'infrastructure']
        direct = [f for f in c['findings'] if f['category'] == 'constructor_effect']
        helper = [f for f in c['findings'] if f['category'] == 'constructor_helper_wait']
        supplied = [f for f in c['findings'] if f['kind'] == 'substitution_candidate']
        contract = [f for f in c['findings'] if f['kind'] == 'runtime_contract']
        class_rows.append({'file': c['file'], 'class': c['class'], 'line': c['line'],
                           'bases': '; '.join(c['bases']),
                           'infrastructure_categories': '; '.join(sorted({f['category'] for f in infra})),
                           'infrastructure_rule_occurrences': len(infra),
                           'direct_constructor_effects': len(direct),
                           'constructor_helper_waits': len(helper),
                           'ros_base_contract': bool(contract),
                           'supplied_collaborator_candidates': len(supplied),
                           'interpretation': 'revisar dependencias de infraestrutura'
                           if infra or contract else 'nenhum indicio direto nas regras; nao comprova isolamento'})
    for name, values in [('classes.csv', class_rows), ('findings.csv', findings)]:
        with (OUT / name).open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(values[0]))
            writer.writeheader()
            writer.writerows(values)
    summary = {'source_files': len(native['paths']['scanned']), 'classes': len(classes),
               'native_rule_occurrences': len(findings), 'occurrences_by_category': dict(sorted(totals.items())),
               'classes_with_direct_infrastructure_findings': sum(bool(c['infrastructure_categories']) for c in class_rows),
               'classes_with_direct_constructor_effects': sum(c['direct_constructor_effects'] > 0 for c in class_rows),
               'classes_with_constructor_helper_wait': sum(c['constructor_helper_waits'] > 0 for c in class_rows),
               'classes_with_ros_base_contract': sum(c['ros_base_contract'] for c in class_rows),
               'classes_with_supplied_collaborator_candidate': sum(c['supplied_collaborator_candidates'] > 0 for c in class_rows),
               'findings_outside_classes': sum(not f['class'] for f in findings),
               'errors': native['errors'], 'testability_score': None}
    (OUT / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    # Verify concrete claims against native tool output, rather than a computed score.
    def has(category, suffix):
        return any(f['category'] == category and f['file'].endswith(suffix) for f in findings)
    assert has('constructor_helper_wait', 'sensor/components/data_processor.py')
    assert has('file_io', 'adaptation/data_access/data_access.py')
    assert has('constructor_effect', 'system_monitor/node_monitor.py')
    assert not any(f['file'].endswith('shared_components/battery.py') for f in findings)
    labels = {'ros_node_base': 'Herança direta de Node/LifecycleNode',
              'ros_parameters': 'Parâmetros via API do node',
              'ros_resource': 'Criação de clientes/serviços/publicadores/subscritores ROS',
              'ros_timer': 'Timers ROS', 'ros_runtime': 'Inicialização/spin ROS',
              'service_wait': 'Esperas por serviços/actions', 'client_call': 'Possíveis chamadas de cliente',
              'file_io': 'Acesso a arquivos', 'directory_io': 'Criação de diretórios',
              'real_clock': 'Relógio real', 'real_sleep': 'Espera temporal',
              'thread_creation': 'Criação de thread/executor de threads',
              'process_execution': 'Processos externos', 'environment_read': 'Variáveis de ambiente',
              'constructor_effect': 'Efeito direto no construtor (sobrepõe outras regras)',
              'constructor_helper_wait': 'Espera em helper chamado pelo construtor (sobreposição)',
              'supplied_collaborator': 'Colaborador recebido: candidato à substituição'}
    lines = ['# Isolamento para testes — análise estática Semgrep', '',
             '**Interpretação do assistente sobre resultados nativos do Semgrep.** '
             'As regras são critérios de auditoria escritos para esta BSN; não são uma métrica '
             'padronizada de testabilidade nem um pacote oficial de regras de testabilidade Semgrep.', '',
             '## Saídas originais', '',
             '- [Texto nativo Semgrep](semgrep-native.txt): achados com arquivos e linhas.',
             '- [JSON nativo](semgrep-native.json) e [SARIF nativo](semgrep-native.sarif): dados sem consolidação.',
             '- [Testes de regras Python](python-rule-tests.txt) e [C++](cpp-rule-tests.txt).',
             '- [Achados dos exemplos Python](validation-python-native.json) e [C++](validation-cpp-native.json).', '',
             f"Escopo: {summary['source_files']} arquivos de produção e {len(classes)} classes. "
             f"Semgrep {native['version']}; 17 regras Python e 11 regras C++ testadas. "
             'A BSN analisada é ROS 2/Python; somente os exemplos de controle usam C++.', '',
             '## Contagens descritivas', '',
             f"Foram emitidas {len(findings)} ocorrências de regras, sem erros de análise. "
             'Uma chamada pode corresponder à regra de infraestrutura e à de contexto do construtor; '
             'portanto, esse total não é um número de dependências, defeitos ou vulnerabilidades.', '',
             '| Categoria | Ocorrências nativas |', '|---|---:|']
    for category, label in labels.items():
        lines.append(f'| {label} | {totals[category]} |')
    lines += ['',
              f"- {summary['classes_with_direct_infrastructure_findings']} classes apresentam chamadas diretas de infraestrutura nas regras.",
              f"- {summary['classes_with_direct_constructor_effects']} classes apresentam efeitos diretos no construtor.",
              f"- {summary['classes_with_constructor_helper_wait']} classe apresenta espera em helper diretamente chamado pelo construtor.",
              f"- {summary['classes_with_supplied_collaborator_candidate']} classes armazenam um colaborador recebido, conforme os nomes considerados pela regra.",
              f"- {summary['findings_outside_classes']} ocorrências ficam fora das classes, sobretudo em funções main. Elas não foram atribuídas à classe do mesmo arquivo.", '',
              'Nenhuma dessas contagens é uma nota de testabilidade. Classes de dados, enums e '
              'adaptadores ROS têm responsabilidades diferentes e não devem ser comparados como uma população homogênea.', '',
              '## Interpretação dos casos principais', '',
              '### DataProcessor: ponto de substituição e espera na construção', '',
              'O Semgrep encontra `self.node = node`, criação de cliente e espera por serviço. '
              'A regra de contexto confirma a estrutura `__init__ → setup_client → wait_for_service` '
              'na mesma classe. O objeto node pode ser substituído, mas o substituto precisa '
              'implementar criação do cliente e controlar o retorno da espera. O timeout de 1 s '
              'é de cada chamada: o loop pode continuar esperando. Isso é um obstáculo concreto '
              'ao isolamento simples, mesmo com LCOM4=1.', '',
              '### DataAccess: ROS e sistema de arquivos', '',
              'Há achados nativos de criação de recursos ROS, timer, arquivos, diretório e relógio. '
              'A inspeção do código complementa os achados: `__init__` chama `_setup_logging` e '
              '`_load_models`, que contêm operações de arquivos. Esses encadeamentos de chamada '
              'são interpretação da fonte, não um call graph produzido pelo Semgrep CE. '
              'Para isolar a classe inteira, o teste precisa controlar ROS e os caminhos/arquivos '
              'ou substituir essas operações.', '',
              '### Enactor: espera indireta por dois serviços', '',
              'O Semgrep localiza clientes, esperas e timer. A revisão manual confirma '
              '`__init__ → setup → _wait_for_services`: há duas chamadas de espera dentro de loops. '
              'A regra de helper cobre somente um nível de chamada e não identifica automaticamente '
              'essa cadeia de dois níveis. Não confundir ausência do alerta de contexto com ausência de espera.', '',
              '### SystemMonitor e ParamAdapter: efeitos diretos', '',
              'As regras encontram criação de recursos ROS e timers diretamente em `__init__`. '
              'São classes de infraestrutura: a dependência é esperada pelo papel de adaptador. '
              'O resultado indica que testar a classe inteira exige um contexto ROS ou substitutos '
              'adequados; não demonstra necessidade de refatoração por si só.', '',
              '### Battery e componentes que recebem node', '',
              'Battery não tem achados nas regras e seus testes existentes a instanciam diretamente. '
              'Isso favorece sua escolha como exemplo de processamento isolado. A ausência de '
              'achados não prova pureza. DataFusionEngine, RiskAnalyzer e Visualizer recebem node; '
              'esses pontos permitem construir objetos de teste, mas o contrato exigido do node '
              'precisa ser inspecionado. A regra não verifica se o substituto é suficiente.', '',
              '### Herança e wrappers', '',
              'A regra de contrato reconhece herança direta de Node/LifecycleNode, mas não resolve '
              'toda a hierarquia local. Sensor e CentralHub herdam RosComponent, que constrói '
              'colaboradores e pode chamar implementações das subclasses. Essas dependências '
              'herdadas não são somadas automaticamente às contagens diretas de cada subclasse.', '',
              '## Evidências por classe', '',
              '| Classe (arquivo) | Categorias diretas | Efeitos diretos em __init__ | Helper com espera | Colaborador recebido |',
              '|---|---|---:|---:|---:|']
    for c in class_rows:
        categories = c['infrastructure_categories'] or 'nenhum achado direto'
        if c['ros_base_contract']:
            categories += '; contrato de Node ROS'
        lines.append(f"| [{c['class']}](../../{c['file']}#L{c['line']}) (`{c['file']}`) | {categories} | {c['direct_constructor_effects']} | {c['constructor_helper_waits']} | {c['supplied_collaborator_candidates']} |")
    lines += ['', '## Evidências com localização', '',
              '| Classe/função | Regra | Localização |', '|---|---|---|']
    for f in findings:
        lines.append(f"| {f['class'] or 'fora de classe'} | {f['category']} | [{f['file']}:{f['line']}](../../{f['file']}#L{f['line']}) |")
    lines += ['', '## Limites e comparação com ROS 1/C++', '',
              '- As regras reconhecem APIs e alguns nomes de variáveis. Chamadas homônimas podem '
              'produzir falsos positivos; wrappers, aliases ou APIs não enumeradas podem ser omitidos.',
              '- Dependências recebidas podem ser objetos de teste, mas a regra de parâmetro armazenado '
              'é somente um indício de ponto de substituição. Ela não mede esforço nem prova substituibilidade.',
              '- Leitura de relógio/arquivo e criação de recurso podem estar em ramos opcionais. '
              'A análise não comprova execução obrigatória em todos os caminhos.',
              '- O contador de efeitos diretos no construtor considera as operações enumeradas '
              'na regra constructor-effect. Não inclui toda interação com parâmetros, '
              'inicialização herdada nem efeitos de todos os métodos chamados.',
              '- Espera em helper está limitada ao padrão validado de uma chamada na mesma classe; '
              'não é análise interprocedural completa.',
              '- C++ tem 11 famílias de regras de infraestrutura correspondentes. ROS 1 usa '
              'advertise/subscribe/serviceClient/createTimer/waitForExistence; ROS 2 usa as APIs Python '
              'listadas nas regras. Construtores, herança, parâmetros ROS, diretórios e pontos de '
              'substituição ainda não têm cobertura equivalente nas regras C++.',
              '- As regras C++ passaram nos exemplos fornecidos, mas não foram aplicadas à BSN ROS 1. '
              'Templates, macros, ponteiros e declarações de streams/threads requerem verificação '
              'nos fontes reais. Os fixtures de stream/thread usam construção explícita por expressão; '
              'não há garantia de cobertura de todas as formas de declaração C++.',
              '- Comparar presença, localização e fase dos efeitos em responsabilidades equivalentes; '
              'não comparar o total bruto de achados das duas linguagens como medida de melhoria.',
              '- O resultado complementa LCOM4/CBO e sustenta conclusões somente sobre indícios '
              'estáticos de isolamento. Não foram executados testes da BSN ou nós ROS.', '',
              '## Autoria e reprodução', '',
              '- Regras: tools/semgrep/isolation-python.yml e isolation-cpp.yml, escritas pelo assistente.',
              '- Motor de análise e saídas originais: Semgrep Community Edition.',
              '- Consolidação por classe, CSV e este texto: assistente; nenhuma nota de testabilidade foi criada.',
              '- Reexecutar: `python3 tools/collect_isolation.py` e `python3 tools/summarize_isolation.py`.',
              '- commands.json contém comandos e códigos de saída; metadata.json contém versões, hashes e escopo.',
              '- Instalação e instruções em [ISOLATION.md](../../tools/ISOLATION.md).', '',
              '## Fontes primárias', '',
              '- https://semgrep.dev/docs/writing-rules/rule-syntax',
              '- https://semgrep.dev/docs/writing-rules/testing-rules',
              '- https://semgrep.dev/products/community-edition/', '']
    (OUT / 'report.md').write_text('\n'.join(lines))
    print(json.dumps({k: v for k, v in summary.items() if k != 'occurrences_by_category'}, indent=2))


if __name__ == '__main__':
    main()
