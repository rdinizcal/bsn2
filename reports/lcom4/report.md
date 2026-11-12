# LCOM4 das classes da BSN ROS 2

**Interpretação e consolidação produzidas pelo assistente.** Valores calculados pelas ferramentas externas, sem implementação própria de LCOM4.

## Relatórios originais das ferramentas

- [HTML nativo do pyscn](pyscn-native.html): resumo e tabela por classe.
- [JSON nativo do pyscn](pyscn.json): valores e grupos de métodos completos.
- [Tabela nativa do potfur/lcom](potfur.txt): segunda medição por classe.
- [CSV consolidado pelo assistente](classes.csv): organização das saídas, não relatório nativo.
- [Equivalência com C++](../../tools/METRICS_COMPARISON.md): ferramentas e limitações verificadas.

Medição estática do código de produção; nenhum nó ROS foi executado ou alterado.

Escopo: 54 arquivos Python; 47 classes inventariadas; 44 classes reportadas pelo pyscn e 47 pelo potfur/lcom.

## Ferramentas e definição

- **pyscn 1.33.2:** grupos conectados por atributos compartilhados ou chamadas internas. Exclui construtores, métodos herdados, estáticos/de classe, abstratos/vazios, métodos sem estado e sem conexão de chamada e certos acessores triviais isolados. Pode reportar 1 mesmo quando não resta nenhum método participante. Enums são omitidos.
- **potfur/lcom 0.1.0**, commit `b8b7d75c149864ab8c2cab7296a06da01a4a3359`: segunda medição. Exclui __init__, métodos sem acesso a self/cls e classmethods. Referências self.metodo() não definido localmente podem conectar métodos pelo mesmo nome, inclusive chamadas herdadas de logging. O resultado não é intercambiável com pyscn.
- **Omen 4.30.0:** executado no mesmo conjunto de arquivos. A implementação instalada calcula **LCOM3**, sem arestas de chamada, apesar de a página consultada anunciar LCOM4. Resultado preservado em omen.json apenas como diagnóstico; não usado como LCOM4.

LCOM4 = 1 significa um grupo conectado; >1 sugere grupos independentes a investigar. Não demonstra responsabilidade única nem testabilidade. Nenhum método participante não constitui evidência de boa coesão.

## Resultados

pyscn, resultado bruto: média 1.0455; máximo 2; distribuição {'1': 42, '2': 2}.

Há 9 classes sem métodos participantes e 4 com apenas um. Esses casos estão separados no CSV e não devem inflar a conclusão de coesão.

Subconjunto com ao menos dois métodos participantes no pyscn: 31 classes; distribuição LCOM4 {1: 29, 2: 2}.

| Pacote | Classes com ≥2 métodos participantes | LCOM4=1 | LCOM4>1 |
|---|---:|---:|---:|
| adaptation | 11 | 10 | 1 |
| central_hub | 5 | 5 | 0 |
| patient | 1 | 1 | 0 |
| sensor | 6 | 6 | 0 |
| shared_components | 5 | 5 | 0 |
| system_monitor | 3 | 2 | 1 |

## Valores por classe

“—” = omitida pelo pyscn; “0 métodos” e “1 método” referem-se aos participantes após suas exclusões.

| Classe (arquivo) | pyscn LCOM4 | potfur LCOM4 | Métodos participantes pyscn |
|---|---:|---:|---:|
| [DataMessage](../../adaptation/adaptation/data_access/data_access.py#L33) (`adaptation/adaptation/data_access/data_access.py`) | 1 | 0 | 0 |
| [StatusMessage](../../adaptation/adaptation/data_access/data_access.py#L53) (`adaptation/adaptation/data_access/data_access.py`) | 1 | 0 | 0 |
| [EventMessage](../../adaptation/adaptation/data_access/data_access.py#L62) (`adaptation/adaptation/data_access/data_access.py`) | 1 | 0 | 0 |
| [EnergyStatusMessage](../../adaptation/adaptation/data_access/data_access.py#L71) (`adaptation/adaptation/data_access/data_access.py`) | 1 | 0 | 0 |
| [UncertaintyMessage](../../adaptation/adaptation/data_access/data_access.py#L82) (`adaptation/adaptation/data_access/data_access.py`) | 1 | 0 | 0 |
| [AdaptationMessage](../../adaptation/adaptation/data_access/data_access.py#L93) (`adaptation/adaptation/data_access/data_access.py`) | 1 | 0 | 0 |
| [DataAccess](../../adaptation/adaptation/data_access/data_access.py#L104) (`adaptation/adaptation/data_access/data_access.py`) | 2 | 2 | 21 |
| [Controller](../../adaptation/adaptation/enactor/controller.py#L10) (`adaptation/adaptation/enactor/controller.py`) | 1 | 1 | 5 |
| [Enactor](../../adaptation/adaptation/enactor/enactor.py#L17) (`adaptation/adaptation/enactor/enactor.py`) | 1 | 1 | 11 |
| [Engine](../../adaptation/adaptation/engines/base_engine.py#L17) (`adaptation/adaptation/engines/base_engine.py`) | 1 | 1 | 8 |
| [ReliabilityEngine](../../adaptation/adaptation/engines/reli_engine.py#L13) (`adaptation/adaptation/engines/reli_engine.py`) | 1 | 2 | 9 |
| [Context](../../adaptation/adaptation/goal_model/context.py#L5) (`adaptation/adaptation/goal_model/context.py`) | 1 | 1 | 9 |
| [Goal](../../adaptation/adaptation/goal_model/goal.py#L3) (`adaptation/adaptation/goal_model/goal.py`) | 1 | 0 | 0 |
| [GoalTree](../../adaptation/adaptation/goal_model/goaltree.py#L7) (`adaptation/adaptation/goal_model/goaltree.py`) | 1 | 1 | 12 |
| [LeafTask](../../adaptation/adaptation/goal_model/leaftask.py#L6) (`adaptation/adaptation/goal_model/leaftask.py`) | 1 | 1 | 10 |
| [Node](../../adaptation/adaptation/goal_model/node.py#L10) (`adaptation/adaptation/goal_model/node.py`) | 1 | 1 | 15 |
| [Property](../../adaptation/adaptation/goal_model/property.py#L5) (`adaptation/adaptation/goal_model/property.py`) | 1 | 1 | 7 |
| [Task](../../adaptation/adaptation/goal_model/task.py#L4) (`adaptation/adaptation/goal_model/task.py`) | 1 | 1 | 0 |
| [FormulaError](../../adaptation/adaptation/model/formula.py#L8) (`adaptation/adaptation/model/formula.py`) | 1 | 0 | 0 |
| [Formula](../../adaptation/adaptation/model/formula.py#L14) (`adaptation/adaptation/model/formula.py`) | 1 | 1 | 14 |
| [CentralHub](../../central_hub/central_hub/central_hub.py#L15) (`central_hub/central_hub/central_hub.py`) | 1 | 1 | 9 |
| [ConfigManager](../../central_hub/central_hub/components/config_manager.py#L12) (`central_hub/central_hub/components/config_manager.py`) | 1 | 1 | 1 |
| [DataFusionEngine](../../central_hub/central_hub/components/data_fusion.py#L10) (`central_hub/central_hub/components/data_fusion.py`) | 1 | 1 | 2 |
| [PublisherManager](../../central_hub/central_hub/components/publishers.py#L15) (`central_hub/central_hub/components/publishers.py`) | 1 | 1 | 2 |
| [SensorDataHandler](../../central_hub/central_hub/components/sensor_data_handler.py#L12) (`central_hub/central_hub/components/sensor_data_handler.py`) | 1 | 1 | 3 |
| [RiskAnalyzer](../../central_hub/central_hub/utils/risk_analyzer.py#L2) (`central_hub/central_hub/utils/risk_analyzer.py`) | 1 | 1 | 1 |
| [Visualizer](../../central_hub/central_hub/utils/visualization.py#L10) (`central_hub/central_hub/utils/visualization.py`) | 1 | 1 | 3 |
| [Patient](../../patient/patient/patient.py#L17) (`patient/patient/patient.py`) | 1 | 1 | 6 |
| [ConfigManager](../../sensor/sensor/components/config_manager.py#L13) (`sensor/sensor/components/config_manager.py`) | 1 | 1 | 2 |
| [DataProcessor](../../sensor/sensor/components/data_processor.py#L14) (`sensor/sensor/components/data_processor.py`) | 1 | 1 | 5 |
| [PublisherManager](../../sensor/sensor/components/publishers.py#L13) (`sensor/sensor/components/publishers.py`) | 1 | 1 | 2 |
| [RiskManager](../../sensor/sensor/components/risk_manager.py#L12) (`sensor/sensor/components/risk_manager.py`) | 1 | 1 | 7 |
| [Sensor](../../sensor/sensor/sensor.py#L21) (`sensor/sensor/sensor.py`) | 1 | 1 | 9 |
| [RiskEvaluator](../../sensor/sensor/utils/risk_evaluator.py#L10) (`sensor/sensor/utils/risk_evaluator.py`) | 1 | 1 | 9 |
| [AdaptationHandler](../../shared_components/shared_components/adaptation_handler.py#L16) (`shared_components/shared_components/adaptation_handler.py`) | 1 | 1 | 3 |
| [Battery](../../shared_components/shared_components/battery.py#L10) (`shared_components/shared_components/battery.py`) | 1 | 1 | 3 |
| [BatteryManager](../../shared_components/shared_components/battery_manager.py#L14) (`shared_components/shared_components/battery_manager.py`) | 1 | 1 | 6 |
| [ConfigManagerBase](../../shared_components/shared_components/core/config_manager_base.py#L6) (`shared_components/shared_components/core/config_manager_base.py`) | 1 | 1 | 1 |
| [PublisherManagerBase](../../shared_components/shared_components/core/publisher_manager_base.py#L12) (`shared_components/shared_components/core/publisher_manager_base.py`) | 1 | 1 | 6 |
| [RosComponent](../../shared_components/shared_components/core/ros_component.py#L13) (`shared_components/shared_components/core/ros_component.py`) | 1 | 1 | 1 |
| [Task](../../shared_components/shared_components/enums.py#L3) (`shared_components/shared_components/enums.py`) | — | 0 | — |
| [StatusContent](../../shared_components/shared_components/enums.py#L17) (`shared_components/shared_components/enums.py`) | — | 0 | — |
| [EventType](../../shared_components/shared_components/enums.py#L23) (`shared_components/shared_components/enums.py`) | — | 0 | — |
| [LifecycleManager](../../shared_components/shared_components/lifecycle_manager.py#L16) (`shared_components/shared_components/lifecycle_manager.py`) | 1 | 1 | 14 |
| [Logger](../../system_monitor/system_monitor/logger.py#L20) (`system_monitor/system_monitor/logger.py`) | 1 | 1 | 12 |
| [SystemMonitor](../../system_monitor/system_monitor/node_monitor.py#L21) (`system_monitor/system_monitor/node_monitor.py`) | 2 | 1 | 6 |
| [ParamAdapter](../../system_monitor/system_monitor/param_adapter.py#L17) (`system_monitor/system_monitor/param_adapter.py`) | 1 | 1 | 3 |

## Grupos desconectados encontrados pelo pyscn

### DataAccess — adaptation/adaptation/data_access/data_access.py

1. `_apply_time_window`, `_calculate_component_cost`, `_calculate_component_reliability`, `_flush_logs`, `_get_component_name`, `_initialize_component_data`, `_load_goal_model`, `_load_models`, `_persist_adaptation`, `_persist_energy_status`, `_persist_event`, `_persist_status`, `_persist_uncertainty`, `_process_engine_energy_status`, `_setup_logging`, `now_seconds`, `process_query`, `process_target_system_data`, `receive_persist_message`, `timer_callback`
2. `_setup_ros_interfaces`

### SystemMonitor — system_monitor/system_monitor/node_monitor.py

1. `check_heartbeats`, `check_state_helper`, `event_callback`, `publish_monitor_status`, `status_callback`
2. `energy_callback`

## Divergências relevantes

- **SystemMonitor:** pyscn=2, potfur=1. energy_callback utiliza um publicador próprio; chamadas comuns a get_logger() herdado podem unir os métodos no potfur. O fixture InheritedLogging reproduz a divergência.
- **ReliabilityEngine:** pyscn=1, potfur=2. get_prefix é um acessor isolado do atributo prefix e é excluído pelo pyscn. O fixture SeparateGetters evidencia a política.
- Casos sem métodos elegíveis têm valores convencionais diferentes: 1 no pyscn e 0 no potfur. Não comparar médias brutas das duas ferramentas.

## Validação de ferramentas

| Exemplo | pyscn | potfur | Observação |
|---|---:|---:|---|
| SharedState | 1 | 1 | Atributo compartilhado: esperado 1 |
| SplitState | 2 | 2 | Dois estados desconectados, construtor excluído: esperado 2 |
| CallBridge | 1 | 1 | Conexão por chamada interna: esperado 1 |
| OnlyConstructor | 1 | 0 | Sem métodos participantes: convenções distintas |
| Stateless | 1 | 0 | Métodos sem estado excluídos: convenções distintas |
| SeparateGetters | 1 | 2 | pyscn exclui os acessores triviais isolados |
| InheritedLogging | 2 | 1 | potfur conecta pela chamada herdada get_logger |

## Escopo, reprodução e limites

- Arquivos de entrada: files.txt. Hashes SHA-256 e versões: metadata.json.
- Testes, auxiliares *_tests.py, test_*.py e test_components foram excluídos. build/install/log, launch e features não entram nos diretórios selecionados.
- bsn_interfaces não contém classes Python de produção escritas manualmente neste escopo; código gerado não foi incluído.
- Reexecutar a medição: `bash tools/run_lcom4.sh`. Instalação fixada em tools/requirements-lcom4.txt.
- Resultados brutos, sem arredondamento: pyscn.json e potfur.txt; tabela consolidada: classes.csv.
- Atributos amplos como self.node podem conectar métodos com responsabilidades distintas. LCOM4=1 é evidência de conectividade estrutural, não garantia de bom projeto.
- Apenas ROS 2 foi medido. Melhora exige resultados ROS 1 com definição e exclusões equivalentes. Não usar diretamente a média desta variante do pyscn contra outra implementação em C++.

## Fontes

- https://github.com/ludo-technologies/pyscn/tree/v1.33.2/internal/analyzer
- https://github.com/potfur/lcom/tree/b8b7d75c149864ab8c2cab7296a06da01a4a3359/src
- https://github.com/panbanda/omen/blob/omen-v4.30.0/src/analyzers/cohesion.rs
