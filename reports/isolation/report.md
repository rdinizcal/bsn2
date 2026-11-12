# Isolamento para testes — análise estática Semgrep

**Interpretação do assistente sobre resultados nativos do Semgrep.** As regras são critérios de auditoria escritos para esta BSN; não são uma métrica padronizada de testabilidade nem um pacote oficial de regras de testabilidade Semgrep.

## Saídas originais

- [Texto nativo Semgrep](semgrep-native.txt): achados com arquivos e linhas.
- [JSON nativo](semgrep-native.json) e [SARIF nativo](semgrep-native.sarif): dados sem consolidação.
- [Testes de regras Python](python-rule-tests.txt) e [C++](cpp-rule-tests.txt).
- [Achados dos exemplos Python](validation-python-native.json) e [C++](validation-cpp-native.json).

Escopo: 54 arquivos de produção e 47 classes. Semgrep 1.185.0; 17 regras Python e 11 regras C++ testadas. A BSN analisada é ROS 2/Python; somente os exemplos de controle usam C++.

## Contagens descritivas

Foram emitidas 251 ocorrências de regras, sem erros de análise. Uma chamada pode corresponder à regra de infraestrutura e à de contexto do construtor; portanto, esse total não é um número de dependências, defeitos ou vulnerabilidades.

| Categoria | Ocorrências nativas |
|---|---:|
| Herança direta de Node/LifecycleNode | 8 |
| Parâmetros via API do node | 92 |
| Criação de clientes/serviços/publicadores/subscritores ROS | 44 |
| Timers ROS | 11 |
| Inicialização/spin ROS | 13 |
| Esperas por serviços/actions | 6 |
| Possíveis chamadas de cliente | 8 |
| Acesso a arquivos | 9 |
| Criação de diretórios | 1 |
| Relógio real | 16 |
| Espera temporal | 6 |
| Criação de thread/executor de threads | 6 |
| Processos externos | 0 |
| Variáveis de ambiente | 0 |
| Efeito direto no construtor (sobrepõe outras regras) | 19 |
| Espera em helper chamado pelo construtor (sobreposição) | 1 |
| Colaborador recebido: candidato à substituição | 11 |

- 21 classes apresentam chamadas diretas de infraestrutura nas regras.
- 7 classes apresentam efeitos diretos no construtor.
- 1 classe apresenta espera em helper diretamente chamado pelo construtor.
- 11 classes armazenam um colaborador recebido, conforme os nomes considerados pela regra.
- 17 ocorrências ficam fora das classes, sobretudo em funções main. Elas não foram atribuídas à classe do mesmo arquivo.

Nenhuma dessas contagens é uma nota de testabilidade. Classes de dados, enums e adaptadores ROS têm responsabilidades diferentes e não devem ser comparados como uma população homogênea.

## Interpretação dos casos principais

### DataProcessor: ponto de substituição e espera na construção

O Semgrep encontra `self.node = node`, criação de cliente e espera por serviço. A regra de contexto confirma a estrutura `__init__ → setup_client → wait_for_service` na mesma classe. O objeto node pode ser substituído, mas o substituto precisa implementar criação do cliente e controlar o retorno da espera. O timeout de 1 s é de cada chamada: o loop pode continuar esperando. Isso é um obstáculo concreto ao isolamento simples, mesmo com LCOM4=1.

### DataAccess: ROS e sistema de arquivos

Há achados nativos de criação de recursos ROS, timer, arquivos, diretório e relógio. A inspeção do código complementa os achados: `__init__` chama `_setup_logging` e `_load_models`, que contêm operações de arquivos. Esses encadeamentos de chamada são interpretação da fonte, não um call graph produzido pelo Semgrep CE. Para isolar a classe inteira, o teste precisa controlar ROS e os caminhos/arquivos ou substituir essas operações.

### Enactor: espera indireta por dois serviços

O Semgrep localiza clientes, esperas e timer. A revisão manual confirma `__init__ → setup → _wait_for_services`: há duas chamadas de espera dentro de loops. A regra de helper cobre somente um nível de chamada e não identifica automaticamente essa cadeia de dois níveis. Não confundir ausência do alerta de contexto com ausência de espera.

### SystemMonitor e ParamAdapter: efeitos diretos

As regras encontram criação de recursos ROS e timers diretamente em `__init__`. São classes de infraestrutura: a dependência é esperada pelo papel de adaptador. O resultado indica que testar a classe inteira exige um contexto ROS ou substitutos adequados; não demonstra necessidade de refatoração por si só.

### Battery e componentes que recebem node

Battery não tem achados nas regras e seus testes existentes a instanciam diretamente. Isso favorece sua escolha como exemplo de processamento isolado. A ausência de achados não prova pureza. DataFusionEngine, RiskAnalyzer e Visualizer recebem node; esses pontos permitem construir objetos de teste, mas o contrato exigido do node precisa ser inspecionado. A regra não verifica se o substituto é suficiente.

### Herança e wrappers

A regra de contrato reconhece herança direta de Node/LifecycleNode, mas não resolve toda a hierarquia local. Sensor e CentralHub herdam RosComponent, que constrói colaboradores e pode chamar implementações das subclasses. Essas dependências herdadas não são somadas automaticamente às contagens diretas de cada subclasse.

## Evidências por classe

| Classe (arquivo) | Categorias diretas | Efeitos diretos em __init__ | Helper com espera | Colaborador recebido |
|---|---|---:|---:|---:|
| [DataMessage](../../adaptation/adaptation/data_access/data_access.py#L33) (`adaptation/adaptation/data_access/data_access.py`) | nenhum achado direto | 0 | 0 | 0 |
| [StatusMessage](../../adaptation/adaptation/data_access/data_access.py#L53) (`adaptation/adaptation/data_access/data_access.py`) | nenhum achado direto | 0 | 0 | 0 |
| [EventMessage](../../adaptation/adaptation/data_access/data_access.py#L62) (`adaptation/adaptation/data_access/data_access.py`) | nenhum achado direto | 0 | 0 | 0 |
| [EnergyStatusMessage](../../adaptation/adaptation/data_access/data_access.py#L71) (`adaptation/adaptation/data_access/data_access.py`) | nenhum achado direto | 0 | 0 | 0 |
| [UncertaintyMessage](../../adaptation/adaptation/data_access/data_access.py#L82) (`adaptation/adaptation/data_access/data_access.py`) | nenhum achado direto | 0 | 0 | 0 |
| [AdaptationMessage](../../adaptation/adaptation/data_access/data_access.py#L93) (`adaptation/adaptation/data_access/data_access.py`) | nenhum achado direto | 0 | 0 | 0 |
| [DataAccess](../../adaptation/adaptation/data_access/data_access.py#L104) (`adaptation/adaptation/data_access/data_access.py`) | directory_io; file_io; real_clock; ros_parameters; ros_resource; ros_timer; contrato de Node ROS | 1 | 0 | 0 |
| [Controller](../../adaptation/adaptation/enactor/controller.py#L10) (`adaptation/adaptation/enactor/controller.py`) | ros_parameters; ros_resource | 1 | 0 | 0 |
| [Enactor](../../adaptation/adaptation/enactor/enactor.py#L17) (`adaptation/adaptation/enactor/enactor.py`) | client_call; ros_parameters; ros_resource; ros_timer; service_wait; contrato de Node ROS | 1 | 0 | 0 |
| [Engine](../../adaptation/adaptation/engines/base_engine.py#L17) (`adaptation/adaptation/engines/base_engine.py`) | client_call; real_sleep; ros_parameters; ros_resource; ros_runtime; contrato de Node ROS | 0 | 0 | 0 |
| [ReliabilityEngine](../../adaptation/adaptation/engines/reli_engine.py#L13) (`adaptation/adaptation/engines/reli_engine.py`) | client_call; ros_parameters; ros_resource; ros_runtime; service_wait | 0 | 0 | 0 |
| [Context](../../adaptation/adaptation/goal_model/context.py#L5) (`adaptation/adaptation/goal_model/context.py`) | nenhum achado direto | 0 | 0 | 0 |
| [Goal](../../adaptation/adaptation/goal_model/goal.py#L3) (`adaptation/adaptation/goal_model/goal.py`) | nenhum achado direto | 0 | 0 | 0 |
| [GoalTree](../../adaptation/adaptation/goal_model/goaltree.py#L7) (`adaptation/adaptation/goal_model/goaltree.py`) | nenhum achado direto | 0 | 0 | 0 |
| [LeafTask](../../adaptation/adaptation/goal_model/leaftask.py#L6) (`adaptation/adaptation/goal_model/leaftask.py`) | nenhum achado direto | 0 | 0 | 0 |
| [Node](../../adaptation/adaptation/goal_model/node.py#L10) (`adaptation/adaptation/goal_model/node.py`) | nenhum achado direto | 0 | 0 | 0 |
| [Property](../../adaptation/adaptation/goal_model/property.py#L5) (`adaptation/adaptation/goal_model/property.py`) | nenhum achado direto | 0 | 0 | 0 |
| [Task](../../adaptation/adaptation/goal_model/task.py#L4) (`adaptation/adaptation/goal_model/task.py`) | nenhum achado direto | 0 | 0 | 0 |
| [FormulaError](../../adaptation/adaptation/model/formula.py#L8) (`adaptation/adaptation/model/formula.py`) | nenhum achado direto | 0 | 0 | 0 |
| [Formula](../../adaptation/adaptation/model/formula.py#L14) (`adaptation/adaptation/model/formula.py`) | nenhum achado direto | 0 | 0 | 0 |
| [CentralHub](../../central_hub/central_hub/central_hub.py#L15) (`central_hub/central_hub/central_hub.py`) | ros_timer | 0 | 0 | 0 |
| [ConfigManager](../../central_hub/central_hub/components/config_manager.py#L12) (`central_hub/central_hub/components/config_manager.py`) | nenhum achado direto | 0 | 0 | 0 |
| [DataFusionEngine](../../central_hub/central_hub/components/data_fusion.py#L10) (`central_hub/central_hub/components/data_fusion.py`) | nenhum achado direto | 0 | 0 | 1 |
| [PublisherManager](../../central_hub/central_hub/components/publishers.py#L15) (`central_hub/central_hub/components/publishers.py`) | real_clock; ros_resource | 0 | 0 | 0 |
| [SensorDataHandler](../../central_hub/central_hub/components/sensor_data_handler.py#L12) (`central_hub/central_hub/components/sensor_data_handler.py`) | ros_resource | 0 | 0 | 1 |
| [RiskAnalyzer](../../central_hub/central_hub/utils/risk_analyzer.py#L2) (`central_hub/central_hub/utils/risk_analyzer.py`) | nenhum achado direto | 0 | 0 | 1 |
| [Visualizer](../../central_hub/central_hub/utils/visualization.py#L10) (`central_hub/central_hub/utils/visualization.py`) | nenhum achado direto | 0 | 0 | 1 |
| [Patient](../../patient/patient/patient.py#L17) (`patient/patient/patient.py`) | ros_parameters; ros_resource; contrato de Node ROS | 1 | 0 | 0 |
| [ConfigManager](../../sensor/sensor/components/config_manager.py#L13) (`sensor/sensor/components/config_manager.py`) | ros_parameters | 0 | 0 | 0 |
| [DataProcessor](../../sensor/sensor/components/data_processor.py#L14) (`sensor/sensor/components/data_processor.py`) | client_call; ros_resource; service_wait | 0 | 1 | 1 |
| [PublisherManager](../../sensor/sensor/components/publishers.py#L13) (`sensor/sensor/components/publishers.py`) | real_clock; ros_resource | 0 | 0 | 0 |
| [RiskManager](../../sensor/sensor/components/risk_manager.py#L12) (`sensor/sensor/components/risk_manager.py`) | nenhum achado direto | 0 | 0 | 1 |
| [Sensor](../../sensor/sensor/sensor.py#L21) (`sensor/sensor/sensor.py`) | ros_timer | 0 | 0 | 0 |
| [RiskEvaluator](../../sensor/sensor/utils/risk_evaluator.py#L10) (`sensor/sensor/utils/risk_evaluator.py`) | nenhum achado direto | 0 | 0 | 0 |
| [AdaptationHandler](../../shared_components/shared_components/adaptation_handler.py#L16) (`shared_components/shared_components/adaptation_handler.py`) | client_call; ros_resource; service_wait | 0 | 0 | 1 |
| [Battery](../../shared_components/shared_components/battery.py#L10) (`shared_components/shared_components/battery.py`) | nenhum achado direto | 0 | 0 | 0 |
| [BatteryManager](../../shared_components/shared_components/battery_manager.py#L14) (`shared_components/shared_components/battery_manager.py`) | real_clock; ros_resource; ros_timer | 2 | 0 | 1 |
| [ConfigManagerBase](../../shared_components/shared_components/core/config_manager_base.py#L6) (`shared_components/shared_components/core/config_manager_base.py`) | ros_parameters | 0 | 0 | 1 |
| [PublisherManagerBase](../../shared_components/shared_components/core/publisher_manager_base.py#L12) (`shared_components/shared_components/core/publisher_manager_base.py`) | ros_resource | 0 | 0 | 1 |
| [RosComponent](../../shared_components/shared_components/core/ros_component.py#L13) (`shared_components/shared_components/core/ros_component.py`) | nenhum achado direto; contrato de Node ROS | 0 | 0 | 0 |
| [Task](../../shared_components/shared_components/enums.py#L3) (`shared_components/shared_components/enums.py`) | nenhum achado direto | 0 | 0 | 0 |
| [StatusContent](../../shared_components/shared_components/enums.py#L17) (`shared_components/shared_components/enums.py`) | nenhum achado direto | 0 | 0 | 0 |
| [EventType](../../shared_components/shared_components/enums.py#L23) (`shared_components/shared_components/enums.py`) | nenhum achado direto | 0 | 0 | 0 |
| [LifecycleManager](../../shared_components/shared_components/lifecycle_manager.py#L16) (`shared_components/shared_components/lifecycle_manager.py`) | real_clock; real_sleep; ros_timer | 0 | 0 | 1 |
| [Logger](../../system_monitor/system_monitor/logger.py#L20) (`system_monitor/system_monitor/logger.py`) | real_clock; ros_parameters; ros_resource; contrato de Node ROS | 0 | 0 | 0 |
| [SystemMonitor](../../system_monitor/system_monitor/node_monitor.py#L21) (`system_monitor/system_monitor/node_monitor.py`) | client_call; real_clock; ros_parameters; ros_resource; ros_timer; contrato de Node ROS | 9 | 0 | 0 |
| [ParamAdapter](../../system_monitor/system_monitor/param_adapter.py#L17) (`system_monitor/system_monitor/param_adapter.py`) | real_clock; ros_parameters; ros_resource; ros_timer; contrato de Node ROS | 4 | 0 | 0 |

## Evidências com localização

| Classe/função | Regra | Localização |
|---|---|---|
| DataAccess | ros_node_base | [adaptation/adaptation/data_access/data_access.py:104](../../adaptation/adaptation/data_access/data_access.py#L104) |
| DataAccess | ros_parameters | [adaptation/adaptation/data_access/data_access.py:132](../../adaptation/adaptation/data_access/data_access.py#L132) |
| DataAccess | ros_parameters | [adaptation/adaptation/data_access/data_access.py:133](../../adaptation/adaptation/data_access/data_access.py#L133) |
| DataAccess | ros_parameters | [adaptation/adaptation/data_access/data_access.py:134](../../adaptation/adaptation/data_access/data_access.py#L134) |
| DataAccess | ros_parameters | [adaptation/adaptation/data_access/data_access.py:135](../../adaptation/adaptation/data_access/data_access.py#L135) |
| DataAccess | ros_parameters | [adaptation/adaptation/data_access/data_access.py:138](../../adaptation/adaptation/data_access/data_access.py#L138) |
| DataAccess | ros_parameters | [adaptation/adaptation/data_access/data_access.py:139](../../adaptation/adaptation/data_access/data_access.py#L139) |
| DataAccess | ros_parameters | [adaptation/adaptation/data_access/data_access.py:140](../../adaptation/adaptation/data_access/data_access.py#L140) |
| DataAccess | ros_parameters | [adaptation/adaptation/data_access/data_access.py:143](../../adaptation/adaptation/data_access/data_access.py#L143) |
| DataAccess | constructor_effect | [adaptation/adaptation/data_access/data_access.py:200](../../adaptation/adaptation/data_access/data_access.py#L200) |
| DataAccess | ros_timer | [adaptation/adaptation/data_access/data_access.py:200](../../adaptation/adaptation/data_access/data_access.py#L200) |
| DataAccess | ros_resource | [adaptation/adaptation/data_access/data_access.py:232](../../adaptation/adaptation/data_access/data_access.py#L232) |
| DataAccess | ros_resource | [adaptation/adaptation/data_access/data_access.py:236](../../adaptation/adaptation/data_access/data_access.py#L236) |
| DataAccess | ros_resource | [adaptation/adaptation/data_access/data_access.py:244](../../adaptation/adaptation/data_access/data_access.py#L244) |
| DataAccess | directory_io | [adaptation/adaptation/data_access/data_access.py:251](../../adaptation/adaptation/data_access/data_access.py#L251) |
| DataAccess | real_clock | [adaptation/adaptation/data_access/data_access.py:254](../../adaptation/adaptation/data_access/data_access.py#L254) |
| DataAccess | file_io | [adaptation/adaptation/data_access/data_access.py:267](../../adaptation/adaptation/data_access/data_access.py#L267) |
| DataAccess | file_io | [adaptation/adaptation/data_access/data_access.py:276](../../adaptation/adaptation/data_access/data_access.py#L276) |
| DataAccess | file_io | [adaptation/adaptation/data_access/data_access.py:295](../../adaptation/adaptation/data_access/data_access.py#L295) |
| DataAccess | file_io | [adaptation/adaptation/data_access/data_access.py:314](../../adaptation/adaptation/data_access/data_access.py#L314) |
| DataAccess | real_clock | [adaptation/adaptation/data_access/data_access.py:357](../../adaptation/adaptation/data_access/data_access.py#L357) |
| DataAccess | file_io | [adaptation/adaptation/data_access/data_access.py:674](../../adaptation/adaptation/data_access/data_access.py#L674) |
| DataAccess | file_io | [adaptation/adaptation/data_access/data_access.py:691](../../adaptation/adaptation/data_access/data_access.py#L691) |
| DataAccess | file_io | [adaptation/adaptation/data_access/data_access.py:708](../../adaptation/adaptation/data_access/data_access.py#L708) |
| DataAccess | file_io | [adaptation/adaptation/data_access/data_access.py:725](../../adaptation/adaptation/data_access/data_access.py#L725) |
| DataAccess | file_io | [adaptation/adaptation/data_access/data_access.py:742](../../adaptation/adaptation/data_access/data_access.py#L742) |
| fora de classe | ros_runtime | [adaptation/adaptation/data_access/data_access.py:767](../../adaptation/adaptation/data_access/data_access.py#L767) |
| Controller | ros_parameters | [adaptation/adaptation/enactor/controller.py:20](../../adaptation/adaptation/enactor/controller.py#L20) |
| Controller | ros_parameters | [adaptation/adaptation/enactor/controller.py:21](../../adaptation/adaptation/enactor/controller.py#L21) |
| Controller | constructor_effect | [adaptation/adaptation/enactor/controller.py:34](../../adaptation/adaptation/enactor/controller.py#L34) |
| Controller | ros_resource | [adaptation/adaptation/enactor/controller.py:34](../../adaptation/adaptation/enactor/controller.py#L34) |
| fora de classe | ros_runtime | [adaptation/adaptation/enactor/controller.py:202](../../adaptation/adaptation/enactor/controller.py#L202) |
| Enactor | ros_node_base | [adaptation/adaptation/enactor/enactor.py:17](../../adaptation/adaptation/enactor/enactor.py#L17) |
| Enactor | ros_parameters | [adaptation/adaptation/enactor/enactor.py:32](../../adaptation/adaptation/enactor/enactor.py#L32) |
| Enactor | ros_parameters | [adaptation/adaptation/enactor/enactor.py:33](../../adaptation/adaptation/enactor/enactor.py#L33) |
| Enactor | ros_parameters | [adaptation/adaptation/enactor/enactor.py:36](../../adaptation/adaptation/enactor/enactor.py#L36) |
| Enactor | ros_parameters | [adaptation/adaptation/enactor/enactor.py:37](../../adaptation/adaptation/enactor/enactor.py#L37) |
| Enactor | constructor_effect | [adaptation/adaptation/enactor/enactor.py:58](../../adaptation/adaptation/enactor/enactor.py#L58) |
| Enactor | ros_timer | [adaptation/adaptation/enactor/enactor.py:58](../../adaptation/adaptation/enactor/enactor.py#L58) |
| Enactor | ros_resource | [adaptation/adaptation/enactor/enactor.py:73](../../adaptation/adaptation/enactor/enactor.py#L73) |
| Enactor | ros_resource | [adaptation/adaptation/enactor/enactor.py:76](../../adaptation/adaptation/enactor/enactor.py#L76) |
| Enactor | ros_resource | [adaptation/adaptation/enactor/enactor.py:80](../../adaptation/adaptation/enactor/enactor.py#L80) |
| Enactor | ros_resource | [adaptation/adaptation/enactor/enactor.py:84](../../adaptation/adaptation/enactor/enactor.py#L84) |
| Enactor | ros_resource | [adaptation/adaptation/enactor/enactor.py:85](../../adaptation/adaptation/enactor/enactor.py#L85) |
| Enactor | service_wait | [adaptation/adaptation/enactor/enactor.py:92](../../adaptation/adaptation/enactor/enactor.py#L92) |
| Enactor | service_wait | [adaptation/adaptation/enactor/enactor.py:95](../../adaptation/adaptation/enactor/enactor.py#L95) |
| Enactor | client_call | [adaptation/adaptation/enactor/enactor.py:135](../../adaptation/adaptation/enactor/enactor.py#L135) |
| Enactor | client_call | [adaptation/adaptation/enactor/enactor.py:219](../../adaptation/adaptation/enactor/enactor.py#L219) |
| Engine | ros_node_base | [adaptation/adaptation/engines/base_engine.py:17](../../adaptation/adaptation/engines/base_engine.py#L17) |
| Engine | ros_parameters | [adaptation/adaptation/engines/base_engine.py:43](../../adaptation/adaptation/engines/base_engine.py#L43) |
| Engine | ros_parameters | [adaptation/adaptation/engines/base_engine.py:44](../../adaptation/adaptation/engines/base_engine.py#L44) |
| Engine | ros_parameters | [adaptation/adaptation/engines/base_engine.py:45](../../adaptation/adaptation/engines/base_engine.py#L45) |
| Engine | ros_parameters | [adaptation/adaptation/engines/base_engine.py:46](../../adaptation/adaptation/engines/base_engine.py#L46) |
| Engine | ros_parameters | [adaptation/adaptation/engines/base_engine.py:53](../../adaptation/adaptation/engines/base_engine.py#L53) |
| Engine | ros_parameters | [adaptation/adaptation/engines/base_engine.py:54](../../adaptation/adaptation/engines/base_engine.py#L54) |
| Engine | ros_parameters | [adaptation/adaptation/engines/base_engine.py:55](../../adaptation/adaptation/engines/base_engine.py#L55) |
| Engine | ros_parameters | [adaptation/adaptation/engines/base_engine.py:56](../../adaptation/adaptation/engines/base_engine.py#L56) |
| Engine | real_sleep | [adaptation/adaptation/engines/base_engine.py:67](../../adaptation/adaptation/engines/base_engine.py#L67) |
| Engine | ros_resource | [adaptation/adaptation/engines/base_engine.py:83](../../adaptation/adaptation/engines/base_engine.py#L83) |
| Engine | ros_resource | [adaptation/adaptation/engines/base_engine.py:86](../../adaptation/adaptation/engines/base_engine.py#L86) |
| Engine | ros_resource | [adaptation/adaptation/engines/base_engine.py:90](../../adaptation/adaptation/engines/base_engine.py#L90) |
| Engine | client_call | [adaptation/adaptation/engines/base_engine.py:102](../../adaptation/adaptation/engines/base_engine.py#L102) |
| Engine | ros_runtime | [adaptation/adaptation/engines/base_engine.py:103](../../adaptation/adaptation/engines/base_engine.py#L103) |
| Engine | ros_runtime | [adaptation/adaptation/engines/base_engine.py:211](../../adaptation/adaptation/engines/base_engine.py#L211) |
| ReliabilityEngine | ros_parameters | [adaptation/adaptation/engines/reli_engine.py:33](../../adaptation/adaptation/engines/reli_engine.py#L33) |
| ReliabilityEngine | ros_parameters | [adaptation/adaptation/engines/reli_engine.py:34](../../adaptation/adaptation/engines/reli_engine.py#L34) |
| ReliabilityEngine | ros_parameters | [adaptation/adaptation/engines/reli_engine.py:35](../../adaptation/adaptation/engines/reli_engine.py#L35) |
| ReliabilityEngine | ros_parameters | [adaptation/adaptation/engines/reli_engine.py:36](../../adaptation/adaptation/engines/reli_engine.py#L36) |
| ReliabilityEngine | ros_parameters | [adaptation/adaptation/engines/reli_engine.py:46](../../adaptation/adaptation/engines/reli_engine.py#L46) |
| ReliabilityEngine | ros_parameters | [adaptation/adaptation/engines/reli_engine.py:47](../../adaptation/adaptation/engines/reli_engine.py#L47) |
| ReliabilityEngine | ros_parameters | [adaptation/adaptation/engines/reli_engine.py:48](../../adaptation/adaptation/engines/reli_engine.py#L48) |
| ReliabilityEngine | ros_parameters | [adaptation/adaptation/engines/reli_engine.py:49](../../adaptation/adaptation/engines/reli_engine.py#L49) |
| ReliabilityEngine | ros_resource | [adaptation/adaptation/engines/reli_engine.py:52](../../adaptation/adaptation/engines/reli_engine.py#L52) |
| ReliabilityEngine | service_wait | [adaptation/adaptation/engines/reli_engine.py:104](../../adaptation/adaptation/engines/reli_engine.py#L104) |
| ReliabilityEngine | client_call | [adaptation/adaptation/engines/reli_engine.py:114](../../adaptation/adaptation/engines/reli_engine.py#L114) |
| ReliabilityEngine | ros_runtime | [adaptation/adaptation/engines/reli_engine.py:115](../../adaptation/adaptation/engines/reli_engine.py#L115) |
| ReliabilityEngine | service_wait | [adaptation/adaptation/engines/reli_engine.py:170](../../adaptation/adaptation/engines/reli_engine.py#L170) |
| ReliabilityEngine | client_call | [adaptation/adaptation/engines/reli_engine.py:180](../../adaptation/adaptation/engines/reli_engine.py#L180) |
| ReliabilityEngine | ros_runtime | [adaptation/adaptation/engines/reli_engine.py:181](../../adaptation/adaptation/engines/reli_engine.py#L181) |
| fora de classe | ros_runtime | [adaptation/adaptation/engines/reli_engine.py:390](../../adaptation/adaptation/engines/reli_engine.py#L390) |
| CentralHub | ros_timer | [central_hub/central_hub/central_hub.py:110](../../central_hub/central_hub/central_hub.py#L110) |
| fora de classe | ros_runtime | [central_hub/central_hub/central_hub.py:215](../../central_hub/central_hub/central_hub.py#L215) |
| fora de classe | thread_creation | [central_hub/central_hub/central_hub.py:223](../../central_hub/central_hub/central_hub.py#L223) |
| DataFusionEngine | supplied_collaborator | [central_hub/central_hub/components/data_fusion.py:42](../../central_hub/central_hub/components/data_fusion.py#L42) |
| PublisherManager | ros_resource | [central_hub/central_hub/components/publishers.py:67](../../central_hub/central_hub/components/publishers.py#L67) |
| PublisherManager | real_clock | [central_hub/central_hub/components/publishers.py:100](../../central_hub/central_hub/components/publishers.py#L100) |
| SensorDataHandler | supplied_collaborator | [central_hub/central_hub/components/sensor_data_handler.py:49](../../central_hub/central_hub/components/sensor_data_handler.py#L49) |
| SensorDataHandler | ros_resource | [central_hub/central_hub/components/sensor_data_handler.py:81](../../central_hub/central_hub/components/sensor_data_handler.py#L81) |
| SensorDataHandler | ros_resource | [central_hub/central_hub/components/sensor_data_handler.py:84](../../central_hub/central_hub/components/sensor_data_handler.py#L84) |
| SensorDataHandler | ros_resource | [central_hub/central_hub/components/sensor_data_handler.py:87](../../central_hub/central_hub/components/sensor_data_handler.py#L87) |
| SensorDataHandler | ros_resource | [central_hub/central_hub/components/sensor_data_handler.py:90](../../central_hub/central_hub/components/sensor_data_handler.py#L90) |
| SensorDataHandler | ros_resource | [central_hub/central_hub/components/sensor_data_handler.py:93](../../central_hub/central_hub/components/sensor_data_handler.py#L93) |
| SensorDataHandler | ros_resource | [central_hub/central_hub/components/sensor_data_handler.py:96](../../central_hub/central_hub/components/sensor_data_handler.py#L96) |
| RiskAnalyzer | supplied_collaborator | [central_hub/central_hub/utils/risk_analyzer.py:6](../../central_hub/central_hub/utils/risk_analyzer.py#L6) |
| Visualizer | supplied_collaborator | [central_hub/central_hub/utils/visualization.py:41](../../central_hub/central_hub/utils/visualization.py#L41) |
| Patient | ros_node_base | [patient/patient/patient.py:17](../../patient/patient/patient.py#L17) |
| Patient | ros_parameters | [patient/patient/patient.py:80](../../patient/patient/patient.py#L80) |
| Patient | ros_parameters | [patient/patient/patient.py:81](../../patient/patient/patient.py#L81) |
| Patient | ros_parameters | [patient/patient/patient.py:84](../../patient/patient/patient.py#L84) |
| Patient | ros_parameters | [patient/patient/patient.py:88](../../patient/patient/patient.py#L88) |
| Patient | ros_parameters | [patient/patient/patient.py:100](../../patient/patient/patient.py#L100) |
| Patient | ros_parameters | [patient/patient/patient.py:101](../../patient/patient/patient.py#L101) |
| Patient | ros_parameters | [patient/patient/patient.py:104](../../patient/patient/patient.py#L104) |
| Patient | ros_parameters | [patient/patient/patient.py:105](../../patient/patient/patient.py#L105) |
| Patient | constructor_effect | [patient/patient/patient.py:113](../../patient/patient/patient.py#L113) |
| Patient | ros_resource | [patient/patient/patient.py:113](../../patient/patient/patient.py#L113) |
| Patient | ros_parameters | [patient/patient/patient.py:140](../../patient/patient/patient.py#L140) |
| Patient | ros_parameters | [patient/patient/patient.py:141](../../patient/patient/patient.py#L141) |
| Patient | ros_parameters | [patient/patient/patient.py:179](../../patient/patient/patient.py#L179) |
| Patient | ros_parameters | [patient/patient/patient.py:180](../../patient/patient/patient.py#L180) |
| Patient | ros_parameters | [patient/patient/patient.py:240](../../patient/patient/patient.py#L240) |
| Patient | ros_parameters | [patient/patient/patient.py:241](../../patient/patient/patient.py#L241) |
| fora de classe | ros_runtime | [patient/patient/patient.py:410](../../patient/patient/patient.py#L410) |
| fora de classe | thread_creation | [patient/patient/patient.py:415](../../patient/patient/patient.py#L415) |
| ConfigManager | ros_parameters | [sensor/sensor/components/config_manager.py:69](../../sensor/sensor/components/config_manager.py#L69) |
| ConfigManager | ros_parameters | [sensor/sensor/components/config_manager.py:71](../../sensor/sensor/components/config_manager.py#L71) |
| ConfigManager | ros_parameters | [sensor/sensor/components/config_manager.py:93](../../sensor/sensor/components/config_manager.py#L93) |
| ConfigManager | ros_parameters | [sensor/sensor/components/config_manager.py:94](../../sensor/sensor/components/config_manager.py#L94) |
| ConfigManager | ros_parameters | [sensor/sensor/components/config_manager.py:95](../../sensor/sensor/components/config_manager.py#L95) |
| ConfigManager | ros_parameters | [sensor/sensor/components/config_manager.py:98](../../sensor/sensor/components/config_manager.py#L98) |
| ConfigManager | ros_parameters | [sensor/sensor/components/config_manager.py:99](../../sensor/sensor/components/config_manager.py#L99) |
| ConfigManager | ros_parameters | [sensor/sensor/components/config_manager.py:100](../../sensor/sensor/components/config_manager.py#L100) |
| ConfigManager | ros_parameters | [sensor/sensor/components/config_manager.py:103](../../sensor/sensor/components/config_manager.py#L103) |
| ConfigManager | ros_parameters | [sensor/sensor/components/config_manager.py:104](../../sensor/sensor/components/config_manager.py#L104) |
| ConfigManager | ros_parameters | [sensor/sensor/components/config_manager.py:105](../../sensor/sensor/components/config_manager.py#L105) |
| ConfigManager | ros_parameters | [sensor/sensor/components/config_manager.py:106](../../sensor/sensor/components/config_manager.py#L106) |
| ConfigManager | ros_parameters | [sensor/sensor/components/config_manager.py:107](../../sensor/sensor/components/config_manager.py#L107) |
| ConfigManager | ros_parameters | [sensor/sensor/components/config_manager.py:110](../../sensor/sensor/components/config_manager.py#L110) |
| ConfigManager | ros_parameters | [sensor/sensor/components/config_manager.py:111](../../sensor/sensor/components/config_manager.py#L111) |
| ConfigManager | ros_parameters | [sensor/sensor/components/config_manager.py:112](../../sensor/sensor/components/config_manager.py#L112) |
| ConfigManager | ros_parameters | [sensor/sensor/components/config_manager.py:113](../../sensor/sensor/components/config_manager.py#L113) |
| ConfigManager | ros_parameters | [sensor/sensor/components/config_manager.py:114](../../sensor/sensor/components/config_manager.py#L114) |
| DataProcessor | supplied_collaborator | [sensor/sensor/components/data_processor.py:54](../../sensor/sensor/components/data_processor.py#L54) |
| DataProcessor | ros_resource | [sensor/sensor/components/data_processor.py:72](../../sensor/sensor/components/data_processor.py#L72) |
| DataProcessor | constructor_helper_wait | [sensor/sensor/components/data_processor.py:73](../../sensor/sensor/components/data_processor.py#L73) |
| DataProcessor | service_wait | [sensor/sensor/components/data_processor.py:73](../../sensor/sensor/components/data_processor.py#L73) |
| DataProcessor | client_call | [sensor/sensor/components/data_processor.py:112](../../sensor/sensor/components/data_processor.py#L112) |
| PublisherManager | ros_resource | [sensor/sensor/components/publishers.py:65](../../sensor/sensor/components/publishers.py#L65) |
| PublisherManager | real_clock | [sensor/sensor/components/publishers.py:90](../../sensor/sensor/components/publishers.py#L90) |
| RiskManager | supplied_collaborator | [sensor/sensor/components/risk_manager.py:47](../../sensor/sensor/components/risk_manager.py#L47) |
| Sensor | ros_timer | [sensor/sensor/sensor.py:177](../../sensor/sensor/sensor.py#L177) |
| fora de classe | ros_runtime | [sensor/sensor/sensor.py:317](../../sensor/sensor/sensor.py#L317) |
| fora de classe | thread_creation | [sensor/sensor/sensor.py:322](../../sensor/sensor/sensor.py#L322) |
| AdaptationHandler | supplied_collaborator | [shared_components/shared_components/adaptation_handler.py:78](../../shared_components/shared_components/adaptation_handler.py#L78) |
| AdaptationHandler | ros_resource | [shared_components/shared_components/adaptation_handler.py:99](../../shared_components/shared_components/adaptation_handler.py#L99) |
| AdaptationHandler | service_wait | [shared_components/shared_components/adaptation_handler.py:102](../../shared_components/shared_components/adaptation_handler.py#L102) |
| AdaptationHandler | client_call | [shared_components/shared_components/adaptation_handler.py:111](../../shared_components/shared_components/adaptation_handler.py#L111) |
| AdaptationHandler | ros_resource | [shared_components/shared_components/adaptation_handler.py:135](../../shared_components/shared_components/adaptation_handler.py#L135) |
| BatteryManager | supplied_collaborator | [shared_components/shared_components/battery_manager.py:57](../../shared_components/shared_components/battery_manager.py#L57) |
| BatteryManager | constructor_effect | [shared_components/shared_components/battery_manager.py:80](../../shared_components/shared_components/battery_manager.py#L80) |
| BatteryManager | ros_resource | [shared_components/shared_components/battery_manager.py:80](../../shared_components/shared_components/battery_manager.py#L80) |
| BatteryManager | constructor_effect | [shared_components/shared_components/battery_manager.py:85](../../shared_components/shared_components/battery_manager.py#L85) |
| BatteryManager | ros_timer | [shared_components/shared_components/battery_manager.py:85](../../shared_components/shared_components/battery_manager.py#L85) |
| BatteryManager | real_clock | [shared_components/shared_components/battery_manager.py:192](../../shared_components/shared_components/battery_manager.py#L192) |
| ConfigManagerBase | supplied_collaborator | [shared_components/shared_components/core/config_manager_base.py:8](../../shared_components/shared_components/core/config_manager_base.py#L8) |
| ConfigManagerBase | ros_parameters | [shared_components/shared_components/core/config_manager_base.py:10](../../shared_components/shared_components/core/config_manager_base.py#L10) |
| ConfigManagerBase | ros_parameters | [shared_components/shared_components/core/config_manager_base.py:11](../../shared_components/shared_components/core/config_manager_base.py#L11) |
| ConfigManagerBase | ros_parameters | [shared_components/shared_components/core/config_manager_base.py:12](../../shared_components/shared_components/core/config_manager_base.py#L12) |
| ConfigManagerBase | ros_parameters | [shared_components/shared_components/core/config_manager_base.py:13](../../shared_components/shared_components/core/config_manager_base.py#L13) |
| ConfigManagerBase | ros_parameters | [shared_components/shared_components/core/config_manager_base.py:14](../../shared_components/shared_components/core/config_manager_base.py#L14) |
| ConfigManagerBase | ros_parameters | [shared_components/shared_components/core/config_manager_base.py:15](../../shared_components/shared_components/core/config_manager_base.py#L15) |
| ConfigManagerBase | ros_parameters | [shared_components/shared_components/core/config_manager_base.py:16](../../shared_components/shared_components/core/config_manager_base.py#L16) |
| ConfigManagerBase | ros_parameters | [shared_components/shared_components/core/config_manager_base.py:18](../../shared_components/shared_components/core/config_manager_base.py#L18) |
| ConfigManagerBase | ros_parameters | [shared_components/shared_components/core/config_manager_base.py:19](../../shared_components/shared_components/core/config_manager_base.py#L19) |
| ConfigManagerBase | ros_parameters | [shared_components/shared_components/core/config_manager_base.py:21](../../shared_components/shared_components/core/config_manager_base.py#L21) |
| ConfigManagerBase | ros_parameters | [shared_components/shared_components/core/config_manager_base.py:22](../../shared_components/shared_components/core/config_manager_base.py#L22) |
| ConfigManagerBase | ros_parameters | [shared_components/shared_components/core/config_manager_base.py:23](../../shared_components/shared_components/core/config_manager_base.py#L23) |
| ConfigManagerBase | ros_parameters | [shared_components/shared_components/core/config_manager_base.py:24](../../shared_components/shared_components/core/config_manager_base.py#L24) |
| ConfigManagerBase | ros_parameters | [shared_components/shared_components/core/config_manager_base.py:25](../../shared_components/shared_components/core/config_manager_base.py#L25) |
| ConfigManagerBase | ros_parameters | [shared_components/shared_components/core/config_manager_base.py:26](../../shared_components/shared_components/core/config_manager_base.py#L26) |
| ConfigManagerBase | ros_parameters | [shared_components/shared_components/core/config_manager_base.py:28](../../shared_components/shared_components/core/config_manager_base.py#L28) |
| PublisherManagerBase | supplied_collaborator | [shared_components/shared_components/core/publisher_manager_base.py:20](../../shared_components/shared_components/core/publisher_manager_base.py#L20) |
| PublisherManagerBase | ros_resource | [shared_components/shared_components/core/publisher_manager_base.py:43](../../shared_components/shared_components/core/publisher_manager_base.py#L43) |
| PublisherManagerBase | ros_resource | [shared_components/shared_components/core/publisher_manager_base.py:44](../../shared_components/shared_components/core/publisher_manager_base.py#L44) |
| RosComponent | ros_node_base | [shared_components/shared_components/core/ros_component.py:13](../../shared_components/shared_components/core/ros_component.py#L13) |
| LifecycleManager | supplied_collaborator | [shared_components/shared_components/lifecycle_manager.py:99](../../shared_components/shared_components/lifecycle_manager.py#L99) |
| LifecycleManager | real_clock | [shared_components/shared_components/lifecycle_manager.py:110](../../shared_components/shared_components/lifecycle_manager.py#L110) |
| LifecycleManager | ros_timer | [shared_components/shared_components/lifecycle_manager.py:150](../../shared_components/shared_components/lifecycle_manager.py#L150) |
| LifecycleManager | ros_timer | [shared_components/shared_components/lifecycle_manager.py:157](../../shared_components/shared_components/lifecycle_manager.py#L157) |
| LifecycleManager | ros_timer | [shared_components/shared_components/lifecycle_manager.py:219](../../shared_components/shared_components/lifecycle_manager.py#L219) |
| LifecycleManager | real_clock | [shared_components/shared_components/lifecycle_manager.py:254](../../shared_components/shared_components/lifecycle_manager.py#L254) |
| LifecycleManager | real_sleep | [shared_components/shared_components/lifecycle_manager.py:497](../../shared_components/shared_components/lifecycle_manager.py#L497) |
| LifecycleManager | real_sleep | [shared_components/shared_components/lifecycle_manager.py:502](../../shared_components/shared_components/lifecycle_manager.py#L502) |
| LifecycleManager | real_sleep | [shared_components/shared_components/lifecycle_manager.py:507](../../shared_components/shared_components/lifecycle_manager.py#L507) |
| Logger | ros_node_base | [system_monitor/system_monitor/logger.py:20](../../system_monitor/system_monitor/logger.py#L20) |
| Logger | ros_parameters | [system_monitor/system_monitor/logger.py:67](../../system_monitor/system_monitor/logger.py#L67) |
| Logger | ros_parameters | [system_monitor/system_monitor/logger.py:68](../../system_monitor/system_monitor/logger.py#L68) |
| Logger | real_clock | [system_monitor/system_monitor/logger.py:71](../../system_monitor/system_monitor/logger.py#L71) |
| Logger | ros_resource | [system_monitor/system_monitor/logger.py:199](../../system_monitor/system_monitor/logger.py#L199) |
| Logger | ros_resource | [system_monitor/system_monitor/logger.py:201](../../system_monitor/system_monitor/logger.py#L201) |
| Logger | ros_resource | [system_monitor/system_monitor/logger.py:203](../../system_monitor/system_monitor/logger.py#L203) |
| Logger | ros_resource | [system_monitor/system_monitor/logger.py:205](../../system_monitor/system_monitor/logger.py#L205) |
| Logger | ros_resource | [system_monitor/system_monitor/logger.py:207](../../system_monitor/system_monitor/logger.py#L207) |
| Logger | real_clock | [system_monitor/system_monitor/logger.py:222](../../system_monitor/system_monitor/logger.py#L222) |
| fora de classe | ros_runtime | [system_monitor/system_monitor/logger.py:371](../../system_monitor/system_monitor/logger.py#L371) |
| fora de classe | thread_creation | [system_monitor/system_monitor/logger.py:376](../../system_monitor/system_monitor/logger.py#L376) |
| SystemMonitor | ros_node_base | [system_monitor/system_monitor/node_monitor.py:21](../../system_monitor/system_monitor/node_monitor.py#L21) |
| SystemMonitor | ros_parameters | [system_monitor/system_monitor/node_monitor.py:70](../../system_monitor/system_monitor/node_monitor.py#L70) |
| SystemMonitor | ros_parameters | [system_monitor/system_monitor/node_monitor.py:71](../../system_monitor/system_monitor/node_monitor.py#L71) |
| SystemMonitor | ros_parameters | [system_monitor/system_monitor/node_monitor.py:72](../../system_monitor/system_monitor/node_monitor.py#L72) |
| SystemMonitor | ros_parameters | [system_monitor/system_monitor/node_monitor.py:73](../../system_monitor/system_monitor/node_monitor.py#L73) |
| SystemMonitor | ros_parameters | [system_monitor/system_monitor/node_monitor.py:74](../../system_monitor/system_monitor/node_monitor.py#L74) |
| SystemMonitor | ros_parameters | [system_monitor/system_monitor/node_monitor.py:77](../../system_monitor/system_monitor/node_monitor.py#L77) |
| SystemMonitor | ros_parameters | [system_monitor/system_monitor/node_monitor.py:78](../../system_monitor/system_monitor/node_monitor.py#L78) |
| SystemMonitor | ros_parameters | [system_monitor/system_monitor/node_monitor.py:79](../../system_monitor/system_monitor/node_monitor.py#L79) |
| SystemMonitor | ros_parameters | [system_monitor/system_monitor/node_monitor.py:80](../../system_monitor/system_monitor/node_monitor.py#L80) |
| SystemMonitor | ros_parameters | [system_monitor/system_monitor/node_monitor.py:81](../../system_monitor/system_monitor/node_monitor.py#L81) |
| SystemMonitor | real_clock | [system_monitor/system_monitor/node_monitor.py:90](../../system_monitor/system_monitor/node_monitor.py#L90) |
| SystemMonitor | constructor_effect | [system_monitor/system_monitor/node_monitor.py:99](../../system_monitor/system_monitor/node_monitor.py#L99) |
| SystemMonitor | ros_resource | [system_monitor/system_monitor/node_monitor.py:99](../../system_monitor/system_monitor/node_monitor.py#L99) |
| SystemMonitor | constructor_effect | [system_monitor/system_monitor/node_monitor.py:106](../../system_monitor/system_monitor/node_monitor.py#L106) |
| SystemMonitor | ros_resource | [system_monitor/system_monitor/node_monitor.py:106](../../system_monitor/system_monitor/node_monitor.py#L106) |
| SystemMonitor | constructor_effect | [system_monitor/system_monitor/node_monitor.py:109](../../system_monitor/system_monitor/node_monitor.py#L109) |
| SystemMonitor | ros_resource | [system_monitor/system_monitor/node_monitor.py:109](../../system_monitor/system_monitor/node_monitor.py#L109) |
| SystemMonitor | constructor_effect | [system_monitor/system_monitor/node_monitor.py:112](../../system_monitor/system_monitor/node_monitor.py#L112) |
| SystemMonitor | ros_resource | [system_monitor/system_monitor/node_monitor.py:112](../../system_monitor/system_monitor/node_monitor.py#L112) |
| SystemMonitor | constructor_effect | [system_monitor/system_monitor/node_monitor.py:118](../../system_monitor/system_monitor/node_monitor.py#L118) |
| SystemMonitor | ros_resource | [system_monitor/system_monitor/node_monitor.py:118](../../system_monitor/system_monitor/node_monitor.py#L118) |
| SystemMonitor | constructor_effect | [system_monitor/system_monitor/node_monitor.py:121](../../system_monitor/system_monitor/node_monitor.py#L121) |
| SystemMonitor | ros_resource | [system_monitor/system_monitor/node_monitor.py:121](../../system_monitor/system_monitor/node_monitor.py#L121) |
| SystemMonitor | constructor_effect | [system_monitor/system_monitor/node_monitor.py:131](../../system_monitor/system_monitor/node_monitor.py#L131) |
| SystemMonitor | ros_resource | [system_monitor/system_monitor/node_monitor.py:131](../../system_monitor/system_monitor/node_monitor.py#L131) |
| SystemMonitor | constructor_effect | [system_monitor/system_monitor/node_monitor.py:140](../../system_monitor/system_monitor/node_monitor.py#L140) |
| SystemMonitor | ros_timer | [system_monitor/system_monitor/node_monitor.py:140](../../system_monitor/system_monitor/node_monitor.py#L140) |
| SystemMonitor | constructor_effect | [system_monitor/system_monitor/node_monitor.py:143](../../system_monitor/system_monitor/node_monitor.py#L143) |
| SystemMonitor | ros_timer | [system_monitor/system_monitor/node_monitor.py:143](../../system_monitor/system_monitor/node_monitor.py#L143) |
| SystemMonitor | real_clock | [system_monitor/system_monitor/node_monitor.py:166](../../system_monitor/system_monitor/node_monitor.py#L166) |
| SystemMonitor | real_clock | [system_monitor/system_monitor/node_monitor.py:225](../../system_monitor/system_monitor/node_monitor.py#L225) |
| SystemMonitor | client_call | [system_monitor/system_monitor/node_monitor.py:263](../../system_monitor/system_monitor/node_monitor.py#L263) |
| SystemMonitor | real_clock | [system_monitor/system_monitor/node_monitor.py:273](../../system_monitor/system_monitor/node_monitor.py#L273) |
| fora de classe | ros_runtime | [system_monitor/system_monitor/node_monitor.py:317](../../system_monitor/system_monitor/node_monitor.py#L317) |
| fora de classe | thread_creation | [system_monitor/system_monitor/node_monitor.py:323](../../system_monitor/system_monitor/node_monitor.py#L323) |
| fora de classe | real_sleep | [system_monitor/system_monitor/node_monitor.py:329](../../system_monitor/system_monitor/node_monitor.py#L329) |
| ParamAdapter | ros_node_base | [system_monitor/system_monitor/param_adapter.py:17](../../system_monitor/system_monitor/param_adapter.py#L17) |
| ParamAdapter | constructor_effect | [system_monitor/system_monitor/param_adapter.py:97](../../system_monitor/system_monitor/param_adapter.py#L97) |
| ParamAdapter | ros_resource | [system_monitor/system_monitor/param_adapter.py:97](../../system_monitor/system_monitor/param_adapter.py#L97) |
| ParamAdapter | ros_parameters | [system_monitor/system_monitor/param_adapter.py:102](../../system_monitor/system_monitor/param_adapter.py#L102) |
| ParamAdapter | ros_parameters | [system_monitor/system_monitor/param_adapter.py:106](../../system_monitor/system_monitor/param_adapter.py#L106) |
| ParamAdapter | constructor_effect | [system_monitor/system_monitor/param_adapter.py:109](../../system_monitor/system_monitor/param_adapter.py#L109) |
| ParamAdapter | ros_resource | [system_monitor/system_monitor/param_adapter.py:109](../../system_monitor/system_monitor/param_adapter.py#L109) |
| ParamAdapter | constructor_effect | [system_monitor/system_monitor/param_adapter.py:116](../../system_monitor/system_monitor/param_adapter.py#L116) |
| ParamAdapter | ros_resource | [system_monitor/system_monitor/param_adapter.py:116](../../system_monitor/system_monitor/param_adapter.py#L116) |
| ParamAdapter | constructor_effect | [system_monitor/system_monitor/param_adapter.py:124](../../system_monitor/system_monitor/param_adapter.py#L124) |
| ParamAdapter | ros_timer | [system_monitor/system_monitor/param_adapter.py:124](../../system_monitor/system_monitor/param_adapter.py#L124) |
| ParamAdapter | ros_resource | [system_monitor/system_monitor/param_adapter.py:153](../../system_monitor/system_monitor/param_adapter.py#L153) |
| ParamAdapter | real_clock | [system_monitor/system_monitor/param_adapter.py:162](../../system_monitor/system_monitor/param_adapter.py#L162) |
| ParamAdapter | real_clock | [system_monitor/system_monitor/param_adapter.py:169](../../system_monitor/system_monitor/param_adapter.py#L169) |
| ParamAdapter | real_clock | [system_monitor/system_monitor/param_adapter.py:226](../../system_monitor/system_monitor/param_adapter.py#L226) |
| fora de classe | ros_runtime | [system_monitor/system_monitor/param_adapter.py:257](../../system_monitor/system_monitor/param_adapter.py#L257) |
| fora de classe | thread_creation | [system_monitor/system_monitor/param_adapter.py:263](../../system_monitor/system_monitor/param_adapter.py#L263) |
| fora de classe | real_sleep | [system_monitor/system_monitor/param_adapter.py:269](../../system_monitor/system_monitor/param_adapter.py#L269) |

## Limites e comparação com ROS 1/C++

- As regras reconhecem APIs e alguns nomes de variáveis. Chamadas homônimas podem produzir falsos positivos; wrappers, aliases ou APIs não enumeradas podem ser omitidos.
- Dependências recebidas podem ser objetos de teste, mas a regra de parâmetro armazenado é somente um indício de ponto de substituição. Ela não mede esforço nem prova substituibilidade.
- Leitura de relógio/arquivo e criação de recurso podem estar em ramos opcionais. A análise não comprova execução obrigatória em todos os caminhos.
- O contador de efeitos diretos no construtor considera as operações enumeradas na regra constructor-effect. Não inclui toda interação com parâmetros, inicialização herdada nem efeitos de todos os métodos chamados.
- Espera em helper está limitada ao padrão validado de uma chamada na mesma classe; não é análise interprocedural completa.
- C++ tem 11 famílias de regras de infraestrutura correspondentes. ROS 1 usa advertise/subscribe/serviceClient/createTimer/waitForExistence; ROS 2 usa as APIs Python listadas nas regras. Construtores, herança, parâmetros ROS, diretórios e pontos de substituição ainda não têm cobertura equivalente nas regras C++.
- As regras C++ passaram nos exemplos fornecidos, mas não foram aplicadas à BSN ROS 1. Templates, macros, ponteiros e declarações de streams/threads requerem verificação nos fontes reais. Os fixtures de stream/thread usam construção explícita por expressão; não há garantia de cobertura de todas as formas de declaração C++.
- Comparar presença, localização e fase dos efeitos em responsabilidades equivalentes; não comparar o total bruto de achados das duas linguagens como medida de melhoria.
- O resultado complementa LCOM4/CBO e sustenta conclusões somente sobre indícios estáticos de isolamento. Não foram executados testes da BSN ou nós ROS.

## Autoria e reprodução

- Regras: tools/semgrep/isolation-python.yml e isolation-cpp.yml, escritas pelo assistente.
- Motor de análise e saídas originais: Semgrep Community Edition.
- Consolidação por classe, CSV e este texto: assistente; nenhuma nota de testabilidade foi criada.
- Reexecutar: `python3 tools/collect_isolation.py` e `python3 tools/summarize_isolation.py`.
- commands.json contém comandos e códigos de saída; metadata.json contém versões, hashes e escopo.
- Instalação e instruções em [ISOLATION.md](../../tools/ISOLATION.md).

## Fontes primárias

- https://semgrep.dev/docs/writing-rules/rule-syntax
- https://semgrep.dev/docs/writing-rules/testing-rules
- https://semgrep.dev/products/community-edition/
