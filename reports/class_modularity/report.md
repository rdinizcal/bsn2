# Modularidade por classes — BSN ROS 2

**Interpretação do assistente, baseada nas saídas nativas abaixo.** CBO e CC são calculados pelas ferramentas. WMC_Lizard é a soma dos CC dos métodos declarados diretamente em cada classe; NOM conta esses métodos. A identificação da classe proprietária usa a AST Python e as linhas reportadas pelo Lizard.

## Relatórios nativos

- [pyscn HTML](pyscn-native.html): CBO por classe e complexidade pela definição do pyscn.
- [pyscn JSON](pyscn.json): dependências reconhecidas por classe e configurações.
- [Lizard HTML](lizard-native.html), [CSV](lizard-native.csv) e [texto](lizard-native.txt): CC por função/método.
- [Pyreverse DOT](classes_BSN.dot) e [SVG renderizado por Graphviz](classes-native.svg): relações de classes.
- [pyscn dependências HTML](deps-native.html) e [JSON](deps-native.json): grafo de módulos, separado do grafo de classes.
- [Omen JSON](omen-native.json): diagnóstico adicional de métricas OO; não usado como referência comparativa.

As tabelas classes.csv, methods.csv, summary.json e a análise de ciclos são derivados explicitamente produzidos pelo assistente. O algoritmo de componentes fortemente conexos é o da biblioteca NetworkX.

## Escopo e valores

54 arquivos de produção, seis pacotes, 47 classes e 318 métodos diretamente declarados. Construtores, destrutores e métodos estáticos/de classe entram no NOM/WMC. Métodos herdados não entram novamente; classes sem métodos têm NOM=0 e WMC=0. Funções globais e locais aninhadas não são atribuídas ao WMC da classe.

| Medida | Resultado |
|---|---:|
| CBO médio (variante pyscn) | 2.8936 |
| CBO máximo | 15 |
| WMC máximo, soma do CC Lizard | 99 |
| CC máxima de método, Lizard | 17 |
| Métodos com CC > 10 | 9 de 318 |
| Classes em ciclos inferidos pelo Pyreverse | 0 |
| Ciclos entre módulos detectados pelo pyscn | 0 |

## Classes prioritárias para inspeção

| Classe | CBO pyscn | WMC Lizard | CC máxima | NOM |
|---|---:|---:|---:|---:|
| [DataAccess](../../adaptation/adaptation/data_access/data_access.py#L104) | 15 | 99 | 17 | 22 |
| [ReliabilityEngine](../../adaptation/adaptation/engines/reli_engine.py#L13) | 3 | 67 | 16 | 13 |
| [LifecycleManager](../../shared_components/shared_components/lifecycle_manager.py#L16) | 1 | 57 | 7 | 15 |
| [Formula](../../adaptation/adaptation/model/formula.py#L14) | 15 | 50 | 12 | 16 |
| [Enactor](../../adaptation/adaptation/enactor/enactor.py#L17) | 7 | 40 | 8 | 15 |
| [Engine](../../adaptation/adaptation/engines/base_engine.py#L17) | 6 | 33 | 6 | 16 |
| [SystemMonitor](../../system_monitor/system_monitor/node_monitor.py#L21) | 6 | 33 | 8 | 7 |
| [Patient](../../patient/patient/patient.py#L17) | 1 | 32 | 7 | 9 |
| [Logger](../../system_monitor/system_monitor/logger.py#L20) | 7 | 32 | 10 | 14 |
| [Controller](../../adaptation/adaptation/enactor/controller.py#L10) | 8 | 31 | 11 | 7 |

DataAccess concentra o maior WMC e está entre as classes de maior CBO. ReliabilityEngine também concentra complexidade. Esses são candidatos à revisão de responsabilidades e dependências, não uma conclusão automática de projeto ruim.

## CBO: definição operacional e limites

- O pyscn inclui usos reconhecidos de classes externas ao arquivo, inclusive bibliotecas/ROS. Exclui classes pares no mesmo arquivo e referências apenas em anotações de tipo. Builtin types são excluídos; não confundir builtin types com todas as classes da biblioteca padrão.
- Formula tem CBO=15, com dependências de tipos ast.* da biblioteca padrão. Esse valor não representa 15 dependências entre classes da BSN.
- DataProcessor tem dependências via self.node que não são completamente resolvidas. O CBO informado não cobre toda a colaboração dinâmica.
- O fixture ConstructedConsumer mede 1. TypedConsumer e UntypedConsumer medem 0. LocalConsumer mede 0 embora instancie outra classe no mesmo arquivo. Esses resultados foram preservados no relatório nativo de validação.
- Não tratar esse CBO como equivalente automático a CBO/TypeCe de uma ferramenta C++.

## Ciclos de classes e módulos

O grafo nativo Pyreverse contém 47 classes e 37 relações distintas após orientar associação/composição do proprietário para o membro. NetworkX não encontrou componentes cíclicos nesse grafo. O fixture A↔B foi detectado pelo mesmo procedimento.

**Ausência de ciclos detectados não comprova ausência de ciclos reais.** O Pyreverse infere herança e atributos, mas referências recebidas sem tipo e chamadas dinâmicas podem não gerar arestas. Por exemplo, componentes que guardam self.node podem referenciar o pai sem essa relação aparecer no diagrama.

O relatório de imports do pyscn é separado, em nível de módulos. Os pacotes ROS foram copiados sem alterar bytes para uma pasta temporária com os nomes importáveis corretos (sensor/sensor.py, adaptation/..., etc.). O mapa de caminhos e hashes está em metadata.json. Os caminhos temporários nos relatórios nativos não persistem, mas os arquivos correspondentes continuam identificáveis pelo mapa. Não se usou package.xml como substituto de classes.

## Complexidade e comparabilidade com C++

O Lizard executou no mesmo código de produção e em exemplos Python/C++ equivalentes. Nos dois exemplos, straight tem CC=1 e branch tem CC=2. Assim, a soma é WMC=3 em ambos. O Lizard é a referência proposta para CC nas duas BSNs; a soma por classe permanece identificada como derivada.

Não usar a métrica class_scopes.complexity do pyscn como WMC: ela descreve o corpo executável da classe, não a soma da complexidade dos métodos.

Omen foi conferido como candidato de WMC/NOM, mas no exemplo de validação retornou WMC=4/NOM=2 em Python e WMC=0/NOM=0 no C++ com métodos inline. Por isso, seus valores nativos foram mantidos como diagnóstico e rejeitados para essa comparação. Não comparar WMC Omen em uma versão contra WMC Lizard na outra.

Para acoplamento e ciclos C++, CppDepend e Doxygen são candidatos existentes com análise/saídas nativas; a equivalência de relações com pyscn/Pyreverse ainda precisa ser validada. Doxygen produz diagramas de classe/colaboração, não automaticamente o mesmo CBO. Nenhuma BSN C++ foi analisada aqui.

## Valores por classe

| Classe e arquivo | CBO pyscn | WMC Lizard | CC máxima | NOM |
|---|---:|---:|---:|---:|
| [DataMessage](../../adaptation/adaptation/data_access/data_access.py#L33) (`adaptation/adaptation/data_access/data_access.py`) | 0 | 1 | 1 | 1 |
| [StatusMessage](../../adaptation/adaptation/data_access/data_access.py#L53) (`adaptation/adaptation/data_access/data_access.py`) | 0 | 1 | 1 | 1 |
| [EventMessage](../../adaptation/adaptation/data_access/data_access.py#L62) (`adaptation/adaptation/data_access/data_access.py`) | 0 | 1 | 1 | 1 |
| [EnergyStatusMessage](../../adaptation/adaptation/data_access/data_access.py#L71) (`adaptation/adaptation/data_access/data_access.py`) | 0 | 1 | 1 | 1 |
| [UncertaintyMessage](../../adaptation/adaptation/data_access/data_access.py#L82) (`adaptation/adaptation/data_access/data_access.py`) | 0 | 1 | 1 | 1 |
| [AdaptationMessage](../../adaptation/adaptation/data_access/data_access.py#L93) (`adaptation/adaptation/data_access/data_access.py`) | 0 | 1 | 1 | 1 |
| [DataAccess](../../adaptation/adaptation/data_access/data_access.py#L104) (`adaptation/adaptation/data_access/data_access.py`) | 15 | 99 | 17 | 22 |
| [Controller](../../adaptation/adaptation/enactor/controller.py#L10) (`adaptation/adaptation/enactor/controller.py`) | 8 | 31 | 11 | 7 |
| [Enactor](../../adaptation/adaptation/enactor/enactor.py#L17) (`adaptation/adaptation/enactor/enactor.py`) | 7 | 40 | 8 | 15 |
| [Engine](../../adaptation/adaptation/engines/base_engine.py#L17) (`adaptation/adaptation/engines/base_engine.py`) | 6 | 33 | 6 | 16 |
| [ReliabilityEngine](../../adaptation/adaptation/engines/reli_engine.py#L13) (`adaptation/adaptation/engines/reli_engine.py`) | 3 | 67 | 16 | 13 |
| [Context](../../adaptation/adaptation/goal_model/context.py#L5) (`adaptation/adaptation/goal_model/context.py`) | 0 | 12 | 2 | 11 |
| [Goal](../../adaptation/adaptation/goal_model/goal.py#L3) (`adaptation/adaptation/goal_model/goal.py`) | 1 | 2 | 1 | 2 |
| [GoalTree](../../adaptation/adaptation/goal_model/goaltree.py#L7) (`adaptation/adaptation/goal_model/goaltree.py`) | 0 | 24 | 4 | 14 |
| [LeafTask](../../adaptation/adaptation/goal_model/leaftask.py#L6) (`adaptation/adaptation/goal_model/leaftask.py`) | 3 | 17 | 5 | 13 |
| [Node](../../adaptation/adaptation/goal_model/node.py#L10) (`adaptation/adaptation/goal_model/node.py`) | 0 | 25 | 3 | 17 |
| [Property](../../adaptation/adaptation/goal_model/property.py#L5) (`adaptation/adaptation/goal_model/property.py`) | 0 | 10 | 2 | 9 |
| [Task](../../adaptation/adaptation/goal_model/task.py#L4) (`adaptation/adaptation/goal_model/task.py`) | 1 | 4 | 1 | 4 |
| [FormulaError](../../adaptation/adaptation/model/formula.py#L8) (`adaptation/adaptation/model/formula.py`) | 0 | 0 | 0 | 0 |
| [Formula](../../adaptation/adaptation/model/formula.py#L14) (`adaptation/adaptation/model/formula.py`) | 15 | 50 | 12 | 16 |
| [CentralHub](../../central_hub/central_hub/central_hub.py#L15) (`central_hub/central_hub/central_hub.py`) | 11 | 31 | 7 | 12 |
| [ConfigManager](../../central_hub/central_hub/components/config_manager.py#L12) (`central_hub/central_hub/components/config_manager.py`) | 1 | 4 | 2 | 2 |
| [DataFusionEngine](../../central_hub/central_hub/components/data_fusion.py#L10) (`central_hub/central_hub/components/data_fusion.py`) | 2 | 22 | 13 | 3 |
| [PublisherManager](../../central_hub/central_hub/components/publishers.py#L15) (`central_hub/central_hub/components/publishers.py`) | 5 | 6 | 3 | 3 |
| [SensorDataHandler](../../central_hub/central_hub/components/sensor_data_handler.py#L12) (`central_hub/central_hub/components/sensor_data_handler.py`) | 0 | 10 | 7 | 4 |
| [RiskAnalyzer](../../central_hub/central_hub/utils/risk_analyzer.py#L2) (`central_hub/central_hub/utils/risk_analyzer.py`) | 2 | 13 | 12 | 2 |
| [Visualizer](../../central_hub/central_hub/utils/visualization.py#L10) (`central_hub/central_hub/utils/visualization.py`) | 0 | 14 | 9 | 4 |
| [Patient](../../patient/patient/patient.py#L17) (`patient/patient/patient.py`) | 1 | 32 | 7 | 9 |
| [ConfigManager](../../sensor/sensor/components/config_manager.py#L13) (`sensor/sensor/components/config_manager.py`) | 1 | 3 | 1 | 3 |
| [DataProcessor](../../sensor/sensor/components/data_processor.py#L14) (`sensor/sensor/components/data_processor.py`) | 4 | 18 | 6 | 6 |
| [PublisherManager](../../sensor/sensor/components/publishers.py#L13) (`sensor/sensor/components/publishers.py`) | 3 | 5 | 2 | 3 |
| [RiskManager](../../sensor/sensor/components/risk_manager.py#L12) (`sensor/sensor/components/risk_manager.py`) | 1 | 13 | 4 | 8 |
| [Sensor](../../sensor/sensor/sensor.py#L21) (`sensor/sensor/sensor.py`) | 9 | 29 | 7 | 12 |
| [RiskEvaluator](../../sensor/sensor/utils/risk_evaluator.py#L10) (`sensor/sensor/utils/risk_evaluator.py`) | 0 | 25 | 7 | 10 |
| [AdaptationHandler](../../shared_components/shared_components/adaptation_handler.py#L16) (`shared_components/shared_components/adaptation_handler.py`) | 1 | 11 | 5 | 4 |
| [Battery](../../shared_components/shared_components/battery.py#L10) (`shared_components/shared_components/battery.py`) | 0 | 11 | 6 | 4 |
| [BatteryManager](../../shared_components/shared_components/battery_manager.py#L14) (`shared_components/shared_components/battery_manager.py`) | 6 | 17 | 8 | 7 |
| [ConfigManagerBase](../../shared_components/shared_components/core/config_manager_base.py#L6) (`shared_components/shared_components/core/config_manager_base.py`) | 1 | 6 | 3 | 3 |
| [PublisherManagerBase](../../shared_components/shared_components/core/publisher_manager_base.py#L12) (`shared_components/shared_components/core/publisher_manager_base.py`) | 7 | 21 | 4 | 8 |
| [RosComponent](../../shared_components/shared_components/core/ros_component.py#L13) (`shared_components/shared_components/core/ros_component.py`) | 4 | 8 | 2 | 6 |
| [Task](../../shared_components/shared_components/enums.py#L3) (`shared_components/shared_components/enums.py`) | 1 | 0 | 0 | 0 |
| [StatusContent](../../shared_components/shared_components/enums.py#L17) (`shared_components/shared_components/enums.py`) | 1 | 0 | 0 | 0 |
| [EventType](../../shared_components/shared_components/enums.py#L23) (`shared_components/shared_components/enums.py`) | 1 | 0 | 0 | 0 |
| [LifecycleManager](../../shared_components/shared_components/lifecycle_manager.py#L16) (`shared_components/shared_components/lifecycle_manager.py`) | 1 | 57 | 7 | 15 |
| [Logger](../../system_monitor/system_monitor/logger.py#L20) (`system_monitor/system_monitor/logger.py`) | 7 | 32 | 10 | 14 |
| [SystemMonitor](../../system_monitor/system_monitor/node_monitor.py#L21) (`system_monitor/system_monitor/node_monitor.py`) | 6 | 33 | 8 | 7 |
| [ParamAdapter](../../system_monitor/system_monitor/param_adapter.py#L17) (`system_monitor/system_monitor/param_adapter.py`) | 1 | 15 | 5 | 4 |

## Validação e reprodução

- Exemplos e saídas originais: validation/. A tipagem, a dependência no mesmo arquivo e o ciclo conhecido documentam o alcance da análise.
- Relatório nativo dos exemplos: [pyscn HTML](validation/pyscn-native.html); [Pyreverse DOT](validation/classes_Cases.dot); [Lizard Python](validation/lizard-python.csv) e [C++](validation/lizard-cpp.csv).
- Executar `.venv-metrics/bin/python tools/collect_class_modularity.py` e depois `.venv-metrics/bin/python tools/summarize_class_modularity.py`.
- Versões e hashes estão em metadata.json. Testes, launch, código gerado e auxiliares de teste não fazem parte da população.

## Fontes primárias

- https://github.com/ludo-technologies/pyscn/blob/v1.33.2/internal/analyzer/cbo.go
- https://github.com/terryyin/lizard
- https://pylint.readthedocs.io/en/latest/additional_tools/pyreverse/index.html
- https://networkx.org/documentation/stable/reference/algorithms/generated/networkx.algorithms.components.strongly_connected_components.html
- https://cppdepend.com/documentation/code-metrics
- https://doxygen.nl/manual/features.html
