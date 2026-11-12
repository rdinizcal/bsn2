# Métricas de modularidade ROS

Para a análise **LCOM4 por classes com ferramentas existentes**, veja
[LCOM4.md](LCOM4.md) e o [relatório](../reports/lcom4/report.md).
O restante desta página descreve a coleta anterior por pacotes.

Para **CBO, ciclos de classes, CC, WMC e NOM**, veja
[CLASS_MODULARITY.md](CLASS_MODULARITY.md) e o
[relatório de classes](../reports/class_modularity/report.md).

Para **isolamento e dependência de infraestrutura**, veja
[ISOLATION.md](ISOLATION.md) e o [relatório Semgrep interpretado](../reports/isolation/report.md).

Execute na raiz do workspace, com Python 3. Sem instalação de dependências ou ROS:

```sh
python3 tools/modularity_metrics.py . --output reports/modularity
```

Para a BSN ROS 1/C++, use o mesmo script e a raiz dos fontes daquela versão:

```sh
python3 /ros_ws/src/tools/modularity_metrics.py /caminho/bsn_ros1/src --output reports/ros1
```

Saídas: `report.md`, `metrics.csv` e `metrics.json`. O JSON inclui as declarações
originais e suas condições para auditoria. Resultados existentes no diretório de
saída são substituídos a cada execução.

- Ca: número de pacotes internos que dependem diretamente do pacote.
- Ce: número de outros pacotes internos dos quais o pacote depende diretamente.
- Instabilidade: Ce / (Ca + Ce); N/A quando ambos são zero. Não é nota de qualidade.
- Ciclos: grupos fortemente conexos, incluindo autorreferências. Não conta todos
  os caminhos circulares possíveis.
- Dependentes transitivos: pacotes que alcançam o pacote por dependências diretas
  ou indiretas, excluindo ele próprio. Mede alcance potencial, não mudanças reais.
- Dependências externas: nomes fora da raiz analisada; não significa necessariamente
  bibliotecas de terceiros.

O escopo padrão inclui depend, build_depend, build_export_depend, exec_depend e
run_depend (ROS 1). Dependências repetidas são contadas uma única vez. Ferramentas
de build e testes entram somente com `--include-buildtool` e `--include-test`.
Dependências de documentação ficam apenas no inventário JSON. Condições não são
avaliadas: as arestas são incluídas conservadoramente e um aviso é registrado.
Grupos de dependências não são expandidos; são sinalizados no relatório.

Compare as versões usando o mesmo escopo e limites equivalentes de pacotes.
Dependências internas são definidas pela raiz fornecida. Pastas ocultas, build,
install e log são ignoradas. Nomes de pacotes duplicados causam erro.

Esta coleta mede dependências **declaradas em package.xml**. Não mede imports,
includes, tópicos/serviços ROS, coesão, complexidade, tamanho ou testes. Manifestos
incompletos subestimam o acoplamento. Nesta BSN, por exemplo, sensor.py importa
shared_components sem declaração correspondente no package.xml de sensor.

Validação dos cálculos:

```sh
python3 -m unittest discover -s tools -p 'test_modularity_metrics.py'
```
