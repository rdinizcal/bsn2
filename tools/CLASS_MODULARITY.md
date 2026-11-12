# Acoplamento, ciclos e complexidade por classe

O cálculo de CBO vem do pyscn e o de CC do Lizard. Pyreverse gera o diagrama
nativo de classes e NetworkX analisa seus componentes cíclicos. O assistente
consolida os resultados, associa os métodos às classes e soma seus CC para WMC.
Nenhum algoritmo próprio substitui CBO, CC ou o cálculo de componentes de grafo.

Na raiz do repositório (`/ros_ws/src`), com o ambiente já instalado:

```sh
.venv-metrics/bin/python tools/collect_class_modularity.py
.venv-metrics/bin/python tools/summarize_class_modularity.py
```

Para preparar outro ambiente: instalar `tools/requirements-class-modularity.txt`
via pip e disponibilizar Graphviz (`dot`). O coletor usa também o binário Omen
4.30.0 instalado na coleta LCOM4, exclusivamente como diagnóstico. Seu resultado
de WMC/NOM foi rejeitado como base comparativa pelo exemplo Python/C++.
As versões e hashes de entrada ficam em `reports/class_modularity/metadata.json`.

Saídas originais das ferramentas:

- `reports/class_modularity/pyscn-native.html` e `pyscn.json`: CBO e complexidade.
- `lizard-native.html`, `lizard-native.csv`, `lizard-native.txt`: CC de funções/métodos.
- `classes_BSN.dot`: diagrama de classes do Pyreverse.
- `classes-native.svg`: renderização Graphviz desse DOT.
- `deps-native.html` e `deps-native.json`: dependências de **módulos**, separadas.
- `omen-native.json`: diagnóstico OO adicional; não referência de comparação.

Saídas derivadas, produzidas pelo assistente:

- `report.md`: interpretação, definições e limites.
- `classes.csv`: CBO, WMC, CC máxima, NOM e relações inferidas por classe.
- `methods.csv`: CC dos métodos declarados, associado à classe por AST e linha.
- `class-cycles-analysis.json`: componentes cíclicos obtidos via NetworkX sobre o DOT.
- `class-graph-networkx.json`: grafo de relações orientado de proprietário/usuário
  para dependência, serializado pela função node_link_data do NetworkX.

O CBO do pyscn inclui usos reconhecidos de tipos externos, mas exclui classes no
mesmo arquivo e dependências apenas em anotações. O grafo do Pyreverse cobre
herança e associações de atributos inferidas; não cobre todas as chamadas e tipos
dinâmicos. Portanto, zero ciclos detectados não demonstra ausência de ciclos reais.

WMC é a soma do CC dos métodos diretamente declarados, incluindo construtores,
destrutores, estáticos e métodos de classe. Métodos herdados não são recontados,
funções internas aninhadas e funções globais não entram. Classes sem métodos
têm WMC=0 e NOM=0. Todas as correspondências de métodos com o CSV Lizard são
verificadas; método ausente causa erro, em vez de ser silenciosamente ignorado.

Os exemplos em `reports/class_modularity/validation` verificam CC em Python/C++,
um ciclo de classes conhecido e os limites de CBO por tipagem/arquivo. São
analisados estaticamente e não importados nem executados.

Critérios e candidatos C++ estão em [METRICS_COMPARISON.md](METRICS_COMPARISON.md).
