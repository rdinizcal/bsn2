# Comparação de métricas Python / C++

Fontes consultadas em 12/11/2026. A escolha é por **métrica e implementação**,
não apenas pela linguagem suportada ou pelo nome comercial da ferramenta.

## Evidências e autoria

Cada coleta deve preservar as saídas nativas e a versão/configuração utilizada.
O relatório interpretativo e o CSV consolidado pelo assistente são derivados;
não substituem as saídas originais nem modificam o cálculo das ferramentas.

Para LCOM4 em ROS 2, `bash tools/run_lcom4.sh` gera:

- `reports/lcom4/pyscn-native.html`: HTML nativo, com resumo e valores por classe.
- `reports/lcom4/pyscn.json`: JSON nativo, incluindo grupos de métodos.
- `reports/lcom4/potfur.txt`: tabela nativa da segunda ferramenta.
- `reports/lcom4/report.md` e `classes.csv`: interpretação e organização derivadas.
- `reports/lcom4/metadata.json` e `files.txt`: versões, hashes e escopo.

## Ferramentas verificadas

| Métrica | Python | C++ | Saídas nativas e estado da comparação |
|---|---|---|---|
| LCOM4 | pyscn 1.33.2; potfur/lcom 0.1.0 | ROSE-LCOM-Tools (LLNL), candidato documental | pyscn: HTML/JSON; potfur: tabela; ROSE: CSV e grafos DOT. Mesmo conceito de componentes por atributos/chamadas, mas populações de métodos diferentes. Equivalência operacional ainda não validada. |
| CC por função/método | Lizard 1.26.0 | Lizard 1.26.0 | Coleta ROS 2 executada, com CSV/HTML/texto nativos. Exemplo equivalente Python/C++ validado com CC 1 e 2. BSN C++ ainda não executada. |
| LCOM3 | Omen 4.30.0 | Omen | Implementação instalada usa componentes por atributos, sem arestas de chamadas. Não substituir LCOM4 por esse número. Python executado apenas como diagnóstico; C++ não executado. |
| CBO por classe | pyscn 1.33.2 | CppDepend, candidato documental de acoplamento de tipos | Coleta Python e HTML/JSON nativos disponíveis. pyscn exclui classes pares no mesmo arquivo e referências só em anotações. Não equivalente automaticamente ao TypeCe/TypeCa do CppDepend. |
| Ciclos entre classes | Pyreverse 4.1.3 (grafo) + NetworkX 3.7 (componentes) | Doxygen (grafo), CppDepend (análise), candidatos | DOT nativo, SVG Graphviz e análise NetworkX disponíveis para Python. Escopo: herança/associações inferidas, não todas as dependências. Sem validação na BSN C++. |
| WMC | Soma do CC Lizard por classe (derivada de CSV nativo) | Mesmo Lizard, soma por classe a preparar | Exemplo CC Python/C++ validado; associação dos métodos a classes C++ ainda não preparada. Omen foi rejeitado para essa comparação: WMC/NOM diferem nos exemplos equivalentes. |
| NOM | Métodos declarados diretamente, inventariados por AST e presentes no CSV Lizard | Contagem equivalente a preparar | Inclui construtores/destrutores/estáticos, exclui herdados. Não comparar contagens com critérios distintos. |
| Indícios de isolamento/infraestrutura | Semgrep CE 1.185.0, 17 regras locais | Mesmo motor, 11 regras ROS 1/C++ locais | ROS 2 executado com TXT/JSON/SARIF nativos. Regras Python e C++ testadas em exemplos; BSN C++ não executada. Só comparar famílias e contextos equivalentes; não é métrica padronizada. |

O README de polyscan lista coesão LCOM4 para Python, Go e Rust, mas não para C++.
Não assumir que o suporte geral de polyscan a C++ inclui LCOM4.

## Nova coleta: acoplamento e complexidade

A análise estática de isolamento já está em [ISOLATION.md](ISOLATION.md) e no
[relatório de isolamento](../reports/isolation/report.md), com saídas nativas
Semgrep e interpretação separadas.

Consulte [CLASS_MODULARITY.md](CLASS_MODULARITY.md) e o
[relatório de classes](../reports/class_modularity/report.md). CBO e CC têm
relatórios nativos; o diagrama de classes também é saída nativa. As contagens de
ciclos via NetworkX e a soma WMC permanecem explicitamente identificadas como
análises derivadas.

Omen 4.30.0 não deve ser usado como referência de WMC/NOM entre linguagens:
no exemplo ComplexityExample retorna WMC=4/NOM=2 em Python e WMC=0/NOM=0 no C++
com métodos inline. As saídas estão preservadas em
`reports/class_modularity/validation/omen-native.json`.

CppDepend oferece acoplamento por tipo e consultas de dependências; Doxygen
oferece diagramas de classe/colaboração. Nenhum dos dois foi instalado ou
executado nesta etapa e nenhum foi homologado como substituto equivalente do
CBO pyscn. São candidatos, não uma promessa de números intercambiáveis.

## LCOM4: o que ainda impede uma comparação direta

O pyscn exclui métodos sem estado, certos acessores isolados e métodos vazios,
além de construtores e outras categorias. O README de ROSE-LCOM-Tools declara
considerar todos os métodos dentro da classe, inclusive sem acesso a atributos.
Assim, mesmo uma tradução fiel pode receber resultados diferentes.

O potfur/lcom também possui regras e limitações próprias, demonstradas pelos
exemplos em `reports/lcom4/validation`. Portanto, ROSE-LCOM-Tools é uma ferramenta
existente de LCOM4 para C++, mas ainda **não foi homologada como equivalente**
à configuração Python usada. Não comparar suas médias diretamente.

Antes de medir a BSN ROS 1:

1. Fixar versão e opções da ferramenta C++ e preparar suas dependências de análise.
2. Traduzir os exemplos de validação para C++, mantendo estado e relações.
3. Conferir construtores/destrutores, métodos vazios/abstratos, acessores,
   estáticos, herança, chamadas externas e classes sem métodos participantes.
4. Conferir acesso a membros aninhados, como `self.node.config` e `node.config`.
5. Só comparar números se as populações e regras puderem ser alinhadas pelas
   ferramentas existentes. Caso contrário, registrar a limitação e escolher
   outra implementação ou métrica comum, sem renomear LCOM3 como LCOM4.

ROSE-LCOM-Tools depende do framework ROSE; sua preparação é mais extensa que uma
instalação pip. Não foi instalado nem executado nesta etapa. Nenhuma BSN C++
foi fornecida/encontrada nos fontes deste workspace.

## Fontes primárias

- [pyscn, versão usada](https://github.com/ludo-technologies/pyscn/tree/v1.33.2)
- [potfur/lcom, commit usado](https://github.com/potfur/lcom/tree/b8b7d75c149864ab8c2cab7296a06da01a4a3359)
- [ROSE-LCOM-Tools: LCOM4, CSV, grafos e limitações](https://github.com/llnl/ROSE-LCOM-Tools)
- [Polyscan: matriz de suporte por linguagem e dimensão](https://github.com/ludo-technologies/polyscan#language-support)
- [Omen: implementação instalada de LCOM3](https://github.com/panbanda/omen/blob/omen-v4.30.0/src/analyzers/cohesion.rs)
- [Lizard: linguagens, relatórios e limitações](https://github.com/terryyin/lizard)
- [CppDepend: métricas por tipo e método](https://cppdepend.com/documentation/code-metrics)
- [Doxygen: diagramas de classe e colaboração](https://doxygen.nl/manual/features.html)
- [Pyreverse: saídas nativas](https://pylint.readthedocs.io/en/latest/additional_tools/pyreverse/index.html)
