# LCOM4 com ferramentas existentes

O cálculo é feito por **pyscn** e **potfur/lcom**, sem importar ou executar a BSN.
`run_lcom4.sh` apenas seleciona arquivos e chama essas ferramentas;
`summarize_lcom4.py` formata os resultados, sem implementar uma métrica de coesão.

Na raiz do repositório (`/ros_ws/src` neste workspace):

```sh
python3 -m venv .venv-metrics
.venv-metrics/bin/python -m pip install -r tools/requirements-lcom4.txt
bash tools/run_lcom4.sh
```

O ambiente já existe e as ferramentas já estão instaladas. Para repetir a coleta,
basta o último comando. Os resultados anteriores de LCOM4 são substituídos.

Arquivos:

- `reports/lcom4/report.md`: valores por classe, grupos, divergências e interpretação.
- `reports/lcom4/classes.csv`: tabela para planilhas, incluindo métodos excluídos.
- `reports/lcom4/pyscn.json`: saída original com os grupos de métodos.
- `reports/lcom4/pyscn-native.html`: relatório HTML original, gerado pelo pyscn.
- `reports/lcom4/potfur.txt`: saída original da segunda ferramenta.
- `reports/lcom4/files.txt`: arquivos de produção selecionados.
- `reports/lcom4/metadata.json`: versões, escopo e hashes dos fontes.
- `reports/lcom4/validation/`: exemplos conhecidos e respectivas saídas.

Para analisar apenas uma classe/arquivo diretamente:

```sh
.venv-metrics/bin/pyscn analyze --select lcom --json --output - \
  central_hub/central_hub/central_hub.py
.venv-metrics/bin/lcom central_hub/central_hub/central_hub.py
```

As duas ferramentas possuem políticas diferentes de inclusão de métodos.
Confira a seção de definições do relatório antes de comparar com C++.
As ferramentas candidatas para C++ e o estado da equivalência estão em
[METRICS_COMPARISON.md](METRICS_COMPARISON.md).
Os três exemplos básicos de validação (estado compartilhado, estados separados e
ponte por chamada) devem produzir 1, 2 e 1 em ambas; a consolidação verifica isso.
Os outros quatro exemplos documentam diferenças de convenção.

## Terceira ferramenta avaliada

Omen 4.30.0 também foi instalado em `.venv-metrics/bin/omen`, a partir do release
oficial, com verificação SHA-256 do arquivo de distribuição:

`12acc59e5084c9b9263e1c5ef01678b060ffd9c91daa80fe01aab3c7ac2c72ac`

Ele foi executado em uma cópia temporária dos mesmos 54 arquivos. A saída está em
`reports/lcom4/omen.json`. Essa execução não faz parte do comando de repetição:
o código dessa versão calcula **LCOM3**, portanto seus números foram descartados
das conclusões de LCOM4. O relatório contém o link para a implementação conferida.

Não confundir este relatório por classes com `reports/modularity/report.md`,
que foi a análise anterior de dependências declaradas por pacotes.
