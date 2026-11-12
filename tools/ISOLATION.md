# Análise estática de isolamento com Semgrep

Motor existente: **Semgrep Community Edition 1.185.0**. As regras foram escritas
para esta BSN e são critérios de auditoria, não uma métrica padronizada ou um
pacote oficial de testabilidade. A saída nativa é mantida separada da interpretação.

## Executar

O ambiente já está preparado. A partir da raiz do repositório, `/ros_ws/src`:

```sh
python3 tools/collect_isolation.py
python3 tools/summarize_isolation.py
```

O primeiro comando testa as regras e executa Semgrep nos seis pacotes Python de
produção; o segundo organiza os achados por classe e gera a interpretação.
Resultados existentes em `reports/isolation` são substituídos.

Para instalar em outro ambiente:

```sh
python3 -m venv .venv-semgrep
.venv-semgrep/bin/python -m pip install -r tools/requirements-isolation.txt
```

O ambiente é separado de `.venv-metrics` para preservar as dependências das
análises anteriores. Configuração e cache temporários do Semgrep ficam em `/tmp`.
A varredura usa apenas regras locais, com métricas e consulta de versão desativadas.
Não utiliza login, serviço remoto de análise, autofix, build ou execução dos fontes ROS.

## Relatórios

Saídas **originais do Semgrep**:

- `reports/isolation/semgrep-native.txt`: achados legíveis, com localização.
- `semgrep-native.json`: dados estruturados do motor.
- `semgrep-native.sarif`: formato de análise estática para ferramentas compatíveis.
- `python-rule-tests.txt` e `cpp-rule-tests.txt`: resultados dos testes das regras.
- `validation-python-native.json` e `validation-cpp-native.json`: achados dos exemplos.

Saídas **derivadas pelo assistente**:

- `report.md`: interpretação dos casos e evidências por classe.
- `classes.csv`: categorias e contextos encontrados em cada classe.
- `findings.csv`: cada ocorrência nativa associada à classe pela localização AST.
- `summary.json`: contagens descritivas; nenhum score de testabilidade.
- `metadata.json`, `files.txt`, `commands.json`: versão, hashes, entradas e comandos.

Todos os 54 arquivos solicitados devem constar entre os arquivos efetivamente
analisados. Erros de parsing, regras omitidas, falhas nos exemplos e escopo
incompleto causam erro na coleta. As fixtures são somente analisadas, não executadas.

## Critérios

As 17 regras Python cobrem recursos, timers e runtime ROS, parâmetros, herança
direta de Node/LifecycleNode, esperas por serviços, possíveis chamadas de cliente,
arquivos/diretórios, relógio, sleep, threads, processos e ambiente. Duas regras
indicam contexto de construção: efeito direto em __init__ e espera em helper da
mesma classe chamado diretamente pelo construtor. Uma regra indica um colaborador
recebido e armazenado, usando uma lista explícita de nomes de parâmetros.

Um colaborador recebido não garante substituição fácil: o objeto pode precisar
implementar várias APIs e retornar respostas específicas para sair de um loop.
Uma espera em helper mais distante, uma dependência herdada ou um wrapper pode
não aparecer como efeito no construtor. Os encadeamentos adicionais descritos
no relatório foram revisados no código e identificados como interpretação.

As contagens não são número de dependências distintas nem número de problemas.
Uma chamada pode gerar um achado de infraestrutura e outro de contexto do
construtor. Os pontos de substituição são indícios favoráveis, não problemas.
Classes de adaptador ROS naturalmente utilizam infraestrutura.

## ROS 1/C++

`tools/semgrep/isolation-cpp.yml` tem 11 famílias correspondentes de infraestrutura,
validadas com exemplos positivos e negativos. Por exemplo:

| Conceito | ROS 2/Python | ROS 1/C++ |
|---|---|---|
| Recurso ROS | create_client/create_publisher/create_subscription | serviceClient/advertise/subscribe |
| Timer | create_timer | createTimer/createWallTimer |
| Espera por serviço | wait_for_service | waitForExistence/waitForService |
| Runtime | rclpy.init/spin | ros::init/spin |
| Tempo | time.time/sleep | ros::Time::now/std::this_thread::sleep_for |

As regras C++ não foram aplicadas à BSN ROS 1. Os exemplos não garantem cobertura
de macros, ponteiros e todas as formas de templates e construção. Por exemplo,
as fixtures de streams e threads usam expressões explícitas de construção, não
todas as formas possíveis de declarações C++.

Ainda não há regras C++ equivalentes para todos os contextos de construtor,
herança, parâmetros ROS, diretórios e pontos de substituição. Portanto, comparar
somente as famílias comuns e as responsabilidades equivalentes após verificar
os fontes reais; não comparar o total bruto das 17 regras Python contra as 11 C++.

Para executar as regras preparadas quando a versão C++ estiver disponível:

```sh
.venv-semgrep/bin/semgrep scan --config tools/semgrep/isolation-cpp.yml \
  --metrics off --disable-version-check --no-git-ignore \
  --json --output /tmp/bsn-ros1-isolation.json /caminho/bsn_ros1/src
```

Esse comando é um modelo: delimitar código de produção e excluir testes/código
gerado antes de comparar. A mesma instalação Semgrep pode analisar ambas as linguagens.

## Fontes

- https://semgrep.dev/docs/writing-rules/rule-syntax
- https://semgrep.dev/docs/writing-rules/testing-rules
- https://semgrep.dev/products/community-edition/
