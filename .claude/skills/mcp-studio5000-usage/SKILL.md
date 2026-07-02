---
name: mcp-studio5000-usage
description: Regras operacionais, limites e recuperação de erros das tools MCP deste workspace (mcp-studio5k, studio5000-ai-assistant, controllogix-docs). Use SEMPRE antes de chamar qualquer tool do mcp-studio5k que escreva no projeto (import_*, save_*, create_*), ao planejar uma sequência de imports L5X, ao listar programas/tags de um .ACD grande, ao ler valores de tags estruturadas, ou quando aparecer qualquer erro "max_bytes", "write limit", "LgxSrv_E_FATAL_ERROR", "LgxSrv_E_SERVER_FAULTED", "TargetType must be", ou falha de index do ai-assistant. Também quando a tarefa mencionar "abrir projeto", "importar L5X", "migrar malha", "exportar rotina" ou automação do Studio 5000 via MCP.
---

# Uso das tools MCP do workspace Studio 5000

Limites reais observados em produção (sessão de migração RC02a, 2026-07-02, projeto de 25.6MB).
O SDK Logix é frágil: um payload errado derruba o engine e **fault é permanente até matar o
processo**. Planejar a sessão de escrita ANTES de começar evita perder trabalho não salvo.

## mcp-studio5k — regras por tool

### Orçamento de escrita (a regra mais importante)

O servidor impõe **limite de ~4 operações de escrita por sessão de projeto aberto**
(imports, saves — todos contam). Ao estourar: `"write limit reached this session;
re-confirm required"`. Fatos:

- `confirmed=true` NÃO destrava; `restart_engine` NÃO reseta o contador.
- `close_project` + `open_project` **reseta** (`health` mostra `write_count: 0`).
- Um import aplicado mas não salvo **pode sobreviver** ao close (o engine persiste imports
  imediatamente em alguns casos), mas não conte com isso — trate como perdido.

**Planeje ciclos de no máximo 3 escritas + 1 save**, depois close/reopen:

```
Ciclo A: import_component (UDT) → import_component (AOI) → save   → close/open
Ciclo B: import_tag ×3 → save                                     → close/open
Ciclo C: import_tag restante → import_routine → save              → close/open
Ciclo D (só leituras): export_l5x → verificação                   → (sem limite)
```

Leituras (`list_*`, `export_l5x`, `get_*`) não contam no limite.

### Sessão e arquivos

| Tool | Regra / limite |
|---|---|
| `open_project` | Recusa se já houver projeto aberto — chame `close_project` antes. Após fault do engine, falha com `LgxSrv_E_SERVER_FAULTED` até seguir a recuperação abaixo. |
| `save_project_as` | Só aceita caminho `.acd` (sem export L5K). **NÃO troca a sessão para o novo arquivo** — `health` continua mostrando o path antigo; escritas seguintes atingiriam o arquivo ORIGINAL. Sempre: `save_project_as` → `close_project` → `open_project` na cópia. Verifique com `health`. |
| `health` | Sempre disponível. Use para confirmar `session.path` (qual arquivo será mutado!) e `write_count` antes de qualquer escrita. |
| `restart_engine` | Respawna o engine child, mas NÃO mata um `RSLogix5000Services` faultado nem reseta o write limit. |

### Leituras em projetos grandes (>20MB)

`list_programs` e `list_programs_routines` **serializam o projeto inteiro** e estouram
`max_bytes` (cap 20MB) em qualquer projeto grande — `page_size` não ajuda. Alternativas:

1. `list_routines(program="NOME")` — funciona, mas exige o nome do programa.
2. Nomes de programas sem listagem: o .ACD embute um XML `LogixTagInfo` (tag database
   completo com programas e tags). Extração offline:
   - localizar streams gzip pelo magic `\x1f\x8b\x08` no binário, descomprimir com
     `zlib.decompressobj(31)`;
   - decodificar UTF-16LE e procurar `<LogixTagInfo` … `</LogixTagInfo>`;
   - dá programas + tags por scope + DataTypes (não contém lógica de rungs).
3. `list_tags(scope=...)` funciona normalmente com filtros (`datatype_filter="PID"` acha
   todos os PID clássicos do controller scope em uma chamada).

`export_l5x` tem cap de retorno de **5MB** — use `x_path` estreito
(`.../Routines/Routine[@Name='X']`, `.../Tags/Tag[@Name='Y']`), nunca `Controller/Programs`.
Resultados grandes chegam como arquivo em `tool-results/…​.txt` (JSON com campo
`data.l5x`) — parseie do disco em vez de reler o output.

### Leitura de valores de tag

`get_tag_value` aceita **apenas tipos escalares** (`bool/real/dint/...`) e um `tag_xpath`
restrito — não aceita tipo `PID` nem membro via XPath de estrutura. Para ler a configuração
de uma tag estruturada (PID, UDT), use `export_l5x` da tag: o `<Data Format="Decorated">`
traz todos os membros com valores (SP, KP, KI, MAXS, MINO, bits de CTL etc.).

### Imports L5X

Gate humano: toda escrita exige `confirmed=true` (sem isso: `"confirmed=True is required
(human gate)"`). Imports que dirigem processo continuam gated por política do workspace —
OK explícito do usuário (um /goal que manda importar conta).

| Tool | Regra / limite |
|---|---|
| `import_component_l5x` | Aceita arquivo com TargetType `DataType` / `AddOnInstructionDefinition`. Importa dependências junto (UDTs do AOI). Ordem: UDTs → AOI → tags → rotinas. |
| `import_tag_l5x` | **Payload deve ter `TargetType="Tag"` — UMA tag por chamada.** `TargetType="Tags"` (plural, como o Studio exporta coleções) é recusado. Estrutura mínima: header + `<Controller Use="Context">` + (`<Tags Use="Context">` ou `<Programs>/<Program Use="Context">/<Tags>`) + `<Tag Use="Target" ...>`. `x_path` = container de destino (`Controller/Tags` ou `Controller/Programs/Program[@Name='P']/Tags`). Scope importa: tag de faceplate → controller; backing de AOI e ONS → program scope do programa que executa a rotina. |
| `import_routine_l5x` | **PERIGO: L5X de rotina com contexto completo (~480KB, como sai do `export_l5x`) causa `LgxSrv_E_FATAL_ERROR` e faulta o servidor.** Antes de importar, ENXUGUE o L5X: manter só a declaração XML, o header `<RSLogix5000Content>`, `<Controller Use="Context">`, `<Programs>/<Program Use="Context">`, `<Routines Use="Context">` e a `<Routine Use="Target">`. Remover TODOS os blocos de contexto `DataTypes`, `AddOnInstructionDefinitions`, `Tags`, `LocalTags` — referências resolvem contra o projeto (importe UDTs/AOI/tags ANTES). Rotina enxuta de 12KB importou limpo onde a de 480KB derrubou o engine. Falha de import faz rollback automático da rotina. |
| `validate_l5x` / `preview_import` | Use antes do import quando disponível; também rode o validador offline da skill `controllogix-ftview-control` (`scripts/validate_l5x.py`) — ele é mais estrito que o importador (exige `Use="Target"` e dependências), então avisos de dependência em payload de tag são aceitáveis. |

### Recuperação de engine faultado

Sintomas: `LgxSrv_E_FATAL_ERROR` num import, depois `LgxSrv_E_SERVER_FAULTED` em qualquer
chamada ("No further calls will be accepted").

1. `restart_engine` sozinho NÃO resolve — o processo `RSLogix5000Services` faultado persiste.
2. PowerShell: `Get-Process RSLogix5000Services` → `Stop-Process -Id <pid> -Force`.
3. `restart_engine` → `open_project` → `health` para confirmar sessão limpa.
4. Verifique o que sobreviveu (tags/rotinas) com leituras antes de refazer escritas —
   parte dos imports não salvos pode ter persistido.

### Verificação pós-import (obrigatória)

Import "ok" não basta. Fechar o ciclo: `save_project` → `close_project` → `open_project` →
`export_l5x` do artefato → conferir conteúdo (ex.: contagem de rungs, `MALHA_03(`=1,
`PID(`=0, pontes presentes). Só então declarar pronto e registrar no
`<PROJETO>-MODIFICATIONS.md`.

## studio5000-ai-assistant

- `index_acd_project` **pode falhar silenciosamente** ("Failed to index project - check
  logs") em ACDs grandes/v31; `get_project_overview`, `search_l5x_content`,
  `analyze_routine_structure` etc. exigem index prévio e retornam "No L5X data indexed".
- Se o index falhar, NÃO insista: caia para grounding direto no mcp-studio5k
  (`list_tags`, `list_routines`, `export_l5x`) e, para estrutura de programas, para a
  extração binária do `LogixTagInfo` descrita acima.
- Nunca use as tools de escrita do ai-assistant (`create_acd_project`,
  `smart_insert_logic`) — mutação de .ACD é exclusiva do mcp-studio5k (regra do CLAUDE.md).

## controllogix-docs

- `answer_query_tool` retorna trechos curtos e fragmentados do 1756-rm003 (PT-BR). Perguntas
  longas/compostas voltam lixo. Faça perguntas **atômicas e com termos do manual**
  ("unidade do ganho integral Ki instrução PID ganhos independentes"), uma por chamada, e
  reformule 1-2 vezes se vier "sem correspondência confiável".
- Confirmações críticas obtidas assim nesta base: PID clássico independente → Ki em 1/s;
  PIDE ISA (dependente) → TI em minutos. Conversão consolidada: `JGP=KP`,
  `JTI=KP/(60·KI)` min, `JTD=KD/(60·KP)` min, `JFF=BIAS`.

## Checklist de sessão de mutação

- [ ] `health` → confirmar `session.path` = cópia de trabalho (nunca o .ACD de produção)
- [ ] Grounding só por leituras do mcp-studio5k (nomes lembrados não contam)
- [ ] Planejar ciclos de ≤3 escritas + save (orçamento de 4)
- [ ] L5X de rotina enxuto (sem contexto de DataTypes/AOI/Tags) antes do import
- [ ] Tags: 1 por `import_tag_l5x`, `TargetType="Tag"`, scope correto
- [ ] Pós-import: save → close → open → export → conferir → MODIFICATIONS.md
