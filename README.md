# Navegação A→B com Desvio de Obstáculos — Khepera IV

Projeto da disciplina **Engenharia de Software de Tempo Real (FTL094)** — CETELI / UFAM.
Um robô **Khepera IV** navega de um ponto **A** a um ponto **B** desviando de todos os
obstáculos, tanto em **simulação (Webots)** quanto no **robô físico real**.

## Algoritmo

Navegação reativa + planejamento local no estilo **Bug2** (Lumelsky & Stepanov, 1987):

1. **Go-to-goal (rastreamento de linha por ponto-P)** — em vez de só mirar em B, o robô
   **segue a linha A→B** usando o erro lateral `d_line`: o rumo desejado une o avanço na
   direção da linha com uma correção proporcional+integral (com anti-windup) do desvio ao
   eixo. É a malha externa (cinemática) de um controle em cascata, adaptada ao acionamento
   diferencial. Alternável por `USE_POINT_P` (fallback = proporcional de rumo clássico).
2. **Wall-following** — ao topar num obstáculo, contorna sua borda mantendo-o de um lado
   a uma distância-alvo (controle proporcional no sensor lateral).
3. **Leave condition** — assim que **recruza a linha A→B mais perto de B**, larga a
   parede e volta ao rastreamento. Com o ponto-P, ao sair do contorno o robô **volta ao
   eixo A→B** em vez de cortar diagonal até B — corrige a re-aquisição da linha no Bug2.

Na simulação a pose vem de **GPS + IMU**; no robô real, de **odometria** pelos encoders
(147,4 pulsos/mm, base entre rodas 105,4 mm).

### Planejamento global sobre o mapa (opcional)

Além do A→B reativo, a interface pode **planejar** um caminho: um mapa de ocupação é gerado
enquanto o robô anda, e um **A\*** sobre esse mapa (`interface/planner.py`) produz waypoints
que o robô percorre em **modo missão** (`--mission`, odometria contínua entre legs). Há
também **geração automática de mapa** (`--slow`): o robô vai de A a B devagar, contorna os
obstáculos e salva o mapa sozinho.

## Estrutura

| Pasta | Conteúdo |
|---|---|
| `worlds/` | Mundo Webots (`first.wbt`) — arena 3×3 com slalom de obstáculos |
| `controllers/goto_avoid/` | Controlador da simulação (C) |
| `khepera_real/` | Versão para o robô físico (`patrulha/main.c`) + guia [`how_to_run.md`](khepera_real/how_to_run.md) |
| `interface/` | Interface web local para controle USB/Wi-Fi, deploy, compilação, parâmetros, telemetria e mapas do robô real |
| `entregaveis/` | Entregáveis FTL094 (LaTeX + PDF): plano de projeto, requisitos, arquitetura, plano/protocolo de testes, código-fonte, apresentação |
| `relatorio/` | Relatório técnico completo (LaTeX + PDF) |

## Como rodar

**Simulação (Webots R2025a):** abra `worlds/first.wbt`; o controlador `goto_avoid` já
está associado ao Khepera4. Compilação via toolchain do próprio Webots (`make`).

**Robô real (Khepera IV):** veja [`khepera_real/how_to_run.md`](khepera_real/how_to_run.md)
— transferência do código por console serial, build no robô (gcc + khepera4toolbox) e
execução na bateria (sem cabo).

**Interface web local (USB/serial ou Wi-Fi/SSH):** veja [`interface/README.md`](interface/README.md).
Ela sobe um servidor Flask em `http://localhost:8340` para conectar ao robô (login `root`,
ou usuário comum com elevação automática via `sudo`), executar missões, usar controle manual
tipo carrinho remoto, acompanhar telemetria, criar mapas em grade/grafo, **gerar mapa A→B
automaticamente**, **navegar por A\* sobre o mapa**, enviar `main.c`, recompilar e ajustar
parâmetros `#define` permitidos.

## Equipe

Gabriel H. S. Nazaré · Luan A. R. Barreto · Matheus R. Canto ·
Matheus Serrão Uchôa · Ricardo M. Braz · Yan F. da Silva
