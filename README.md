# Navegação A→B com Desvio de Obstáculos — Khepera IV

Projeto da disciplina **Engenharia de Software de Tempo Real (FTL094)** — CETELI / UFAM.
Um robô **Khepera IV** navega de um ponto **A** a um ponto **B** desviando de todos os
obstáculos, tanto em **simulação (Webots)** quanto no **robô físico real**.

## Algoritmo

Navegação reativa + planejamento local no estilo **Bug2** (Lumelsky & Stepanov, 1987):

1. **Go-to-goal** — mira em B e anda pela linha A→B (controle de rumo).
2. **Wall-following** — ao topar num obstáculo, contorna sua borda mantendo-o de um lado
   a uma distância-alvo (controle proporcional no sensor lateral).
3. **Leave condition** — assim que **recruza a linha A→B mais perto de B**, larga a
   parede e volta a mirar o alvo. É isso que faz o robô **voltar ao eixo** em vez de
   sair vagando após o desvio.

Na simulação a pose vem de **GPS + IMU**; no robô real, de **odometria** pelos encoders
(147,4 pulsos/mm, base entre rodas 105,4 mm).

## Estrutura

| Pasta | Conteúdo |
|---|---|
| `worlds/` | Mundo Webots (`first.wbt`) — arena 3×3 com slalom de obstáculos |
| `controllers/goto_avoid/` | Controlador da simulação (C) |
| `khepera_real/` | Versão para o robô físico (`patrulha/main.c`) + guia [`how_to_run.md`](khepera_real/how_to_run.md) |
| `entregaveis/` | Entregáveis FTL094 (LaTeX + PDF): plano de projeto, requisitos, arquitetura, plano/protocolo de testes, código-fonte, apresentação |
| `relatorio/` | Relatório técnico completo (LaTeX + PDF) |

## Como rodar

**Simulação (Webots R2025a):** abra `worlds/first.wbt`; o controlador `goto_avoid` já
está associado ao Khepera4. Compilação via toolchain do próprio Webots (`make`).

**Robô real (Khepera IV):** veja [`khepera_real/how_to_run.md`](khepera_real/how_to_run.md)
— transferência do código por console serial, build no robô (gcc + khepera4toolbox) e
execução na bateria (sem cabo).

## Equipe

Gabriel H. S. Nazaré · Luan A. R. Barreto · Matheus R. Canto ·
Matheus Serrão Uchôa · Ricardo M. Braz · Yan F. da Silva
