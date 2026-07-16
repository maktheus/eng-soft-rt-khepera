# Navegação A→B com Desvio de Obstáculos — Khepera IV

Projeto da disciplina **Engenharia de Software de Tempo Real (FTL094)** — CETELI / UFAM.
Um robô **Khepera IV** navega de um ponto **A** a um ponto **B** desviando de todos os
obstáculos, tanto em **simulação (Webots)** quanto no **robô físico real**.

## Algoritmo

O objetivo principal do projeto e navegar sem mapa previo. O robo conhece apenas
o alvo B no referencial da odometria e descobre obstaculos ao toca-los com o IR.

Navegacao tatil orientada ao alvo:

1. **GOAL** - mantem o rumo de B por odometria. Os dois motores avancam e as
   correcoes acontecem em arco, sem rotacao continua no proprio eixo.
2. **CONTATO** - uma leitura IR alta registra um ponto do obstaculo no mapa tatil.
3. **RECUO** - afasta 85 mm, medidos pelos encoders.
4. **GIRO** - escolhe um lado persistente e faz um unico giro limitado, sobre
   uma roda, para assumir a direcao tangencial ao obstaculo.
5. **CONTORNO** - avanca um trecho medido pela odometria, sem depender de manter
   a parede visivel. O trecho cresce quando o obstaculo e maior.
6. **ALINHA/TESTE** - volta a apontar para B e confirma o caminho avancando. Se
   tocar novamente, continua pelo mesmo lado; se reconhecer um loop ou falta de
   progresso, troca de lado uma decisao por vez.

A simulacao e o robo fisico compartilham o mesmo \`controller_core.h\`. No robo
real, a pose vem da odometria (147,4 pulsos/mm, base entre rodas 105,4 mm).

### Planejamento global sobre o mapa (opcional / diagnostico)

Depois de uma missao real, a interface pode **analisar** o mapa observado: um
**A\*** sobre esse mapa (`interface/planner.py`) produz um caminho idealizado
para diagnostico/comparacao. Esse modo pode dirigir o robo por waypoints em
**modo missao** (`--mission`), mas nao e o objetivo principal. A geracao
automatica de mapa (`--slow`) continua rodando A->B com sensores locais e salva
o mapa observado ao final.

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

**Interface web local (USB/serial ou Wi-Fi/SSH):** da raiz do repositorio, rode:

```bash
make run
```

O comando cria/atualiza `.venv`, instala as dependencias de `interface/requirements.txt`
e sobe `http://localhost:8340`. O backend Flask tambem serve o frontend estatico de
`interface/static/`, entao front e back ficam prontos juntos para chamar as rotas `/api`
que comunicam com o robo. Veja [`interface/README.md`](interface/README.md).
No Windows sem GNU Make instalado, use `.\make.cmd run` ou instale o `make` para usar
exatamente `make run`.
Ela sobe um servidor Flask em `http://localhost:8340` para conectar ao robô (login `root`,
ou usuário comum com elevação automática via `sudo`), executar missões, usar controle manual
tipo carrinho remoto, acompanhar telemetria, criar mapas em grade/grafo, **gerar mapa A→B
automaticamente**, **diagnosticar/rodar A\* sobre mapa salvo**, enviar `main.c`, recompilar e ajustar
parâmetros `#define` permitidos.

## Equipe

Gabriel H. S. Nazaré · Luan A. R. Barreto · Matheus R. Canto ·
Matheus Serrão Uchôa · Ricardo M. Braz · Yan F. da Silva
