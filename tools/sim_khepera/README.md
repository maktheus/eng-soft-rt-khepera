# Visualizador OpenGL do Khepera

Ferramenta local em C para simular a logica que sera compilada e enviada ao robo.

O ponto importante: `khepera_gl.c` inclui diretamente:

```c
#include "../../khepera_real/patrulha/controller_core.h"
```

Esse `controller_core.h` tambem e usado por `khepera_real/patrulha/main.c`. Assim, a janela
OpenGL e o robo fisico executam a mesma maquina de estados `GOAL/WALLF/BEIRADA`; a simulacao
troca apenas hardware por raycast de sensores IR e cinematica diferencial aproximada.

A escala esta em milimetros. O desenho separa:

- corpo do Khepera: raio aproximado de `70 mm`;
- footprint/folga de seguranca: raio de `90 mm`;
- sensores IR: raios ate `350 mm`;
- grade: celulas de `250 mm`;
- obstaculos: caixas retangulares em dimensoes reais aproximadas, como `150x150`,
  `200x200`, `250x250`, paredes e blocos maiores.

## Mundos em JSON

O simulador carrega `tools/sim_khepera/worlds_1000.json` por padrao. Esse arquivo
tem 1000 cenarios progressivos, todos em milimetros:

```json
{
  "name": "case_0001_single_pair",
  "difficulty": 1,
  "goal": [1918, -110],
  "bounds": [-450, 2438, -900, 900],
  "obstacles": [
    {"name": "box_main", "rect": [805.6, -38.5, 188.1, 258.1]}
  ]
}
```

Para recriar a bateria:

```powershell
python tools\sim_khepera\generate_worlds.py --count 1000 --out tools\sim_khepera\worlds_1000.json
```

## Rodar no Windows

Modo batch, gera imagens e metricas:

```powershell
.\tools\sim_khepera\build.ps1
```

Modo interativo, abre a janela OpenGL:

```powershell
.\tools\sim_khepera\build.ps1 -Interactive
```

Teclas:

- `N`/`P` ou setas: troca cenario;
- `1`..`9`: vai para um dos primeiros cenarios;
- `R`: reinicia o cenario atual;
- `F`: alterna tempo real `1x` e acelerado `5x`;
- `Espaco`: pausa/continua;
- `Esc`: sai.

## Saidas

O modo batch gera:

```text
tools/sim_khepera/out_gl/
```

Arquivos principais:

- `summary.txt`: metricas por cenario;
- `*.bmp`: frames renderizados pelo OpenGL do primeiro cenario, do ultimo e de falhas;
- `batch_stdout.txt`: log completo opcional quando a validacao e rodada pela interface.

## Pela interface web

Rode:

```powershell
cd interface
python app.py
```

Em `http://localhost:8340`, use o painel `Simulacao OpenGL` para:

- gerar o JSON com ate 1000 mundos;
- validar o lote inteiro em batch;
- abrir a janela OpenGL interativa;
- ver o total de mundos, chegadas, falhas e timeouts do ultimo `summary.txt`.

## Leitura da visualizacao

- verde: estado `GOAL`, rastreando a linha A->B;
- azul: estado `WALLF`, contornando obstaculo;
- marrom: obstaculo;
- retangulo claro ao redor do obstaculo: area inflada pelo raio aproximado do robo;
- linhas amarelas/vermelhas: sensores IR livres ou com deteccao;
- linha cinza: linha ideal A->B;
- disco amarelo: robo e orientacao final.
