# Interface web de controle

Painel local para operar o Khepera IV pelo console serial USB ou por Wi-Fi/SSH.
Ele usa o `app.py` como ponte entre o navegador e o robo:

- lista portas seriais;
- faz login automatico no console como `root`;
- conecta por SSH quando o robo estiver na mesma rede Wi-Fi;
- executa `patrulha` com alvo, trim, loop, diagnostico e timeout;
- executa `patrulha --teleop` para controle manual tipo carrinho remoto;
- **gera mapa A->B automaticamente** (`Mapear A->B (auto)` -> `patrulha --slow`):
  vai de A a B devagar, contorna os obstaculos e salva o mapa sozinho;
- **navega por A\*** (`Navegar pelo mapa (A*)`): planeja sobre o ultimo mapa salvo
  e dirige o robo pelos waypoints via `patrulha --mission`;
- mostra telemetria, sensores, bateria, console e trajetoria;
- cria mapa passivo com grade de ocupacao e grafo topologico;
- envia `main.c` local para o robo e recompila;
- altera parametros `#define` permitidos e recompila.

## Conexoes

A interface deve ser aberta pelo servidor Flask, porque e o processo Python que
acessa a porta serial ou cria a sessao SSH com o robo.

Nao abra `static/index.html` direto pelo navegador para operar o robo; nesse
modo a pagina nao consegue chamar as rotas `/api`.

### USB

Use o modo **USB** quando o cabo estiver conectado ao console serial do Khepera.
A interface lista as portas locais e faz login automatico como `root`.

### Wi-Fi

Use o modo **Wi-Fi** quando o Khepera ja estiver conectado na mesma rede do
computador/tablet que abre a pagina. O fluxo recomendado agora e:

1. Clique em `Procurar robos no Wi-Fi`.
2. A interface procura dispositivos com SSH aberto na rede local.
3. Se encontrar um candidato, selecione-o na lista.
4. Clique em `Conectar`.

Se a busca nao encontrar o robo, ainda da para preencher manualmente:

- IP ou host do robo;
- porta SSH, normalmente `22`;
- usuario, normalmente `root`;
- senha, se houver.

O Wi-Fi usa uma sessao SSH interativa por `paramiko`, mas os comandos enviados
ao robo continuam os mesmos do modo USB. A descoberta automatica usa uma
varredura curta do `/24` da rede local, nomes comuns como `khepera.local` e a
tabela ARP da maquina.

### Multiplos usuarios (root ou nao-root)

O controle do robo (I2C dos motores/sensores) e a compilacao em `/home/root`
exigem root. A interface lida com isso automaticamente:

- Login como `root` (USB ou SSH): usado direto, sem sudo.
- Login como usuario comum (ex.: `msuchoa`): ao ver o primeiro prompt, a
  interface eleva a sessao para root com `sudo -s`, reutilizando a mesma senha
  do login para responder ao prompt do sudo. A partir dai deploy, compilacao,
  missao e teleop rodam como root, igual ao modo USB.

Para isso o usuario precisa ter permissao de sudo no robo (estar no grupo
`sudo`/`wheel`). Se o sudo estiver como `NOPASSWD`, a senha nem chega a ser
pedida; caso contrario, a senha do login e usada.

## Rodar

```powershell
cd interface
python -m pip install -r requirements.txt
python app.py
```

Abra `http://localhost:8340` na maquina que rodou o `app.py`.

Por padrao o servidor escuta em `0.0.0.0`, ou seja, tambem fica acessivel de
**outros dispositivos na mesma rede** (PC, celular, tablet) em
`http://<IP-da-maquina-que-rodou-o-app.py>:8340` — o IP correto e impresso no
terminal ao iniciar. Isso permite, por exemplo, rodar o `app.py` no celular
conectado ao robo por USB e controlar tudo pelo navegador do PC na mesma
rede Wi-Fi (ou vice-versa).

> Sem autenticacao: qualquer um na mesma rede local pode abrir a pagina e
> mandar o robo andar. Para restringir, defina `HOST=127.0.0.1` antes de
> rodar (`HOST=127.0.0.1 python app.py`) e volte ao acesso so-local.

## Fluxo recomendado

1. Ligue o Khepera IV e escolha o modo `USB` ou `Wi-Fi`.
2. No USB, selecione a porta serial. No Wi-Fi, use `Procurar robos no Wi-Fi`.
3. Use `Enviar + compilar` para sincronizar o `khepera_real/patrulha/main.c`.
4. Teste primeiro em `Diagnostico` ou com as rodas suspensas.
5. Para controle manual, clique em `Entrar no controle` e segure os botoes de direcao.
6. Para missao autonoma A->B, defina o alvo `(X,Y)` e clique `Iniciar`.
7. Para **gerar o mapa**, clique `Mapear A->B (auto)` (ele mapeia e salva sozinho) — ou
   ligue `Iniciar mapa` manualmente antes de rodar um A->B.
8. Com um mapa salvo, clique `Navegar pelo mapa (A*)` para ir ao alvo pelo caminho planejado.

Para comandos manuais, use o console da propria pagina. `Ctrl-C` e enviado pelo
botao `Parar`.

No modo manual, a interface reenvia comandos enquanto o botao esta pressionado.
Ao soltar, ela envia freio. O programa no robo tambem para os motores se ficar
0,8 s sem receber comando. Por seguranca, o botao `Entrar no controle` so fica
habilitado depois de `Enviar + compilar` concluir com sucesso na sessao atual.

## Mapas

Cada sessao de mapeamento e salva em `interface/maps/<sessao>/`:

- `telemetry.ndjson`: uma linha JSON por amostra de telemetria;
- `grid.json`: grade de ocupacao com celulas `free`, `visited` e `occupied`;
- `graph.json`: grafo topologico com nos e arestas navegaveis.

O mapa observa pose, sensores frontais e trajeto para construir uma grade de
ocupacao limpa. Ele ja alimenta o planejador A* (abaixo).

## Geracao automatica de mapa (Mapear A->B)

O botao **Mapear A->B (auto)** faz tudo num passo: liga o mapeamento, roda o robo
de A ate o alvo `(X,Y)` em **modo lento** (`patrulha --slow`, metade da velocidade
linear) para contornar os obstaculos devagar e amostrar melhor, e ao chegar em B
(ou parar) encerra e **salva o mapa** em `interface/maps/<sessao>/`.
Endpoint: `POST /api/map/explore {gx,gy}`.

## Planejamento e navegacao por A\* (Navegar pelo mapa)

O botao **Navegar pelo mapa (A*)** planeja um caminho sobre o **ultimo mapa salvo**
e dirige o robo pelos waypoints:

1. `planner.py` roda um A* 8-conexo sobre o `grid.json`: celulas `occupied` dilatadas
   pelo raio do robo = bloqueadas; o resto (inclusive nao observado) = livre; o caminho
   e simplificado por linha-de-visada em poucos waypoints.
2. O robo sobe em **modo missao** (`patrulha --mission`) e recebe cada `goto X Y` pela
   serial, mantendo a **odometria continua** entre os trechos; o Bug2 reativo ainda
   cuida de obstaculos nao mapeados.

Endpoint: `POST /api/map/navigate {gx,gy}`.

> **Referencial:** a odometria zera a cada missao, entao o mapa e o alvo valem no
> **mesmo referencial** (robo comeca na origem do mapa). Sem localizacao persistente,
> mapas de sessoes diferentes nao se alinham — gere o mapa e navegue a partir do
> mesmo ponto de partida.

## Depois

Proximos incrementos uteis:

- localizacao persistente (corrigir a deriva da odometria entre sessoes) para reusar mapas;
- expor os ganhos do ponto-P (`K_CT`, `LOOKAHEAD_MM`, `KI_CT`) no card de parametros;
- metricas ISE/ITSE do erro lateral a partir do `telemetry.ndjson` (validacao quantitativa);
- HTTPS/autenticacao para uso fora da maquina local;
- layout mobile dedicado para controle manual em tablet/celular.
