# Interface web de controle

Painel local para operar o Khepera IV pelo console serial USB ou por Wi-Fi/SSH.
Ele usa o `app.py` como ponte entre o navegador e o robo:

- lista portas seriais;
- faz login automatico no console como `root`;
- conecta por SSH quando o robo estiver na mesma rede Wi-Fi;
- executa `patrulha` com alvo, trim, loop, diagnostico e timeout;
- executa `patrulha --teleop` para controle manual tipo carrinho remoto;
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

## Rodar

```powershell
cd interface
python -m pip install -r requirements.txt
python app.py
```

Abra `http://localhost:8340`.

## Fluxo recomendado

1. Ligue o Khepera IV e escolha o modo `USB` ou `Wi-Fi`.
2. No USB, selecione a porta serial. No Wi-Fi, use `Procurar robos no Wi-Fi`.
3. Use `Enviar + compilar` para sincronizar o `khepera_real/patrulha/main.c`.
4. Teste primeiro em `Diagnostico` ou com as rodas suspensas.
5. Para mapear, clique em `Iniciar mapa` antes de mover o robo.
6. Para controle manual, clique em `Entrar no controle` e segure os botoes de direcao.
7. Para missao autonoma, rode com timeout curto e acompanhe a telemetria.

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

O mapa e passivo: ele nao muda a decisao do robo ainda. Ele observa pose,
sensores frontais e trajeto para construir uma representacao limpa que depois
pode alimentar A*, Dijkstra ou outro planejador.

## Depois

Proximos incrementos uteis para Wi-Fi/tablet:

- identificacao mais precisa do Khepera quando houver varios dispositivos SSH na rede;
- HTTPS/autenticacao para uso fora da maquina local;
- layout mobile dedicado para controle manual em tablet/celular;
- perfis salvos para mais de um robo.
