# Khepera IV — versão de hardware (`goto_avoid_real`)

Patrulha **vai-e-volta infinita** por odometria (dead-reckoning) **desviando de qualquer obstáculo**
(camada reativa por infravermelho). Roda no robô **físico** usando `libkhepera` — não é controlador Webots.

> ⚠️ **Pré-requisito que ainda falta:** o **login do robô** (usuário/senha). O robô é do CETELI
> (nº 0471) — peça as credenciais ao professor/laboratório ou consulte o manual K-Team. Sem isso
> não é possível transferir/compilar no robô.

## 1. Transferir o código para o robô
**Opção A — Wi-Fi (recomendada):** conecte o robô à rede e copie por `scp`:
```
scp goto_avoid_real.c Makefile root@<IP_DO_ROBO>:/home/root/
```
**Opção B — console serial (COM6):** no console do robô, cole o arquivo com um *heredoc*:
```
cat > goto_avoid_real.c << 'EOF'
   (cole aqui o conteúdo do .c)
EOF
```
(ou use `rz`/ZMODEM se o robô tiver, mais prático para arquivos grandes).

## 2. Compilar (no robô)
```
make            # ou: gcc -O2 -o goto_avoid_real goto_avoid_real.c -lkhepera -lpthread -lm
```

## 3. Calibrar (olhando a telemetria de ~1 Hz)
O programa imprime estado, distância da perna e os IR. Ajuste no `.c` e recompile:
- **Índices dos IR** (`IR_F/IR_FL/IR_FR`): ponha a mão na frente do robô e veja qual valor sobe.
- **`IR_OBSTACLE` / `IR_CLEAR`**: defina acima do nível de "espaço livre" medido (mesma lição da simulação: **medir, não chutar**).
- **Sinal das rodas** (`CRUISE`): se o robô andar para trás, inverta o sinal.
- **`WHEELBASE_MM`**: se a meia-volta não fechar ~180°, ajuste.

## 4. Rodar — com SEGURANÇA
1. **Primeiro com o robô suspenso** (rodas no ar, sobre um suporte): valide a lógica, os sinais e os IR sem risco.
2. Depois, no **chão em área livre**, longe de quedas/obstáculos frágeis.
3. Comece com `CRUISE` baixo.
```
./goto_avoid_real
```
Pare a qualquer momento com **Ctrl-C** (faz parada segura dos motores).

## Constantes já confirmadas (manual K-Team)
- Encoder: **147,4 pulsos/mm** → `PULSE_TO_MM = 1/147.4`
- Velocidade: **0,678181 mm/s** por unidade de `kh4_set_speed`
