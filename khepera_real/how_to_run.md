# Como rodar o `patrulha` no Khepera IV — dois caminhos

Guia prático para executar o controlador **`patrulha`** (patrulha vai-e-volta com
desvio reativo contínuo) no robô físico **Khepera IV** (`khepera4_1401`).

> 🧭 **Convenção de ícones — leia isto primeiro!**
> - 🪟 = comando roda **no PowerShell do Windows** (no seu PC, prompt `PS U:\>`)
> - 🤖 = comando roda **no shell Linux do robô** (depois de logar como `root`)
>
> A maior fonte de erro é colar um comando 🤖 no PowerShell 🪟 (dá erro de `&&`).
> No robô (Linux) use `&&`/`;`; no Windows (PowerShell) use `;`.

---

## Onde está tudo

| Item | Caminho |
|---|---|
| Projeto (robô) | `/home/root/khepera4toolbox/app/patrulha/` |
| Fonte | `main.c` (+ `i2cal.c` corrigido, `Makefile`) |
| Binário | `patrulha` |
| Login do robô | usuário `root`, **sem senha** (só Enter) |
| Serial (USB) | porta **COM6**, 115200 8N1 |
| Wi-Fi do robô | SSID `Botchas_2.4G`, IP `192.168.1.33` |
| Chave SSH (PC) | `~/.ssh/claude_edgebench` |

---

## Caminho A — USB (cabo / console serial)

Use quando: vai **compilar/editar**, fazer **teste com rodas no ar**, ou o Wi-Fi caiu.
O robô fica **preso pelo cabo** → bom pra bancada, ruim pra andar no chão.

### A.1 — Abrir o terminal do robô
**Opção PuTTY (recomendado):** Connection type **Serial** · Serial line `COM6` ·
Speed `115200` → **Open** → Enter → no `login:` digite `root` → Enter (sem senha).

**Opção sem PuTTY (pelo PowerShell do Windows):** 🪟 abre uma "ponte" e manda 1 comando ao robô:
```powershell
$p=New-Object System.IO.Ports.SerialPort('COM6',115200,'None',8,'One');$p.DtrEnable=$true;$p.RtsEnable=$true;$p.Open();Start-Sleep 1;$p.Write("root`r");Start-Sleep 2;$p.Write("`r");Start-Sleep 1;$p.Write("SEU_COMANDO_AQUI`r");Start-Sleep 3;Write-Host $p.ReadExisting();$p.Close()
```
(troque `SEU_COMANDO_AQUI` pelo comando Linux desejado).

### A.2 — Rodar (já logado no robô) 🤖
```sh
cd ~/khepera4toolbox/app/patrulha
./patrulha
```
Pare com **Ctrl-C** (faz parada segura dos motores).

> ⚠️ Com USB, o cabo limita o movimento. Para validar a lógica, **apoie o robô com as
> rodas no ar** antes de pôr no chão.

---

## Caminho B — Wi-Fi (SSH, sem fio) ✅ recomendado para o chão

Use quando: quer o robô **andando livre no chão**, controlado à distância.

### Pré-requisitos (já feitos, mas repita se reiniciar o robô)
1. **PC e robô na MESMA banda 2.4 GHz** (o Wi-Fi do Khepera é só 2.4 GHz).
   Conecte seu PC ao `Botchas_2.4G` (senão o roteador isola um do outro).
2. **Robô no Wi-Fi** — *a conexão NÃO sobrevive a reboot*. Se reiniciou, reconecte
   pelo USB (Caminho A) rodando no robô 🤖:
   ```sh
   wpa_passphrase Botchas_2.4G '<SUA_SENHA_WIFI>' > /tmp/wpa.conf
   wpa_supplicant -B -i wlan0 -c /tmp/wpa.conf -D nl80211,wext
   udhcpc -i wlan0
   ip addr show wlan0 | grep inet      # confirma o IP (192.168.1.33)
   ```
3. **Chave SSH instalada** no robô (1x; também não esquece em reboot pois fica no eMMC):
   no robô 🤖 `cat ~/.ssh/authorized_keys` deve conter `...claude-deploy@edgebench`.

### B.1 — Testar o SSH 🪟
```powershell
ssh -i "$env:USERPROFILE\.ssh\claude_edgebench" -o UserKnownHostsFile=NUL -o StrictHostKeyChecking=no root@192.168.1.33 "echo OK; uname -n"
```
Deve imprimir `OK` e `khepera4_1401`.

### B.2 — Rodar no chão (burst seguro com auto-parada) 🪟
O `timeout` garante que o robô **para sozinho** ao fim — mesmo se o Wi-Fi cair:
```powershell
ssh -i "$env:USERPROFILE\.ssh\claude_edgebench" -o UserKnownHostsFile=NUL -o StrictHostKeyChecking=no root@192.168.1.33 "cd ~/khepera4toolbox/app/patrulha; timeout 12 ./patrulha"
```
(roda 12 s e para; aumente o número pra rodar mais tempo.)

> 🔌 Para andar livre de verdade, **desconecte o cabo USB** (o controle continua pelo Wi-Fi).
> 🤚 Mantenha alguém por perto, pronto pra pegar o robô.

---

## Recompilar (depois de editar `main.c`)

No robô 🤖 (uma linha só):
```sh
cd ~/khepera4toolbox/app/patrulha && gcc -Wall \
  -I/home/root/khepera4toolbox/Modules/khepera4 \
  -I/home/root/khepera4toolbox/Modules/commandline \
  -I/home/root/khepera4toolbox/Modules/i2cal \
  main.c i2cal.c /home/root/khepera4toolbox/Modules/khepera4/*.c \
  /home/root/khepera4toolbox/Modules/commandline/commandline.c \
  -lm -o patrulha && echo BUILD_OK
```
> Por que não `make`? O build system do toolbox só monta `.a` (não linka o executável).
> E usamos `i2cal.c` **local corrigido** (o do toolbox tem `/dev/i2c-3` chumbado; neste
> robô o dsPIC está em `/dev/i2c-2`).

---

## Calibração (parâmetros no topo do `main.c`)

| Parâmetro | Hoje | Efeito |
|---|---|---|
| `SPEED` | 12000 | velocidade geral (↑ mais rápido) |
| `K_STEER` | 0.0014 | força da curva de desvio (↑ desvia mais cedo/forte) |
| `FRONT_SPAN` | 450 | quão cedo desacelera ao ver obstáculo |
| `IR_BASE` | 120 | base livre dos sensores (ruído) |
| `PATROL_MM` | 1000 | comprimento de cada "perna" da patrulha |
| `WHEELBASE_MM` | 105.4 | ajuste se a meia-volta não fechar ~180° |

Editou → **recompile** (seção acima) → rode de novo.

---

## Segurança

- 1º teste sempre com **rodas no ar**; só depois no chão.
- No chão: **área livre**, sem quedas/escadas, longe de coisas frágeis.
- Use **bursts com `timeout`** (auto-parada) e fique pronto pra pegar o robô.
- **Ctrl-C** sempre faz parada segura dos motores.

---

## Troubleshooting

| Sintoma | Causa / solução |
|---|---|
| `O token '&&' não é válido` | Você colou comando 🤖 no PowerShell 🪟. Rode no shell do robô. |
| SSH `Connection timed out` | PC e robô em bandas diferentes (PC no 5 GHz). Ponha o PC no `Botchas_2.4G`. |
| `ping` falha mas SSH funciona | Normal: o Wi-Fi descarta ICMP. Use a **porta 22** como referência. |
| Robô sumiu da rede após ligar | Wi-Fi é temporário: reconecte (Caminho B, passo 2) via USB. |
| `COM6` ocupada | Outra conexão serial aberta; feche o PuTTY/PowerShell anterior. |
| `unable to open the I2C bus` | Use o `i2cal.c` local (`/dev/i2c-2`), não o do toolbox. |

---

## Resumo rápido (cheat sheet)

```text
USB:   PuTTY COM6 115200 -> login root -> cd ~/khepera4toolbox/app/patrulha -> ./patrulha   (Ctrl-C)
WiFi:  🪟 ssh -i ~/.ssh/claude_edgebench root@192.168.1.33 "cd ~/khepera4toolbox/app/patrulha; timeout 12 ./patrulha"
```
