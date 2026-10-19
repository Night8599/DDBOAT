# DDBOAT

Biblioteca da equipe do barco 04 para Guerledan. Um codigo de missao roda igual
no barco real, no simulador Unity e num simulador Python (modelo de 8 estados do
polycopie do Jaulin), e tudo sai num log completo por volta do laco.

## Estrutura

```
main.py                    ponto de entrada unico (python3 main.py -h)
config/boat04.json         TODOS os parametros (null = ainda nao definido)
ddboat/
  core/        angles (unica fonte de convencao), geo, linalg (Python puro),
               config, clock (laco de frequencia fixa), shutdown, keyboard
  hal/         real (drivers ENSTA), unity (UDP), pysim (simulador), nmea
  estimation/  navigator, filters, attitude, kalman
  calib/       mag2d (metodo do cap_nord), mag3d (elipsoide + N/O), imu, unity_conv
  control/     pid, mixer, heading
  guidance/    ir ate ponto, seguir linha (campo de vetores / lookahead), troca de segmento
  mission/     steps (missao em JSON), safety (STOP, RTH, geofence, GPS), runner
  io/          logger (CSV + JSON de metadados), telemetry (UDP), status
  apps/        comandos do main.py
missions/      exemplos de missao
tools/         so no PC: plot_log, log2kml (KML/GPX), fr_excel, telemetry_viewer
scripts/       send-git.sh / send-git.ps1, fetch-data.sh / fetch-data.ps1
tests/         60 testes (python3 -m unittest discover -s tests)
src/DriversDDBOAT_2026/   drivers da ENSTA, intocados
data/          gerado em execucao (logs, calib, STOP/RTH), fora do git
```

## Convencoes (valem para todo o codigo)

- SI interno: m, s, rad. Referencial local ENU: x leste, y norte.
- Rumo interno `theta`: rad, anti-horario a partir do leste (convencao do modelo do Jaulin).
- Rumo de bussola (0 = N, horario) so na tela, no log e nas missoes (`compass_deg`).
- Comando de giro > 0 = virar para a esquerda (motor direito mais forte).
- Texto impresso em ASCII e codigo compativel com Python 3.5+ (vermin: minimo 3.3).

## Enviar para o barco e trazer os dados

Linux/Ubuntu: `./scripts/send-git.sh`   |   Windows: `powershell -ExecutionPolicy Bypass -File scripts\send-git.ps1`

Sem `sshpass` (sempre no Windows) o ssh pede a senha `ue32`.
Logs e calibracoes ficam em `data/` no barco (fora do git, para o push nunca ser recusado).
Para trazer: `./scripts/fetch-data.sh` ou `scripts\fetch-data.ps1` (vai para `data/boat/`).

## Comandos principais (no barco: `cd ~/DDBOAT`)

| Comando | O que faz |
|---|---|
| `python3 main.py selftest` | checa drivers, IMU, Arduino, GPS e calibracoes, sem motor |
| `python3 main.py motors 50 0 --dur 2` | antigo comPropulseurs |
| `python3 main.py calib-mag2d` | calibracao do cap_nord (agora recusa volta incompleta) |
| `python3 main.py calib-mag2d --import-file calib_cap.json` | reaproveita a calibracao antiga |
| `python3 main.py heading` | rumo ao vivo sem motor (antigo `cap_nord check`) |
| `python3 main.py teleop --backend real` | teclado w a s d v q, com log |
| `python3 main.py run missions/X.json --backend real` | executa uma missao |
| `python3 main.py imu-raw --dur 10` e `--bdu` | teste dos picos de 256 |
| `python3 main.py gps --mode raw` | GPS com checksum e status 'A' |
| `python3 main.py calib-gyro`, `check-gyro`, `calib-axes`, `calib-mag3d` | calibracoes extras |
| `python main.py teleop --backend unity` / `unity-conv` | simulador Unity (PC) |
| `python main.py run missions/pysim_demo.json --backend pysim` | simulador Python (PC) |

Para rodar desligado do Wi-Fi: `nohup python3 main.py run missions/X.json --backend real --quiet &`.
Para parar pelo SSH: `touch data/STOP`. Para voltar ao pontao: `touch data/RTH`.

## Seguranca

- `safety.geofence_latlon = null` faz `run --backend real` recusar (use `--sem-geofence` so segurando o barco).
- Saiu da geofence, tempo maximo ou `data/RTH` -> volta ao pontao; sem `home_latlon` -> para.
- GPS velho num passo que usa posicao -> motores em 0 ate o GPS voltar.
- Ctrl+C, SIGTERM e SIGHUP (queda do SSH) zeram os motores; um watchdog zera se o laco travar.
