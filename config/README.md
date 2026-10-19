# config/boat04.json

Nao existe valor padrao escondido no codigo: chave ausente = erro. `null` = ainda
nao definido, e a funcao que precisa do valor recusa rodar (ou avisa, quando o efeito e so de precisao).

Pendentes de definir no lago: `safety.home_latlon`, `safety.geofence_latlon`,
`nav.declination_deg`, `nav.origin_latlon` (null = primeiro fix vira origem), `telemetry.pc_ip`.

Opcoes desligadas ate um teste no barco: `real.imu.bdu`, `real.gps.mode = "raw"`,
`nav.gyro_fusion` (validar com `check-gyro` antes), `nav.heading_source = "mag3d"` + `tilt_compensation`.

Ganhos (`control`, `guidance`) sao iniciais, ajustados so no simulador Python.
Os parametros do `pysim` (inclusive `u_per_cmd` e a calibracao `mag2d` ficticia) nao foram medidos no barco.
