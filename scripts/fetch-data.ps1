# Baixa data/logs e data/calib do barco para data\boat\ (Windows PowerShell, scp do OpenSSH).
param([string]$Config = "config\boat04.json")
Set-Location (Split-Path -Parent $PSScriptRoot)
$cfg = Get-Content $Config -Raw | ConvertFrom-Json
$src = "$($cfg.boat.user)@$($cfg.boat.ip):$($cfg.boat.remote_dir)/data"
New-Item -ItemType Directory -Force -Path "data\boat" | Out-Null
scp -r "$src/logs" "$src/calib" "data\boat\"
if ($LASTEXITCODE -eq 0) { Write-Host "OK: copiado para data\boat\" } else { Write-Host "FALHA no scp" }
