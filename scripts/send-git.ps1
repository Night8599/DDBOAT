# Envia a branch atual para o DDBOAT por git push (Windows PowerShell).
# Equivalente ao send-git.sh. O Windows nao tem sshpass: o ssh pede a senha (ue32).
# Uso (na raiz do repositorio):  powershell -ExecutionPolicy Bypass -File scripts\send-git.ps1
param([string]$Config = "config\boat04.json")

Set-Location (Split-Path -Parent $PSScriptRoot)
$cfg = Get-Content $Config -Raw | ConvertFrom-Json
$IP = $cfg.boat.ip
$USUARIO = $cfg.boat.user
$CAMINHO_REMOTO = $cfg.boat.remote_dir

$BRANCH_ATUAL = (git branch --show-current).Trim()
if (-not $BRANCH_ATUAL) {
    Write-Host "Erro: voce ainda nao fez nenhum commit. Faca um commit antes de enviar."
    exit 1
}

Write-Host "Enviando a branch '$BRANCH_ATUAL' para o DDBOAT ($USUARIO@${IP}:$CAMINHO_REMOTO)..."
$remoto = "$USUARIO@${IP}:$CAMINHO_REMOTO/"
git remote get-url ddboat 2>$null | Out-Null
if ($LASTEXITCODE -eq 0) { git remote set-url ddboat $remoto } else { git remote add ddboat $remoto }

Write-Host "(digite a senha ue32 quando o ssh pedir)"
git push ddboat $BRANCH_ATUAL
if ($LASTEXITCODE -eq 0) {
    Write-Host "OK: commits enviados para o barco."
} else {
    Write-Host "FALHA ao enviar. Veja os erros acima."
    exit 1
}
