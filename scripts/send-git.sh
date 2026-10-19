#!/bin/bash
# Envia a branch atual para o DDBOAT por git push (Linux/Ubuntu, Git Bash).
# Le IP, usuario e pasta do config/boat04.json (ou do arquivo passado em $1).
# Com sshpass instalado a senha e preenchida sozinha; sem ele, o ssh pede a senha.

cd "$(dirname "$0")/.." || exit 1
CONFIG="${1:-config/boat04.json}"
SENHA="ue32"

ler() { python3 -c "import json,sys; print(json.load(open('$CONFIG'))['boat']['$1'])" 2>/dev/null; }
IP="$(ler ip)"; USUARIO="$(ler user)"; CAMINHO_REMOTO="$(ler remote_dir)"
if [ -z "$IP" ]; then
    echo "Nao consegui ler $CONFIG com python3; usando os valores fixos."
    IP="172.20.25.204"; USUARIO="ue32"; CAMINHO_REMOTO="~/DDBOAT"
fi

BRANCH_ATUAL=$(git branch --show-current)
if [ -z "$BRANCH_ATUAL" ]; then
    echo "Erro: voce ainda nao fez nenhum commit. Faca um commit antes de enviar."
    exit 1
fi

echo "Enviando a branch '$BRANCH_ATUAL' para o DDBOAT ($USUARIO@$IP:$CAMINHO_REMOTO)..."
git remote set-url ddboat "$USUARIO@$IP:$CAMINHO_REMOTO/" 2>/dev/null || git remote add ddboat "$USUARIO@$IP:$CAMINHO_REMOTO/"

if command -v sshpass >/dev/null 2>&1; then
    sshpass -p "$SENHA" git push ddboat "$BRANCH_ATUAL"
else
    echo "(sshpass nao encontrado: digite a senha $SENHA quando o ssh pedir)"
    git push ddboat "$BRANCH_ATUAL"
fi

if [ $? -eq 0 ]; then
    echo "OK: commits enviados para o barco."
else
    echo "FALHA ao enviar. Veja os erros acima."
    exit 1
fi
