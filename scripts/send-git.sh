#!/bin/bash

# Configurações de acesso ao DDBOAT
IP="172.20.25.204"
USUARIO="ue32"
SENHA="ue32"
CAMINHO_REMOTO="~/DDBOAT/"

# 1. Identifica o nome da branch atual automaticamente (main, master, etc)
BRANCH_ATUAL=$(git branch --show-current)

if [ -z "$BRANCH_ATUAL" ]; then
    echo "Erro: Você ainda não fez nenhum commit. Faça um commit antes de enviar."
    exit 1
fi

echo "Preparando para enviar a branch '$BRANCH_ATUAL' para o DDBOAT ($IP)..."

# 2. Configura ou atualiza o endereço do repositório remoto garantindo que use o IP
git remote set-url ddboat "$USUARIO@$IP:$CAMINHO_REMOTO" 2>/dev/null || git remote add ddboat "$USUARIO@$IP:$CAMINHO_REMOTO"

# 3. Faz o push usando o sshpass para preencher a senha
sshpass -p "$SENHA" git push ddboat "$BRANCH_ATUAL"

# Verifica se o comando anterior deu certo
if [ $? -eq 0 ]; then
    echo "✅ Commits enviados com sucesso para o barco!"
else
    echo "❌ Falha ao enviar. Verifique os erros acima."
fi