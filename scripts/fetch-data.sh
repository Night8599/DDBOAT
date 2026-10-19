#!/bin/bash
# Baixa data/logs e data/calib do barco para data/boat/ (Linux/Ubuntu).
cd "$(dirname "$0")/.." || exit 1
CONFIG="${1:-config/boat04.json}"
ler() { python3 -c "import json; print(json.load(open('$CONFIG'))['boat']['$1'])"; }
IP="$(ler ip)"; USUARIO="$(ler user)"; DIR="$(ler remote_dir)"
mkdir -p data/boat
scp -r "$USUARIO@$IP:$DIR/data/logs" "$USUARIO@$IP:$DIR/data/calib" data/boat/ && echo "OK: copiado para data/boat/"
