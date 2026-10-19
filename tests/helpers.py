import copy
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from ddboat.core.config import Config, DEFAULT_CONFIG  # noqa: E402

# os testes escrevem arquivos temporarios em data/ (num clone novo ela nao existe)
if not os.path.isdir(os.path.join(ROOT, "data")):
    os.makedirs(os.path.join(ROOT, "data"))


def cfg(**changes):
    """Copia do config real com alteracoes: cfg(**{"nav.origin_latlon": [..]})."""
    with open(DEFAULT_CONFIG) as f:
        d = json.load(f)
    for k, v in changes.items():
        node = d
        parts = k.split(".")
        for p in parts[:-1]:
            node = node[p]
        if parts[-1] not in node:
            raise KeyError(k)
        node[parts[-1]] = v
    return Config(d, DEFAULT_CONFIG)
