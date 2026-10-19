# -*- coding: utf-8 -*-
"""Leitura do config JSON do barco.

Nao existe valor padrao escondido: toda chave usada pelo codigo tem que estar
no arquivo. Chave ausente -> erro claro. Valor null -> "nao definido", e quem
usa decide o que fazer (em geral, recusar a funcao que precisa dele).
"""
import json
import os

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_CONFIG = os.path.join(REPO_ROOT, "config", "boat04.json")


class ConfigError(Exception):
    pass


class Config(object):
    def __init__(self, data, path="<dict>"):
        self.data = data
        self.path = path

    @classmethod
    def load(cls, path=None):
        path = path or DEFAULT_CONFIG
        with open(path) as f:
            return cls(json.load(f), os.path.abspath(path))

    def get(self, key):
        """get("a.b.c"). Erro se alguma parte nao existir (null e permitido)."""
        node = self.data
        for part in key.split("."):
            if not isinstance(node, dict) or part not in node:
                raise ConfigError("chave '%s' ausente em %s" % (key, self.path))
            node = node[part]
        return node

    def require(self, key):
        """Como get, mas null tambem e erro."""
        v = self.get(key)
        if v is None:
            raise ConfigError("chave '%s' esta null em %s: defina antes de usar" % (key, self.path))
        return v

    def path_of(self, key):
        """Caminho de arquivo do config, relativo a raiz do repositorio."""
        p = self.require(key)
        if not os.path.isabs(p):
            p = os.path.join(REPO_ROOT, p)
        return p
