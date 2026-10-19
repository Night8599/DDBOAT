# -*- coding: utf-8 -*-
"""Converte um log (',' e ponto decimal) para Excel em frances (';' e virgula decimal).
Uso: python tools/fr_excel.py data/logs/ARQ.csv   -> ARQ_fr.csv"""
import os
import sys


def main():
    if len(sys.argv) != 2:
        print("uso: python tools/fr_excel.py LOG.csv")
        return 1
    src = sys.argv[1]
    dst = os.path.splitext(src)[0] + "_fr.csv"
    with open(src) as f, open(dst, "w") as g:
        for line in f:
            g.write(";".join(v.replace(".", ",") for v in line.rstrip("\n").split(",")) + "\n")
    print("salvo em", dst)
    return 0


if __name__ == "__main__":
    sys.exit(main())
