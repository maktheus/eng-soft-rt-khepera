#!/usr/bin/env python3
"""Ferramentas locais para subir a interface web do Khepera IV."""

from __future__ import annotations

import argparse
import hashlib
import os
import shutil
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
INTERFACE_DIR = ROOT / "interface"
REQUIREMENTS = INTERFACE_DIR / "requirements.txt"
VENV_DIR = ROOT / ".venv"
REQ_STAMP = VENV_DIR / ".interface-requirements.sha256"


def log(message: str) -> None:
    print(message, flush=True)


def venv_python() -> Path:
    if os.name == "nt":
        return VENV_DIR / "Scripts" / "python.exe"
    return VENV_DIR / "bin" / "python"


def run(
    cmd: list[str],
    cwd: Path | None = None,
    env: dict[str, str] | None = None,
    label: str = "setup",
) -> None:
    shown = " ".join(cmd)
    log(f"[{label}] {shown}")
    subprocess.run(cmd, cwd=str(cwd or ROOT), env=env, check=True)


def file_sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def ensure_venv() -> Path:
    py = venv_python()
    if not py.exists():
        log(f"[setup] criando ambiente virtual em {VENV_DIR}")
        run([sys.executable, "-m", "venv", str(VENV_DIR)])

    current_hash = file_sha256(REQUIREMENTS)
    previous_hash = REQ_STAMP.read_text(encoding="utf-8").strip() if REQ_STAMP.exists() else ""
    if current_hash != previous_hash:
        log("[setup] instalando dependencias da interface")
        run([str(py), "-m", "pip", "install", "-r", str(REQUIREMENTS)])
        REQ_STAMP.write_text(current_hash + "\n", encoding="utf-8")
    else:
        log("[setup] dependencias ja estao atualizadas")

    return py


def cmd_setup(_: argparse.Namespace) -> None:
    ensure_venv()
    log("[setup] pronto")


def cmd_run(args: argparse.Namespace) -> None:
    py = ensure_venv()
    env = os.environ.copy()
    env["HOST"] = args.host
    env["PORT"] = str(args.port)
    env.setdefault("PYTHONUNBUFFERED", "1")

    log("")
    log("[run] backend Flask + frontend estatico")
    log(f"[run] local: http://localhost:{args.port}")
    if args.host not in {"127.0.0.1", "localhost"}:
        log("[run] tambem fica acessivel na rede local pelo IP desta maquina")
    log("[run] pare com Ctrl-C")
    log("")

    try:
        run([str(py), "app.py"], cwd=INTERFACE_DIR, env=env, label="run")
    except KeyboardInterrupt:
        log("\n[run] encerrado")


def cmd_clean(_: argparse.Namespace) -> None:
    if VENV_DIR.exists():
        log(f"[clean] removendo {VENV_DIR}")
        shutil.rmtree(VENV_DIR)
    else:
        log("[clean] nada para remover")


def cmd_help(_: argparse.Namespace) -> None:
    log("Comandos disponiveis:")
    log("  make setup           cria .venv e instala dependencias")
    log("  make run             sobe backend Flask e frontend em http://localhost:8340")
    log("  make run PORT=9000   usa outra porta")
    log("  make run HOST=127.0.0.1  restringe acesso a esta maquina")
    log("  make clean           remove .venv")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Setup local da interface Khepera IV")
    sub = parser.add_subparsers(dest="command", required=True)

    setup = sub.add_parser("setup", help="cria .venv e instala dependencias")
    setup.set_defaults(func=cmd_setup)

    run_parser = sub.add_parser("run", help="sobe a interface web")
    run_parser.add_argument("--host", default="0.0.0.0")
    run_parser.add_argument("--port", default=8340, type=int)
    run_parser.set_defaults(func=cmd_run)

    clean = sub.add_parser("clean", help="remove .venv")
    clean.set_defaults(func=cmd_clean)

    help_parser = sub.add_parser("help", help="mostra comandos disponiveis")
    help_parser.set_defaults(func=cmd_help)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    args.func(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
