"""Interface web para o Khepera IV (console USB ou Wi-Fi/SSH).

Servidor local que faz a ponte entre o navegador e o robo:
  - conexao/login automatico no console serial USB (root, sem senha)
  - conexao Wi-Fi por SSH interativo
  - execucao do controlador `patrulha` com parametros de missao
  - telemetria ao vivo (pose, sensores IR, bateria, trajetoria)
  - mapeamento passivo em grade + grafo com persistencia em JSON/NDJSON
  - deploy do main.c local + recompilacao no robo
  - edicao de parametros de compilacao (#define) via sed + rebuild
  - configuracao do Wi-Fi do robo (wpa_supplicant + udhcpc) usando a rede
    atual da maquina que roda esta interface

Uso:  python app.py   ->   http://localhost:8340
"""
import base64
import concurrent.futures
import csv
import glob
import hashlib
import ipaddress
import json
import math
import os
import platform
import re
import shlex
import socket
import subprocess
import threading
import time
import warnings

import serial
import serial.tools.list_ports
from flask import Flask, jsonify, request, send_from_directory

try:
    warnings.filterwarnings("ignore", message="Blowfish has been deprecated.*")
    import paramiko
except ImportError:  # dependencia opcional para manter USB funcionando
    paramiko = None

import planner  # A* sobre o mapa de ocupacao gerado

BASE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(BASE, ".."))
LOCAL_MAIN = os.path.normpath(os.path.join(BASE, "..", "khepera_real", "patrulha", "main.c"))
LOCAL_CONTROLLER_CONFIG = os.path.normpath(os.path.join(BASE, "..", "khepera_real", "patrulha", "controller_config.h"))
LOCAL_CONTROLLER_CORE = os.path.normpath(os.path.join(BASE, "..", "khepera_real", "patrulha", "controller_core.h"))
MAPS_DIR = os.path.join(BASE, "maps")
SIM_DIR = os.path.join(ROOT, "tools", "sim_khepera")
SIM_GENERATOR = os.path.join(SIM_DIR, "generate_worlds.py")
SIM_BUILD = os.path.join(SIM_DIR, "build.ps1")
SIM_WORLD_DEFAULT = os.path.join(SIM_DIR, "worlds_1000.json")
SIM_OUT_DIR = os.path.join(SIM_DIR, "out_gl")
SIM_SELECTED = {"worlds": SIM_WORLD_DEFAULT, "out_dir": SIM_OUT_DIR}
REMOTE_DIR = "/home/root/khepera4toolbox/app/patrulha"
BUILD_CMD = (
    f"cd {REMOTE_DIR} && gcc -Wall "
    "-I/home/root/khepera4toolbox/Modules/khepera4 "
    "-I/home/root/khepera4toolbox/Modules/commandline "
    "-I/home/root/khepera4toolbox/Modules/i2cal "
    "main.c i2cal.c /home/root/khepera4toolbox/Modules/khepera4/*.c "
    "/home/root/khepera4toolbox/Modules/commandline/commandline.c "
    "-lm -o patrulha && echo BUILD_OK || echo BUILD_FAIL"
)

# parametros de compilacao expostos na UI (whitelist p/ o sed)
DEFINES = [
    "SPEED", "KP_HEAD", "HEAD_SLOW", "IR_BASE", "FRONT_SPAN", "FRONT_BLOCK",
    "FRONT_CRIT", "FRONT_CLEAR", "WALL_TARGET", "KP_WALL", "ARRIVE_MM",
    "CLIFF_THR", "LINE_EPS", "PROGRESS_MM", "WALL_LOST", "WALL_MIN_MM",
    "WHEELBASE_MM", "BATT_LOW_MV",
]
MAIN_DEFINES = {"SPEED", "WHEELBASE_MM", "BATT_LOW_MV"}

TELEM_RE = re.compile(
    r"\[(GOAL|WALLF)(?:-([ED]))?\s*\]\s*pose=\((-?\d+),(-?\d+),(-?\d+)d\)\s*"
    r"d_goal=(-?\d+)\s*d_line=([+-]?\d+)\s*herr=([+-]?\d+)d\s*\|\s*"
    r"F/FL/FR=(\d+)/(\d+)/(\d+)\s*\|\s*chao=(-?\d+)\s*bat=(\d+)mV")
BEIRADA_RE = re.compile(r"\[BEIRADA\]\s*chao=(-?\d+)")
DIAG_RE = re.compile(
    r"IR\[BL L FL F FR R BR B\]=(\d+) (\d+) (\d+) (\d+) (\d+) (\d+) (\d+) (\d+)"
    r" \| chao\[(\d+) (\d+) (\d+) (\d+)\] \| enc L/R=(-?\d+)/(-?\d+) \| bat=(\d+)mV")
MANUAL_RE = re.compile(
    r"\[MANUAL\]\s*(?:pose=\((-?\d+),(-?\d+),(-?\d+)d\)\s*)?"
    r"fwd=([+-]?\d+(?:\.\d+)?)\s*diff=([+-]?\d+(?:\.\d+)?)\s*"
    r"guard=(\w+)\s*\|\s*F/FL/FR=(\d+)/(\d+)/(\d+)\s*\|\s*chao=(-?\d+)\s*bat=(\d+)mV")
# Reconhece o prompt de qualquer usuario (root termina em '#', demais em '$').
PROMPT_RE = re.compile(r"[\w.-]+@[\w.-]+:[^\n#$]*[#$] ?")
IPV4_RE = re.compile(r"(?<!\d)(?:\d{1,3}\.){3}\d{1,3}(?!\d)")
COMMON_ROBOT_HOSTS = [
    "khepera",
    "khepera.local",
    "khepera4",
    "khepera4.local",
    "khepera-iv",
    "khepera-iv.local",
]


def _repo_rel(path):
    try:
        return os.path.relpath(path, ROOT).replace("\\", "/")
    except ValueError:
        return path


def _repo_path(value, default_path):
    raw = (value or "").strip()
    path = raw if raw else default_path
    if not os.path.isabs(path):
        path = os.path.join(ROOT, path)
    path = os.path.abspath(os.path.normpath(path))
    root = os.path.abspath(ROOT)
    if path != root and not path.startswith(root + os.sep):
        raise ValueError("caminho precisa ficar dentro do repositorio")
    return path


def _sim_world_count(worlds_path):
    try:
        with open(worlds_path, encoding="utf-8") as f:
            data = json.load(f)
        meta_count = int(data.get("metadata", {}).get("count") or 0)
        scenarios = data.get("scenarios") or []
        return meta_count or len(scenarios)
    except Exception:
        return 0


def sim_snapshot(worlds_path=None, out_dir=None):
    worlds_path = worlds_path or SIM_WORLD_DEFAULT
    out_dir = out_dir or SIM_OUT_DIR
    summary_path = os.path.join(out_dir, "summary.txt")
    out = {
        "worlds": _repo_rel(worlds_path),
        "world_exists": os.path.exists(worlds_path),
        "world_count": _sim_world_count(worlds_path) if os.path.exists(worlds_path) else 0,
        "out_dir": _repo_rel(out_dir),
        "summary_exists": os.path.exists(summary_path),
        "total": 0,
        "arrived": 0,
        "failed": 0,
        "collisions": 0,
        "timeouts": 0,
        "failures": [],
        "summary": _repo_rel(summary_path),
    }
    if not os.path.exists(summary_path):
        return out
    try:
        with open(summary_path, newline="", encoding="utf-8", errors="replace") as f:
            for row in csv.DictReader(f):
                status = (row.get("status") or "").strip()
                name = (row.get("scenario") or "").strip()
                out["total"] += 1
                if status == "ARRIVED":
                    out["arrived"] += 1
                else:
                    out["failed"] += 1
                    if status == "COLLISION":
                        out["collisions"] += 1
                    elif status == "TIMEOUT":
                        out["timeouts"] += 1
                    if len(out["failures"]) < 12:
                        out["failures"].append({
                            "scenario": name,
                            "status": status,
                            "time_s": row.get("time_s", ""),
                            "final_goal_mm": row.get("final_goal_mm", ""),
                            "wall_entries": row.get("wall_entries", ""),
                        })
        out["mtime"] = int(os.path.getmtime(summary_path))
    except Exception as e:
        out["summary_error"] = str(e)
    return out


def local_defines():
    """Valores default dos #define lidos do main.c do repositorio."""
    vals = {}
    for path in (LOCAL_MAIN, LOCAL_CONTROLLER_CONFIG):
        try:
            with open(path, encoding="utf-8", errors="replace") as f:
                for line in f:
                    m = re.match(r"#define\s+(\w+)\s+([0-9.eE+-]+)", line)
                    if m and m.group(1) in DEFINES:
                        vals[m.group(1)] = m.group(2)
        except OSError:
            pass
    return vals


class MappingService:
    def __init__(self):
        self.lock = threading.Lock()
        self.active = False
        self.session_id = None
        self.session_dir = None
        self.resolution = 50.0
        self.node_spacing = 150.0
        self.telemetry_count = 0
        self.cells = {}
        self.nodes = []
        self.edges = []
        self.last_node_id = None
        self.last_saved = 0.0

    def start(self, name="", resolution=50.0):
        safe = re.sub(r"[^A-Za-z0-9_.-]+", "-", name.strip()).strip("-")[:32]
        stamp = time.strftime("%Y%m%d_%H%M%S")
        session_id = f"{stamp}_{safe}" if safe else stamp
        session_dir = os.path.join(MAPS_DIR, session_id)
        os.makedirs(session_dir, exist_ok=True)
        with self.lock:
            self.active = True
            self.session_id = session_id
            self.session_dir = session_dir
            self.resolution = max(20.0, min(200.0, float(resolution)))
            self.node_spacing = 3.0 * self.resolution
            self.telemetry_count = 0
            self.cells = {}
            self.nodes = []
            self.edges = []
            self.last_node_id = None
            self.last_saved = 0.0
        self.save()
        return self.snapshot()

    def stop(self):
        with self.lock:
            self.active = False
        self.save()

    def clear(self):
        with self.lock:
            self.active = False
            self.session_id = None
            self.session_dir = None
            self.telemetry_count = 0
            self.cells = {}
            self.nodes = []
            self.edges = []
            self.last_node_id = None
            self.last_saved = 0.0

    def ingest(self, telem):
        if "x" not in telem or "y" not in telem or "th" not in telem:
            return
        with self.lock:
            if not self.active:
                return
            point = dict(telem)
            point["t"] = time.time()
            self.telemetry_count += 1
            self._mark_pose(point)
            self._mark_sensors(point)
            self._update_graph(point)
            session_dir = self.session_dir
            should_save = time.time() - self.last_saved > 1.0
        self._append_telemetry(session_dir, point)
        if should_save:
            self.save()

    def _cell_key(self, x, y):
        return (int(math.floor(float(x) / self.resolution)),
                int(math.floor(float(y) / self.resolution)))

    def _cell_center(self, i, j):
        return ((i + 0.5) * self.resolution, (j + 0.5) * self.resolution)

    def _cell(self, key):
        return self.cells.setdefault(key, {"free": 0.0, "occupied": 0.0, "visited": 0.0})

    def _add_score(self, x, y, field, amount):
        cell = self._cell(self._cell_key(x, y))
        cell[field] = max(0.0, cell.get(field, 0.0) + amount)

    def _mark_pose(self, telem):
        self._add_score(telem["x"], telem["y"], "visited", 1.0)
        self._add_score(telem["x"], telem["y"], "free", 1.0)

    def _mark_sensors(self, telem):
        sensor_angles = {"F": 0.0, "FL": 35.0, "FR": -35.0}
        for name, rel_deg in sensor_angles.items():
            raw = telem.get(name)
            if raw is None:
                continue
            excess = max(0.0, float(raw) - 120.0)
            blocked = excess >= 120.0
            if blocked:
                hit_dist = max(80.0, min(240.0, 260.0 - 0.45 * excess))
                self._mark_ray(telem, rel_deg, hit_dist - self.resolution, "free", 0.35)
                self._mark_ray_endpoint(telem, rel_deg, hit_dist, "occupied", 1.2)
            else:
                self._mark_ray(telem, rel_deg, 230.0, "free", 0.25)

    def _mark_ray(self, telem, rel_deg, length, field, amount):
        if length <= 0:
            return
        steps = max(1, int(length / max(20.0, self.resolution * 0.5)))
        theta = math.radians(float(telem["th"]) + rel_deg)
        for n in range(1, steps + 1):
            dist = length * n / steps
            x = float(telem["x"]) + math.cos(theta) * dist
            y = float(telem["y"]) + math.sin(theta) * dist
            self._add_score(x, y, field, amount)

    def _mark_ray_endpoint(self, telem, rel_deg, dist, field, amount):
        theta = math.radians(float(telem["th"]) + rel_deg)
        x = float(telem["x"]) + math.cos(theta) * dist
        y = float(telem["y"]) + math.sin(theta) * dist
        self._add_score(x, y, field, amount)

    def _update_graph(self, telem):
        x, y = float(telem["x"]), float(telem["y"])
        if not self.nodes:
            self._add_node(x, y, "start", telem.get("state", ""))
            return
        last = self.nodes[-1]
        dist = math.hypot(x - last["x"], y - last["y"])
        state_changed = telem.get("state") and telem.get("state") != last.get("state")
        if dist >= self.node_spacing or (state_changed and dist >= self.resolution):
            new_id = self._add_node(x, y, "waypoint", telem.get("state", ""))
            self._add_edge(last["id"], new_id, dist, telem)

    def _add_node(self, x, y, kind, state):
        node_id = f"N{len(self.nodes)}"
        self.nodes.append({
            "id": node_id, "x": round(x, 1), "y": round(y, 1),
            "kind": kind, "state": state or "",
        })
        self.last_node_id = node_id
        return node_id

    def _add_edge(self, source, target, dist, telem):
        clearance = self._front_clearance(telem)
        self.edges.append({
            "from": source, "to": target,
            "cost": round(dist, 1), "distance": round(dist, 1),
            "clearance": clearance,
        })

    def _front_clearance(self, telem):
        vals = [telem.get(k) for k in ("F", "FL", "FR") if telem.get(k) is not None]
        if not vals:
            return None
        pressure = max(vals) - 120
        return round(max(30.0, min(250.0, 260.0 - 0.45 * pressure)), 1)

    def _state_for(self, cell):
        if cell.get("occupied", 0.0) > cell.get("free", 0.0) + 0.8:
            return "occupied"
        if cell.get("visited", 0.0) > 0.0:
            return "visited"
        if cell.get("free", 0.0) > 0.2:
            return "free"
        return "unknown"

    def _snapshot_unlocked(self):
        cells = []
        for (i, j), cell in self.cells.items():
            state = self._state_for(cell)
            if state == "unknown":
                continue
            cx, cy = self._cell_center(i, j)
            cells.append({
                "i": i, "j": j, "x": round(cx, 1), "y": round(cy, 1),
                "state": state,
                "free": round(cell.get("free", 0.0), 2),
                "occupied": round(cell.get("occupied", 0.0), 2),
                "visited": round(cell.get("visited", 0.0), 2),
            })
        cells.sort(key=lambda c: (c["i"], c["j"]))
        return {
            "active": self.active,
            "session_id": self.session_id,
            "resolution_mm": self.resolution,
            "telemetry_count": self.telemetry_count,
            "cells": cells,
            "graph": {"nodes": list(self.nodes), "edges": list(self.edges)},
        }

    def snapshot(self):
        with self.lock:
            return self._snapshot_unlocked()

    def save(self):
        with self.lock:
            if not self.session_dir:
                return
            snapshot = self._snapshot_unlocked()
            session_dir = self.session_dir
            self.last_saved = time.time()
        os.makedirs(session_dir, exist_ok=True)
        with open(os.path.join(session_dir, "grid.json"), "w", encoding="utf-8") as f:
            json.dump({
                "session_id": snapshot["session_id"],
                "resolution_mm": snapshot["resolution_mm"],
                "cells": snapshot["cells"],
            }, f, indent=2)
        with open(os.path.join(session_dir, "graph.json"), "w", encoding="utf-8") as f:
            json.dump(snapshot["graph"], f, indent=2)

    def _append_telemetry(self, session_dir, point):
        if not session_dir:
            return
        os.makedirs(session_dir, exist_ok=True)
        with open(os.path.join(session_dir, "telemetry.ndjson"), "a", encoding="utf-8") as f:
            f.write(json.dumps(point, sort_keys=True) + "\n")

    def sessions(self):
        if not os.path.isdir(MAPS_DIR):
            return []
        out = []
        for name in sorted(os.listdir(MAPS_DIR), reverse=True):
            path = os.path.join(MAPS_DIR, name)
            if not os.path.isdir(path):
                continue
            grid_path = os.path.join(path, "grid.json")
            graph_path = os.path.join(path, "graph.json")
            out.append({
                "session_id": name,
                "has_grid": os.path.exists(grid_path),
                "has_graph": os.path.exists(graph_path),
            })
        return out


mapper = MappingService()


def _valid_ipv4(value):
    try:
        ip = ipaddress.ip_address(value)
    except ValueError:
        return None
    if ip.version != 4 or ip.is_loopback or ip.is_link_local or ip.is_multicast or ip.is_reserved:
        return None
    return str(ip)


def _run_text(cmd, timeout=2.5):
    try:
        p = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=timeout,
        )
    except Exception:
        return ""
    return (p.stdout or "") + "\n" + (p.stderr or "")


def _local_ipv4s():
    ips = set()
    try:
        hostname = socket.gethostname()
        for item in socket.getaddrinfo(hostname, None, socket.AF_INET):
            ip = _valid_ipv4(item[4][0])
            if ip:
                ips.add(ip)
    except Exception:
        pass
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("8.8.8.8", 80))
            ip = _valid_ipv4(s.getsockname()[0])
            if ip:
                ips.add(ip)
        finally:
            s.close()
    except Exception:
        pass
    ipconfig = _run_text(["ipconfig"], timeout=2.5)
    for line in ipconfig.splitlines():
        if "IPv4" not in line:
            continue
        for match in IPV4_RE.findall(line):
            ip = _valid_ipv4(match)
            if ip:
                ips.add(ip)
    return sorted(ips)


def _arp_ipv4s():
    ips = set()
    table = _run_text(["arp", "-a"], timeout=2.5)
    for match in IPV4_RE.findall(table):
        ip = _valid_ipv4(match)
        if ip:
            ips.add(ip)
    return sorted(ips)


IFACE_RE = re.compile(r"^[a-zA-Z0-9_-]+$")


def _detect_local_wifi():
    """Melhor esforco para achar o SSID da rede Wi-Fi da maquina que roda a interface."""
    system = platform.system()
    try:
        if system == "Windows":
            out = _run_text(["netsh", "wlan", "show", "interfaces"], timeout=3.0)
            for line in out.splitlines():
                line = line.strip()
                if line.lower().startswith("ssid") and "bssid" not in line.lower():
                    _, _, value = line.partition(":")
                    value = value.strip()
                    if value:
                        return value
            return None
        if system == "Darwin":
            airport = (
                "/System/Library/PrivateFrameworks/Apple80211.framework"
                "/Versions/Current/Resources/airport"
            )
            out = _run_text([airport, "-I"], timeout=3.0)
            for line in out.splitlines():
                line = line.strip()
                if line.startswith("SSID:"):
                    value = line.split(":", 1)[1].strip()
                    if value:
                        return value
            return None
        out = _run_text(["nmcli", "-t", "-f", "active,ssid", "dev", "wifi"], timeout=3.0)
        for line in out.splitlines():
            if line.startswith("yes:"):
                value = line.split(":", 1)[1].strip()
                if value:
                    return value
        out = _run_text(["iwgetid", "-r"], timeout=2.0).strip()
        return out or None
    except Exception:
        return None


def _resolve_common_hosts(timeout=1.6):
    found = {}

    def resolve(host):
        out = []
        try:
            for item in socket.getaddrinfo(host, 22, socket.AF_INET, socket.SOCK_STREAM):
                ip = _valid_ipv4(item[4][0])
                if ip:
                    out.append(ip)
        except Exception:
            pass
        return host, out

    with concurrent.futures.ThreadPoolExecutor(max_workers=min(8, len(COMMON_ROBOT_HOSTS))) as pool:
        futures = [pool.submit(resolve, host) for host in COMMON_ROBOT_HOSTS]
        deadline = time.time() + timeout
        try:
            for fut in concurrent.futures.as_completed(futures, timeout=timeout):
                if time.time() > deadline:
                    break
                host, ips = fut.result()
                for ip in ips:
                    found.setdefault(ip, set()).add(f"nome:{host}")
        except concurrent.futures.TimeoutError:
            pass
    return found


def _subnet_ipv4s(local_ips, max_hosts=254):
    ips = []
    seen = set(local_ips)
    for local in local_ips:
        try:
            network = ipaddress.ip_network(f"{local}/24", strict=False)
        except ValueError:
            continue
        for ip in network.hosts():
            value = str(ip)
            if value in seen:
                continue
            seen.add(value)
            ips.append(value)
            if len(ips) >= max_hosts:
                return ips
    return ips


def _probe_ssh(host, port=22, timeout=0.35):
    started = time.time()
    try:
        with socket.create_connection((host, int(port)), timeout=timeout) as sock:
            sock.settimeout(timeout)
            banner = ""
            try:
                banner = sock.recv(128).decode("utf-8", "replace").strip()
            except Exception:
                pass
    except Exception as e:
        return None
    elapsed_ms = int((time.time() - started) * 1000)
    return {
        "host": host,
        "port": int(port),
        "label": f"{host}:{int(port)}",
        "banner": banner,
        "latency_ms": elapsed_ms,
    }


def discover_wifi_devices(timeout=4.0, include_subnet=True):
    timeout = max(0.5, min(8.0, float(timeout)))
    local_ips = _local_ipv4s()
    sources = _resolve_common_hosts(timeout=min(1.5, timeout / 2.0))
    for ip in _arp_ipv4s():
        sources.setdefault(ip, set()).add("arp")
    if include_subnet:
        for ip in _subnet_ipv4s(local_ips):
            sources.setdefault(ip, set()).add("rede-local")

    for ip in list(sources):
        if ip in local_ips:
            sources.pop(ip, None)

    hosts = sorted(sources, key=lambda ip: (0 if any(s.startswith("nome:") for s in sources[ip]) else 1, ip))
    devices = []
    deadline = time.time() + timeout
    max_workers = min(64, max(1, len(hosts)))
    if not hosts:
        return {"local_ips": local_ips, "devices": devices}

    with concurrent.futures.ThreadPoolExecutor(max_workers=max_workers) as pool:
        futures = {pool.submit(_probe_ssh, host, 22): host for host in hosts}
        try:
            for fut in concurrent.futures.as_completed(futures, timeout=timeout):
                if time.time() > deadline:
                    break
                item = fut.result()
                if not item:
                    continue
                source = sorted(sources.get(item["host"], []))
                score = 40
                if any(s.startswith("nome:") for s in source):
                    score += 35
                if "arp" in source:
                    score += 15
                banner = item.get("banner", "").lower()
                if "dropbear" in banner or "openssh" in banner or "ssh-" in banner:
                    score += 10
                item.update({
                    "source": source,
                    "score": score,
                    "username": "root",
                    "detail": "SSH aberto; candidato para Khepera IV",
                })
                devices.append(item)
        except concurrent.futures.TimeoutError:
            pass
    devices.sort(key=lambda d: (-d["score"], d["host"]))
    return {"local_ips": local_ips, "devices": devices[:20]}


class SerialConsoleTransport:
    mode = "usb"
    username = "root"   # console serial faz auto-login como root

    def __init__(self, port):
        self.port = port
        self.ser = None

    @property
    def label(self):
        return self.port

    @property
    def is_open(self):
        return bool(self.ser and self.ser.is_open)

    def open(self):
        self.ser = serial.Serial(self.port, 115200, timeout=0.05)
        self.ser.dtr = True
        self.ser.rts = True

    def write(self, text):
        self.ser.write(text.encode())

    def read(self, size):
        return self.ser.read(size)

    def close(self):
        if self.ser:
            self.ser.close()


class SSHConsoleTransport:
    mode = "wifi"

    def __init__(self, host, username="root", password="", port=22):
        self.host = host
        self.username = username or "root"
        self.password = password
        self.port = int(port or 22)
        self.client = None
        self.channel = None

    @property
    def label(self):
        return f"{self.username}@{self.host}:{self.port}"

    @property
    def is_open(self):
        return bool(self.channel and not self.channel.closed)

    def open(self):
        if paramiko is None:
            raise RuntimeError("dependencia paramiko nao instalada; rode: python -m pip install -r interface/requirements.txt")
        self.client = paramiko.SSHClient()
        self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.client.connect(
            hostname=self.host,
            port=self.port,
            username=self.username,
            password=self.password,
            timeout=8,
            banner_timeout=8,
            auth_timeout=8,
            look_for_keys=True,
            allow_agent=True,
        )
        self.channel = self.client.invoke_shell(term="vt100", width=120, height=32)
        self.channel.settimeout(0.05)
        time.sleep(0.25)

    def write(self, text):
        self.channel.send(text)

    def read(self, size):
        try:
            if not self.channel.recv_ready():
                time.sleep(0.03)
                return b""
            return self.channel.recv(size)
        except socket.timeout:
            return b""

    def close(self):
        if self.channel:
            self.channel.close()
        if self.client:
            self.client.close()


class RobotLink:
    def __init__(self):
        self.transport = None
        self.mode = None
        self.port = None
        self.wlock = threading.Lock()   # escrita no console remoto
        self.clock = threading.Lock()   # console/telemetria
        self.console = []
        self.console_base = 0
        self.scan = ""                  # janela de busca p/ wait_for
        self.telem = {}
        self.trail = []
        self.goal = {"x": 700.0, "y": 0.0}
        self.running = False
        self.manual = False
        self.teleop_ready = False
        self.run_started = 0.0
        self.arrived = False
        self.job = None                 # {'name','status','detail'}
        self.overrides = {}             # #define aplicados nesta sessao
        self.echo_off = False
        self._alive = False
        self.wifi_ip = None             # IP obtido pelo ultimo job_wifi_setup
        self.need_sudo = False          # usuario nao-root -> elevar p/ root
        self.escalated = True           # ja "root" por padrao (USB/root SSH)
        self.sudo_password = ""         # senha do login, reusada no sudo -S
        self.awaiting_pw = False        # esperando o prompt do sudo p/ enviar a senha

    # ---------------- conexao ----------------
    def _connect_transport(self, transport):
        self.disconnect()
        transport.open()
        self.transport = transport
        self.mode = transport.mode
        self.port = transport.label
        self._alive = True
        self.echo_off = False
        # Multi-usuario: se o login nao for root, eleva com sudo -s ao ver o
        # primeiro prompt, para que hardware (I2C), deploy e compilacao (em
        # /home/root) rodem como root, igual ao console serial.
        user = getattr(transport, "username", "root")
        self.need_sudo = (user != "root")
        self.escalated = not self.need_sudo
        self.sudo_password = getattr(transport, "password", "") or ""
        threading.Thread(target=self._read_loop, daemon=True).start()
        if transport.mode == "wifi":
            self._log(f"[interface] conectado via Wi-Fi/SSH em {transport.label}")
        else:
            self._log(f"[interface] conectado via USB em {transport.label}")
        self._write("\r")               # provoca prompt ou login

    def connect_usb(self, port):
        if not port:
            raise ValueError("porta serial nao informada")
        self._connect_transport(SerialConsoleTransport(port))

    def connect_wifi(self, host, username="root", password="", port=22):
        host = (host or "").strip()
        if not host:
            raise ValueError("host/IP do robo nao informado")
        self._connect_transport(SSHConsoleTransport(host, username, password, port))

    def disconnect(self):
        self._alive = False
        if self.transport:
            try:
                self._write("stty echo\r")
                time.sleep(0.2)
                self.transport.close()
            except Exception:
                pass
        self.transport = None
        self.mode = None
        self.port = None
        self.running = False
        self.manual = False

    @property
    def connected(self):
        return self.transport is not None and self._alive and self.transport.is_open

    # ---------------- console remoto ----------------
    def _write(self, s):
        with self.wlock:
            self.transport.write(s)

    def cmd(self, s, echo=True):
        if echo:
            self._log("$ " + s)
        try:
            self._write(s + "\r")
        except Exception as e:
            self._log(f"[interface] erro na conexao: {e}")
            self.disconnect()

    def stop(self):
        self._log("$ [Ctrl-C]")
        try:
            self._write("\x03")
        except Exception:
            pass
        self.running = False
        self.manual = False

    # ---------------- leitura ----------------
    def _read_loop(self):
        part = ""
        while self._alive:
            try:
                data = self.transport.read(4096)
            except Exception:
                if self._alive:
                    self._log("[interface] conexao caiu")
                    self._alive = False
                break
            if not data:
                if self.transport and not self.transport.is_open:
                    self._log("[interface] conexao encerrada pelo robo")
                    self._alive = False
                    break
                continue
            part += data.decode("utf-8", "replace").replace("\r", "")
            if self.mode == "usb" and (part.endswith("login: ") or part.endswith("login:")):
                self._write("root\r")
                self._log("[interface] login automatico: root")
                part = ""
                continue
            if self.mode == "usb" and (part.endswith("Password: ") or part.endswith("Password:")):
                self._write("\r")
                part = ""
                continue
            if self.awaiting_pw and "password" in part.lower() and ":" in part:
                # sudo esta pedindo a senha ("Password:" ou "[sudo] password for..:")
                self.awaiting_pw = False
                self._write(self.sudo_password + "\r")
                part = ""
                continue
            while "\n" in part:
                line, part = part.split("\n", 1)
                self._handle_line(PROMPT_RE.sub("", line))
            if PROMPT_RE.search(part):
                part = PROMPT_RE.sub("", part)
                if self.need_sudo and not self.escalated:
                    # primeiro prompt como usuario comum -> virar root.
                    # sudo -S le a senha do stdin; enviamos a senha do login
                    # so quando o marcador do prompt do sudo aparecer (timing).
                    self.escalated = True
                    self.echo_off = False   # refaz stty -echo no shell root
                    self.awaiting_pw = bool(self.sudo_password)
                    self._log("[interface] elevando privilegios: sudo -s")
                    self._write("stty -echo\r")     # nao ecoar a senha
                    self._write("sudo -S -s\r")
                    continue
                if not self.echo_off:
                    self.echo_off = True
                    self._write("stty -echo\r")
                if self.running and time.time() - self.run_started > 1.0:
                    self.running = False    # prompt de volta = programa acabou
                    self.manual = False

    def _log(self, line):
        with self.clock:
            self.console.append(line)
            if len(self.console) > 3000:
                cut = 1000
                self.console = self.console[cut:]
                self.console_base += cut

    def _run_local_process(self, cmd, detail_prefix="sim"):
        pretty = subprocess.list2cmdline([str(x) for x in cmd])
        self._log(f"[{detail_prefix}] $ {pretty}")
        flags = getattr(subprocess, "CREATE_NO_WINDOW", 0) if platform.system() == "Windows" else 0
        proc = subprocess.Popen(
            [str(x) for x in cmd],
            cwd=ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            creationflags=flags,
        )
        assert proc.stdout is not None
        last_line = ""
        for line in proc.stdout:
            line = line.rstrip()
            if not line:
                continue
            last_line = line
            self._log(f"[{detail_prefix}] {line}")
            if self.job and self.job.get("status") == "running":
                self.job["detail"] = line[-160:]
        code = proc.wait()
        if code != 0:
            raise RuntimeError(f"processo local falhou ({code}): {last_line or pretty}")

    def job_sim_generate(self, count, out_path):
        self.job["detail"] = f"gerando {count} mundos..."
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        self._run_local_process(
            ["python", SIM_GENERATOR, "--count", str(count), "--out", out_path],
            detail_prefix="sim-gen",
        )
        self.job.update(status="ok", detail=f"JSON gerado: {_repo_rel(out_path)} ({count} mundos)")

    def job_sim_batch(self, worlds_path, out_dir):
        if not os.path.exists(worlds_path):
            raise RuntimeError(f"JSON nao encontrado: {_repo_rel(worlds_path)}")
        os.makedirs(out_dir, exist_ok=True)
        self.job["detail"] = f"validando {_repo_rel(worlds_path)}..."
        self._run_local_process(
            [
                "powershell",
                "-ExecutionPolicy", "Bypass",
                "-File", SIM_BUILD,
                "-OutDir", out_dir,
                "-Worlds", worlds_path,
            ],
            detail_prefix="sim-batch",
        )
        snap = sim_snapshot(worlds_path, out_dir)
        if snap.get("failed"):
            self.job.update(
                status="error",
                detail=f"{snap['failed']} falha(s): {snap['arrived']}/{snap['total']} chegaram",
            )
        else:
            self.job.update(status="ok", detail=f"{snap['arrived']}/{snap['total']} chegaram")

    def _handle_line(self, line):
        if not line.strip():
            return
        self._log(line)
        with self.clock:
            self.scan += line + "\n"
            if len(self.scan) > 65536:
                self.scan = self.scan[-32768:]
        self._parse(line)

    def _parse(self, line):
        m = TELEM_RE.search(line)
        if m:
            st = m.group(1) + ("-" + m.group(2) if m.group(2) else "")
            t = {
                "state": st,
                "x": int(m.group(3)), "y": int(m.group(4)), "th": int(m.group(5)),
                "d_goal": int(m.group(6)), "d_line": int(m.group(7)),
                "herr": int(m.group(8)),
                "F": int(m.group(9)), "FL": int(m.group(10)), "FR": int(m.group(11)),
                "chao": int(m.group(12)), "bat": int(m.group(13)),
            }
            with self.clock:
                self.telem.update(t)
                p = self.trail[-1] if self.trail else None
                if p is None or abs(p[0] - t["x"]) + abs(p[1] - t["y"]) > 5:
                    self.trail.append([t["x"], t["y"]])
                    if len(self.trail) > 4000:
                        self.trail = self.trail[::2]
            mapper.ingest(t)
            return
        m = BEIRADA_RE.search(line)
        if m:
            with self.clock:
                self.telem.update({"state": "BEIRADA", "chao": int(m.group(1))})
            return
        m = DIAG_RE.search(line)
        if m:
            g = [int(x) for x in m.groups()]
            with self.clock:
                self.telem.update({
                    "state": "DIAG", "ir": g[0:8], "ground": g[8:12],
                    "encL": g[12], "encR": g[13], "bat": g[14],
                })
            return
        m = MANUAL_RE.search(line)
        if m:
            t = {
                "state": "MANUAL",
                "fwd": float(m.group(4)), "diff": float(m.group(5)),
                "guard": m.group(6),
                "F": int(m.group(7)), "FL": int(m.group(8)), "FR": int(m.group(9)),
                "chao": int(m.group(10)), "bat": int(m.group(11)),
            }
            if m.group(1) is not None:
                t.update({"x": int(m.group(1)), "y": int(m.group(2)), "th": int(m.group(3))})
            with self.clock:
                self.telem.update(t)
                if "x" in t:
                    p = self.trail[-1] if self.trail else None
                    if p is None or abs(p[0] - t["x"]) + abs(p[1] - t["y"]) > 5:
                        self.trail.append([t["x"], t["y"]])
                        if len(self.trail) > 4000:
                            self.trail = self.trail[::2]
            mapper.ingest(t)
            return
        if line.startswith("TELEOP:"):
            with self.clock:
                self.telem["state"] = "MANUAL"
            return
        if "TELEOP encerrado" in line:
            self.running = False
            self.manual = False
            with self.clock:
                self.telem["state"] = "PARADO"
            return
        if "== CHEGOU" in line:
            with self.clock:
                self.arrived = True
                self.telem["state"] = "CHEGOU"
        elif "Motores parados" in line:
            self.running = False
            self.manual = False
            with self.clock:
                self.telem["state"] = "PARADO"

    # ---------------- espera por padrao ----------------
    def wait_for(self, pattern, timeout):
        rx = re.compile(pattern)
        end = time.time() + timeout
        while time.time() < end:
            with self.clock:
                m = rx.search(self.scan)
            if m:
                return m
            time.sleep(0.1)
        return None

    def scan_reset(self):
        with self.clock:
            self.scan = ""

    # ---------------- acoes ----------------
    def run(self, gx, gy, trim, loop, diag, duration):
        with self.clock:
            self.trail = []
            self.arrived = False
            self.telem = {}
        self.goal = {"x": gx, "y": gy}
        self.manual = False
        prog = f"timeout {int(duration)} ./patrulha" if duration > 0 else "./patrulha"
        if diag:
            args = " --diag"
        else:
            args = f" --gx {gx:g} --gy {gy:g} -t {trim:g}" + (" --loop" if loop else "")
        self.running = True
        self.run_started = time.time()
        self.cmd(f"cd {REMOTE_DIR} && {prog}{args}")

    def manual_start(self):
        with self.clock:
            self.trail = []
            self.arrived = False
            self.telem = {"state": "MANUAL"}
        self.running = True
        self.manual = True
        self.run_started = time.time()
        self.cmd(f"cd {REMOTE_DIR} && ./patrulha --teleop")

    def manual_drive(self, fwd, diff):
        fwd = max(-1.0, min(1.0, float(fwd)))
        diff = max(-0.9, min(0.9, float(diff)))
        try:
            self._write(f"v {fwd:.3f} {diff:.3f}\r")
        except Exception as e:
            self._log(f"[interface] erro no controle manual: {e}")
            self.disconnect()

    def manual_zero(self):
        try:
            self._write("s\r")
        except Exception:
            pass

    # ---------------- jobs (deploy/compile/defines) ----------------
    def start_job(self, name, fn, *a):
        if self.job and self.job["status"] == "running":
            return False
        self.job = {"name": name, "status": "running", "detail": ""}

        def wrap():
            try:
                fn(*a)
            except Exception as e:
                self.job.update(status="error", detail=str(e))
        threading.Thread(target=wrap, daemon=True).start()
        return True

    def _upload_file_b64(self, local_path, remote_name):
        with open(local_path, "rb") as f:
            data = f.read().replace(b"\r\n", b"\n")
        md5 = hashlib.md5(data).hexdigest()
        b64 = base64.b64encode(data).decode()
        chunks = [b64[i:i + 120] for i in range(0, len(b64), 120)]
        self._log(f"[interface] enviando {remote_name} ({len(data)} bytes, md5 {md5[:8]}...)")
        self.cmd("rm -f /tmp/up.b64", echo=False)
        time.sleep(0.3)
        for i, ch in enumerate(chunks):
            self.cmd(f"echo {ch} >> /tmp/up.b64", echo=False)
            time.sleep(0.03)
            self.job["detail"] = f"enviando {remote_name} {i + 1}/{len(chunks)}"
        time.sleep(0.5)
        self.cmd(f"base64 -d /tmp/up.b64 > {REMOTE_DIR}/{remote_name}", echo=False)
        time.sleep(0.5)
        self.scan_reset()
        self.cmd(f"md5sum {REMOTE_DIR}/{remote_name}", echo=False)
        if not self.wait_for(md5, 10):
            self.job.update(status="error", detail=f"md5 nao confere apos envio de {remote_name}")
            return False
        self._log(f"[interface] {remote_name} OK (md5 confere)")
        return True

    def job_deploy(self, compile_after):
        files = (
            (LOCAL_MAIN, "main.c"),
            (LOCAL_CONTROLLER_CONFIG, "controller_config.h"),
            (LOCAL_CONTROLLER_CORE, "controller_core.h"),
        )
        for local_path, remote_name in files:
            if not self._upload_file_b64(local_path, remote_name):
                return
        self.overrides = {}
        if compile_after:
            if self._compile():
                self.teleop_ready = True
        else:
            self.job.update(status="ok", detail="enviado")

    def job_navigate(self, gx, gy, waypoints):
        """Executa a missao: sobe o patrulha em modo --mission e envia os
        waypoints do planejador A* (mapa) um a um, com odometria continua."""
        self.running = True
        self.manual = False
        self.scan_reset()
        self.cmd(f"cd {REMOTE_DIR} && ./patrulha --mission")
        if not self.wait_for(r"aguardando 'goto", 10):
            self.job.update(status="error", detail="modo missao nao iniciou")
            self.running = False
            return
        n = len(waypoints)
        for k, (wx, wy) in enumerate(waypoints):
            if not self.running:
                break
            self.job["detail"] = f"waypoint {k+1}/{n} -> ({wx:.0f},{wy:.0f})"
            self.scan_reset()
            self.cmd(f"goto {wx:.0f} {wy:.0f}", echo=False)
            if not self.wait_for(r"CHEGOU", 180):
                self.cmd("quit", echo=False)
                self.job.update(status="error", detail=f"timeout no waypoint {k+1}/{n}")
                self.running = False
                return
        self.cmd("quit", echo=False)
        self.running = False
        self.job.update(status="ok", detail=f"missao concluida ({n} waypoints)")

    def job_map_explore(self, gx, gy):
        """Geracao automatica de mapa: liga o mapeamento, roda A->B em modo
        LENTO (contorna os obstaculos devagar p/ mapear limpo) e, ao chegar
        em B (ou parar), encerra e salva o mapa."""
        with self.clock:
            self.trail = []; self.arrived = False; self.telem = {}
        self.goal = {"x": gx, "y": gy}
        mapper.start("auto_" + time.strftime("%Y%m%d_%H%M%S"), 50.0)
        self.running = True; self.manual = False
        self.run_started = time.time()
        self.job["detail"] = "mapeando A->B (contorno lento)..."
        self.scan_reset()
        self.cmd(f"cd {REMOTE_DIR} && ./patrulha --slow --gx {gx:g} --gy {gy:g}")
        arrived = self.wait_for(r"CHEGOU", 300)
        if not arrived:
            self.stop()          # Ctrl-C: para o robo
        self.running = False
        time.sleep(0.6)
        mapper.stop()
        mapper.save()
        snap = mapper.snapshot()
        cells = len(snap.get("cells", []))
        if arrived:
            self.job.update(status="ok", detail=f"mapa A->B salvo ({cells} celulas)")
        else:
            self.job.update(status="error", detail=f"timeout; mapa parcial salvo ({cells} celulas)")

    def job_compile(self):
        self._compile()

    def _compile(self):
        self.job["detail"] = "compilando (~1 min)..."
        self.scan_reset()
        self.cmd(BUILD_CMD, echo=False)
        self._log("[interface] compilando no robo...")
        m = self.wait_for(r"BUILD_OK|BUILD_FAIL", 300)
        if m and m.group(0) == "BUILD_OK":
            self.job.update(status="ok", detail="compilado com sucesso")
            return True
        else:
            self.job.update(status="error", detail="falha na compilacao (veja o console)")
            self.teleop_ready = False
            return False

    def job_defines(self, changes):
        for name, val in changes.items():
            remote_file = "main.c" if name in MAIN_DEFINES else "controller_config.h"
            sed = (f"sed -r -i \"s/^(#define[[:space:]]+{name}[[:space:]]+)"
                   f"[0-9.eE+-]+/\\1{val}/\" {REMOTE_DIR}/{remote_file}")
            self.cmd(sed, echo=False)
            time.sleep(0.15)
            self.overrides[name] = val
        names = "|".join(changes)
        self.cmd(f"grep -E '^#define +({names})' {REMOTE_DIR}/main.c {REMOTE_DIR}/controller_config.h", echo=False)
        time.sleep(0.8)
        self._compile()

    def job_wifi_setup(self, ssid, password, iface):
        q_ssid = shlex.quote(ssid)
        self.job["detail"] = "gravando wpa_supplicant.conf..."
        if password:
            self.cmd(f"wpa_passphrase {q_ssid} {shlex.quote(password)} > /tmp/wpa.conf", echo=False)
        else:
            self.cmd(
                "printf 'network={\\n  ssid=\"%s\"\\n  key_mgmt=NONE\\n}\\n' "
                f"{q_ssid} > /tmp/wpa.conf",
                echo=False,
            )
        time.sleep(0.4)

        self.job["detail"] = "reiniciando wpa_supplicant..."
        self.cmd("killall wpa_supplicant 2>/dev/null; sleep 1", echo=False)
        time.sleep(1.2)
        self.scan_reset()
        self.cmd(f"wpa_supplicant -B -i {iface} -c /tmp/wpa.conf -D nl80211,wext", echo=False)
        time.sleep(2.0)

        self.job["detail"] = "obtendo IP (udhcpc)..."
        self.scan_reset()
        self.cmd(f"udhcpc -i {iface}", echo=False)
        m = self.wait_for(r"\binet\b[^\d]*(\d{1,3}(?:\.\d{1,3}){3})", 20)
        if not m:
            self.scan_reset()
            self.cmd(f"ip addr show {iface} | grep inet", echo=False)
            m = self.wait_for(r"inet\s+(\d{1,3}(?:\.\d{1,3}){3})", 8)

        if m:
            self.wifi_ip = m.group(1)
            self.job.update(status="ok", detail=f"conectado, IP {self.wifi_ip}")
        else:
            self.wifi_ip = None
            self.job.update(status="error", detail="sem IP -- confira SSID/senha e tente de novo")


link = RobotLink()
app = Flask(__name__, static_folder="static")


@app.get("/")
def index():
    return send_from_directory(os.path.join(BASE, "static"), "index.html")


@app.get("/api/ports")
def ports():
    out = []
    for p in serial.tools.list_ports.comports():
        out.append({"port": p.device, "desc": p.description})
    out.sort(key=lambda x: ("USB" not in x["desc"].upper(), x["port"]))
    return jsonify(out)


@app.get("/api/wifi/discover")
def wifi_discover():
    try:
        timeout = float(request.args.get("timeout", 4.0))
    except ValueError:
        timeout = 4.0
    include_subnet = request.args.get("scan", "1") != "0"
    result = discover_wifi_devices(timeout=timeout, include_subnet=include_subnet)
    return jsonify({"ok": True, **result})


@app.get("/api/wifi/current")
def wifi_current():
    """SSID da rede Wi-Fi da maquina que roda esta interface (para preencher o formulario)."""
    ssid = _detect_local_wifi()
    return jsonify({"ok": True, "ssid": ssid})


@app.post("/api/wifi/setup")
def wifi_setup():
    err = _need_free_for_job()
    if err:
        return err
    j = request.json or {}
    ssid = (j.get("ssid") or "").strip()
    password = j.get("password") or ""
    iface = (j.get("iface") or "wlan0").strip() or "wlan0"
    if not ssid:
        return jsonify({"ok": False, "error": "SSID nao informado"}), 400
    if password and len(password) < 8:
        return jsonify({"ok": False, "error": "senha WPA/WPA2 deve ter ao menos 8 caracteres"}), 400
    if not IFACE_RE.match(iface):
        return jsonify({"ok": False, "error": f"interface invalida: {iface}"}), 400
    ok = link.start_job("wifi", link.job_wifi_setup, ssid, password, iface)
    if not ok:
        return jsonify({"ok": False, "error": "ja existe um job em andamento"}), 409
    return jsonify({"ok": True})


@app.post("/api/connect")
def connect():
    j = request.json or {}
    mode = j.get("mode", "usb")
    try:
        if mode == "wifi":
            link.connect_wifi(
                j.get("host", ""),
                j.get("username", "root"),
                j.get("password", ""),
                int(j.get("ssh_port", 22) or 22),
            )
        else:
            link.connect_usb(j.get("port", ""))
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 400
    return jsonify({"ok": True})


@app.post("/api/disconnect")
def disconnect():
    link.disconnect()
    return jsonify({"ok": True})


@app.get("/api/state")
def state():
    since = int(request.args.get("since", 0))
    with link.clock:
        lo = max(0, since - link.console_base)
        lines = link.console[lo:]
        nxt = link.console_base + len(link.console)
        telem = dict(link.telem)
        trail = list(link.trail[-1500:])
    job = dict(link.job) if link.job else None
    return jsonify({
        "connected": link.connected, "mode": link.mode, "port": link.port,
        "running": link.running, "manual": link.manual, "arrived": link.arrived,
        "teleop_ready": link.teleop_ready,
        "job": job, "busy": bool(job and job["status"] == "running"),
        "telem": telem, "trail": trail, "goal": link.goal,
        "map": mapper.snapshot(),
        "sim": sim_snapshot(SIM_SELECTED["worlds"], SIM_SELECTED["out_dir"]),
        "console": lines, "console_next": nxt,
        "wifi_ip": link.wifi_ip,
    })


@app.get("/api/defines")
def defines():
    vals = local_defines()
    vals.update(link.overrides)
    return jsonify({"order": DEFINES, "values": vals})


@app.get("/api/map/state")
def map_state():
    return jsonify(mapper.snapshot())


@app.get("/api/map/sessions")
def map_sessions():
    return jsonify({"sessions": mapper.sessions()})


def _latest_grid():
    """Grid.json mais recente (com celulas) salvo em maps/, ou None."""
    files = sorted(glob.glob(os.path.join(MAPS_DIR, "*", "grid.json")))
    for f in reversed(files):
        try:
            g = json.load(open(f, encoding="utf-8"))
            if g.get("cells"):
                return g, os.path.basename(os.path.dirname(f))
        except Exception:
            continue
    return None, None


@app.post("/api/map/navigate")
def map_navigate():
    """Planeja A* sobre o mapa gerado (frame de odometria = start em 0,0) e
    dirige o robo pelos waypoints via modo --mission.
    NOTA: como a odometria zera a cada missao, o mapa e o alvo sao
    interpretados no MESMO referencial (robo comeca na origem do mapa)."""
    err = _need_free_for_job()
    if err:
        return err
    j = request.json or {}
    try:
        gx = float(j.get("gx")); gy = float(j.get("gy"))
    except (TypeError, ValueError):
        return jsonify({"ok": False, "error": "alvo (gx,gy) invalido"}), 400
    grid, sess = _latest_grid()
    if grid is None:
        return jsonify({"ok": False, "error": "nenhum mapa salvo em maps/ (rode e salve um mapa antes)"}), 400
    wps = planner.plan_waypoints(grid, (0.0, 0.0), (gx, gy))
    if not wps:
        return jsonify({"ok": False, "error": "sem caminho ate o alvo no mapa (alvo dentro de obstaculo?)"}), 409
    ok = link.start_job("navigate", link.job_navigate, gx, gy, wps)
    if not ok:
        return jsonify({"ok": False, "error": "ja existe um job em andamento"}), 409
    return jsonify({"ok": True, "map": sess, "waypoints": wps})


@app.post("/api/map/explore")
def map_explore():
    """Geracao automatica de mapa: roda A->B em modo lento com o mapeamento
    ligado e salva o mapa ao final. Alvo (gx,gy) do card Missao."""
    err = _need_free_for_job()
    if err:
        return err
    j = request.json or {}
    try:
        gx = float(j.get("gx")); gy = float(j.get("gy"))
    except (TypeError, ValueError):
        return jsonify({"ok": False, "error": "alvo (gx,gy) invalido"}), 400
    ok = link.start_job("explore", link.job_map_explore, gx, gy)
    if not ok:
        return jsonify({"ok": False, "error": "ja existe um job em andamento"}), 409
    return jsonify({"ok": True})


@app.post("/api/map/start")
def map_start():
    j = request.json or {}
    try:
        resolution = float(j.get("resolution", 50))
    except ValueError:
        return jsonify({"ok": False, "error": "resolucao invalida"}), 400
    snapshot = mapper.start(j.get("name", ""), resolution)
    return jsonify({"ok": True, "map": snapshot})


@app.post("/api/map/stop")
def map_stop():
    mapper.stop()
    return jsonify({"ok": True, "map": mapper.snapshot()})


@app.post("/api/map/save")
def map_save():
    mapper.save()
    return jsonify({"ok": True, "map": mapper.snapshot()})


@app.post("/api/map/clear")
def map_clear():
    mapper.clear()
    return jsonify({"ok": True, "map": mapper.snapshot()})


def _need_local_job_free():
    if link.job and link.job.get("status") == "running":
        return jsonify({"ok": False, "error": "ja existe um job em andamento"}), 409
    return None


@app.get("/api/sim/state")
def sim_state():
    try:
        worlds = _repo_path(request.args.get("worlds", ""), SIM_WORLD_DEFAULT)
        out_dir = _repo_path(request.args.get("out_dir", ""), SIM_OUT_DIR)
    except ValueError as e:
        return jsonify({"ok": False, "error": str(e)}), 400
    return jsonify({"ok": True, **sim_snapshot(worlds, out_dir)})


@app.post("/api/sim/generate")
def sim_generate():
    err = _need_local_job_free()
    if err:
        return err
    j = request.json or {}
    try:
        count = int(j.get("count", 1000))
        out_path = _repo_path(j.get("out", ""), SIM_WORLD_DEFAULT)
    except (TypeError, ValueError) as e:
        return jsonify({"ok": False, "error": f"parametro invalido: {e}"}), 400
    if count < 1 or count > 1000:
        return jsonify({"ok": False, "error": "quantidade precisa ficar entre 1 e 1000"}), 400
    SIM_SELECTED["worlds"] = out_path
    ok = link.start_job("sim-generate", link.job_sim_generate, count, out_path)
    if not ok:
        return jsonify({"ok": False, "error": "ja existe um job em andamento"}), 409
    return jsonify({"ok": True})


@app.post("/api/sim/batch")
def sim_batch():
    err = _need_local_job_free()
    if err:
        return err
    j = request.json or {}
    try:
        worlds = _repo_path(j.get("worlds", ""), SIM_WORLD_DEFAULT)
        out_dir = _repo_path(j.get("out_dir", ""), SIM_OUT_DIR)
    except ValueError as e:
        return jsonify({"ok": False, "error": str(e)}), 400
    SIM_SELECTED["worlds"] = worlds
    SIM_SELECTED["out_dir"] = out_dir
    ok = link.start_job("sim-batch", link.job_sim_batch, worlds, out_dir)
    if not ok:
        return jsonify({"ok": False, "error": "ja existe um job em andamento"}), 409
    return jsonify({"ok": True})


@app.post("/api/sim/open")
def sim_open():
    j = request.json or {}
    try:
        worlds = _repo_path(j.get("worlds", ""), SIM_WORLD_DEFAULT)
    except ValueError as e:
        return jsonify({"ok": False, "error": str(e)}), 400
    if not os.path.exists(worlds):
        return jsonify({"ok": False, "error": f"JSON nao encontrado: {_repo_rel(worlds)}"}), 400
    SIM_SELECTED["worlds"] = worlds
    flags = getattr(subprocess, "CREATE_NO_WINDOW", 0) if platform.system() == "Windows" else 0
    subprocess.Popen(
        [
            "powershell",
            "-ExecutionPolicy", "Bypass",
            "-File", SIM_BUILD,
            "-Interactive",
            "-Worlds", worlds,
        ],
        cwd=ROOT,
        creationflags=flags,
    )
    return jsonify({"ok": True})


def _need_link():
    if not link.connected:
        return jsonify({"ok": False, "error": "robo nao conectado"}), 400
    return None


def _need_free_for_job():
    err = _need_link()
    if err:
        return err
    if link.running:
        return jsonify({"ok": False, "error": "pare a missao antes desta acao"}), 409
    if link.job and link.job.get("status") == "running":
        return jsonify({"ok": False, "error": "ja existe um job em andamento"}), 409
    return None


@app.post("/api/run")
def run():
    err = _need_link()
    if err:
        return err
    if link.job and link.job.get("status") == "running":
        return jsonify({"ok": False, "error": "aguarde o job atual terminar"}), 409
    if link.running:
        return jsonify({"ok": False, "error": "pare a execucao atual antes de iniciar outra"}), 409
    j = request.json or {}
    try:
        gx = float(j.get("gx", 700))
        gy = float(j.get("gy", 0))
        trim = float(j.get("trim", 0))
        duration = float(j.get("duration", 15))
    except ValueError:
        return jsonify({"ok": False, "error": "parametro invalido"}), 400
    link.run(gx, gy, trim, bool(j.get("loop")), bool(j.get("diag")), duration)
    return jsonify({"ok": True})


@app.post("/api/stop")
def stop():
    err = _need_link()
    if err:
        return err
    link.stop()
    return jsonify({"ok": True})


@app.post("/api/manual/start")
def manual_start():
    err = _need_link()
    if err:
        return err
    if link.job and link.job.get("status") == "running":
        return jsonify({"ok": False, "error": "aguarde o job atual terminar"}), 409
    if not link.teleop_ready:
        return jsonify({"ok": False, "error": "use Enviar + compilar antes do controle manual"}), 409
    if link.running and not link.manual:
        return jsonify({"ok": False, "error": "pare a missao atual antes do controle manual"}), 409
    if not link.manual:
        link.manual_start()
    return jsonify({"ok": True})


@app.post("/api/manual/drive")
def manual_drive():
    err = _need_link()
    if err:
        return err
    if not link.manual:
        return jsonify({"ok": False, "error": "controle manual nao iniciado"}), 409
    j = request.json or {}
    try:
        fwd = float(j.get("fwd", 0))
        diff = float(j.get("diff", 0))
    except ValueError:
        return jsonify({"ok": False, "error": "comando manual invalido"}), 400
    link.manual_drive(fwd, diff)
    return jsonify({"ok": True})


@app.post("/api/manual/zero")
def manual_zero():
    err = _need_link()
    if err:
        return err
    if link.manual:
        link.manual_zero()
    return jsonify({"ok": True})


@app.post("/api/deploy")
def deploy():
    err = _need_free_for_job()
    if err:
        return err
    ok = link.start_job("deploy", link.job_deploy, True)
    if not ok:
        return jsonify({"ok": False, "error": "ja existe um job em andamento"}), 409
    return jsonify({"ok": True})


@app.post("/api/compile")
def compile_():
    err = _need_free_for_job()
    if err:
        return err
    ok = link.start_job("compile", link.job_compile)
    if not ok:
        return jsonify({"ok": False, "error": "ja existe um job em andamento"}), 409
    return jsonify({"ok": True})


@app.post("/api/defines")
def set_defines():
    err = _need_free_for_job()
    if err:
        return err
    j = request.json or {}
    changes = {}
    for k, v in j.items():
        if k not in DEFINES:
            return jsonify({"ok": False, "error": f"parametro desconhecido: {k}"}), 400
        try:
            float(v)
        except ValueError:
            return jsonify({"ok": False, "error": f"valor invalido p/ {k}: {v}"}), 400
        changes[k] = str(v)
    if not changes:
        return jsonify({"ok": False, "error": "nada para aplicar"}), 400
    ok = link.start_job("defines", link.job_defines, changes)
    if not ok:
        return jsonify({"ok": False, "error": "ja existe um job em andamento"}), 409
    return jsonify({"ok": True})


@app.post("/api/cmd")
def raw_cmd():
    err = _need_link()
    if err:
        return err
    c = (request.json or {}).get("cmd", "").strip()
    if c:
        link.cmd(c)
    return jsonify({"ok": True})


if __name__ == "__main__":
    port = int(os.environ.get("PORT", "8340"))
    host = os.environ.get("HOST", "0.0.0.0")
    print(f"Interface Khepera IV -> http://localhost:{port}   (main.c local: {LOCAL_MAIN})")
    if host not in ("127.0.0.1", "localhost"):
        for ip in _local_ipv4s():
            print(f"  tambem acessivel na mesma rede em: http://{ip}:{port}")
    app.run(host=host, port=port, threaded=True)
