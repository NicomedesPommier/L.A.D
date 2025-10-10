# core/ip_config.py  (ajusta la ruta si tu settings.py está en otro paquete)
"""Utilities for reading the shared network/IP configuration."""
from __future__ import annotations

import json
import os
import ipaddress
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict

# Apunta a <repo>/config/ip_config.json (ajusta parents[n] si cambia tu layout)
_BASE_DIR = Path(__file__).resolve().parents[2]
_CONFIG_FILE = _BASE_DIR / "config" / "ip_config.json"

# Defaults seguros
_DEFAULTS: Dict[str, Any] = {
    "exposed_ip": "192.168.100.116",
    "frontend_port": 3000,
    "backend_port": 8000,
    "rosbridge_port": 9090,
    "scheme_http": "http",  # o "https"
    "scheme_ws": "ws",      # o "wss"
}


@dataclass(frozen=True)
class NetworkConfig:
    """Typed representation of the network configuration."""

    exposed_ip: str
    frontend_port: int = 3000
    backend_port: int = 8000
    rosbridge_port: int = 9090
    scheme_http: str = "http"
    scheme_ws: str = "ws"

    @property
    def frontend_origin(self) -> str:
        return f"{self.scheme_http}://{self.exposed_ip}:{self.frontend_port}"

    @property
    def backend_origin(self) -> str:
        return f"{self.scheme_http}://{self.exposed_ip}:{self.backend_port}"

    @property
    def rosbridge_ws(self) -> str:
        return f"{self.scheme_ws}://{self.exposed_ip}:{self.rosbridge_port}"

    def as_dict(self) -> Dict[str, Any]:
        return {
            "exposed_ip": self.exposed_ip,
            "frontend_origin": self.frontend_origin,
            "backend_origin": self.backend_origin,
            "rosbridge_ws": self.rosbridge_ws,
            "frontend_port": self.frontend_port,
            "backend_port": self.backend_port,
            "rosbridge_port": self.rosbridge_port,
            "scheme_http": self.scheme_http,
            "scheme_ws": self.scheme_ws,
        }


def _load_raw_config() -> Dict[str, Any]:
    """Lee el JSON y mezcla con defaults (sin romper si falta el archivo)."""
    if not _CONFIG_FILE.exists():
        return dict(_DEFAULTS)

    try:
        data = json.loads(_CONFIG_FILE.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid JSON in {_CONFIG_FILE}: {exc}") from exc

    if not isinstance(data, dict):
        raise ValueError(f"Configuration in {_CONFIG_FILE} must be a JSON object")

    merged = dict(_DEFAULTS)
    merged.update({k: v for k, v in data.items() if v is not None})
    return merged


def _is_valid_ip(s: str) -> bool:
    try:
        ipaddress.ip_address(s)
        return True
    except ValueError:
        return False


def _coerce_int(value: Any, default: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _normalize_scheme(value: str, allowed: set[str], default: str) -> str:
    v = (value or "").strip().lower()
    return v if v in allowed else default


def load_network_config() -> NetworkConfig:
    """Load the shared network configuration file, with env overrides."""
    data = _load_raw_config()

    # Overrides por variables de entorno (si las pones, mandan sobre el JSON)
    env_ip = os.getenv("EXPOSED_IP", "").strip()
    env_front = os.getenv("FRONTEND_PORT", "").strip()
    env_back = os.getenv("BACKEND_PORT", "").strip()
    env_ros = os.getenv("ROSBRIDGE_PORT", "").strip()
    env_http = os.getenv("SCHEME_HTTP", "").strip()
    env_ws = os.getenv("SCHEME_WS", "").strip()

    # IP
    exposed = env_ip or str(data.get("exposed_ip", "")).strip() or _DEFAULTS["exposed_ip"]
    if not _is_valid_ip(exposed):
        exposed = _DEFAULTS["exposed_ip"]

    # Puertos
    frontend_port = _coerce_int(env_front or data.get("frontend_port"), _DEFAULTS["frontend_port"])
    backend_port = _coerce_int(env_back or data.get("backend_port"), _DEFAULTS["backend_port"])
    rosbridge_port = _coerce_int(env_ros or data.get("rosbridge_port"), _DEFAULTS["rosbridge_port"])

    # Esquemas
    scheme_http = _normalize_scheme(env_http or data.get("scheme_http"), {"http", "https"}, _DEFAULTS["scheme_http"])
    scheme_ws = _normalize_scheme(env_ws or data.get("scheme_ws"), {"ws", "wss"}, _DEFAULTS["scheme_ws"])

    return NetworkConfig(
        exposed_ip=exposed,
        frontend_port=frontend_port,
        backend_port=backend_port,
        rosbridge_port=rosbridge_port,
        scheme_http=scheme_http,
        scheme_ws=scheme_ws,
    )


__all__ = ["NetworkConfig", "load_network_config"]
