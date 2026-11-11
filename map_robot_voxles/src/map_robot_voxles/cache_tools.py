"""Configuration and provenance helpers for explicit cache tools."""

import hashlib
import json
from pathlib import Path

import yaml


def sha256(path):
    path = Path(path)
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_config(config_file):
    config_file = Path(config_file).expanduser().resolve()
    with config_file.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    workspace_root = config.get("workspace_root", ".")
    workspace_root = (config_file.parent / workspace_root).resolve()
    config["_config_file"] = str(config_file)
    config["_workspace_root"] = str(workspace_root)
    return config


def resolve_path(config, value):
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = Path(config["_workspace_root"]) / path
    return path.resolve()


def write_manifest(path, kind, config, sources, outputs, status="generated"):
    path = Path(path).expanduser().resolve()
    records = []
    for role, item in list(sources.items()) + list(outputs.items()):
        item = Path(item).expanduser().resolve()
        if not item.is_file():
            raise FileNotFoundError("manifest input does not exist: {}".format(item))
        records.append({
            "role": role,
            "path": str(item),
            "size": item.stat().st_size,
            "sha256": sha256(item),
            "output": role in outputs,
        })
    payload = {
        "schema_version": 1,
        "kind": kind,
        "status": status,
        "config_file": config.get("_config_file"),
        "parameters": {
            key: value for key, value in config.items() if not key.startswith("_")
        },
        "files": records,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    return path
