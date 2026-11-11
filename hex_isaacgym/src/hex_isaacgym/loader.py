"""Isaac Gym loader that enforces its required import order."""

import importlib
import os
from pathlib import Path
import sys
import tempfile


DEFAULT_ISAACGYM_PYTHON = Path(
    "/home/val/Downloads/IsaacGym_Preview_4_Package/isaacgym/python"
)


def load_isaacgym(python_path=None):
    """Load Isaac Gym before importing Torch and return its two API modules."""
    if "isaacgym" not in sys.modules and "torch" in sys.modules:
        raise RuntimeError(
            "Torch was imported before Isaac Gym. Start a fresh process and "
            "import/create hex_isaacgym before importing Torch."
        )
    path = DEFAULT_ISAACGYM_PYTHON if python_path is None else Path(python_path)
    path = path.expanduser().resolve()
    if not (path / "isaacgym/__init__.py").is_file():
        raise FileNotFoundError("Isaac Gym Python package not found at {}".format(path))
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
    # Preview 4 builds gymtorch on first import. Home may be read-only in ROS
    # deployments and managed runners, so use an explicit disposable cache.
    extension_dir = Path(tempfile.gettempdir()) / "hex_isaacgym_torch_extensions"
    extension_dir.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("TORCH_EXTENSIONS_DIR", str(extension_dir))
    gymapi = importlib.import_module("isaacgym.gymapi")
    gymtorch = importlib.import_module("isaacgym.gymtorch")
    return gymapi, gymtorch
