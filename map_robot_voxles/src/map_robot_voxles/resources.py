"""Locate package resources in source, devel, and installed workspaces."""
from pathlib import Path
import sys


def package_resource(package, relative):
    candidates = [Path(__file__).resolve().parents[2].parent / package]
    for parent in Path(__file__).resolve().parents:
        candidates.extend((parent / 'src' / package, parent / 'share' / package))
    try:
        import rospkg
        try:
            candidates.append(Path(rospkg.RosPack().get_path(package)))
        except rospkg.ResourceNotFound:
            pass
    except ImportError:
        pass
    candidates.append(Path(sys.prefix) / 'share' / package)
    for base in candidates:
        result = base / relative
        if result.exists():
            return result
    raise FileNotFoundError('package resource not found: {}/{}'.format(package, relative))
