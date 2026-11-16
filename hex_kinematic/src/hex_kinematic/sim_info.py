"""Plain YAML handoff between the runner and the simulator."""
from pathlib import Path

import yaml


def write_sim_info(filename, values):
    filename = Path(filename).expanduser()
    filename.parent.mkdir(parents=True, exist_ok=True)
    content = yaml.safe_dump(values, sort_keys=False)
    with filename.open('w') as stream:
        stream.write(content)


def read_sim_info(filename):
    with Path(filename).expanduser().open() as stream:
        return yaml.safe_load(stream)
