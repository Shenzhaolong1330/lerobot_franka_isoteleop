from datetime import datetime
from pathlib import Path
import re


def generate_dataset_name(cfg):
    """Return the next dated dataset version."""
    if cfg.resume:
        return cfg.resume_dataset, cfg.resume_dataset.rsplit("_", 1)[-1]

    user, description = cfg.repo_id.split("/", 1)
    base_path = Path(cfg.dataset_path).parent
    base_path.mkdir(parents=True, exist_ok=True)
    pattern = re.compile(rf"^{re.escape(description)}_\d{{8}}_v(\d+)$")
    versions = (
        int(match.group(1))
        for path in base_path.iterdir()
        if path.is_dir() and (match := pattern.match(path.name))
    )
    version_str = f"v{max(versions, default=0) + 1:02d}"
    date = datetime.today().strftime("%Y%m%d")
    return f"{user}/{description}_{date}_{version_str}", version_str


def update_dataset_info(cfg, dataset_name, version_str):
    """Append one entry to dataset_info.txt."""
    info_path = Path(cfg.dataset_path).parent
    info_file = info_path / "dataset_info.txt"

    if info_file.exists():
        with info_file.open("r", encoding="utf-8") as file:
            lines = [line for line in file if line.strip()]
        record_id = len(lines) + 1
    else:
        record_id = 1

    info_line = (
        f'record_id="{record_id}", name="{dataset_name}", '
        f'task="{cfg.task_description}", '
        f'date="{datetime.now():%Y-%m-%d %H:%M:%S}", version="{version_str}", '
        f'user_info="{cfg.user_info}", '
        f'type="{"resumed" if cfg.resume else "record"}"\n'
    )
    with info_file.open("a", encoding="utf-8") as file:
        file.write(info_line)
