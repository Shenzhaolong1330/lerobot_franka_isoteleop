from datetime import datetime
from pathlib import Path
import re
import shutil

import yaml

from lerobot.utils.constants import HF_LEROBOT_HOME


def clean_dataset_info():
    parent_path = Path(__file__).resolve().parent
    cfg_path = parent_path.parent / "config" / "cfg.yaml"
    with cfg_path.open("r", encoding="utf-8") as file:
        cfg = yaml.safe_load(file)

    repo_id = cfg["record"]["repo_id"]
    user_name = repo_id.split("/", 1)[0]

    base_path = Path(HF_LEROBOT_HOME) / user_name
    info_file = base_path / "dataset_info.txt"

    if not info_file.exists():
        print(f"====== [ERROR] dataset_info.txt not found at {info_file} ======")
        return

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_file = (
        base_path
        / "dataset_info_backup"
        / f"dataset_info_backup_{timestamp}.txt"
    )
    backup_file.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(info_file, backup_file)
    print(f"====== [BACKUP] Created backup file: {backup_file} ======")

    existing_folders = {p.name for p in base_path.iterdir() if p.is_dir()}
    print(f"====== [INFO] Found {len(existing_folders)} existing dataset folders ======")

    with info_file.open("r", encoding="utf-8") as file:
        lines = file.readlines()

    kept_lines = []
    removed_lines = []

    for line in lines:
        match = re.search(r'name="([^"]+)"', line)
        if match:
            full_name = match.group(1)
            folder_name = full_name.split("/", 1)[1] if "/" in full_name else full_name
            if folder_name in existing_folders:
                kept_lines.append(line)
            else:
                removed_lines.append(line)
        else:
            kept_lines.append(line)

    updated_lines = [
        re.sub(r'record_id="[^"]*"', f'record_id="{index}"', line)
        for index, line in enumerate(kept_lines, start=1)
    ]
    with info_file.open("w", encoding="utf-8") as file:
        file.writelines(updated_lines)

    print("====== [CLEANUP COMPLETE] ======")
    print(f"Kept {len(updated_lines)} lines, removed {len(removed_lines)} invalid entries.")
    print(f"Backup saved at: {backup_file}")
    if removed_lines:
        print("Removed entries:")
        for line in removed_lines:
            print(" -", line.strip())

def main():
    clean_dataset_info()


if __name__ == "__main__":
    main()
