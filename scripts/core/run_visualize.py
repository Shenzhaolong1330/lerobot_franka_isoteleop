import argparse
import gc
import json
import logging
import tempfile
import time
import webbrowser
from collections.abc import Iterator
from pathlib import Path

import numpy as np
import rerun as rr
import torch
import torch.utils.data
import tqdm
import yaml

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.constants import ACTION, DONE, OBS_STATE, REWARD


def get_feature_names(features: dict, key: str) -> list[str]:
    names = features[key]["names"]
    if isinstance(names, dict):
        if all(isinstance(name, int) for name in names):
            return [names[index] for index in sorted(names)]
        return [name for name, _ in sorted(names.items(), key=lambda item: item[1])]
    return list(names)


def tensor_row_to_named_dict(row: torch.Tensor, names: list[str]) -> dict[str, float]:
    return {name: float(row[idx].item()) for idx, name in enumerate(names)}


def obs_group_name(name: str) -> str:
    return name.split(".", 1)[0]


def select_obs_group(values: dict[str, float], group: str | None) -> dict[str, float]:
    if not group:
        return {}
    return {
        name: value
        for name, value in values.items()
        if name == group or name.startswith(f"{group}.")
    }


def format_named_values(values: dict[str, float]) -> str:
    return ", ".join(f"{name}={value:.6f}" for name, value in values.items())


def green(text: str) -> str:
    return f"\033[32m{text}\033[0m"


def write_data_window_html(
    repo_id: str,
    episode_index: int,
    rows: list[dict],
    obs_names: list[str],
    default_obs_group: str | None = None,
    output_dir: Path | None = None,
) -> Path:
    if output_dir is None:
        output_dir = Path(tempfile.gettempdir()) / "franka_keyteleop_visualize"
    output_dir.mkdir(parents=True, exist_ok=True)

    html_path = output_dir / f"{repo_id.replace('/', '_')}_episode_{episode_index}_data.html"
    payload = {
        "repo_id": repo_id,
        "episode_index": episode_index,
        "obs_names": obs_names,
        "default_obs_group": default_obs_group,
        "rows": rows,
    }
    data_json = json.dumps(payload)
    html = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Franka Episode Data</title>
  <style>
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      font-family: Arial, sans-serif;
      background: #eef3f1;
      color: #17201d;
    }}
    header {{
      padding: 14px 18px;
      background: #173c35;
      color: white;
      display: flex;
      justify-content: space-between;
      gap: 16px;
      align-items: center;
    }}
    header h1 {{
      margin: 0;
      font-size: 17px;
      font-weight: 600;
    }}
    header span {{
      font-size: 13px;
      color: #cfe6de;
    }}
    .layout {{
      display: grid;
      grid-template-columns: 430px 1fr;
      height: calc(100vh - 54px);
      min-height: 520px;
    }}
    aside {{
      border-right: 1px solid #c9d7d2;
      background: #fbfdfc;
      overflow: hidden;
      display: flex;
      flex-direction: column;
    }}
    .controls {{
      padding: 12px;
      border-bottom: 1px solid #d8e3df;
      display: grid;
      gap: 10px;
    }}
    .field {{
      display: grid;
      gap: 5px;
    }}
    .field label {{
      font-size: 11px;
      font-weight: 700;
      color: #56635f;
      text-transform: uppercase;
    }}
    input[type="search"], input[type="range"], select {{
      width: 100%;
    }}
    input[type="search"], select {{
      height: 32px;
      padding: 0 9px;
      border: 1px solid #b9ccc5;
      border-radius: 5px;
      font-size: 13px;
      background: white;
    }}
    select {{
      border-color: #9db5ca;
      color: #1e466f;
      font-weight: 700;
      background: #f7fbff;
    }}
    .list {{
      overflow: auto;
      font-size: 12px;
    }}
    .row {{
      padding: 8px 10px;
      border-bottom: 1px solid #edf3f1;
      cursor: pointer;
      display: grid;
      gap: 3px;
    }}
    .row:hover, .row.active {{
      background: #eef1f6;
    }}
    .row-title {{
      font-weight: 700;
      color: #111827;
    }}
    .row-sub {{
      white-space: nowrap;
      overflow: hidden;
      text-overflow: ellipsis;
      color: #53625e;
    }}
    main {{
      overflow: auto;
      padding: 18px;
    }}
    .summary {{
      display: grid;
      grid-template-columns: repeat(3, minmax(120px, 1fr));
      gap: 10px;
      margin-bottom: 16px;
    }}
    .metric {{
      background: #f8fafc;
      border: 1px solid #c8d2df;
      border-radius: 6px;
      padding: 10px;
    }}
    .metric:nth-child(1) {{
      border-left: 4px solid #9aa9bd;
    }}
    .metric:nth-child(2) {{
      border-left: 4px solid #8fb3c9;
    }}
    .metric:nth-child(3) {{
      border-left: 4px solid #8bbba7;
    }}
    .metric label {{
      display: block;
      color: #66736f;
      font-size: 12px;
      margin-bottom: 5px;
    }}
    .metric div {{
      font-weight: 700;
      font-size: 16px;
    }}
    section {{
      background: #fbfdfc;
      border: 1px solid #d2dfda;
      border-radius: 6px;
      margin-bottom: 14px;
      overflow: hidden;
    }}
    section.obs-section {{
      border-color: #7ab59f;
    }}
    section.action-section {{
      border-color: #8aa9d6;
    }}
    section h2 {{
      margin: 0;
      padding: 10px 12px;
      font-size: 14px;
      border-bottom: 1px solid #d2dfda;
    }}
    .obs-section h2 {{
      background: #e0f2ea;
      color: #17533f;
    }}
    .action-section h2 {{
      background: #e6eefb;
      color: #214f88;
    }}
    table {{
      width: 100%;
      border-collapse: collapse;
      font-size: 13px;
    }}
    th, td {{
      padding: 7px 10px;
      border-bottom: 1px solid #edf3f1;
      text-align: left;
      font-variant-numeric: tabular-nums;
    }}
    th {{
      width: 240px;
      color: #4f5f5a;
      font-weight: 600;
      background: #f7faf9;
    }}
    .empty {{
      padding: 12px;
      color: #15803d;
      font-weight: 600;
    }}
  </style>
</head>
<body>
  <header>
    <h1>Franka Episode Data</h1>
    <span id="meta"></span>
  </header>
  <div class="layout">
    <aside>
      <div class="controls">
        <div class="field">
          <label for="obsGroup">Observation Group</label>
          <select id="obsGroup"></select>
        </div>
        <div class="field">
          <label for="search">Search</label>
          <input id="search" type="search" placeholder="Search action or selected obs values">
        </div>
        <div class="field">
          <label for="slider">Step</label>
          <input id="slider" type="range" min="0" max="0" value="0">
        </div>
      </div>
      <div id="list" class="list"></div>
    </aside>
    <main>
      <div class="summary">
        <div class="metric"><label>Frame</label><div id="frameValue">-</div></div>
        <div class="metric"><label>Timestamp</label><div id="timeValue">-</div></div>
        <div class="metric"><label>Displayed Obs Fields</label><div id="obsCount">0</div></div>
      </div>
      <section class="obs-section">
        <h2>Observation</h2>
        <div id="obsEmpty" class="empty" hidden>No observation fields in this group.</div>
        <table id="obsTable"></table>
      </section>
      <section class="action-section">
        <h2>Action</h2>
        <table id="actionTable"></table>
      </section>
    </main>
  </div>
  <script>
    const data = {data_json};
    const rows = data.rows || [];
    const obsGroups = [...new Set((data.obs_names || []).map((name) => name.split(".")[0]))];
    let filtered = rows.map((_, index) => index);
    let selectedIndex = 0;
    let selectedObsGroup = data.default_obs_group && obsGroups.includes(data.default_obs_group)
      ? data.default_obs_group
      : (obsGroups[0] || "");

    const meta = document.getElementById("meta");
    const list = document.getElementById("list");
    const search = document.getElementById("search");
    const slider = document.getElementById("slider");
    const obsGroup = document.getElementById("obsGroup");

    meta.textContent = `${{data.repo_id}} | episode ${{data.episode_index}} | ${{rows.length}} frames`;
    slider.max = Math.max(rows.length - 1, 0);
    obsGroup.innerHTML = obsGroups.map((group) => `<option value="${{group}}">${{group}}</option>`).join("");
    obsGroup.value = selectedObsGroup;

    function valueText(value) {{
      return Number.isFinite(value) ? value.toFixed(6) : String(value);
    }}

    function dictSummary(values) {{
      return Object.entries(values || {{}})
        .map(([key, value]) => `${{key}}=${{valueText(value)}}`)
        .join(", ");
    }}

    function selectedObs(values) {{
      return Object.fromEntries(
        Object.entries(values || {{}}).filter(([key]) => key === selectedObsGroup || key.startsWith(`${{selectedObsGroup}}.`))
      );
    }}

    function renderTable(tableId, values) {{
      const table = document.getElementById(tableId);
      table.innerHTML = "";
      for (const [key, value] of Object.entries(values || {{}})) {{
        const tr = document.createElement("tr");
        const th = document.createElement("th");
        const td = document.createElement("td");
        th.textContent = key;
        td.textContent = valueText(value);
        tr.appendChild(th);
        tr.appendChild(td);
        table.appendChild(tr);
      }}
    }}

    function renderDetail(rowIndex) {{
      if (!rows.length) return;
      selectedIndex = Math.max(0, Math.min(rowIndex, rows.length - 1));
      const row = rows[selectedIndex];
      const obs = selectedObs(row.obs);
      document.getElementById("frameValue").textContent = row.frame_index;
      document.getElementById("timeValue").textContent = row.timestamp.toFixed(3);
      document.getElementById("obsCount").textContent = Object.keys(obs).length;
      renderTable("actionTable", row.action);
      renderTable("obsTable", obs);
      document.getElementById("obsEmpty").hidden = Object.keys(obs).length > 0;
      slider.value = selectedIndex;
      document.querySelectorAll(".row").forEach((el) => {{
        el.classList.toggle("active", Number(el.dataset.index) === selectedIndex);
      }});
    }}

    function renderList() {{
      list.innerHTML = "";
      for (const rowIndex of filtered) {{
        const row = rows[rowIndex];
        const item = document.createElement("div");
        item.className = "row";
        item.dataset.index = rowIndex;
        item.innerHTML = `
          <div class="row-title">Frame ${{row.frame_index}} | t=${{row.timestamp.toFixed(3)}}</div>
          <div class="row-sub">action: ${{dictSummary(row.action)}}</div>
          <div class="row-sub">${{selectedObsGroup}}: ${{dictSummary(selectedObs(row.obs))}}</div>
        `;
        item.addEventListener("click", () => renderDetail(rowIndex));
        list.appendChild(item);
      }}
      renderDetail(filtered[0] ?? 0);
    }}

    search.addEventListener("input", () => {{
      const q = search.value.toLowerCase().trim();
      filtered = rows
        .map((row, index) => [row, index])
        .filter(([row]) => {{
          if (!q) return true;
          return `${{row.frame_index}} ${{row.timestamp}} ${{dictSummary(row.action)}} ${{dictSummary(selectedObs(row.obs))}}`
            .toLowerCase()
            .includes(q);
        }})
        .map(([, index]) => index);
      renderList();
    }});

    obsGroup.addEventListener("change", () => {{
      selectedObsGroup = obsGroup.value;
      renderList();
    }});
    slider.addEventListener("input", () => renderDetail(Number(slider.value)));
    slider.addEventListener("wheel", (event) => {{
      event.preventDefault();
      const direction = event.deltaY > 0 || event.deltaX > 0 ? 1 : -1;
      const nextIndex = Math.max(0, Math.min(rows.length - 1, Number(slider.value) + direction));
      renderDetail(nextIndex);
    }}, {{ passive: false }});
    renderList();
  </script>
</body>
</html>
"""
    html_path.write_text(html, encoding="utf-8")
    return html_path


class EpisodeSampler(torch.utils.data.Sampler):
    def __init__(self, dataset: LeRobotDataset, episode_index: int):
        from_idx = dataset.meta.episodes["dataset_from_index"][episode_index]
        to_idx = dataset.meta.episodes["dataset_to_index"][episode_index]
        self.frame_ids = range(from_idx, to_idx)

    def __iter__(self) -> Iterator:
        return iter(self.frame_ids)

    def __len__(self) -> int:
        return len(self.frame_ids)


def to_hwc_uint8_numpy(chw_float32_torch: torch.Tensor) -> np.ndarray:
    assert chw_float32_torch.dtype == torch.float32
    assert chw_float32_torch.ndim == 3
    c, h, w = chw_float32_torch.shape
    assert c < h and c < w, f"expect channel first images, but instead {chw_float32_torch.shape}"
    hwc_uint8_numpy = (chw_float32_torch * 255).type(torch.uint8).permute(1, 2, 0).numpy()
    return hwc_uint8_numpy


def visualize_dataset(
    dataset: LeRobotDataset,
    episode_index: int,
    batch_size: int = 32,
    num_workers: int = 0,
    mode: str = "local",
    web_port: int = 9090,
    ws_port: int = 9087,
    save: bool = False,
    output_dir: Path | None = None,
    default_obs_group: str | None = None,
    data_window: bool = True,
    print_to_terminal: bool = False,
) -> Path | None:
    if save:
        assert output_dir is not None, (
            "Set an output directory where to write .rrd files with `--output-dir path/to/directory`."
        )

    repo_id = dataset.repo_id
    features = dataset.meta.features
    action_names = (
        get_feature_names(features, "action") if "action" in features else []
    )
    obs_state_names = (
        get_feature_names(features, "observation.state")
        if "observation.state" in features
        else []
    )
    obs_groups = list(dict.fromkeys(obs_group_name(name) for name in obs_state_names))
    if default_obs_group and default_obs_group not in obs_groups:
        fallback = obs_groups[0] if obs_groups else ""
        print(
            green(
                f"Observation group '{default_obs_group}' was not found. "
                f"Defaulting to '{fallback}'."
            )
        )
        default_obs_group = None
    active_obs_group = default_obs_group or (obs_groups[0] if obs_groups else None)
    data_rows = []
    logging.info("Loading dataloader")
    episode_sampler = EpisodeSampler(dataset, episode_index)
    dataloader = torch.utils.data.DataLoader(
        dataset,
        num_workers=num_workers,
        batch_size=batch_size,
        sampler=episode_sampler,
    )

    logging.info("Starting Rerun")

    if mode not in ["local", "distant"]:
        raise ValueError(mode)

    spawn_local_viewer = mode == "local" and not save
    rr.init(f"{repo_id}/episode_{episode_index}", spawn=spawn_local_viewer)

    # Work around a Rerun multiprocessing flush hang.
    gc.collect()

    if mode == "distant":
        rr.serve(open_browser=False, web_port=web_port, ws_port=ws_port)

    logging.info("Logging to Rerun")

    for batch in tqdm.tqdm(dataloader, total=len(dataloader)):
        for i in range(len(batch["index"])):
            rr.set_time("frame_index", sequence=batch["frame_index"][i].item())
            rr.set_time("timestamp", timestamp=batch["timestamp"][i].item())

            for key in dataset.meta.camera_keys:
                rr.log(key, rr.Image(to_hwc_uint8_numpy(batch[key][i])))

            if "task" in batch:
                rr.log("task", rr.TextLog(batch["task"][i]))

            if ACTION in batch:
                for dim_idx, val in enumerate(batch[ACTION][i]):
                    rr.log(
                        f"{ACTION}/{action_names[dim_idx]}",
                        rr.Scalars(val.item()),
                    )

            if OBS_STATE in batch:
                for dim_idx, val in enumerate(batch[OBS_STATE][i]):
                    rr.log(
                        f"observation.state/{obs_state_names[dim_idx]}",
                        rr.Scalars(val.item()),
                    )

            frame_index = int(batch["frame_index"][i].item())
            timestamp = float(batch["timestamp"][i].item())
            action_values = (
                tensor_row_to_named_dict(batch[ACTION][i], action_names)
                if ACTION in batch
                else {}
            )
            obs_values = (
                tensor_row_to_named_dict(batch[OBS_STATE][i], obs_state_names)
                if OBS_STATE in batch
                else {}
            )
            if data_window:
                data_rows.append(
                    {
                        "frame_index": frame_index,
                        "timestamp": timestamp,
                        "action": action_values,
                        "obs": obs_values,
                    }
                )

            if print_to_terminal:
                selected_obs = select_obs_group(obs_values, active_obs_group)
                line = (
                    f"[EP {episode_index} | frame {frame_index} | t={timestamp:.3f}] "
                    f"action: {format_named_values(action_values)}"
                )
                if selected_obs:
                    line += f" | obs: {format_named_values(selected_obs)}"
                print(line)

            if DONE in batch:
                rr.log(DONE, rr.Scalars(batch[DONE][i].item()))

            if REWARD in batch:
                rr.log(REWARD, rr.Scalars(batch[REWARD][i].item()))

            if "next.success" in batch:
                rr.log("next.success", rr.Scalars(batch["next.success"][i].item()))

    if data_window:
        html_path = write_data_window_html(
            repo_id=repo_id,
            episode_index=episode_index,
            rows=data_rows,
            obs_names=obs_state_names,
            default_obs_group=active_obs_group,
            output_dir=output_dir,
        )
        print(green(f"Episode data window: {html_path}"))
        webbrowser.open(html_path.as_uri())

    if mode == "local" and save:
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        repo_id_str = repo_id.replace("/", "_")
        rrd_path = output_dir / f"{repo_id_str}_episode_{episode_index}.rrd"
        rr.save(rrd_path)
        return rrd_path

    if mode == "distant":
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("Ctrl-C received. Exiting.")


def main():
    config_path = Path(__file__).parents[1] / "config" / "cfg.yaml"
    with config_path.open("r", encoding="utf-8") as file:
        cfg = yaml.safe_load(file)

    parser = argparse.ArgumentParser()
    visualize_cfg = cfg["visualize"]

    parser.add_argument(
        "--dataset-name",
        "--repo-id",
        dest="dataset_name",
        type=str,
        default=visualize_cfg.get("dataset_name", visualize_cfg.get("repo_id")),
        help="Name of the LeRobotDataset dataset (e.g. `lerobot/pusht`).",
    )
    parser.add_argument(
        "--episode-index",
        type=int,
        default=visualize_cfg["episode_index"],
        help="Episode to visualize.",
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=None,
        help="Optional local dataset root.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Directory path to write a .rrd file when `--save 1` is set.",
    )
    parser.add_argument(
        "--batch-size",
        type=int,
        default=32,
        help="Batch size loaded by DataLoader.",
    )
    parser.add_argument(
        "--num-workers",
        type=int,
        default=4,
        help="Number of processes of Dataloader for loading the data.",
    )
    parser.add_argument(
        "--mode",
        type=str,
        default="local",
        help=(
            "Use 'local' for a viewer or 'distant' for a Rerun web server."
        ),
    )
    parser.add_argument(
        "--web-port",
        type=int,
        default=9090,
        help="Web port for rerun.io when `--mode distant` is set.",
    )
    parser.add_argument(
        "--ws-port",
        type=int,
        default=9087,
        help="Web socket port for rerun.io when `--mode distant` is set.",
    )
    parser.add_argument(
        "--save",
        type=int,
        default=0,
        help=(
            "Save a .rrd file in the directory provided by `--output-dir`. "
            "It also deactivates the spawning of a viewer. "
            "Visualize the data by running `rerun path/to/file.rrd` on your local machine."
        ),
    )

    parser.add_argument(
        "--tolerance-s",
        type=float,
        default=1e-4,
        help="Dataset timestamp tolerance in seconds.",
    )
    parser.add_argument(
        "--default-obs-group",
        type=str,
        default=visualize_cfg.get("default_obs_group", None),
        help="Observation group selected by default, e.g. `tcp_pose` or `tcp_force`.",
    )
    parser.add_argument(
        "--data-window",
        type=int,
        default=int(visualize_cfg.get("data_window", True)),
        help="Open per-frame actions and observations in HTML.",
    )
    parser.add_argument(
        "--print-to-terminal",
        type=int,
        default=int(visualize_cfg.get("print_to_terminal", False)),
        help="Print per-frame action and selected observation data to the terminal.",
    )

    args = parser.parse_args()
    args.data_window = bool(args.data_window)
    args.print_to_terminal = bool(args.print_to_terminal)
    kwargs = vars(args)
    dataset_name = kwargs.pop("dataset_name")
    root = kwargs.pop("root")
    tolerance_s = kwargs.pop("tolerance_s")

    logging.info("Loading dataset")
    dataset = LeRobotDataset(
        dataset_name,
        episodes=[args.episode_index],
        root=root,
        tolerance_s=tolerance_s,
    )

    visualize_dataset(dataset, **kwargs)


if __name__ == "__main__":
    main()
