import logging
from pathlib import Path

import torch
import yaml
from tqdm import tqdm

from lerobot.datasets.lerobot_dataset import LeRobotDataset

logging.basicConfig(level=logging.INFO)


class EpisodeSampler(torch.utils.data.Sampler):
    """Iterate over one episode."""

    def __init__(self, dataset: LeRobotDataset, episode_index: int):
        from_idx = dataset.meta.episodes["dataset_from_index"][episode_index]
        to_idx = dataset.meta.episodes["dataset_to_index"][episode_index]
        self.frame_ids = range(from_idx, to_idx)

    def __iter__(self):
        return iter(self.frame_ids)

    def __len__(self):
        return len(self.frame_ids)


def check_dataset(repo_id: str):
    logging.info("Loading dataset metadata only")
    dataset = LeRobotDataset(repo_id, episodes=None)

    num_episodes = len(dataset.meta.episodes["dataset_from_index"])
    logging.info(f"Found {num_episodes} episodes in dataset {repo_id}")

    errors = []
    success_count = 0

    for episode_index in tqdm(range(num_episodes), desc="Episodes"):
        ep_display = episode_index + 1

        sampler = EpisodeSampler(dataset, episode_index)
        dataloader = torch.utils.data.DataLoader(
            dataset, batch_size=1, sampler=sampler
        )

        frame_idx = 0

        try:
            for _ in tqdm(
                dataloader,
                total=len(sampler),
                desc=f"Episode {ep_display}/{num_episodes} | total frames {len(sampler)}",
                leave=False,
            ):
                frame_idx += 1

            success_count += 1

        except Exception as e:
            errors.append(
                {
                    "episode_idx": episode_index,
                    "episode_display": ep_display,
                    "frame": frame_idx,
                    "error": str(e),
                }
            )

    print("\n" + "=" * 50)
    print("DATASET CHECK SUMMARY")
    print("=" * 50)
    print(f"Total episodes: {num_episodes}")
    print(f"Success: {success_count}")
    print(f"Failed: {len(errors)}")

    if errors:
        print("\nFailed episodes detail:")
        for err in errors:
            print(
                f"  - Episode {err['episode_display']} "
                f"(idx={err['episode_idx']}) | "
                f"Frame {err['frame']} | Error: {err['error']}"
            )


def main():
    parent_path = Path(__file__).resolve().parent
    cfg_path = parent_path.parent / "config" / "cfg.yaml"

    with cfg_path.open("r", encoding="utf-8") as file:
        cfg = yaml.safe_load(file)

    repo_id = cfg["check_dataset"]["dataset_name"]
    check_dataset(repo_id)


if __name__ == "__main__":
    main()
