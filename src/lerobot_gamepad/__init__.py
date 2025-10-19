from pathlib import Path

# Path to the "robot_asset" directory inside this package
ASSET_ROOT = (Path(__file__).resolve().parent / "robot_asset").resolve()

__all__ = ["ASSET_ROOT"]
