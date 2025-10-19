# lekiwi_gamepad
## Installation
init env
```
uv venv --python=python3.11
```

Install lerobot
```
uv pip install opencv-python-headless==4.11.0.86
uv pip install torch==2.7.0 --torch-backend=auto
uv pip install -e submodules/lerobot
```
Install feetch sdk
```
# For SO101
uv pip install submodules/lerobot[feetech]
```
Install kinematics:
```
uv pip install ../../submodules/lerobot[kinematics]
```
Install lekiwi GamePad Control
```
uv pip install -e submodules/lerobot-teleoperator-deltas-gamepad
```
