# 🧩 Troubleshooting `/dev/video*` Camera Errors

If you encounter an error like this when running `lerobot-find-cameras` or any OpenCV-based script:

```

ERROR: Failed to connect or configure OpenCV camera /dev/video2: OpenCVCamera(/dev/video2) read failed (status=False)

````

it means OpenCV could not access or properly configure your camera device.

---

## ✅ Step 1. Check Available Camera Devices

Run this command to see all connected video devices:

```bash
v4l2-ctl --list-devices
````

Example output:

```
USB 2.0 Camera: USB Camera (usb-0000:00:14.0-6):
    /dev/video0
    /dev/video2
```

Note which `/dev/video*` device you want to use.

---

## ✅ Step 2. Check Supported Formats

Once you know the correct device, check its supported formats:

```bash
v4l2-ctl --list-formats-ext -d /dev/video2
```

Example output:

```
ioctl: VIDIOC_ENUM_FMT
        Type: Video Capture

        [0]: 'MJPG' (Motion-JPEG, compressed)
                Size: Discrete 1920x1080
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 640x480
                        Interval: Discrete 0.033s (30.000 fps)
        [1]: 'YUYV' (YUYV 4:2:2)
                Size: Discrete 640x480
                        Interval: Discrete 0.033s (30.000 fps)
```

If your camera supports **MJPG**, it’s usually more stable than **YUYV**.
To explicitly set MJPG mode in OpenCV, add:

```python
self.videocapture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
```

---

## ⚙️ Step 3. Fix Common Camera Issues

| Problem                       | Cause                                   | Solution                                                             |
| ----------------------------- | --------------------------------------- | -------------------------------------------------------------------- |
| `read failed (status=False)`  | Unsupported or mismatched camera format | Use MJPG format                                                      |
| No `/dev/video*` device found | Driver or permission issue              | Run `ls /dev/video*` to check detection                              |
| Permission denied             | Current user lacks access               | Add yourself to the `video` group:<br>`sudo usermod -aG video $USER` |

---

## 🧪 Step 4. Quick Camera Test (Optional)

You can verify camera access with this short Python script:

```python
import cv2

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

if not cap.isOpened():
    print("❌ Failed to open camera.")
else:
    print("✅ Camera opened successfully.")
    ret, frame = cap.read()
    if ret:
        print("✅ Frame captured successfully.")
        cv2.imshow("Camera Test", frame)
        cv2.waitKey(0)
    else:
        print("⚠️ Failed to read frame.")
    cap.release()
    cv2.destroyAllWindows()
```

---

If the test works, your camera setup is correct.
If not, recheck your formats with `v4l2-ctl --list-formats-ext` and adjust accordingly.

## 🧪 Step 5. Override Camera Settings

If your camera is not fully compatible with the default `lerobot` OpenCV camera—e.g., it only supports MJPG format—you can create a custom camera class to override the default behavior.

1. **Create a custom camera file**
   Go to `src/lerobot_gamepad/cameras/` and create a new Python file, e.g., `opencv_camera_MJPG.py`.
   You can refer to the existing example in `src/lerobot_gamepad/cameras/opencv_camera_MJPG.py`.

2. **Install your changes in editable mode**
   From the root of this repository, run:

   ```bash
   uv pip install -e .
   ```

3. **Import and use your custom camera**
   For example:

   ```python
   from lerobot_gamepad.cameras.opencv_camera_MJPG import OpenCVCameraMJPG
   from lerobot.cameras.opencv import OpenCVCameraConfig

   config = OpenCVCameraConfig(index_or_path="/dev/video2", width=640, height=480, fps=30)
   camera = OpenCVCameraMJPG(config)
   camera.connect()
   frame = camera.read()
   camera.disconnect()
   ```

This way, you can enforce settings like MJPG format or other camera-specific configurations without modifying the original site-packages code.

### Step 6. Override Robot Settings

1. **Create a custom robot file**
   Go to `src/lerobot_gamepad/robots/` and create a new Python file, e.g., `so1010_follower_with_MJPG_camera.py`.
   You can refer to the existing example in `src/lerobot_gamepad/cameras/so1010_follower_with_MJPG_camera.py`.

2. **Install your changes in editable mode**
   From the root of this repository, run:

   ```bash
   uv pip install -e .
   ```