from lerobot.cameras.opencv import OpenCVCamera
import cv2
import platform
import logging

logger = logging.getLogger(__name__)

class OpenCVCameraMJPG(OpenCVCamera):
    """
    Subclass of OpenCVCamera that forces MJPG format on Linux.
    """

    def _configure_capture_settings(self) -> None:
        """
        Override the original function to force MJPG on Linux,
        then call the original configuration.
        """
        if self.is_connected and platform.system() == "Linux":
            # Force MJPG before setting resolution/FPS
            fourcc = cv2.VideoWriter_fourcc(*"MJPG")
            self.videocapture.set(cv2.CAP_PROP_FOURCC, fourcc)
            actual_fourcc = int(self.videocapture.get(cv2.CAP_PROP_FOURCC))
            actual_str = "".join(
                [chr((actual_fourcc >> 8 * i) & 0xFF) for i in range(4)]
            )
            logger.info(f"{self} actual FOURCC after forcing MJPG: {actual_str}")

        # Call the original configuration method for FPS/width/height
        super()._configure_capture_settings()
