"""
Dims: 6 
Right -> Left [-2, 2]; unbounded?

"""

import cv2, numpy as np, threading, time
from contextlib import AbstractContextManager
from pathlib import Path
from ur_interface.ur import UR

class CameraRecorder(AbstractContextManager):
    """
    Context manager that records an RTSP stream in the background.
      • Saves an .mp4 *and/or* stacked NumPy array on exit.
      • Continues recording while you do arbitrary work inside the with‑block.
    """

    def __init__(
        self,
        src_url: str,
        out_mp4: str | None = "capture.mp4",
        save_npy: str | None = None,
        max_frames: int | None = None,
    ):
        self.src_url, self.out_mp4, self.save_npy, self.max_frames = (
            src_url,
            out_mp4,
            save_npy,
            max_frames,
        )
        self._stop = threading.Event()
        self._frames: list[np.ndarray] = []

    # ── context‑manager interface ──────────────────────────────────────────
    def __enter__(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self                    # you could expose helpers here if desired

    def __exit__(self, exc_type, exc, tb):
        self._stop.set()
        self._thread.join()
        if self.save_npy and self._frames:
            np.save(self.save_npy, np.stack(self._frames))
        # return False → propagate any exception from inside the with‑block
        return False

    # ── background capture loop ───────────────────────────────────────────
    def _run(self):
        cap = cv2.VideoCapture(self.src_url)
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open stream: {self.src_url}")

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        fps = cap.get(cv2.CAP_PROP_FPS) or 30
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        writer = (
            cv2.VideoWriter(self.out_mp4, fourcc, fps, (w, h))
            if self.out_mp4
            else None
        )

        i = 0
        while not self._stop.is_set():
            ret, frame = cap.read()
            if not ret:
                break
            if writer:
                writer.write(frame)
            if self.save_npy:
                self._frames.append(frame.copy())
            i += 1
            if self.max_frames and i >= self.max_frames:
                break
            # optional live preview
            # cv2.imshow("feed", frame); cv2.waitKey(1)

        cap.release()
        if writer:
            writer.release()
        cv2.destroyAllWindows()
        
robot = UR("146.137.240.38")

with CameraRecorder(
        src_url="rtsp://146.137.240.5:554/s0",
        out_mp4="video_capture.mp4",
        save_npy="video_frames.npy",
) as rec:

    # start pose
    robot.home([0, -1.57, 0, 0, 0, 0])
    # mid pose 
    robot.home([1.57, -1.75, 0, 1.57, 0, 0])
    # end pose 
    robot.home([1.57, -1.57, 0, 0, 0, 0])
    
robot.disconnect()