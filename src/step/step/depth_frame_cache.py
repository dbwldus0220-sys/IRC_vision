"""Share the newest aligned-depth frame between vision analyzers."""

from __future__ import annotations

import threading
import time

from cv_bridge import CvBridge
import numpy as np
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image


class DepthFrameCache:
    """Thread-safe holder for one immutable, most-recent depth frame."""

    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.image: np.ndarray | None = None
        self.received_at: float | None = None
        self.width: int | None = None
        self.height: int | None = None

    def update(
        self,
        image: np.ndarray,
        *,
        width: int,
        height: int,
        received_at: float | None = None,
    ) -> None:
        """Atomically replace the cached frame and its metadata."""
        with self.lock:
            self.image = image
            self.received_at = (
                time.monotonic() if received_at is None else received_at
            )
            self.width = width
            self.height = height


class DepthFrameConsumer:
    """Compatibility properties for analyzers backed by a depth cache."""

    depth_cache: DepthFrameCache

    def _get_depth_cache(self) -> DepthFrameCache:
        """Lazily support analyzer helpers constructed without ROS setup."""
        cache = getattr(self, "depth_cache", None)
        if cache is None:
            cache = DepthFrameCache()
            self.depth_cache = cache
        return cache

    @property
    def latest_depth_image(self) -> np.ndarray | None:
        cache = self._get_depth_cache()
        with cache.lock:
            return cache.image

    @latest_depth_image.setter
    def latest_depth_image(self, value: np.ndarray | None) -> None:
        cache = self._get_depth_cache()
        with cache.lock:
            cache.image = value

    @property
    def latest_depth_time(self) -> float | None:
        cache = self._get_depth_cache()
        with cache.lock:
            return cache.received_at

    @latest_depth_time.setter
    def latest_depth_time(self, value: float | None) -> None:
        cache = self._get_depth_cache()
        with cache.lock:
            cache.received_at = value

    @property
    def latest_image_width(self) -> int | None:
        cache = self._get_depth_cache()
        with cache.lock:
            return cache.width

    @latest_image_width.setter
    def latest_image_width(self, value: int | None) -> None:
        cache = self._get_depth_cache()
        with cache.lock:
            cache.width = value

    @property
    def latest_image_height(self) -> int | None:
        cache = self._get_depth_cache()
        with cache.lock:
            return cache.height

    @latest_image_height.setter
    def latest_image_height(self, value: int | None) -> None:
        cache = self._get_depth_cache()
        with cache.lock:
            cache.height = value

    def _store_depth_message(
        self,
        message: Image,
        bridge: CvBridge,
    ) -> None:
        depth = bridge.imgmsg_to_cv2(
            message,
            desired_encoding="passthrough",
        )
        self.depth_cache.update(
            np.asarray(depth),
            width=int(message.width),
            height=int(message.height),
        )


class SharedDepthSubscriber(Node):
    """Convert aligned depth once for all analyzers in the unified process."""

    def __init__(self, topic: str, cache: DepthFrameCache) -> None:
        super().__init__("vision_depth_cache")
        self._cache = cache
        self._bridge = CvBridge()
        self._callback_group = MutuallyExclusiveCallbackGroup()
        latest_depth_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Image,
            topic,
            self._depth_callback,
            latest_depth_qos,
            callback_group=self._callback_group,
        )
        self.get_logger().info(f"Sharing aligned depth: {topic}")

    def _depth_callback(self, message: Image) -> None:
        try:
            depth = self._bridge.imgmsg_to_cv2(
                message,
                desired_encoding="passthrough",
            )
            self._cache.update(
                np.asarray(depth),
                width=int(message.width),
                height=int(message.height),
            )
        except Exception as exc:
            self.get_logger().warning(f"Could not read depth image: {exc}")
