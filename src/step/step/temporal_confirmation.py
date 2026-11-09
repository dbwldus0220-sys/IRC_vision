#!/usr/bin/env python3
"""Reusable temporal and spatial confirmation for vision detections."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math


def depth_is_within_range(
    depth_valid: bool,
    depth_m: float | None,
    maximum_depth_m: float,
) -> bool:
    """Return whether a fresh object depth is inside an inclusive gate."""
    if not depth_valid or depth_m is None or isinstance(depth_m, bool):
        return False
    try:
        depth = float(depth_m)
        maximum = float(maximum_depth_m)
    except (TypeError, ValueError):
        return False
    return bool(
        math.isfinite(depth)
        and math.isfinite(maximum)
        and depth > 0.0
        and maximum > 0.0
        and depth <= maximum
    )


@dataclass(frozen=True)
class ConfirmationResult:
    """State returned after processing one video frame."""

    raw_detected: bool
    confirmed: bool
    matched_previous: bool
    hit_count: int
    required_hits: int
    window_size: int
    missed_frames: int
    track_id: int | None = None
    spatial_pending: bool = False

    def as_dict(self, prefix: str = "confirmation") -> dict[str, object]:
        """Return JSON-friendly diagnostic fields."""
        raw_key = (
            "raw_detected"
            if prefix == "confirmation"
            else f"{prefix}_raw_detected"
        )
        fields = {
            raw_key: self.raw_detected,
            f"{prefix}_confirmed": self.confirmed,
            f"{prefix}_matched_previous": self.matched_previous,
            f"{prefix}_hits": self.hit_count,
            f"{prefix}_required_hits": self.required_hits,
            f"{prefix}_window_size": self.window_size,
            f"{prefix}_missed_frames": self.missed_frames,
        }
        if self.track_id is not None:
            fields[f"{prefix}_track_id"] = self.track_id
            fields[f"{prefix}_spatial_pending"] = self.spatial_pending
        return fields


class TemporalConfirmationFilter:
    """Confirm repeated, spatially consistent detections in a frame window.

    The filter can also confirm a plain boolean condition by omitting ``bbox``.
    A confirmed track tolerates a small number of misses internally, but the
    returned ``confirmed`` flag is false on a frame with no current detection.
    This prevents a motion planner from acting on stale image geometry while
    allowing a reacquired target to recover without starting from zero.
    """

    EDGE_MARGIN_PX = 2
    EDGE_MAX_CENTER_SHIFT_NORM = 0.05
    EDGE_MIN_OVERLAP_RATIO = 0.5

    def __init__(
        self,
        *,
        window_size: int,
        required_hits: int,
        max_missed_frames: int = 2,
        max_center_shift_norm: float = 0.18,
        min_area_ratio: float = 0.40,
        spatial_matching: bool = True,
        edge_area_grace: bool = False,
    ) -> None:
        self.window_size = max(1, int(window_size))
        self.required_hits = max(
            1,
            min(self.window_size, int(required_hits)),
        )
        self.max_missed_frames = max(0, int(max_missed_frames))
        self.max_center_shift_norm = max(0.0, float(max_center_shift_norm))
        self.min_area_ratio = min(1.0, max(0.0, float(min_area_ratio)))
        self.spatial_matching = bool(spatial_matching)
        self.edge_area_grace = bool(edge_area_grace)
        self._track_id = 0
        self._edge_bbox = None

        self._hits: deque[bool] = deque(maxlen=self.window_size)
        self._last_bbox: tuple[float, float, float, float] | None = None
        self._missed_frames = 0
        self._track_confirmed = False

    def reset(self) -> None:
        """Forget the current candidate and all temporal history."""
        self._track_id += 1
        self._edge_bbox = None
        self._hits.clear()
        self._last_bbox = None
        self._missed_frames = 0
        self._track_confirmed = False

    def update(
        self,
        detected: bool,
        *,
        bbox: list[int] | tuple[int, int, int, int] | None = None,
        image_width: int | None = None,
        image_height: int | None = None,
        allow_edge_recovery: bool = True,
    ) -> ConfirmationResult:
        """Process one frame and return the current confirmation state."""
        current_bbox = self._normalize_bbox(bbox)
        matched = bool(detected)
        edge_recovered = False
        if self._edge_bbox is not None:
            # Only the next observation may resolve the uncertain area change.
            edge_recovered = bool(detected and current_bbox is not None
                and allow_edge_recovery and (
                    self._matches_previous(current_bbox, image_width, image_height)
                    or (self._matches_previous(current_bbox, image_width, image_height,
                                               previous_bbox=self._edge_bbox)
                        and self._edge_area_change(current_bbox, image_width, image_height))))
            if not edge_recovered:
                self.reset()
            self._edge_bbox = None
        if detected and self.spatial_matching:
            matched = edge_recovered or (current_bbox is not None and self._matches_previous(
                current_bbox,
                image_width,
                image_height,
            ))
            if not matched:
                if (self.edge_area_grace and allow_edge_recovery and not self._track_confirmed
                        and current_bbox is not None
                        and self._edge_area_change(current_bbox, image_width, image_height)):
                    self._edge_bbox = current_bbox
                else:
                    self.reset()
                    matched = current_bbox is not None

        if detected and matched:
            self._hits.append(True)
            self._missed_frames = 0
            if current_bbox is not None:
                self._last_bbox = current_bbox
            if sum(self._hits) >= self.required_hits:
                self._track_confirmed = True
        else:
            self._hits.append(False)
            self._missed_frames += 1
            if self._missed_frames > self.max_missed_frames:
                self.reset()

        confirmed_now = bool(
            detected and matched and self._track_confirmed
        )
        return ConfirmationResult(
            raw_detected=bool(detected),
            confirmed=confirmed_now,
            matched_previous=matched,
            hit_count=sum(self._hits),
            required_hits=self.required_hits,
            window_size=self.window_size,
            missed_frames=self._missed_frames,
            track_id=self._track_id if self.edge_area_grace else None,
            spatial_pending=self._edge_bbox is not None,
        )

    @staticmethod
    def _normalize_bbox(
        bbox: list[int] | tuple[int, int, int, int] | None,
    ) -> tuple[float, float, float, float] | None:
        if bbox is None or len(bbox) != 4:
            return None
        left, top, right, bottom = (float(value) for value in bbox)
        if not all(math.isfinite(value) for value in (left, top, right, bottom)):
            return None
        if right <= left or bottom <= top:
            return None
        return left, top, right, bottom

    def _matches_previous(
        self,
        bbox: tuple[float, float, float, float],
        image_width: int | None,
        image_height: int | None,
        *,
        previous_bbox: tuple[float, float, float, float] | None = None,
    ) -> bool:
        previous_bbox = self._last_bbox if previous_bbox is None else previous_bbox
        if previous_bbox is None:
            return True
        left, top, right, bottom = bbox
        old_left, old_top, old_right, old_bottom = previous_bbox
        center_x = (left + right) / 2.0
        center_y = (top + bottom) / 2.0
        old_center_x = (old_left + old_right) / 2.0
        old_center_y = (old_top + old_bottom) / 2.0
        diagonal = math.hypot(
            max(1, int(image_width or 0)),
            max(1, int(image_height or 0)),
        )
        center_shift = math.hypot(
            center_x - old_center_x,
            center_y - old_center_y,
        ) / diagonal
        area = (right - left) * (bottom - top)
        old_area = (old_right - old_left) * (old_bottom - old_top)
        area_ratio = min(area, old_area) / max(area, old_area, 1.0)
        return (
            center_shift <= self.max_center_shift_norm
            and area_ratio >= self.min_area_ratio
        )

    def _edge_area_change(self, bbox, width, height) -> bool:
        """Allow one uncertain frame only for overlapping, nearby edge boxes."""
        old = self._last_bbox
        if old is None or not width or not height or min(width, height) <= 0:
            return False
        if any(not (0 <= b[0] < b[2] <= width and 0 <= b[1] < b[3] <= height)
               for b in (old, bbox)):
            return False
        margin = self.EDGE_MARGIN_PX
        if not any(b[0] <= margin or b[1] <= margin
                   or b[2] >= width - margin or b[3] >= height - margin
                   for b in (old, bbox)):
            return False
        areas = [(b[2] - b[0]) * (b[3] - b[1]) for b in (old, bbox)]
        overlap = (max(0., min(old[2], bbox[2]) - max(old[0], bbox[0]))
                   * max(0., min(old[3], bbox[3]) - max(old[1], bbox[1])))
        shift = math.hypot((old[0] + old[2] - bbox[0] - bbox[2]) / 2,
                           (old[1] + old[3] - bbox[1] - bbox[3]) / 2) / math.hypot(width, height)
        return (min(areas) / max(areas) < self.min_area_ratio
                and overlap / min(areas) >= self.EDGE_MIN_OVERLAP_RATIO
                and shift <= min(self.max_center_shift_norm, self.EDGE_MAX_CENTER_SHIFT_NORM))
