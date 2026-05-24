from collections import deque
from dataclasses import dataclass, field
from math import hypot
from typing import Deque


@dataclass
class PositionStabilityGate:
    sample_count_target: int
    stable_distance_m: float
    max_unstable_samples: int
    _samples: Deque[tuple[float, float]] = field(default_factory=deque, init=False)
    _unstable_samples: int = field(default=0, init=False)
    _accepted: bool = field(default=False, init=False)

    def update(self, x: float, y: float) -> bool:
        if self._accepted:
            return True

        current = (x, y)
        if len(self._samples) < self.sample_count_target:
            self._samples.append(current)
            return False

        stable = all(
            hypot(x - previous_x, y - previous_y) <= self.stable_distance_m
            for previous_x, previous_y in self._samples
        )
        self._samples.popleft()
        self._samples.append(current)

        if stable:
            self._accepted = True
            return True

        self._unstable_samples += 1
        if self._unstable_samples >= self.max_unstable_samples:
            self._accepted = True
            return True

        return False
