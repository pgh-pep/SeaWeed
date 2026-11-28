from collections import deque


class UARTClockSync:
    def __init__(self, window_s: float = 5.0, bucket_s: float = 0.5) -> None:
        self.bucket_ns: int = int(bucket_s * 1e9)
        self.mins: deque[int] = deque(maxlen=int(window_s / bucket_s))
        self.cur_min: int | None = None
        self._cur_start: int = 0
        self.wraps: int = 0
        self.prev_raw: int | None = None
        self.prev_out: int = 0

    def to_host_ns(self, stamp_ms: int, host_ns: int) -> int:
        # handle wrap and resetting
        if self.prev_raw is not None:
            if stamp_ms < self.prev_raw - 2**31:
                self.wraps += 1 << 32
            elif stamp_ms < self.prev_raw - 1000:
                self.reset()  # assume board reboot
        self.prev_raw = stamp_ms

        board_ns = (stamp_ms + self.wraps) * 1_000_000
        sample = host_ns - board_ns

        # track sliding window per bucket
        if self.cur_min is None:
            self.cur_min, self._cur_start = sample, host_ns
        else:
            self.cur_min = min(self.cur_min, sample)
            if host_ns - self._cur_start >= self.bucket_ns:
                self.mins.append(self.cur_min)
                self.cur_min, self._cur_start = None, host_ns

        candidates: list[int] = list(self.mins)
        if self.cur_min is not None:
            candidates.append(self.cur_min)
        offset: int = min(candidates)
        # ensure stamps are non-decreasing
        out = max(board_ns + offset, self.prev_out)
        self.prev_out = out
        return out

    def reset(self) -> None:
        self.mins.clear()
        self.cur_min = None
        self.wraps = 0
        self.prev_out = 0
