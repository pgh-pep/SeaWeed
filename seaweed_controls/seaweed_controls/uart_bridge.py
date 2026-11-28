import struct
import threading
import time
from collections.abc import Callable

import serial
from uart_bridge_state import REST, RX_PACKET_FIELDS, SOFT_ESTOP, UARTBridgeState

# packet_rx_t (jetson -> board)
# 8 bytes: state, fl, fr, bl, br, launch_action, launcher_angle (degrees)
RX_FORMAT: str = "<B4bBH"
RX_FIELDS: list[str] = ["state", "fl", "fr", "bl", "br", "launch_action", "launcher_angle"]

# packet_tx_t (board -> jetson)
# 49 bytes, following RX_PACKET_FIELDS
TX_FORMAT: str = "<I6iH3iBHi"
TX_SIZE: int = struct.calcsize(TX_FORMAT)

# enum names for logging, match common_types.h
STATE_NAMES: dict[int, str] = {1: "SOFT_ESTOP", 2: "MANUAL", 3: "AUTONOMOUS", 4: "STATIONKEEPING"}
ACTION_NAMES: dict[int, str] = {1: "REST", 2: "FIRE_CANNON", 3: "FIRE_PUMP"}


def enum_name(names: dict[int, str], v: int) -> str:
    return names.get(v, f"UNKNOWN({v})")


def format_tx(p: dict[str, int]) -> str:
    return (
        f"{enum_name(STATE_NAMES, p['state'])} | "
        f"thr fl {p['fl']:4d} fr {p['fr']:4d} bl {p['bl']:4d} br {p['br']:4d} | "
        f"{enum_name(ACTION_NAMES, p['launch_action'])} @ {p['launcher_angle']} deg"
    )


def format_rx(d: dict[str, int]) -> str:
    return (
        f"t {d['stamp_ms']} ms | {d['voltage'] / 1000:.2f} V {d['current'] / 1000:.2f} A | "
        f"gyro [{d['gyro_x']} {d['gyro_y']} {d['gyro_z']}] "
        f"accel [{d['accel_x']} {d['accel_y']} {d['accel_z']}]"
    )


def crc16(data: bytes, poly: int = 0x1021, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ poly if crc & 0x8000 else crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out, idx, code = bytearray([0]), 0, 1
    for i, b in enumerate(data):
        if b == 0:
            out[idx] = code
            idx = len(out)
            out.append(0)
            code = 1
        else:
            out.append(b)
            code += 1
            if code == 0xFF and i + 1 < len(data):
                out[idx] = code
                idx = len(out)
                out.append(0)
                code = 1
    out[idx] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes | None:
    out, i = bytearray(), 0
    while i < len(data):
        code = data[i]
        i += 1
        if code == 0 or i + code - 1 > len(data):
            return None
        out += data[i : i + code - 1]
        i += code - 1
        if code != 0xFF and i < len(data):
            out.append(0)
    return bytes(out)


def make_frame(counter: int, payload: bytes) -> bytes:
    raw = bytes([counter, len(payload)]) + payload
    raw += struct.pack("<H", crc16(raw))
    return cobs_encode(raw) + b"\x00"


def parse_frame(frame: bytes) -> tuple[int, bytes] | None:
    # return (counter, payload), none if frame invalid
    raw = cobs_decode(frame)
    if not raw or len(raw) < 4 or raw[1] != len(raw) - 4:
        return None
    if crc16(raw[:-2]) != struct.unpack("<H", raw[-2:])[0]:
        return None
    return raw[0], raw[2:-2]


class UARTBridge:
    # runs on seperate thread
    def __init__(
        self,
        port: str,
        state: UARTBridgeState | None = None,
        on_rx: Callable[[dict[str, int]], None] | None = None,
        timeout: float = 0.5,
        tx_period: float = 0.02,
        debug: bool = False,
        debug_period: float = 0.2,
        log: Callable[[str], object] = print,
    ) -> None:
        self.port: str = port
        self.state: UARTBridgeState = state or UARTBridgeState()
        self.on_rx: Callable[[dict[str, int]], None] | None = on_rx
        self.timeout: float = timeout
        self.tx_period: float = tx_period
        self.debug: bool = debug
        self.debug_period: float = debug_period
        self.log: Callable[[str], object] = log
        self.stale: list[str] = []  # inputs replaced by defaults in last payload
        self._stop: threading.Event = threading.Event()
        self.thread: threading.Thread = threading.Thread(target=self.run, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self._stop.set()
        self.thread.join(timeout=2.0)

    def get_payload(self) -> bytes:
        s, now = self.state, time.time()

        def fresh(t: float | None) -> bool:
            return t is not None and (self.timeout <= 0 or now - t <= self.timeout)

        def clamp(v: float) -> int:
            return max(-128, min(127, int(v)))

        with s.lock:
            st: int = s.state if fresh(s.state_time) else SOFT_ESTOP
            th: tuple[int, int, int, int] = (s.fl, s.fr, s.bl, s.br) if fresh(s.thruster_time) else (0, 0, 0, 0)
            if fresh(s.launch_time):
                la, ang = s.launch_action, s.launcher_angle
            else:
                la, ang = REST, 0
            # track which inputs fell back to defaults, for logging
            self.stale = [
                name
                for name, t in (("state", s.state_time), ("thrusters", s.thruster_time), ("launch", s.launch_time))
                if not fresh(t)
            ]
        return struct.pack(RX_FORMAT, st, *map(clamp, th), la, int(ang) % 360)

    def run(self) -> None:
        while not self._stop.is_set():
            try:
                with serial.Serial(self.port, 115200, timeout=0.002) as ser:
                    with self.state.lock:
                        self.state.connected = True
                    self._loop(ser)
            except (serial.SerialException, OSError):
                pass
            with self.state.lock:
                self.state.connected = False
            self._stop.wait(1.0)  # retry

    def _loop(self, ser: serial.Serial) -> None:
        s = self.state
        buf: bytearray = bytearray()
        counter: int = 0
        last_tx: float = 0.0
        last_dbg: float = 0.0
        sent: dict[str, int] | None = None  # store latest packet for logging
        recv: dict[str, int] | None = None
        while not self._stop.is_set():
            now = time.monotonic()
            if now - last_tx >= self.tx_period:
                payload = self.get_payload()
                ser.write(make_frame(counter, payload))
                if self.debug:
                    sent = dict(zip(RX_FIELDS, struct.unpack(RX_FORMAT, payload)))
                counter = (counter + 1) & 0xFF
                last_tx = now
                with s.lock:
                    s.tx_count += 1

            # returns when any byte arrives (or after timeout)
            buf += ser.read(max(1, ser.in_waiting))
            while 0 in buf:
                i = buf.index(0)
                frame = bytes(buf[:i])
                del buf[: i + 1]
                if not frame:
                    continue
                r = parse_frame(frame)
                if r is None or len(r[1]) != TX_SIZE:
                    with s.lock:
                        s.rx_bad += 1
                    continue
                ctr, rxget_payload = r
                vals: dict[str, int] = dict(zip(RX_PACKET_FIELDS, struct.unpack(TX_FORMAT, rxget_payload)))
                recv = vals
                with s.lock:
                    for k, v in vals.items():
                        setattr(s, k, v)
                    s.rx_counter = ctr
                    s.rx_time = time.time()
                    s.rx_ok += 1
                if self.on_rx:
                    try:
                        self.on_rx(vals)
                    except Exception:  # noqa: BLE001, S110
                        pass  # dont kill bridge regardless?

            if self.debug and now - last_dbg >= self.debug_period:
                last_dbg = now
                with s.lock:
                    age = "NEVER" if s.rx_time is None else f"{time.time() - s.rx_time:.2f}s ago"
                    stale = ", ".join(self.stale) if self.stale else "none"
                    self.log(
                        f"\n--- tx {s.tx_count} | rx ok {s.rx_ok} bad {s.rx_bad} "
                        f"| last rx ctr {s.rx_counter} ({age})\n"
                        f"  sent : {format_tx(sent) if sent else '(nothing yet)'}\n"
                        f"  stale: {stale}\n"
                        f"  recv : {format_rx(recv) if recv else '(nothing yet)'}"
                    )
