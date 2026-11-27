import threading

# NOTE: this should be redone, needs a better standardized method
# ex ros msg with defined enums or some like that
SOFT_ESTOP, MANUAL, AUTONOMOUS, STATIONKEEPING = 1, 2, 3, 4
REST, FIRE_CANNON, FIRE_PUMP = 1, 2, 3

# IN WIRE ORDER:
RX_PACKET_FIELDS = [
    "stamp_ms",
    "gyro_x",
    "gyro_y",
    "gyro_z",
    "accel_x",
    "accel_y",
    "accel_z",
    "imu_status",
    "mag_x",
    "mag_y",
    "mag_z",
    "mag_status",
    "voltage",
    "current",
]


class UARTBridgeState:
    def __init__(self):
        self.lock = threading.Lock()

        # jetson to board
        self.state: int = SOFT_ESTOP
        self.state_time: float | None = None
        self.fl: int = 0
        self.fr: int = 0
        self.bl: int = 0
        self.br: int = 0
        self.thruster_time: float | None = None
        self.launch_action: int = REST
        self.launcher_angle: int = 0
        self.launch_time: float | None = None

        # board to jetson
        self.stamp_ms: int | None = None
        self.gyro_x: int | None = None
        self.gyro_y: int | None = None
        self.gyro_z: int | None = None
        self.accel_x: int | None = None
        self.accel_y: int | None = None
        self.accel_z: int | None = None
        self.imu_status: int | None = None
        self.mag_x: int | None = None
        self.mag_y: int | None = None
        self.mag_z: int | None = None
        self.mag_status: int | None = None
        self.voltage: int | None = None
        self.current: int | None = None
        self.rx_time: float | None = None

        # uart link status
        self.connected: bool = False
        self.tx_count: int = 0
        self.rx_ok: int = 0
        self.rx_bad: int = 0
        self.rx_counter: int | None = None
