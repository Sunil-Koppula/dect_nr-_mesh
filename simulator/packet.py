"""
Packet definitions for DECT NR+ mesh network simulator.
Mirrors the C packet.h structures.
"""

from dataclasses import dataclass
from enum import IntEnum


class DeviceType(IntEnum):
    UNKNOWN = 0x00
    GATEWAY = 0x01
    ANCHOR = 0x02
    SENSOR = 0x03


class PacketType(IntEnum):
    PAIR_REQUEST = 0x01
    PAIR_RESPONSE = 0x02
    PAIR_CONFIRM = 0x03
    DATA = 0x04
    DATA_ACK = 0x05
    OTA_INIT = 0x0C
    OTA_ACK = 0x0D


# Pairing status codes
PAIR_STATUS_SUCCESS = 0x00
PAIR_STATUS_FAILURE = 0x01


@dataclass
class Version:
    major: int = 1
    minor: int = 0
    patch: int = 0

    def __str__(self):
        return f"{self.major}.{self.minor}.{self.patch}"

    def is_newer_than(self, other: "Version") -> bool:
        if self.major != other.major:
            return self.major > other.major
        if self.minor != other.minor:
            return self.minor > other.minor
        return self.patch > other.patch


@dataclass
class PairRequest:
    packet_type: int = PacketType.PAIR_REQUEST
    device_type: int = DeviceType.UNKNOWN
    device_id: int = 0
    random_num: int = 0
    version: Version = None

    def __post_init__(self):
        if self.version is None:
            self.version = Version()


@dataclass
class PairResponse:
    packet_type: int = PacketType.PAIR_RESPONSE
    device_type: int = DeviceType.UNKNOWN
    device_id: int = 0
    dst_device_id: int = 0
    hash: int = 0
    hop_num: int = 0


@dataclass
class PairConfirm:
    packet_type: int = PacketType.PAIR_CONFIRM
    device_type: int = DeviceType.UNKNOWN
    device_id: int = 0
    dst_device_id: int = 0
    status: int = PAIR_STATUS_SUCCESS
    version: Version = None

    def __post_init__(self):
        if self.version is None:
            self.version = Version()


def compute_pair_hash(device_id: int, random_num: int) -> int:
    """Simple hash for pairing verification."""
    h = device_id ^ random_num
    h = ((h << 13) ^ h) & 0xFFFFFFFF
    h = (h * 0x5BD1E995) & 0xFFFFFFFF
    return h


def device_type_str(dt: int) -> str:
    return {
        DeviceType.GATEWAY: "GATEWAY",
        DeviceType.ANCHOR: "ANCHOR",
        DeviceType.SENSOR: "SENSOR",
    }.get(dt, "UNKNOWN")
