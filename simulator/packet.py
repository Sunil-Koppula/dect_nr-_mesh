"""
Packet definitions for DECT NR+ mesh network simulator.
Mirrors src/protocol.h exactly.
"""

from dataclasses import dataclass
from enum import IntEnum


# === Device Type (mirrors device_type_t) ===

class DeviceType(IntEnum):
    UNKNOWN = 0x00
    GATEWAY = 0x01
    ANCHOR = 0x02
    SENSOR = 0x03


# === Packet Types (mirrors packet_type_t) ===

class PacketType(IntEnum):
    PAIR_REQUEST = 0x01
    PAIR_RESPONSE = 0x02
    PAIR_CONFIRM = 0x03
    DATA = 0x04
    DATA_ACK = 0x05
    PARENT_QUERY = 0x0F
    PARENT_RESPONSE = 0x10
    REPAIR = 0x11
    SET_RSSI = 0x12


# === Status Codes (mirrors STATUS_*) ===

STATUS_SUCCESS = 0x00
STATUS_FAILURE = 0x01
STATUS_CRC_FAIL = 0x02


# === Version ===

@dataclass
class Version:
    major: int = 1
    minor: int = 0
    patch: int = 0

    def __str__(self):
        return f"{self.major}.{self.minor}.{self.patch}"


# === Pair Request (mirrors pair_request_packet_t — 12 bytes) ===

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


# === Pair Response (mirrors pair_response_packet_t — 15 bytes) ===

@dataclass
class PairResponse:
    packet_type: int = PacketType.PAIR_RESPONSE
    device_type: int = DeviceType.UNKNOWN
    device_id: int = 0          # responder's ID
    dst_device_id: int = 0      # requester's ID
    hash: int = 0
    hop_num: int = 0
    version: Version = None     # responder's version

    def __post_init__(self):
        if self.version is None:
            self.version = Version()


# === Pair Confirm (mirrors pair_confirm_packet_t — 11 bytes) ===

@dataclass
class PairConfirm:
    packet_type: int = PacketType.PAIR_CONFIRM
    device_type: int = DeviceType.UNKNOWN
    device_id: int = 0          # confirmer's ID
    dst_device_id: int = 0      # responder's ID
    status: int = STATUS_SUCCESS
    version: Version = None     # confirmer's version

    def __post_init__(self):
        if self.version is None:
            self.version = Version()


# === Data Packet (mirrors data_packet_t) ===

@dataclass
class DataPacket:
    packet_type: int = PacketType.DATA
    src_device_id: int = 0
    dst_device_id: int = 0
    payload_len: int = 0


# === Data ACK (mirrors data_ack_packet_t — 7 bytes) ===

@dataclass
class DataAck:
    packet_type: int = PacketType.DATA_ACK
    src_device_id: int = 0
    dst_device_id: int = 0
    hop_num: int = 0
    status: int = STATUS_SUCCESS


# === Hash (mirrors compute_pair_hash in mesh.c) ===

@dataclass
class ParentQuery:
    packet_type: int = PacketType.PARENT_QUERY
    src_device_id: int = 0
    dst_device_id: int = 0
    target_id: int = 0  # 0 = all, else specific device


@dataclass
class ParentResponse:
    packet_type: int = PacketType.PARENT_RESPONSE
    src_device_id: int = 0
    dst_device_id: int = 0
    device_type: int = DeviceType.UNKNOWN
    parent_id: int = 0
    parent_type: int = DeviceType.UNKNOWN
    hop_num: int = 0


@dataclass
class RepairPacket:
    packet_type: int = PacketType.REPAIR
    src_device_id: int = 0
    dst_device_id: int = 0


@dataclass
class SetRssiPacket:
    packet_type: int = PacketType.SET_RSSI
    src_device_id: int = 0
    dst_device_id: int = 0
    rssi_dbm: int = -75


def compute_pair_hash(device_id: int, random_num: int) -> int:
    h = device_id ^ random_num
    h = (((h << 13) | (h >> 19)) ^ (h * 0x5BD1E995)) & 0xFFFFFFFF
    return h


def device_type_str(dt: int) -> str:
    return {
        DeviceType.GATEWAY: "GATEWAY",
        DeviceType.ANCHOR: "ANCHOR",
        DeviceType.SENSOR: "SENSOR",
    }.get(dt, "UNKNOWN")
