"""
Simulated radio environment for DECT NR+ mesh network.
Calculates RSSI from distance and determines packet delivery.
"""

import math
import random


# Radio parameters
BROADCAST_RANGE = 300       # pixels — max range for packet delivery
RSSI_AT_1M = -30            # dBm at 1 meter (reference)
PATH_LOSS_EXPONENT = 2.5    # free-space ~2, indoor ~2.5-3.5
PIXELS_PER_METER = 30       # canvas scale

# Packet loss increases with distance
LOSS_RATE_MIN = 0.0         # at close range
LOSS_RATE_MAX = 0.3         # at max range


def distance(x1: float, y1: float, x2: float, y2: float) -> float:
    """Euclidean distance in pixels."""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def distance_meters(x1: float, y1: float, x2: float, y2: float) -> float:
    """Distance in meters (from pixel coordinates)."""
    return distance(x1, y1, x2, y2) / PIXELS_PER_METER


def calc_rssi(x1: float, y1: float, x2: float, y2: float) -> int:
    """Calculate RSSI in dBm based on distance between two nodes."""
    d = distance_meters(x1, y1, x2, y2)
    if d < 0.1:
        d = 0.1  # avoid log(0)
    rssi = RSSI_AT_1M - 10 * PATH_LOSS_EXPONENT * math.log10(d)
    return int(rssi)


def in_range(x1: float, y1: float, x2: float, y2: float) -> bool:
    """Check if two nodes are within broadcast range."""
    return distance(x1, y1, x2, y2) <= BROADCAST_RANGE


def packet_lost(x1: float, y1: float, x2: float, y2: float) -> bool:
    """Determine if a packet is lost based on distance (probabilistic)."""
    d = distance(x1, y1, x2, y2)
    if d > BROADCAST_RANGE:
        return True
    # Linear interpolation of loss rate
    ratio = d / BROADCAST_RANGE
    loss_rate = LOSS_RATE_MIN + (LOSS_RATE_MAX - LOSS_RATE_MIN) * ratio
    return random.random() < loss_rate
