"""
Mesh network model for DECT NR+ simulator.
Contains node classes, pairing state machine, and mesh topology.

Topology rules:
  - Gateway is the root (hop 0), always paired.
  - Anchors form a MESH — each anchor connects to ALL reachable
    paired anchors/gateway (up to MAX_ANCHORS peers).
  - Anchor hop = 1 + min(hop of connected peers).
  - Sensors connect to ONE parent (gateway or anchor), tree-style.

Pairing priority for anchors:
  1. Gateway (if in range and has capacity)
  2. Paired anchors with RSSI > -60 dBm (lowest hop first)
"""

import random
import time
from dataclasses import dataclass, field
from typing import Optional, Callable

from packet import (
    DeviceType, Version, PairRequest, PairResponse, PairConfirm,
    compute_pair_hash, device_type_str,
    PAIR_STATUS_SUCCESS, PAIR_STATUS_FAILURE,
)
from radio_sim import in_range, calc_rssi, packet_lost

# Mesh limits (matches firmware)
MAX_ANCHORS = 8
MAX_SENSORS = 16
RSSI_THRESHOLD = -60  # dBm — minimum RSSI for anchor-to-anchor link


class NodeState:
    IDLE = "idle"
    PAIRING = "pairing"
    PAIRED = "paired"


@dataclass
class PairedDevice:
    device_id: int = 0
    device_type: int = DeviceType.UNKNOWN
    version: Version = None

    def __post_init__(self):
        if self.version is None:
            self.version = Version()


@dataclass
class Node:
    device_id: int = 0
    device_type: int = DeviceType.UNKNOWN
    version: Version = None
    hop: int = 0
    x: float = 0.0
    y: float = 0.0
    state: str = NodeState.IDLE
    parent_id: Optional[int] = None
    # Mesh peers (anchors connect to multiple peers)
    paired_anchors: list = field(default_factory=list)
    paired_sensors: list = field(default_factory=list)
    # All mesh links (set of device_ids this node is connected to)
    mesh_links: set = field(default_factory=set)

    def __post_init__(self):
        if self.version is None:
            self.version = Version()
        if self.device_id == 0:
            self.device_id = random.randint(1000, 65000)

    @property
    def is_gateway(self):
        return self.device_type == DeviceType.GATEWAY

    @property
    def is_anchor(self):
        return self.device_type == DeviceType.ANCHOR

    @property
    def is_sensor(self):
        return self.device_type == DeviceType.SENSOR

    @property
    def can_accept_children(self):
        if self.is_gateway:
            return True
        if self.is_anchor and self.state == NodeState.PAIRED:
            return True
        return False

    @property
    def can_accept_anchor(self):
        return self.can_accept_children and len(self.paired_anchors) < MAX_ANCHORS

    @property
    def can_accept_sensor(self):
        return self.can_accept_children and len(self.paired_sensors) < MAX_SENSORS

    def has_link_to(self, device_id: int) -> bool:
        return device_id in self.mesh_links

    def add_link(self, peer_id: int):
        self.mesh_links.add(peer_id)

    def add_child(self, child: PairedDevice) -> bool:
        # Avoid duplicates
        if child.device_type == DeviceType.ANCHOR:
            if any(c.device_id == child.device_id for c in self.paired_anchors):
                return True
            if len(self.paired_anchors) < MAX_ANCHORS:
                self.paired_anchors.append(child)
                return True
        elif child.device_type == DeviceType.SENSOR:
            if any(c.device_id == child.device_id for c in self.paired_sensors):
                return True
            if len(self.paired_sensors) < MAX_SENSORS:
                self.paired_sensors.append(child)
                return True
        return False

    def __str__(self):
        return (f"{device_type_str(self.device_type)} "
                f"ID:{self.device_id} v{self.version} hop:{self.hop}")


class LogEntry:
    def __init__(self, timestamp: float, message: str, level: str = "INF"):
        self.timestamp = timestamp
        self.message = message
        self.level = level

    def __str__(self):
        t = time.strftime("%H:%M:%S", time.gmtime(self.timestamp))
        return f"[{t}] <{self.level.lower()}> {self.message}"


class MeshNetwork:
    """Manages all nodes and orchestrates pairing."""

    def __init__(self):
        self.nodes: list[Node] = []
        self.log_entries: list[LogEntry] = []
        self.start_time = time.time()
        self.on_log: Optional[Callable] = None
        self.on_topology_changed: Optional[Callable] = None
        self.pending_packets: list = []

    def _log(self, msg: str, level: str = "INF"):
        entry = LogEntry(time.time() - self.start_time, msg, level)
        self.log_entries.append(entry)
        if self.on_log:
            self.on_log(entry)

    def add_node(self, device_type: int, x: float, y: float,
                 version: Version = None) -> Node:
        node = Node(
            device_type=device_type,
            version=version or Version(),
            x=x, y=y,
        )
        if device_type == DeviceType.GATEWAY:
            node.hop = 0
            node.state = NodeState.PAIRED
        self.nodes.append(node)
        self._log(f"Added {node}")
        if self.on_topology_changed:
            self.on_topology_changed()
        return node

    def remove_node(self, node: Node):
        nid = node.device_id
        # Remove links and children referencing this node
        for n in self.nodes:
            n.mesh_links.discard(nid)
            n.paired_anchors = [c for c in n.paired_anchors
                                if c.device_id != nid]
            n.paired_sensors = [c for c in n.paired_sensors
                                if c.device_id != nid]
        # Unpair children
        for n in self.nodes:
            if n.parent_id == nid:
                n.parent_id = None
                n.state = NodeState.IDLE
                n.hop = 0
                n.mesh_links.clear()
        self.nodes.remove(node)
        self._recalculate_hops()
        self._log(f"Removed {device_type_str(node.device_type)} ID:{nid}")
        if self.on_topology_changed:
            self.on_topology_changed()

    def find_node(self, device_id: int) -> Optional[Node]:
        for n in self.nodes:
            if n.device_id == device_id:
                return n
        return None

    def get_nodes_in_range(self, node: Node) -> list[Node]:
        return [n for n in self.nodes
                if n.device_id != node.device_id
                and in_range(node.x, node.y, n.x, n.y)]

    def get_all_links(self) -> list[tuple[Node, Node]]:
        """Get all unique mesh links as (node_a, node_b) pairs."""
        seen = set()
        links = []
        for node in self.nodes:
            for peer_id in node.mesh_links:
                key = (min(node.device_id, peer_id),
                       max(node.device_id, peer_id))
                if key not in seen:
                    seen.add(key)
                    peer = self.find_node(peer_id)
                    if peer:
                        links.append((node, peer))
            # Also include sensor->parent links
            if node.is_sensor and node.parent_id is not None:
                parent = self.find_node(node.parent_id)
                if parent:
                    key = (min(node.device_id, node.parent_id),
                           max(node.device_id, node.parent_id))
                    if key not in seen:
                        seen.add(key)
                        links.append((node, parent))
        return links

    def _recalculate_hops(self):
        """Recalculate hop numbers for all anchors based on mesh links.
        hop = 1 + min(hop of connected peers that are paired)."""
        # Gateway is always hop 0
        # BFS from gateway through mesh links
        for n in self.nodes:
            if not n.is_gateway:
                n.hop = 99  # reset

        changed = True
        while changed:
            changed = False
            for n in self.nodes:
                if n.is_gateway or n.state != NodeState.PAIRED:
                    continue
                if n.is_sensor:
                    # Sensor hop = parent hop + 1
                    parent = self.find_node(n.parent_id)
                    if parent:
                        new_hop = parent.hop + 1
                        if new_hop != n.hop:
                            n.hop = new_hop
                            changed = True
                    continue

                # Anchor: hop = 1 + min(hop of mesh peers)
                min_peer_hop = 99
                for peer_id in n.mesh_links:
                    peer = self.find_node(peer_id)
                    if peer and peer.state == NodeState.PAIRED:
                        min_peer_hop = min(min_peer_hop, peer.hop)
                if min_peer_hop < 99:
                    new_hop = min_peer_hop + 1
                    if new_hop != n.hop:
                        n.hop = new_hop
                        changed = True

    def unpair_all(self):
        """Reset all nodes to unpaired state, keeping positions."""
        for n in self.nodes:
            if n.is_gateway:
                n.paired_anchors.clear()
                n.paired_sensors.clear()
                n.mesh_links.clear()
            else:
                n.state = NodeState.IDLE
                n.parent_id = None
                n.hop = 0
                n.paired_anchors.clear()
                n.paired_sensors.clear()
                n.mesh_links.clear()
        self._log("All nodes unpaired")
        if self.on_topology_changed:
            self.on_topology_changed()

    # ========== Pairing Protocol ==========

    def _create_mesh_link(self, node_a: Node, node_b: Node):
        """Create a bidirectional mesh link between two nodes."""
        node_a.add_link(node_b.device_id)
        node_b.add_link(node_a.device_id)

        # Add to paired lists (bidirectional)
        child_a = PairedDevice(
            device_id=node_a.device_id,
            device_type=node_a.device_type,
            version=node_a.version,
        )
        child_b = PairedDevice(
            device_id=node_b.device_id,
            device_type=node_b.device_type,
            version=node_b.version,
        )

        if node_a.is_anchor:
            node_b.add_child(child_a)
        if node_b.is_anchor:
            node_a.add_child(child_b)

    def start_pairing(self, node: Node) -> list:
        """
        Initiate pairing for a sensor or anchor.
        Anchors connect to ALL reachable paired nodes (mesh).
        Sensors connect to ONE best parent (tree).
        Returns animation steps.
        """
        if node.is_gateway:
            self._log("Gateway doesn't need pairing", "WRN")
            return []

        if node.is_sensor and node.state == NodeState.PAIRED:
            self._log(f"Sensor ID:{node.device_id} already paired", "WRN")
            return []

        node.state = NodeState.PAIRING
        rand_num = random.randint(0, 0xFFFFFFFF)
        expected_hash = compute_pair_hash(node.device_id, rand_num)

        pair_req = PairRequest(
            device_type=node.device_type,
            device_id=node.device_id,
            random_num=rand_num,
            version=node.version,
        )

        self._log(f"Pair request from {device_type_str(node.device_type)} "
                  f"ID:{node.device_id}")

        candidates = []
        animation_steps = []
        nearby = self.get_nodes_in_range(node)

        # Broadcast animation
        animation_steps.append({
            "type": "broadcast",
            "src": node,
            "packet": pair_req,
            "delay": 0,
        })

        # Collect responses from eligible nodes
        for n in nearby:
            if not n.can_accept_children:
                continue

            # Already linked?
            if node.is_anchor and node.has_link_to(n.device_id):
                continue

            # Check capacity
            if node.is_anchor and not n.can_accept_anchor:
                continue
            if node.is_sensor and not n.can_accept_sensor:
                continue

            # Packet loss
            if packet_lost(node.x, node.y, n.x, n.y):
                self._log(f"Pair request lost to ID:{n.device_id}", "WRN")
                continue

            rssi = calc_rssi(node.x, node.y, n.x, n.y)

            # RSSI threshold for anchor-to-anchor links
            if not n.is_gateway and rssi < RSSI_THRESHOLD:
                self._log(f"ID:{n.device_id} RSSI too weak ({rssi} dBm)", "WRN")
                continue

            pair_hash = compute_pair_hash(node.device_id, rand_num)

            resp = PairResponse(
                device_type=n.device_type,
                device_id=n.device_id,
                dst_device_id=node.device_id,
                hash=pair_hash,
                hop_num=n.hop,
            )

            candidates.append((n, resp, rssi))

            self._log(f"Pair response from {device_type_str(n.device_type)} "
                      f"ID:{n.device_id} hop:{n.hop} RSSI:{rssi}")

            animation_steps.append({
                "type": "unicast",
                "src": n,
                "dst": node,
                "packet": resp,
                "delay": 500,
            })

        if not candidates:
            self._log(f"No responses for ID:{node.device_id}", "WRN")
            node.state = NodeState.IDLE
            return animation_steps

        if node.is_sensor:
            # Sensor: pick ONE best parent (gateway > lowest hop > best RSSI)
            candidates.sort(
                key=lambda c: (0 if c[0].is_gateway else 1, c[0].hop, -c[2]))
            best_node, best_resp, best_rssi = candidates[0]

            if best_resp.hash != expected_hash:
                self._log(f"Hash mismatch from ID:{best_node.device_id}", "WRN")
                node.state = NodeState.IDLE
                return animation_steps

            self._log(f"Best: {device_type_str(best_node.device_type)} "
                      f"ID:{best_node.device_id} hop:{best_node.hop} "
                      f"RSSI:{best_rssi}")

            confirm = PairConfirm(
                device_type=node.device_type,
                device_id=node.device_id,
                dst_device_id=best_node.device_id,
                status=PAIR_STATUS_SUCCESS,
                version=node.version,
            )

            def on_sensor_confirm():
                node.parent_id = best_node.device_id
                node.hop = best_node.hop + 1
                node.state = NodeState.PAIRED
                child_info = PairedDevice(
                    device_id=node.device_id,
                    device_type=node.device_type,
                    version=node.version,
                )
                best_node.add_child(child_info)
                self._log(f"Paired: SENSOR ID:{node.device_id} -> "
                          f"{device_type_str(best_node.device_type)} "
                          f"ID:{best_node.device_id} (hop:{node.hop})")
                if self.on_topology_changed:
                    self.on_topology_changed()

            animation_steps.append({
                "type": "unicast",
                "src": node,
                "dst": best_node,
                "packet": confirm,
                "delay": 500,
                "callback": on_sensor_confirm,
            })

        else:
            # Anchor: connect to ALL valid candidates (mesh links)
            # Sort: gateway first, then lowest hop, then best RSSI
            candidates.sort(
                key=lambda c: (0 if c[0].is_gateway else 1, c[0].hop, -c[2]))

            # Limit to MAX_ANCHORS links total
            max_new = MAX_ANCHORS - len(node.mesh_links)
            selected = candidates[:max_new]

            for peer_node, resp, rssi in selected:
                if resp.hash != expected_hash:
                    continue

                confirm = PairConfirm(
                    device_type=node.device_type,
                    device_id=node.device_id,
                    dst_device_id=peer_node.device_id,
                    status=PAIR_STATUS_SUCCESS,
                    version=node.version,
                )

                # Capture variables for closure
                def make_callback(pn, r):
                    def on_anchor_confirm():
                        self._create_mesh_link(node, pn)
                        node.state = NodeState.PAIRED
                        self._recalculate_hops()
                        self._log(f"Mesh link: ANCHOR ID:{node.device_id} <-> "
                                  f"{device_type_str(pn.device_type)} "
                                  f"ID:{pn.device_id} "
                                  f"(hop:{node.hop})")
                        if self.on_topology_changed:
                            self.on_topology_changed()
                    return on_anchor_confirm

                animation_steps.append({
                    "type": "unicast",
                    "src": node,
                    "dst": peer_node,
                    "packet": confirm,
                    "delay": 300,
                    "callback": make_callback(peer_node, rssi),
                })

        return animation_steps

    def pair_all_unpaired(self) -> list:
        """Pair all unpaired nodes. Multi-round for mesh building."""
        all_steps = []

        # Multi-round anchor pairing
        max_rounds = 10
        for round_num in range(max_rounds):
            unpaired = [n for n in self.nodes
                        if n.is_anchor and n.state != NodeState.PAIRED]
            if not unpaired:
                break

            paired_count = 0
            for node in unpaired:
                steps = self.start_pairing(node)
                all_steps.extend(steps)
                if node.state == NodeState.PAIRED:
                    paired_count += 1

            if paired_count == 0:
                break

            self._log(f"Pairing round {round_num + 1}: "
                      f"{paired_count} anchors paired")

        # Second pass: already-paired anchors discover new mesh neighbors
        for node in self.nodes:
            if node.is_anchor and node.state == NodeState.PAIRED:
                nearby_paired = [
                    n for n in self.get_nodes_in_range(node)
                    if n.state == NodeState.PAIRED
                    and (n.is_gateway or n.is_anchor)
                    and not node.has_link_to(n.device_id)
                    and len(node.mesh_links) < MAX_ANCHORS
                    and calc_rssi(node.x, node.y, n.x, n.y) >= RSSI_THRESHOLD
                ]
                for peer in nearby_paired:
                    if not peer.can_accept_anchor:
                        continue
                    self._create_mesh_link(node, peer)
                    self._log(f"Mesh link: ID:{node.device_id} <-> "
                              f"ID:{peer.device_id}")
                    all_steps.append({
                        "type": "unicast",
                        "src": node,
                        "dst": peer,
                        "packet": PairConfirm(
                            device_type=node.device_type,
                            device_id=node.device_id,
                            dst_device_id=peer.device_id,
                            status=PAIR_STATUS_SUCCESS,
                            version=node.version,
                        ),
                        "delay": 200,
                    })

        self._recalculate_hops()

        # Then pair sensors
        for node in self.nodes:
            if node.is_sensor and node.state != NodeState.PAIRED:
                steps = self.start_pairing(node)
                all_steps.extend(steps)

        if self.on_topology_changed:
            self.on_topology_changed()

        return all_steps
