"""
Mesh network model for DECT NR+ simulator.
Mirrors the firmware logic exactly:

Gateway (gateway.c):
  - hop 0, always paired
  - Responds to pair requests from anchors/sensors
  - handle_pair_confirm: stores anchors in anchor_store, sensors in sensor_store
  - handle_data: ACK back, no relay (gateway is final destination)

Anchor (anchor.c):
  - True mesh — pairs with ALL reachable gateways/anchors (device_store, max 8)
  - Paired sensors stored in sensor_store (max 16)
  - anchor_do_mesh_pairing: broadcast pair_request, collect responses,
    discovery_sort_mesh (gateway/min-hop first, filter RSSI > -75 dBm),
    send pair_confirm to ALL valid, store in device_store
  - handle_pair_confirm: gateway/anchor -> device_store + neighbor_add;
    sensor -> sensor_store
  - handle_data: ACK sender, relay to ALL devices in device_store (skip sender)
  - Rediscovery: every 10 min until device_store full (8 slots)

Sensor (sensor.c):
  - Tree topology — picks ONE best parent (gateway > lowest hop > best RSSI)
  - sensor_send_data: send to parent, wait for ACK

Mesh (mesh.c):
  - discovery_sort_mesh: sort by gateway first, then min hop, then best RSSI;
    filter anchors below -75 dBm
  - discovery_best: gateway wins > lower hop > better RSSI (used by sensor)
  - neighbor_best_route: gateway wins > lower hop > better RSSI
  - discovery_add_response: reject sensors as candidates for anchors/sensors
"""

import random
import time
from dataclasses import dataclass, field
from typing import Optional, Callable

from packet import (
    DeviceType, Version, PairRequest, PairResponse, PairConfirm,
    DataPacket, DataAck,
    compute_pair_hash, device_type_str,
    STATUS_SUCCESS, STATUS_CRC_FAIL,
)
from radio_sim import in_range, calc_rssi, packet_lost

# === Firmware constants ===
MAX_DEVICES = 8       # NVS_ANCHOR_MAX — anchor's device_store (gateways + anchors)
MAX_SENSORS = 16      # NVS_SENSOR_MAX
MAX_CANDIDATES = 8    # MAX_CANDIDATES in mesh.h
MAX_NEIGHBORS = 8     # MAX_NEIGHBORS in mesh.h
RSSI_THRESHOLD = -75  # MESH_RSSI_THRESHOLD_2 / 2 in mesh.h
PAIR_RETRY_MAX = 5    # anchor.c / sensor.c


class NodeState:
    IDLE = "idle"
    PAIRING = "pairing"
    PAIRED = "paired"


@dataclass
class PairedDevice:
    """Mirrors paired_device_info_t in identity.h"""
    device_id: int = 0
    device_type: int = DeviceType.UNKNOWN
    version: Version = None

    def __post_init__(self):
        if self.version is None:
            self.version = Version()


@dataclass
class Node:
    """Represents a device in the mesh network."""
    device_id: int = 0
    device_type: int = DeviceType.UNKNOWN
    version: Version = None
    hop: int = 0
    x: float = 0.0
    y: float = 0.0
    state: str = NodeState.IDLE

    # --- Gateway stores (gateway.c) ---
    # anchor_store: paired anchors (max 8)
    # sensor_store: paired sensors (max 16)

    # --- Anchor stores (anchor.c) ---
    # device_store: gateways + anchors combined (max 8)
    # sensor_store: paired sensors (max 16)

    # --- Sensor ---
    # parent_id: single parent (tree)

    parent_id: Optional[int] = None      # sensor only
    paired_devices: list = field(default_factory=list)   # device_store (anchor) / anchor_store (gateway)
    paired_sensors: list = field(default_factory=list)    # sensor_store
    mesh_links: set = field(default_factory=set)          # bidirectional link set

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
        """Gateway or paired anchor can accept children."""
        if self.is_gateway:
            return True
        if self.is_anchor and self.state == NodeState.PAIRED:
            return True
        return False

    @property
    def device_store_full(self):
        return len(self.paired_devices) >= MAX_DEVICES

    @property
    def sensor_store_full(self):
        return len(self.paired_sensors) >= MAX_SENSORS

    def has_link_to(self, device_id: int) -> bool:
        return device_id in self.mesh_links

    def add_link(self, peer_id: int):
        self.mesh_links.add(peer_id)

    def device_store_contains(self, device_id: int) -> bool:
        """Mirrors paired_store_contains(&device_store, id)"""
        return any(d.device_id == device_id for d in self.paired_devices)

    def device_store_add(self, dev: PairedDevice) -> bool:
        """Mirrors paired_store_add(&device_store, ...)"""
        # Update if already exists
        for d in self.paired_devices:
            if d.device_id == dev.device_id:
                d.version = dev.version
                return True
        if len(self.paired_devices) < MAX_DEVICES:
            self.paired_devices.append(dev)
            return True
        return False

    def sensor_store_add(self, dev: PairedDevice) -> bool:
        """Mirrors paired_store_add(&sensor_store, ...)"""
        for d in self.paired_sensors:
            if d.device_id == dev.device_id:
                d.version = dev.version
                return True
        if len(self.paired_sensors) < MAX_SENSORS:
            self.paired_sensors.append(dev)
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
    """Manages all nodes and orchestrates pairing protocol."""

    def __init__(self):
        self.nodes: list[Node] = []
        self.log_entries: list[LogEntry] = []
        self.start_time = time.time()
        self.on_log: Optional[Callable] = None
        self.on_topology_changed: Optional[Callable] = None

    def _log(self, msg: str, level: str = "INF"):
        entry = LogEntry(time.time() - self.start_time, msg, level)
        self.log_entries.append(entry)
        if self.on_log:
            self.on_log(entry)

    # ========== Node management ==========

    def add_node(self, device_type: int, x: float, y: float,
                 version: Version = None) -> Node:
        node = Node(device_type=device_type, version=version or Version(),
                    x=x, y=y)
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
        for n in self.nodes:
            n.mesh_links.discard(nid)
            n.paired_devices = [d for d in n.paired_devices if d.device_id != nid]
            n.paired_sensors = [d for d in n.paired_sensors if d.device_id != nid]
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
        seen = set()
        links = []
        for node in self.nodes:
            for peer_id in node.mesh_links:
                key = (min(node.device_id, peer_id), max(node.device_id, peer_id))
                if key not in seen:
                    seen.add(key)
                    peer = self.find_node(peer_id)
                    if peer:
                        links.append((node, peer))
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
        """BFS hop recalculation. Mirrors _recalculate_hops in firmware."""
        for n in self.nodes:
            if not n.is_gateway:
                n.hop = 99
        changed = True
        while changed:
            changed = False
            for n in self.nodes:
                if n.is_gateway or n.state != NodeState.PAIRED:
                    continue
                if n.is_sensor:
                    parent = self.find_node(n.parent_id)
                    if parent and parent.hop + 1 != n.hop:
                        n.hop = parent.hop + 1
                        changed = True
                    continue
                # Anchor: hop = 1 + min(hop of mesh peers)
                min_peer_hop = 99
                for pid in n.mesh_links:
                    peer = self.find_node(pid)
                    if peer and peer.state == NodeState.PAIRED:
                        min_peer_hop = min(min_peer_hop, peer.hop)
                if min_peer_hop < 99 and min_peer_hop + 1 != n.hop:
                    n.hop = min_peer_hop + 1
                    changed = True

    def unpair_all(self):
        for n in self.nodes:
            if n.is_gateway:
                n.paired_devices.clear()
                n.paired_sensors.clear()
                n.mesh_links.clear()
            else:
                n.state = NodeState.IDLE
                n.parent_id = None
                n.hop = 0
                n.paired_devices.clear()
                n.paired_sensors.clear()
                n.mesh_links.clear()
        self._log("All nodes unpaired")
        if self.on_topology_changed:
            self.on_topology_changed()

    # ========== Discovery (mirrors mesh.c) ==========

    def _discovery_add_response(self, candidates, requester, responder, rssi):
        """Mirrors discovery_add_response in mesh.c.
        Rejects sensors as candidates for anchors/sensors."""
        if len(candidates) >= MAX_CANDIDATES:
            return
        if ((requester.is_sensor or requester.is_anchor)
                and responder.is_sensor):
            return
        candidates.append((responder, rssi))

    def _discovery_sort_mesh(self, candidates):
        """Mirrors discovery_sort_mesh in mesh.c.
        Sort: gateway first > min hop > best RSSI.
        Filter: discard anchors below RSSI_THRESHOLD."""
        # Sort
        candidates.sort(key=lambda c: (
            0 if c[0].is_gateway else 1,
            c[0].hop,
            -c[1],  # higher RSSI is better
        ))
        # Filter: gateways always kept, anchors need RSSI >= threshold
        valid = []
        for node, rssi in candidates:
            if node.is_gateway:
                valid.append((node, rssi))
            elif rssi >= RSSI_THRESHOLD:
                valid.append((node, rssi))
        return valid

    def _discovery_best(self, candidates):
        """Mirrors discovery_best in mesh.c.
        Used by sensor: gateway wins > lower hop > better RSSI."""
        if not candidates:
            return None
        best_node, best_rssi = candidates[0]
        for node, rssi in candidates[1:]:
            # Gateway always wins
            if node.is_gateway and not best_node.is_gateway:
                best_node, best_rssi = node, rssi
                continue
            if best_node.is_gateway and not node.is_gateway:
                continue
            # Lower hop
            if node.hop < best_node.hop:
                best_node, best_rssi = node, rssi
                continue
            if node.hop > best_node.hop:
                continue
            # Better RSSI
            if rssi > best_rssi:
                best_node, best_rssi = node, rssi
        return best_node, best_rssi

    # ========== Gateway handle_pair_confirm (gateway.c:76-115) ==========

    def _gateway_handle_pair_confirm(self, gateway, pkt_type, pkt_id, pkt_version):
        """Gateway stores anchors in anchor_store (paired_devices),
        sensors in sensor_store."""
        if pkt_type == DeviceType.SENSOR:
            gateway.sensor_store_add(PairedDevice(
                device_id=pkt_id, device_type=pkt_type, version=pkt_version))
        elif pkt_type == DeviceType.ANCHOR:
            gateway.device_store_add(PairedDevice(
                device_id=pkt_id, device_type=pkt_type, version=pkt_version))

    # ========== Anchor handle_pair_confirm (anchor.c:253-298) ==========

    def _anchor_handle_pair_confirm(self, anchor, pkt_type, pkt_id, pkt_version):
        """Anchor: gateway/anchor -> device_store + neighbor_add;
        sensor -> sensor_store."""
        if pkt_type == DeviceType.GATEWAY:
            anchor.device_store_add(PairedDevice(
                device_id=pkt_id, device_type=pkt_type, version=pkt_version))
            anchor.add_link(pkt_id)
        elif pkt_type == DeviceType.ANCHOR:
            anchor.device_store_add(PairedDevice(
                device_id=pkt_id, device_type=pkt_type, version=pkt_version))
            anchor.add_link(pkt_id)
        elif pkt_type == DeviceType.SENSOR:
            anchor.sensor_store_add(PairedDevice(
                device_id=pkt_id, device_type=pkt_type, version=pkt_version))

    # ========== Mesh link creation ==========

    def _create_mesh_link(self, node_a: Node, node_b: Node):
        """Bidirectional mesh link + paired_devices on both sides."""
        node_a.add_link(node_b.device_id)
        node_b.add_link(node_a.device_id)
        node_a.device_store_add(PairedDevice(
            device_id=node_b.device_id, device_type=node_b.device_type,
            version=node_b.version))
        node_b.device_store_add(PairedDevice(
            device_id=node_a.device_id, device_type=node_a.device_type,
            version=node_a.version))

    # ========== Anchor pairing (anchor.c:67-232) ==========

    def _anchor_do_mesh_pairing(self, anchor) -> list:
        """Mirrors anchor_do_mesh_pairing exactly.
        Broadcast pair_request, collect responses, sort/filter,
        pair with ALL valid candidates."""
        animation_steps = []
        rand_num = random.randint(0, 0xFFFFFFFF)
        expected_hash = compute_pair_hash(anchor.device_id, rand_num)

        # Broadcast pair request
        self._log(f"Pair request from ANCHOR ID:{anchor.device_id}")
        animation_steps.append({
            "type": "broadcast", "src": anchor,
            "packet": PairRequest(device_type=anchor.device_type,
                                  device_id=anchor.device_id,
                                  random_num=rand_num, version=anchor.version),
            "delay": 0,
        })

        # Collect responses from nearby paired nodes
        candidates = []
        for n in self.get_nodes_in_range(anchor):
            if not n.can_accept_children:
                continue
            if anchor.has_link_to(n.device_id):
                continue
            if n.device_store_full:
                continue
            if packet_lost(anchor.x, anchor.y, n.x, n.y):
                self._log(f"Pair request lost to ID:{n.device_id}", "WRN")
                continue

            rssi = calc_rssi(anchor.x, anchor.y, n.x, n.y)
            self._discovery_add_response(candidates, anchor, n, rssi)

            if not n.is_gateway and rssi < RSSI_THRESHOLD:
                candidates = [(nn, r) for nn, r in candidates if nn.device_id != n.device_id]
                self._log(f"ID:{n.device_id} RSSI too weak ({rssi} dBm)", "WRN")
                continue

            hash_val = compute_pair_hash(anchor.device_id, rand_num)
            self._log(f"Pair response from {device_type_str(n.device_type)} "
                      f"ID:{n.device_id} hop:{n.hop} RSSI:{rssi}")
            animation_steps.append({
                "type": "unicast", "src": n, "dst": anchor,
                "packet": PairResponse(device_type=n.device_type,
                                       device_id=n.device_id,
                                       dst_device_id=anchor.device_id,
                                       hash=hash_val, hop_num=n.hop,
                                       version=n.version),
                "delay": 500,
            })

        if not candidates:
            self._log(f"No responses for ANCHOR ID:{anchor.device_id}", "WRN")
            anchor.state = NodeState.IDLE
            return animation_steps

        # discovery_sort_mesh
        valid = self._discovery_sort_mesh(candidates)
        if not valid:
            self._log(f"No candidates above RSSI threshold", "WRN")
            anchor.state = NodeState.IDLE
            return animation_steps

        # Limit to MAX_DEVICES
        max_new = MAX_DEVICES - len(anchor.paired_devices)
        selected = valid[:max_new]

        # Pair with ALL valid candidates
        for peer_node, rssi in selected:
            pair_hash = compute_pair_hash(anchor.device_id, rand_num)
            if pair_hash != expected_hash:
                continue

            def make_callback(pn):
                def cb():
                    self._create_mesh_link(anchor, pn)
                    anchor.state = NodeState.PAIRED
                    self._recalculate_hops()
                    self._log(f"Mesh link: ANCHOR ID:{anchor.device_id} <-> "
                              f"{device_type_str(pn.device_type)} "
                              f"ID:{pn.device_id} (hop:{anchor.hop})")
                    if self.on_topology_changed:
                        self.on_topology_changed()
                return cb

            animation_steps.append({
                "type": "unicast", "src": anchor, "dst": peer_node,
                "packet": PairConfirm(device_type=anchor.device_type,
                                      device_id=anchor.device_id,
                                      dst_device_id=peer_node.device_id,
                                      status=STATUS_SUCCESS,
                                      version=anchor.version),
                "delay": 300,
                "callback": make_callback(peer_node),
            })

        return animation_steps

    # ========== Anchor rediscovery (anchor.c:450-553) ==========

    def _anchor_rediscovery(self, anchor) -> list:
        """Mirrors anchor_rediscovery — single attempt, skip already paired."""
        if anchor.device_store_full:
            return []

        animation_steps = []
        rand_num = random.randint(0, 0xFFFFFFFF)
        expected_hash = compute_pair_hash(anchor.device_id, rand_num)

        self._log(f"Rediscovery: ANCHOR ID:{anchor.device_id} "
                  f"({len(anchor.paired_devices)}/{MAX_DEVICES} slots)")
        animation_steps.append({
            "type": "broadcast", "src": anchor,
            "packet": PairRequest(device_type=anchor.device_type,
                                  device_id=anchor.device_id,
                                  random_num=rand_num, version=anchor.version),
            "delay": 0,
        })

        candidates = []
        for n in self.get_nodes_in_range(anchor):
            if not n.can_accept_children:
                continue
            if anchor.device_store_contains(n.device_id):
                continue
            if packet_lost(anchor.x, anchor.y, n.x, n.y):
                continue
            rssi = calc_rssi(anchor.x, anchor.y, n.x, n.y)
            if not n.is_gateway and rssi < RSSI_THRESHOLD:
                continue
            self._discovery_add_response(candidates, anchor, n, rssi)

        valid = self._discovery_sort_mesh(candidates)
        for peer_node, rssi in valid:
            if anchor.device_store_full:
                break
            pair_hash = compute_pair_hash(anchor.device_id, rand_num)
            if pair_hash != expected_hash:
                continue

            def make_cb(pn):
                def cb():
                    self._create_mesh_link(anchor, pn)
                    self._recalculate_hops()
                    self._log(f"Rediscovery: ANCHOR ID:{anchor.device_id} <-> "
                              f"ID:{pn.device_id}")
                    if self.on_topology_changed:
                        self.on_topology_changed()
                return cb

            animation_steps.append({
                "type": "unicast", "src": anchor, "dst": peer_node,
                "packet": PairConfirm(device_type=anchor.device_type,
                                      device_id=anchor.device_id,
                                      dst_device_id=peer_node.device_id,
                                      status=STATUS_SUCCESS,
                                      version=anchor.version),
                "delay": 200,
                "callback": make_cb(peer_node),
            })

        return animation_steps

    # ========== Sensor pairing (sensor.c:34-138) ==========

    def _sensor_do_pairing(self, sensor) -> list:
        """Mirrors sensor_do_pairing exactly.
        Pick ONE best parent: gateway > lowest hop > best RSSI."""
        animation_steps = []
        rand_num = random.randint(0, 0xFFFFFFFF)
        expected_hash = compute_pair_hash(sensor.device_id, rand_num)

        self._log(f"Pair request from SENSOR ID:{sensor.device_id}")
        animation_steps.append({
            "type": "broadcast", "src": sensor,
            "packet": PairRequest(device_type=sensor.device_type,
                                  device_id=sensor.device_id,
                                  random_num=rand_num, version=sensor.version),
            "delay": 0,
        })

        candidates = []
        for n in self.get_nodes_in_range(sensor):
            if not n.can_accept_children:
                continue
            if n.sensor_store_full:
                continue
            if packet_lost(sensor.x, sensor.y, n.x, n.y):
                self._log(f"Pair request lost to ID:{n.device_id}", "WRN")
                continue
            rssi = calc_rssi(sensor.x, sensor.y, n.x, n.y)
            self._discovery_add_response(candidates, sensor, n, rssi)
            hash_val = compute_pair_hash(sensor.device_id, rand_num)

            self._log(f"Pair response from {device_type_str(n.device_type)} "
                      f"ID:{n.device_id} hop:{n.hop} RSSI:{rssi}")
            animation_steps.append({
                "type": "unicast", "src": n, "dst": sensor,
                "packet": PairResponse(device_type=n.device_type,
                                       device_id=n.device_id,
                                       dst_device_id=sensor.device_id,
                                       hash=hash_val, hop_num=n.hop,
                                       version=n.version),
                "delay": 500,
            })

        if not candidates:
            self._log(f"No responses for SENSOR ID:{sensor.device_id}", "WRN")
            sensor.state = NodeState.IDLE
            return animation_steps

        # discovery_best: gateway > lower hop > better RSSI
        best_node, best_rssi = self._discovery_best(candidates)
        pair_hash = compute_pair_hash(sensor.device_id, rand_num)
        if pair_hash != expected_hash:
            self._log(f"Hash mismatch from ID:{best_node.device_id}", "WRN")
            sensor.state = NodeState.IDLE
            return animation_steps

        self._log(f"Best: {device_type_str(best_node.device_type)} "
                  f"ID:{best_node.device_id} hop:{best_node.hop} RSSI:{best_rssi}")

        def on_confirm():
            sensor.parent_id = best_node.device_id
            sensor.hop = best_node.hop + 1
            sensor.state = NodeState.PAIRED
            # Parent stores sensor (gateway: sensor_store; anchor: sensor_store)
            best_node.sensor_store_add(PairedDevice(
                device_id=sensor.device_id, device_type=sensor.device_type,
                version=sensor.version))
            self._log(f"Paired: SENSOR ID:{sensor.device_id} -> "
                      f"{device_type_str(best_node.device_type)} "
                      f"ID:{best_node.device_id} (hop:{sensor.hop})")
            if self.on_topology_changed:
                self.on_topology_changed()

        animation_steps.append({
            "type": "unicast", "src": sensor, "dst": best_node,
            "packet": PairConfirm(device_type=sensor.device_type,
                                  device_id=sensor.device_id,
                                  dst_device_id=best_node.device_id,
                                  status=STATUS_SUCCESS, version=sensor.version),
            "delay": 500,
            "callback": on_confirm,
        })

        return animation_steps

    # ========== Public pairing API ==========

    def start_pairing(self, node: Node) -> list:
        if node.is_gateway:
            self._log("Gateway doesn't need pairing", "WRN")
            return []
        if node.is_sensor and node.state == NodeState.PAIRED:
            self._log(f"Sensor ID:{node.device_id} already paired", "WRN")
            return []

        node.state = NodeState.PAIRING
        if node.is_anchor:
            return self._anchor_do_mesh_pairing(node)
        else:
            return self._sensor_do_pairing(node)

    @staticmethod
    def _execute_callbacks(steps: list):
        for step in steps:
            cb = step.get("callback")
            if cb:
                cb()

    def pair_all_unpaired(self) -> list:
        """Multi-round pairing. Mirrors firmware boot sequence."""
        all_steps = []

        # Round 1+: pair unpaired anchors
        for round_num in range(10):
            unpaired = [n for n in self.nodes
                        if n.is_anchor and n.state != NodeState.PAIRED]
            if not unpaired:
                break
            paired_count = 0
            for node in unpaired:
                steps = self.start_pairing(node)
                self._execute_callbacks(steps)
                all_steps.extend(steps)
                if node.state == NodeState.PAIRED:
                    paired_count += 1
            if paired_count == 0:
                break
            self._log(f"Pairing round {round_num + 1}: {paired_count} anchors paired")

        # Rediscovery pass: already-paired anchors find new mesh neighbors
        for node in self.nodes:
            if node.is_anchor and node.state == NodeState.PAIRED:
                steps = self._anchor_rediscovery(node)
                self._execute_callbacks(steps)
                all_steps.extend(steps)

        self._recalculate_hops()

        # Pair sensors
        for node in self.nodes:
            if node.is_sensor and node.state != NodeState.PAIRED:
                steps = self.start_pairing(node)
                self._execute_callbacks(steps)
                all_steps.extend(steps)

        if self.on_topology_changed:
            self.on_topology_changed()
        return all_steps

    # ========== Data send/relay (matches firmware exactly) ==========

    def _anchor_handle_data(self, anchor, sender_id, animation_steps, visited):
        """Mirrors anchor.c handle_data: ACK sender, relay to ALL paired
        devices (skip sender). Each receiving anchor also runs handle_data
        (mesh flood with visited set to prevent loops)."""
        for dev in anchor.paired_devices:
            if dev.device_id == sender_id:
                continue  # don't relay back to sender
            if dev.device_id in visited:
                continue  # already received this data
            relay_node = self.find_node(dev.device_id)
            if not relay_node:
                continue

            visited.add(dev.device_id)

            self._log(f"Data relayed: ANCHOR ID:{anchor.device_id} -> "
                      f"{device_type_str(dev.device_type)} ID:{dev.device_id}")
            animation_steps.append({
                "type": "unicast", "src": anchor, "dst": relay_node,
                "packet": DataPacket(src_device_id=anchor.device_id,
                                     dst_device_id=dev.device_id),
                "delay": 800,
            })

            # If relay target is an anchor, it also runs handle_data
            # (relay to ALL its paired devices, skip the sender)
            if relay_node.is_anchor:
                self._anchor_handle_data(relay_node, anchor.device_id,
                                         animation_steps, visited)

    def simulate_data_send(self, sensor: Node) -> list:
        """Mirrors sensor_send_data (sensor.c:145-201) +
        anchor handle_data (anchor.c:300-387) — mesh flood +
        gateway handle_data (gateway.c:117-157) — ACK only."""
        if not sensor.is_sensor or sensor.state != NodeState.PAIRED:
            self._log(f"Sensor ID:{sensor.device_id} not paired", "WRN")
            return []

        parent = self.find_node(sensor.parent_id)
        if not parent:
            self._log(f"Sensor ID:{sensor.device_id} parent not found", "ERR")
            return []

        animation_steps = []

        # sensor_send_data: send DATA to parent
        self._log(f"Data from SENSOR ID:{sensor.device_id} -> "
                  f"{device_type_str(parent.device_type)} ID:{parent.device_id}")
        animation_steps.append({
            "type": "unicast", "src": sensor, "dst": parent,
            "packet": DataPacket(src_device_id=sensor.device_id,
                                 dst_device_id=parent.device_id),
            "delay": 0,
        })

        # Parent sends DATA_ACK back to sensor
        self._log(f"Data ACK: {device_type_str(parent.device_type)} "
                  f"ID:{parent.device_id} -> SENSOR ID:{sensor.device_id}")
        animation_steps.append({
            "type": "unicast", "src": parent, "dst": sensor,
            "packet": DataAck(src_device_id=parent.device_id,
                              dst_device_id=sensor.device_id,
                              hop_num=parent.hop, status=STATUS_SUCCESS),
            "delay": 1000,
        })

        # Anchor: relay to ALL paired devices, each anchor continues the flood
        # Gateway: ACK only, no relay (final destination)
        if parent.is_anchor:
            # Track visited nodes to prevent infinite loops in mesh
            visited = {sensor.device_id, parent.device_id}
            self._anchor_handle_data(parent, sensor.device_id,
                                     animation_steps, visited)

        return animation_steps
