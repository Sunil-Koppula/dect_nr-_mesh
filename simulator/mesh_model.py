"""
Mesh network model for DECT NR+ simulator.
Mirrors the firmware logic exactly:

Gateway (gateway.c):
  - hop 0, always paired
  - Responds to pair requests from anchors/sensors
  - handle_pair_confirm: stores anchors in anchor_store, sensors in sensor_store
  - handle_data: ACK back, no relay (gateway is final destination)
  - AT commands: PARENT_QUERY, REPAIR, SET_RSSI dispatched from gateway

Anchor (anchor.c):
  - True mesh — pairs with ALL reachable gateways/anchors (device_store, max 8)
  - Paired sensors stored in sensor_store (max 16)
  - anchor_do_mesh_pairing: broadcast pair_request, collect responses,
    discovery_sort_mesh (gateway/min-hop first, filter RSSI > threshold),
    send pair_confirm to ALL valid, store in device_store
  - handle_pair_confirm: gateway/anchor -> device_store + neighbor_add;
    sensor -> sensor_store
  - handle_data: ACK sender, relay UPSTREAM only (devices with lower hop)
  - handle_parent_query: respond with own info, forward downstream
  - handle_repair: forward downstream, then factory reset
  - handle_set_rssi: forward downstream, store new threshold
  - Rediscovery: every 10 min until device_store full (8 slots);
    also updates hop numbers of already-paired devices

Sensor (sensor.c):
  - Tree topology — picks ONE best parent (gateway > lowest hop > best RSSI)
  - sensor_send_data: send to parent, wait for ACK

Mesh (mesh.c):
  - discovery_sort_mesh: sort by gateway first, then min hop, then best RSSI;
    filter anchors below rssi_threshold (mutable, default -75 dBm)
  - discovery_best: gateway wins > lower hop > better RSSI (used by sensor)
  - neighbor_best_route: gateway wins > lower hop > better RSSI
  - discovery_add_response: reject sensors as candidates for anchors/sensors
"""

import random
import time
from dataclasses import dataclass, field
from typing import Optional, Callable

from packet import (
    DeviceType, PacketType, Version, PairRequest, PairResponse, PairConfirm,
    DataPacket, DataAck,
    ParentQuery, ParentResponse, RepairPacket, SetRssiPacket,
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

    def is_upstream_of(self, other_node) -> bool:
        """True if this node's hop < other_node's hop (upstream)."""
        return self.hop < other_node.hop

    def is_downstream_of(self, other_node) -> bool:
        """True if this node's hop > other_node's hop (downstream)."""
        return self.hop > other_node.hop

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
        self.rssi_threshold: int = RSSI_THRESHOLD

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
        Filter: discard anchors below rssi_threshold."""
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
            elif rssi >= self.rssi_threshold:
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

            if not n.is_gateway and rssi < self.rssi_threshold:
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
        """Mirrors anchor_rediscovery — single attempt, skip already paired.
        Also updates hop numbers of already-paired devices that respond."""
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
            if packet_lost(anchor.x, anchor.y, n.x, n.y):
                continue
            rssi = calc_rssi(anchor.x, anchor.y, n.x, n.y)
            if not n.is_gateway and rssi < self.rssi_threshold:
                continue
            # Update hop numbers for already-paired devices
            if anchor.device_store_contains(n.device_id):
                for d in anchor.paired_devices:
                    if d.device_id == n.device_id:
                        d.version = n.version
                        break
                self._log(f"Rediscovery: updated neighbor ID:{n.device_id} "
                          f"hop:{n.hop}")
                continue
            self._discovery_add_response(candidates, anchor, n, rssi)

        if anchor.device_store_full:
            return animation_steps

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
        """Relay data UPSTREAM only (to devices with lower hop).
        Each receiving anchor also runs handle_data with visited set
        to prevent loops."""
        for dev in anchor.paired_devices:
            if dev.device_id == sender_id:
                continue  # don't relay back to sender
            if dev.device_id in visited:
                continue  # already received this data
            relay_node = self.find_node(dev.device_id)
            if not relay_node:
                continue
            # Only relay upstream
            if relay_node.hop >= anchor.hop:
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
            # (relay upstream, skip the sender)
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

        # Anchor: relay upstream only; Gateway: ACK only (final destination)
        if parent.is_anchor:
            # Track visited nodes to prevent infinite loops in mesh
            visited = {sensor.device_id, parent.device_id}
            self._anchor_handle_data(parent, sensor.device_id,
                                     animation_steps, visited)

        return animation_steps

    # ========== Downstream forwarding helper ==========

    def _forward_downstream(self, anchor, sender_id, packet_factory, label,
                            animation_steps):
        """Forward a packet to downstream anchors (hop > my_hop) + all sensors."""
        for dev in anchor.paired_devices:
            if dev.device_id == sender_id:
                continue
            peer = self.find_node(dev.device_id)
            if not peer or peer.hop <= anchor.hop:
                continue  # skip upstream and same-level peers
            pkt = packet_factory(dev.device_id)
            animation_steps.append({
                "type": "unicast", "src": anchor, "dst": peer,
                "packet": pkt, "delay": 200,
            })
        for dev in anchor.paired_sensors:
            peer = self.find_node(dev.device_id)
            if not peer:
                continue
            pkt = packet_factory(dev.device_id)
            animation_steps.append({
                "type": "unicast", "src": anchor, "dst": peer,
                "packet": pkt, "delay": 200,
            })

    # ========== Neighbor best route (mirrors mesh.c) ==========

    def _neighbor_best_route(self, anchor):
        """Return the best upstream neighbor: gateway > lower hop > better RSSI."""
        best = None
        best_rssi = -999
        for dev in anchor.paired_devices:
            peer = self.find_node(dev.device_id)
            if not peer:
                continue
            if not peer.is_upstream_of(anchor):
                continue
            rssi = calc_rssi(anchor.x, anchor.y, peer.x, peer.y)
            if best is None:
                best, best_rssi = peer, rssi
                continue
            if peer.is_gateway and not best.is_gateway:
                best, best_rssi = peer, rssi
                continue
            if best.is_gateway and not peer.is_gateway:
                continue
            if peer.hop < best.hop:
                best, best_rssi = peer, rssi
                continue
            if peer.hop == best.hop and rssi > best_rssi:
                best, best_rssi = peer, rssi
        return best

    # ========== Parent Query (AT+SENSOR_*, AT+ANCHOR_*) ==========

    def simulate_parent_query(self, target_id: int) -> list:
        """Query parent info for a specific device or all devices.
        If target_id != 0: specific query.  If target_id == 0: query all."""
        gateway = None
        for n in self.nodes:
            if n.is_gateway:
                gateway = n
                break
        if not gateway:
            self._log("No gateway found", "ERR")
            return []

        animation_steps = []
        seen_ids = set()

        if target_id != 0:
            # Check gateway's sensor_store first
            for dev in gateway.paired_sensors:
                if dev.device_id == target_id:
                    self._log(f"Parent query: SENSOR ID:{target_id} -> "
                              f"parent GATEWAY ID:{gateway.device_id} hop:1")
                    return []
            # Check gateway's anchor_store (device_store)
            for dev in gateway.paired_devices:
                if dev.device_id == target_id:
                    self._log(f"Parent query: ANCHOR ID:{target_id} -> "
                              f"parent GATEWAY ID:{gateway.device_id} hop:1")
                    return []

        # Send PARENT_QUERY to all paired anchors
        for dev in gateway.paired_devices:
            peer = self.find_node(dev.device_id)
            if not peer:
                continue
            pkt = ParentQuery(src_device_id=gateway.device_id,
                              dst_device_id=dev.device_id,
                              target_id=target_id)
            animation_steps.append({
                "type": "unicast", "src": gateway, "dst": peer,
                "packet": pkt, "delay": 200,
            })
            self._anchor_handle_parent_query(
                peer, gateway.device_id, target_id, animation_steps, seen_ids)

        if target_id == 0:
            # Also query directly paired sensors
            for dev in gateway.paired_sensors:
                peer = self.find_node(dev.device_id)
                if not peer:
                    continue
                resp = ParentResponse(
                    src_device_id=peer.device_id,
                    dst_device_id=gateway.device_id,
                    device_type=peer.device_type,
                    parent_id=peer.parent_id or 0,
                    parent_type=DeviceType.GATEWAY,
                    hop_num=peer.hop,
                )
                animation_steps.append({
                    "type": "unicast", "src": peer, "dst": gateway,
                    "packet": resp, "delay": 300,
                })
                self._log(f"Parent response: SENSOR ID:{peer.device_id} -> "
                          f"parent GATEWAY ID:{gateway.device_id} hop:{peer.hop}")

        return animation_steps

    def simulate_parent_query_all(self, filter_type: int) -> list:
        """Query all devices of a given type (DeviceType.SENSOR or ANCHOR).
        Equivalent to AT+SENSOR_ALL / AT+ANCHOR_ALL."""
        gateway = None
        for n in self.nodes:
            if n.is_gateway:
                gateway = n
                break
        if not gateway:
            self._log("No gateway found", "ERR")
            return []

        type_str = "SENSOR" if filter_type == DeviceType.SENSOR else "ANCHOR"
        self._log(f"AT+{type_str}_ALL: querying all {type_str.lower()}s")

        animation_steps = []
        seen_ids = set()

        # Gateway logs its own directly paired devices of the filter type
        if filter_type == DeviceType.SENSOR:
            for dev in gateway.paired_sensors:
                peer = self.find_node(dev.device_id)
                if peer:
                    self._log(f"  {type_str} ID:{peer.device_id} -> "
                              f"parent GATEWAY ID:{gateway.device_id} "
                              f"hop:{peer.hop}")
                    seen_ids.add(peer.device_id)
        elif filter_type == DeviceType.ANCHOR:
            for dev in gateway.paired_devices:
                peer = self.find_node(dev.device_id)
                if peer and peer.is_anchor:
                    self._log(f"  {type_str} ID:{peer.device_id} -> "
                              f"parent GATEWAY ID:{gateway.device_id} "
                              f"hop:{peer.hop}")
                    seen_ids.add(peer.device_id)

        # Send PARENT_QUERY (target_id=0) to all paired anchors
        for dev in gateway.paired_devices:
            peer = self.find_node(dev.device_id)
            if not peer or not peer.is_anchor:
                continue
            pkt = ParentQuery(src_device_id=gateway.device_id,
                              dst_device_id=dev.device_id,
                              target_id=0)
            animation_steps.append({
                "type": "unicast", "src": gateway, "dst": peer,
                "packet": pkt, "delay": 200,
            })
            self._anchor_handle_parent_query_all(
                peer, gateway.device_id, filter_type, animation_steps, seen_ids)

        return animation_steps

    def _anchor_handle_parent_query(self, anchor, sender_id, target_id,
                                    animation_steps, seen_ids):
        """Anchor handles PARENT_QUERY: respond if match, forward downstream."""
        if anchor.device_id in seen_ids:
            return
        seen_ids.add(anchor.device_id)

        # Find best upstream route for responses
        upstream = self._neighbor_best_route(anchor)
        if not upstream:
            upstream = self.find_node(sender_id)

        if target_id != 0:
            # Specific target query
            if anchor.device_id == target_id:
                # Respond with own info
                parent_node = self._neighbor_best_route(anchor)
                parent_id = parent_node.device_id if parent_node else 0
                parent_type = parent_node.device_type if parent_node else DeviceType.UNKNOWN
                resp = ParentResponse(
                    src_device_id=anchor.device_id,
                    dst_device_id=sender_id,
                    device_type=anchor.device_type,
                    parent_id=parent_id,
                    parent_type=parent_type,
                    hop_num=anchor.hop,
                )
                if upstream:
                    animation_steps.append({
                        "type": "unicast", "src": anchor, "dst": upstream,
                        "packet": resp, "delay": 300,
                    })
                self._log(f"Parent response: ANCHOR ID:{anchor.device_id} -> "
                          f"parent {device_type_str(parent_type)} "
                          f"ID:{parent_id} hop:{anchor.hop}")
                return

            # Check local sensor_store
            for dev in anchor.paired_sensors:
                if dev.device_id == target_id:
                    peer = self.find_node(dev.device_id)
                    hop = peer.hop if peer else anchor.hop + 1
                    resp = ParentResponse(
                        src_device_id=dev.device_id,
                        dst_device_id=sender_id,
                        device_type=DeviceType.SENSOR,
                        parent_id=anchor.device_id,
                        parent_type=anchor.device_type,
                        hop_num=hop,
                    )
                    if upstream:
                        animation_steps.append({
                            "type": "unicast", "src": anchor, "dst": upstream,
                            "packet": resp, "delay": 300,
                        })
                    self._log(f"Parent response: SENSOR ID:{dev.device_id} -> "
                              f"parent ANCHOR ID:{anchor.device_id} hop:{hop}")
                    return

            # Check downstream device_store
            for dev in anchor.paired_devices:
                if dev.device_id == target_id:
                    peer = self.find_node(dev.device_id)
                    if peer and peer.is_downstream_of(anchor):
                        p_node = self._neighbor_best_route(peer) if peer else None
                        p_id = p_node.device_id if p_node else anchor.device_id
                        p_type = p_node.device_type if p_node else anchor.device_type
                        resp = ParentResponse(
                            src_device_id=dev.device_id,
                            dst_device_id=sender_id,
                            device_type=dev.device_type,
                            parent_id=p_id,
                            parent_type=p_type,
                            hop_num=peer.hop if peer else 0,
                        )
                        if upstream:
                            animation_steps.append({
                                "type": "unicast", "src": anchor, "dst": upstream,
                                "packet": resp, "delay": 300,
                            })
                        self._log(f"Parent response: ANCHOR ID:{dev.device_id} -> "
                                  f"parent {device_type_str(p_type)} "
                                  f"ID:{p_id} hop:{peer.hop if peer else 0}")
                        return

        # Not found locally — forward query.
        # query_all: downstream only (hop > my_hop).
        # specific: peers + downstream (hop >= my_hop) to reach devices
        # behind same-level anchors. Skip sender and upstream.
        for dev in anchor.paired_devices:
            if dev.device_id == sender_id:
                continue
            peer = self.find_node(dev.device_id)
            if not peer or not peer.is_anchor:
                continue
            if target_id != 0:
                # Specific: skip upstream only (allow peers + downstream)
                if peer.is_upstream_of(anchor):
                    continue
            else:
                # Query all: downstream only
                if peer.hop <= anchor.hop:
                    continue
            pkt = ParentQuery(src_device_id=anchor.device_id,
                              dst_device_id=dev.device_id,
                              target_id=target_id)
            animation_steps.append({
                "type": "unicast", "src": anchor, "dst": peer,
                "packet": pkt, "delay": 200,
            })
            self._anchor_handle_parent_query(
                peer, anchor.device_id, target_id, animation_steps, seen_ids)

        # Always forward to all paired sensors
        for dev in anchor.paired_sensors:
            peer = self.find_node(dev.device_id)
            if not peer:
                continue
            pkt = ParentQuery(src_device_id=anchor.device_id,
                              dst_device_id=dev.device_id,
                              target_id=target_id)
            animation_steps.append({
                "type": "unicast", "src": anchor, "dst": peer,
                "packet": pkt, "delay": 200,
            })

    def _anchor_handle_parent_query_all(self, anchor, sender_id, filter_type,
                                        animation_steps, seen_ids):
        """Handle query-all: report own info + sensors, forward downstream."""
        if anchor.device_id in seen_ids:
            return
        seen_ids.add(anchor.device_id)

        upstream = self._neighbor_best_route(anchor)
        if not upstream:
            upstream = self.find_node(sender_id)

        type_str = "SENSOR" if filter_type == DeviceType.SENSOR else "ANCHOR"

        # Report own info if matching filter
        if filter_type == DeviceType.ANCHOR:
            parent_node = self._neighbor_best_route(anchor)
            parent_id = parent_node.device_id if parent_node else 0
            parent_type = parent_node.device_type if parent_node else DeviceType.UNKNOWN
            resp = ParentResponse(
                src_device_id=anchor.device_id,
                dst_device_id=sender_id,
                device_type=anchor.device_type,
                parent_id=parent_id,
                parent_type=parent_type,
                hop_num=anchor.hop,
            )
            if upstream:
                animation_steps.append({
                    "type": "unicast", "src": anchor, "dst": upstream,
                    "packet": resp, "delay": 300,
                })
            self._log(f"  {type_str} ID:{anchor.device_id} -> "
                      f"parent {device_type_str(parent_type)} "
                      f"ID:{parent_id} hop:{anchor.hop}")

        # Report sensors if filter is SENSOR
        if filter_type == DeviceType.SENSOR:
            for dev in anchor.paired_sensors:
                if dev.device_id in seen_ids:
                    continue
                seen_ids.add(dev.device_id)
                peer = self.find_node(dev.device_id)
                hop = peer.hop if peer else anchor.hop + 1
                resp = ParentResponse(
                    src_device_id=dev.device_id,
                    dst_device_id=sender_id,
                    device_type=DeviceType.SENSOR,
                    parent_id=anchor.device_id,
                    parent_type=anchor.device_type,
                    hop_num=hop,
                )
                if upstream:
                    animation_steps.append({
                        "type": "unicast", "src": anchor, "dst": upstream,
                        "packet": resp, "delay": 300,
                    })
                self._log(f"  {type_str} ID:{dev.device_id} -> "
                          f"parent ANCHOR ID:{anchor.device_id} hop:{hop}")

        # Forward downstream to other anchors
        for dev in anchor.paired_devices:
            if dev.device_id == sender_id:
                continue
            peer = self.find_node(dev.device_id)
            if not peer or not peer.is_anchor:
                continue
            if peer.hop <= anchor.hop:
                continue
            pkt = ParentQuery(src_device_id=anchor.device_id,
                              dst_device_id=dev.device_id,
                              target_id=0)
            animation_steps.append({
                "type": "unicast", "src": anchor, "dst": peer,
                "packet": pkt, "delay": 200,
            })
            self._anchor_handle_parent_query_all(
                peer, anchor.device_id, filter_type, animation_steps, seen_ids)

    # ========== Repair (AT+REPAIR) ==========

    def simulate_repair(self) -> list:
        """Gateway sends REPAIR to all paired anchors + sensors.
        Each anchor forwards downstream then factory resets.
        Each sensor factory resets. Gateway factory resets itself."""
        gateway = None
        for n in self.nodes:
            if n.is_gateway:
                gateway = n
                break
        if not gateway:
            self._log("No gateway found", "ERR")
            return []

        self._log("AT+REPAIR: initiating network repair")
        animation_steps = []

        # Send REPAIR to all paired anchors
        for dev in list(gateway.paired_devices):
            peer = self.find_node(dev.device_id)
            if not peer:
                continue
            pkt = RepairPacket(src_device_id=gateway.device_id,
                               dst_device_id=dev.device_id)
            animation_steps.append({
                "type": "unicast", "src": gateway, "dst": peer,
                "packet": pkt, "delay": 200,
            })
            self._anchor_handle_repair(peer, gateway.device_id, animation_steps)

        # Send REPAIR to all paired sensors
        for dev in list(gateway.paired_sensors):
            peer = self.find_node(dev.device_id)
            if not peer:
                continue
            pkt = RepairPacket(src_device_id=gateway.device_id,
                               dst_device_id=dev.device_id)
            animation_steps.append({
                "type": "unicast", "src": gateway, "dst": peer,
                "packet": pkt, "delay": 200,
            })
            # Sensor factory reset
            peer.parent_id = None
            peer.hop = 0
            peer.state = NodeState.IDLE
            peer.mesh_links.clear()
            self._log(f"REPAIR: SENSOR ID:{peer.device_id} factory reset")

        # Gateway factory resets itself
        gateway.paired_devices.clear()
        gateway.paired_sensors.clear()
        gateway.mesh_links.clear()
        self._log("REPAIR: GATEWAY factory reset")

        if self.on_topology_changed:
            self.on_topology_changed()
        return animation_steps

    def _anchor_handle_repair(self, anchor, sender_id, animation_steps):
        """Anchor forwards REPAIR downstream then factory resets."""
        # Forward downstream (not upstream, not peers)
        for dev in list(anchor.paired_devices):
            if dev.device_id == sender_id:
                continue
            peer = self.find_node(dev.device_id)
            if not peer or peer.hop <= anchor.hop:
                continue
            pkt = RepairPacket(src_device_id=anchor.device_id,
                               dst_device_id=dev.device_id)
            animation_steps.append({
                "type": "unicast", "src": anchor, "dst": peer,
                "packet": pkt, "delay": 200,
            })
            if peer.is_anchor:
                self._anchor_handle_repair(peer, anchor.device_id,
                                           animation_steps)

        # Forward to paired sensors
        for dev in list(anchor.paired_sensors):
            peer = self.find_node(dev.device_id)
            if not peer:
                continue
            pkt = RepairPacket(src_device_id=anchor.device_id,
                               dst_device_id=dev.device_id)
            animation_steps.append({
                "type": "unicast", "src": anchor, "dst": peer,
                "packet": pkt, "delay": 200,
            })
            # Sensor factory reset
            peer.parent_id = None
            peer.hop = 0
            peer.state = NodeState.IDLE
            peer.mesh_links.clear()
            self._log(f"REPAIR: SENSOR ID:{peer.device_id} factory reset")

        # Anchor factory reset
        anchor.paired_devices.clear()
        anchor.paired_sensors.clear()
        anchor.mesh_links.clear()
        anchor.state = NodeState.IDLE
        anchor.hop = 0
        self._log(f"REPAIR: ANCHOR ID:{anchor.device_id} factory reset")

    # ========== SET_RSSI (AT+SET_RSSI) ==========

    def simulate_set_rssi(self, rssi_dbm: int) -> list:
        """Gateway sends SET_RSSI to all anchors + sensors.
        Each anchor forwards downstream + stores new threshold.
        Each sensor stores. Gateway stores. Updates self.rssi_threshold."""
        gateway = None
        for n in self.nodes:
            if n.is_gateway:
                gateway = n
                break
        if not gateway:
            self._log("No gateway found", "ERR")
            return []

        self._log(f"AT+SET_RSSI: setting threshold to {rssi_dbm} dBm")
        animation_steps = []

        # Send SET_RSSI to all paired anchors
        for dev in gateway.paired_devices:
            peer = self.find_node(dev.device_id)
            if not peer:
                continue
            pkt = SetRssiPacket(src_device_id=gateway.device_id,
                                dst_device_id=dev.device_id,
                                rssi_dbm=rssi_dbm)
            animation_steps.append({
                "type": "unicast", "src": gateway, "dst": peer,
                "packet": pkt, "delay": 200,
            })
            self._anchor_handle_set_rssi(peer, gateway.device_id, rssi_dbm,
                                         animation_steps)

        # Send SET_RSSI to all paired sensors
        for dev in gateway.paired_sensors:
            peer = self.find_node(dev.device_id)
            if not peer:
                continue
            pkt = SetRssiPacket(src_device_id=gateway.device_id,
                                dst_device_id=dev.device_id,
                                rssi_dbm=rssi_dbm)
            animation_steps.append({
                "type": "unicast", "src": gateway, "dst": peer,
                "packet": pkt, "delay": 200,
            })
            self._log(f"SET_RSSI: SENSOR ID:{peer.device_id} stored {rssi_dbm} dBm")

        # Gateway stores
        self.rssi_threshold = rssi_dbm
        self._log(f"SET_RSSI: GATEWAY threshold now {rssi_dbm} dBm")

        return animation_steps

    def _anchor_handle_set_rssi(self, anchor, sender_id, rssi_dbm,
                                animation_steps):
        """Anchor forwards SET_RSSI downstream + stores new threshold."""
        # Forward downstream
        for dev in anchor.paired_devices:
            if dev.device_id == sender_id:
                continue
            peer = self.find_node(dev.device_id)
            if not peer or peer.hop <= anchor.hop:
                continue
            pkt = SetRssiPacket(src_device_id=anchor.device_id,
                                dst_device_id=dev.device_id,
                                rssi_dbm=rssi_dbm)
            animation_steps.append({
                "type": "unicast", "src": anchor, "dst": peer,
                "packet": pkt, "delay": 200,
            })
            if peer.is_anchor:
                self._anchor_handle_set_rssi(peer, anchor.device_id, rssi_dbm,
                                             animation_steps)

        # Forward to paired sensors
        for dev in anchor.paired_sensors:
            peer = self.find_node(dev.device_id)
            if not peer:
                continue
            pkt = SetRssiPacket(src_device_id=anchor.device_id,
                                dst_device_id=dev.device_id,
                                rssi_dbm=rssi_dbm)
            animation_steps.append({
                "type": "unicast", "src": anchor, "dst": peer,
                "packet": pkt, "delay": 200,
            })
            self._log(f"SET_RSSI: SENSOR ID:{peer.device_id} stored {rssi_dbm} dBm")

        # Store locally
        self._log(f"SET_RSSI: ANCHOR ID:{anchor.device_id} stored {rssi_dbm} dBm")
