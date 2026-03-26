"""
Canvas widget for visualizing the DECT NR+ mesh topology.
Draws nodes, connections, and animates packet flow.
"""

from PyQt5.QtWidgets import QWidget, QMenu, QAction
from PyQt5.QtCore import Qt, QTimer, QPointF, QRectF
from PyQt5.QtGui import (QPainter, QPen, QBrush, QColor, QFont,
                          QRadialGradient, QPainterPath)

from packet import DeviceType, device_type_str
from mesh_model import Node, NodeState, MeshNetwork
from radio_sim import BROADCAST_RANGE

# Node colors
NODE_COLORS = {
    DeviceType.GATEWAY: QColor(41, 128, 185),    # blue
    DeviceType.ANCHOR:  QColor(39, 174, 96),      # green
    DeviceType.SENSOR:  QColor(230, 126, 34),      # orange
}

NODE_RADIUS = 22
ANIM_SPEED = 2.0     # pixels per frame
ANIM_FPS = 60


class PacketAnimation:
    """Represents an animated packet moving between nodes."""

    def __init__(self, src: Node, dst: Node, color: QColor,
                 label: str = "", callback=None):
        self.src = src
        self.dst = dst
        self.color = color
        self.label = label
        self.callback = callback
        self.progress = 0.0   # 0.0 to 1.0
        self.done = False

    def update(self, dt: float):
        """Advance animation. dt is time in seconds."""
        self.progress += dt * 2.0  # 0.5 seconds per packet
        if self.progress >= 1.0:
            self.progress = 1.0
            self.done = True
            if self.callback:
                self.callback()
                self.callback = None

    @property
    def current_pos(self) -> QPointF:
        x = self.src.x + (self.dst.x - self.src.x) * self.progress
        y = self.src.y + (self.dst.y - self.src.y) * self.progress
        return QPointF(x, y)


class BroadcastAnimation:
    """Expanding circle for broadcast packets."""

    def __init__(self, src: Node, color: QColor, label: str = ""):
        self.src = src
        self.color = color
        self.label = label
        self.radius = 0.0
        self.max_radius = BROADCAST_RANGE
        self.done = False

    def update(self, dt: float):
        self.radius += dt * 400  # pixels per second
        if self.radius >= self.max_radius:
            self.done = True


class MeshCanvas(QWidget):
    """Main canvas for drawing mesh topology."""

    def __init__(self, mesh: MeshNetwork, parent=None):
        super().__init__(parent)
        self.mesh = mesh
        self.setMinimumSize(600, 400)
        self.setMouseTracking(True)

        # Drag state
        self.dragging_node = None
        self.drag_offset = QPointF(0, 0)

        # Selection
        self.selected_node = None
        self.on_node_selected = None   # callback

        # Animations
        self.animations = []
        self.animation_queue = []  # queued steps from pairing
        self.queue_timer = QTimer()
        self.queue_timer.setSingleShot(True)
        self.queue_timer.timeout.connect(self._process_next_animation)

        # Animation timer
        self.anim_timer = QTimer()
        self.anim_timer.timeout.connect(self._tick_animations)
        self.anim_timer.start(int(1000 / ANIM_FPS))

    def _node_at(self, pos: QPointF) -> Node:
        """Find node under cursor."""
        for node in reversed(self.mesh.nodes):
            dx = pos.x() - node.x
            dy = pos.y() - node.y
            if dx * dx + dy * dy <= NODE_RADIUS * NODE_RADIUS:
                return node
        return None

    # ===== Mouse events =====

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            node = self._node_at(event.pos())
            if node:
                self.dragging_node = node
                self.drag_offset = QPointF(
                    event.pos().x() - node.x,
                    event.pos().y() - node.y,
                )
                self.selected_node = node
                if self.on_node_selected:
                    self.on_node_selected(node)
            else:
                self.selected_node = None
                if self.on_node_selected:
                    self.on_node_selected(None)
            self.update()

    def mouseMoveEvent(self, event):
        if self.dragging_node:
            self.dragging_node.x = event.pos().x() - self.drag_offset.x()
            self.dragging_node.y = event.pos().y() - self.drag_offset.y()
            self.update()

    def mouseReleaseEvent(self, event):
        self.dragging_node = None

    def contextMenuEvent(self, event):
        menu = QMenu(self)
        node = self._node_at(event.pos())

        if node:
            # Context menu for existing node
            info = menu.addAction(f"{device_type_str(node.device_type)} "
                                  f"ID:{node.device_id}")
            info.setEnabled(False)
            menu.addSeparator()

            if node.state != NodeState.PAIRED and not node.is_gateway:
                pair_act = menu.addAction("Start Pairing")
                pair_act.triggered.connect(lambda: self._start_pairing(node))

            remove_act = menu.addAction("Remove Node")
            remove_act.triggered.connect(lambda: self._remove_node(node))
        else:
            # Context menu for empty area
            x, y = event.pos().x(), event.pos().y()

            gw_act = menu.addAction("Add Gateway")
            gw_act.triggered.connect(
                lambda: self._add_node(DeviceType.GATEWAY, x, y))

            anchor_act = menu.addAction("Add Anchor")
            anchor_act.triggered.connect(
                lambda: self._add_node(DeviceType.ANCHOR, x, y))

            sensor_act = menu.addAction("Add Sensor")
            sensor_act.triggered.connect(
                lambda: self._add_node(DeviceType.SENSOR, x, y))

        menu.exec_(event.globalPos())

    def _add_node(self, device_type, x, y):
        self.mesh.add_node(device_type, x, y)
        self.update()

    def _remove_node(self, node):
        self.mesh.remove_node(node)
        if self.selected_node == node:
            self.selected_node = None
            if self.on_node_selected:
                self.on_node_selected(None)
        self.update()

    def _start_pairing(self, node):
        steps = self.mesh.start_pairing(node)
        self.queue_animation_steps(steps)

    # ===== Animation =====

    def queue_animation_steps(self, steps: list):
        """Queue animation steps from a pairing sequence."""
        self.animation_queue.extend(steps)
        if not self.queue_timer.isActive():
            self._process_next_animation()

    def _process_next_animation(self):
        if not self.animation_queue:
            return

        step = self.animation_queue.pop(0)

        if step["type"] == "broadcast":
            src = step["src"]
            anim = BroadcastAnimation(
                src, NODE_COLORS.get(src.device_type, QColor(200, 200, 200)),
                "PAIR_REQ",
            )
            self.animations.append(anim)
        elif step["type"] == "unicast":
            src = step["src"]
            dst = step["dst"]
            color = NODE_COLORS.get(src.device_type, QColor(200, 200, 200))
            label = type(step["packet"]).__name__
            callback = step.get("callback")
            anim = PacketAnimation(src, dst, color, label, callback)
            self.animations.append(anim)

        delay = step.get("delay", 300)
        if self.animation_queue:
            self.queue_timer.start(delay)

    def _tick_animations(self):
        if not self.animations:
            return

        dt = 1.0 / ANIM_FPS

        for anim in self.animations:
            anim.update(dt)

        # Remove finished animations
        self.animations = [a for a in self.animations if not a.done]
        self.update()

    # ===== Drawing =====

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Background
        painter.fillRect(self.rect(), QColor(30, 30, 40))

        # Draw connections (paired links)
        self._draw_connections(painter)

        # Draw broadcast range for selected node
        if self.selected_node:
            self._draw_range(painter, self.selected_node)

        # Draw animations
        self._draw_animations(painter)

        # Draw nodes
        for node in self.mesh.nodes:
            self._draw_node(painter, node)

        painter.end()

    def _draw_connections(self, painter: QPainter):
        """Draw all mesh links."""
        # Mesh links (anchor-to-anchor, anchor-to-gateway)
        pen = QPen(QColor(100, 100, 120), 2, Qt.SolidLine)
        painter.setPen(pen)

        for node_a, node_b in self.mesh.get_all_links():
            painter.drawLine(
                int(node_a.x), int(node_a.y),
                int(node_b.x), int(node_b.y),
            )

    def _draw_range(self, painter: QPainter, node: Node):
        """Draw translucent broadcast range circle."""
        pen = QPen(QColor(255, 255, 255, 30), 1, Qt.DashLine)
        painter.setPen(pen)
        painter.setBrush(QBrush(QColor(255, 255, 255, 8)))
        painter.drawEllipse(
            QPointF(node.x, node.y),
            BROADCAST_RANGE, BROADCAST_RANGE,
        )

    def _draw_node(self, painter: QPainter, node: Node):
        """Draw a single node."""
        color = NODE_COLORS.get(node.device_type, QColor(200, 200, 200))
        cx, cy = int(node.x), int(node.y)
        r = NODE_RADIUS

        # Selection highlight
        if node == self.selected_node:
            painter.setPen(QPen(QColor(255, 255, 255), 3))
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(QPointF(cx, cy), r + 4, r + 4)

        # Node circle
        if node.state == NodeState.IDLE:
            # Dimmed for unpaired
            dim_color = QColor(color.red() // 2, color.green() // 2,
                               color.blue() // 2)
            painter.setBrush(QBrush(dim_color))
            painter.setPen(QPen(color, 2, Qt.DashLine))
        elif node.state == NodeState.PAIRING:
            painter.setBrush(QBrush(color.darker(120)))
            painter.setPen(QPen(QColor(255, 255, 100), 2))
        else:
            painter.setBrush(QBrush(color))
            painter.setPen(QPen(color.lighter(150), 2))

        painter.drawEllipse(QPointF(cx, cy), r, r)

        # Label
        painter.setPen(QPen(Qt.white))
        painter.setFont(QFont("Consolas", 8, QFont.Bold))

        # Device type letter
        letter = {
            DeviceType.GATEWAY: "G",
            DeviceType.ANCHOR: "A",
            DeviceType.SENSOR: "S",
        }.get(node.device_type, "?")

        painter.drawText(QRectF(cx - r, cy - 10, r * 2, 14),
                         Qt.AlignCenter, letter)

        # ID below
        painter.setFont(QFont("Consolas", 7))
        painter.drawText(QRectF(cx - r, cy + 1, r * 2, 12),
                         Qt.AlignCenter, str(node.device_id))

        # Hop number (top-right)
        if node.state == NodeState.PAIRED:
            painter.setFont(QFont("Consolas", 7))
            painter.setPen(QPen(QColor(200, 200, 200)))
            painter.drawText(cx + r + 2, cy - r + 10,
                             f"h{node.hop}")

    def _draw_animations(self, painter: QPainter):
        """Draw active packet animations."""
        for anim in self.animations:
            if isinstance(anim, BroadcastAnimation):
                # Expanding circle
                alpha = max(0, int(180 * (1.0 - anim.radius / anim.max_radius)))
                color = QColor(anim.color)
                color.setAlpha(alpha)
                painter.setPen(QPen(color, 2))
                painter.setBrush(Qt.NoBrush)
                painter.drawEllipse(
                    QPointF(anim.src.x, anim.src.y),
                    anim.radius, anim.radius,
                )
                # Label
                if anim.radius < 60:
                    painter.setFont(QFont("Consolas", 7))
                    painter.setPen(QPen(QColor(255, 255, 200)))
                    painter.drawText(int(anim.src.x) + 10,
                                     int(anim.src.y) - int(anim.radius) - 5,
                                     anim.label)

            elif isinstance(anim, PacketAnimation):
                pos = anim.current_pos
                # Dot
                painter.setPen(Qt.NoPen)
                painter.setBrush(QBrush(anim.color.lighter(150)))
                painter.drawEllipse(pos, 5, 5)
                # Label
                painter.setFont(QFont("Consolas", 7))
                painter.setPen(QPen(QColor(255, 255, 200)))
                painter.drawText(int(pos.x()) + 8, int(pos.y()) - 5,
                                 anim.label)
