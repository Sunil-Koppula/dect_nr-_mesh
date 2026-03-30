"""
DECT NR+ Mesh Network Simulator
Main window with canvas, node info panel, and log.
"""

import sys
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QPushButton, QLabel, QTextEdit, QSplitter, QGroupBox, QFrame,
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont, QColor

from packet import DeviceType, device_type_str, Version
from mesh_model import MeshNetwork, Node, NodeState
from mesh_view import MeshCanvas


class NodeInfoPanel(QGroupBox):
    """Panel showing details of the selected node."""

    def __init__(self, parent=None):
        super().__init__("Node Info", parent)
        self.setFixedWidth(220)
        self.setStyleSheet("""
            QGroupBox {
                color: #ccc;
                border: 1px solid #444;
                border-radius: 4px;
                margin-top: 8px;
                padding-top: 16px;
                font-weight: bold;
            }
            QLabel {
                color: #bbb;
                font-family: Consolas;
                font-size: 11px;
            }
        """)

        layout = QVBoxLayout()
        layout.setSpacing(4)

        self.lbl_type = QLabel("Type: -")
        self.lbl_id = QLabel("ID: -")
        self.lbl_version = QLabel("Version: -")
        self.lbl_hop = QLabel("Hop: -")
        self.lbl_state = QLabel("State: -")
        self.lbl_parent = QLabel("Parent: -")
        self.lbl_pos = QLabel("Pos: -")
        self.lbl_devices = QLabel("Devices: -")
        self.lbl_sensors = QLabel("Sensors: -")

        for lbl in [self.lbl_type, self.lbl_id, self.lbl_version,
                     self.lbl_hop, self.lbl_state, self.lbl_parent,
                     self.lbl_pos, self.lbl_devices, self.lbl_sensors]:
            layout.addWidget(lbl)

        layout.addStretch()
        self.setLayout(layout)

    def update_info(self, node):
        if node is None:
            for lbl in [self.lbl_type, self.lbl_id, self.lbl_version,
                         self.lbl_hop, self.lbl_state, self.lbl_parent,
                         self.lbl_pos, self.lbl_devices, self.lbl_sensors]:
                lbl.setText(lbl.text().split(":")[0] + ": -")
            return

        self.lbl_type.setText(f"Type: {device_type_str(node.device_type)}")
        self.lbl_id.setText(f"ID: {node.device_id}")
        self.lbl_version.setText(f"Version: v{node.version}")
        self.lbl_hop.setText(f"Hop: {node.hop}")
        self.lbl_state.setText(f"State: {node.state}")
        self.lbl_parent.setText(f"Parent: {node.parent_id or 'none'}")
        self.lbl_pos.setText(f"Pos: ({int(node.x)}, {int(node.y)})")

        device_ids = [str(d.device_id) for d in node.paired_devices]
        sensor_ids = [str(d.device_id) for d in node.paired_sensors]
        self.lbl_devices.setText(
            f"Devices ({len(device_ids)}/{8}): "
            + (", ".join(device_ids) if device_ids else "none"))
        self.lbl_sensors.setText(
            f"Sensors ({len(sensor_ids)}/16): "
            + (", ".join(sensor_ids) if sensor_ids else "none"))


class LogPanel(QTextEdit):
    """Scrolling log output."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self.setMaximumHeight(180)
        self.setFont(QFont("Consolas", 9))
        self.setStyleSheet("""
            QTextEdit {
                background-color: #1a1a2e;
                color: #a0a0b0;
                border: 1px solid #333;
            }
        """)

    def add_log(self, entry):
        color_map = {
            "INF": "#a0d0a0",
            "WRN": "#e0c060",
            "ERR": "#e06060",
        }
        color = color_map.get(entry.level, "#a0a0b0")
        self.append(f'<span style="color:{color}">{entry}</span>')
        self.verticalScrollBar().setValue(
            self.verticalScrollBar().maximum())


class MainWindow(QMainWindow):
    """Main application window."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("DECT NR+ Mesh Simulator")
        self.setMinimumSize(1000, 700)
        self.setStyleSheet("""
            QMainWindow { background-color: #2b2b3b; }
            QPushButton {
                background-color: #3a3a5a;
                color: #ccc;
                border: 1px solid #555;
                border-radius: 4px;
                padding: 6px 14px;
                font-family: Consolas;
                font-size: 11px;
            }
            QPushButton:hover { background-color: #4a4a6a; }
            QPushButton:pressed { background-color: #2a2a4a; }
        """)

        # Model
        self.mesh = MeshNetwork()

        # Views
        self.canvas = MeshCanvas(self.mesh)
        self.info_panel = NodeInfoPanel()
        self.log_panel = LogPanel()

        # Wire up callbacks
        self.mesh.on_log = self.log_panel.add_log
        self.mesh.on_topology_changed = self._on_topology_changed
        self.canvas.on_node_selected = self._on_node_selected

        # Toolbar
        toolbar = QWidget()
        toolbar_layout = QHBoxLayout(toolbar)
        toolbar_layout.setContentsMargins(8, 4, 8, 4)

        btn_gw = QPushButton("+ Gateway")
        btn_gw.clicked.connect(self._add_gateway)

        btn_anchor = QPushButton("+ Anchor")
        btn_anchor.clicked.connect(self._add_anchor)

        btn_sensor = QPushButton("+ Sensor")
        btn_sensor.clicked.connect(self._add_sensor)

        btn_pair_all = QPushButton("Pair All")
        btn_pair_all.clicked.connect(self._pair_all)
        btn_pair_all.setStyleSheet(
            btn_pair_all.styleSheet() +
            "QPushButton { background-color: #2a6a4a; }")

        btn_unpair_all = QPushButton("Unpair All")
        btn_unpair_all.clicked.connect(self._unpair_all)
        btn_unpair_all.setStyleSheet(
            btn_unpair_all.styleSheet() +
            "QPushButton { background-color: #6a5a2a; }")

        btn_randomize = QPushButton("Randomize")
        btn_randomize.clicked.connect(self._randomize)
        btn_randomize.setStyleSheet(
            btn_randomize.styleSheet() +
            "QPushButton { background-color: #2a4a6a; }")

        btn_send_data = QPushButton("Send Data")
        btn_send_data.clicked.connect(self._send_data)
        btn_send_data.setStyleSheet(
            btn_send_data.styleSheet() +
            "QPushButton { background-color: #5a2a6a; }")

        btn_clear = QPushButton("Clear All")
        btn_clear.clicked.connect(self._clear_all)
        btn_clear.setStyleSheet(
            btn_clear.styleSheet() +
            "QPushButton { background-color: #6a2a2a; }")

        for btn in [btn_gw, btn_anchor, btn_sensor, btn_pair_all,
                     btn_unpair_all, btn_send_data, btn_randomize, btn_clear]:
            toolbar_layout.addWidget(btn)
        toolbar_layout.addStretch()

        # Layout
        right_panel = QVBoxLayout()
        right_panel.addWidget(self.info_panel)
        right_panel.addStretch()

        right_widget = QWidget()
        right_widget.setLayout(right_panel)

        canvas_and_info = QHBoxLayout()
        canvas_and_info.addWidget(self.canvas, 1)
        canvas_and_info.addWidget(right_widget)

        canvas_info_widget = QWidget()
        canvas_info_widget.setLayout(canvas_and_info)

        splitter = QSplitter(Qt.Vertical)
        splitter.addWidget(canvas_info_widget)
        splitter.addWidget(self.log_panel)
        splitter.setSizes([500, 180])

        central = QVBoxLayout()
        central.setContentsMargins(0, 0, 0, 0)
        central.setSpacing(0)
        central.addWidget(toolbar)
        central.addWidget(splitter, 1)

        container = QWidget()
        container.setLayout(central)
        self.setCentralWidget(container)

        # Add a default gateway
        self.mesh.add_node(DeviceType.GATEWAY, 400, 200)

    def _add_gateway(self):
        import random
        x = random.randint(100, self.canvas.width() - 100)
        y = random.randint(80, self.canvas.height() - 80)
        self.mesh.add_node(DeviceType.GATEWAY, x, y)
        self.canvas.update()

    def _add_anchor(self):
        import random
        x = random.randint(100, self.canvas.width() - 100)
        y = random.randint(80, self.canvas.height() - 80)
        self.mesh.add_node(DeviceType.ANCHOR, x, y)
        self.canvas.update()

    def _add_sensor(self):
        import random
        x = random.randint(100, self.canvas.width() - 100)
        y = random.randint(80, self.canvas.height() - 80)
        self.mesh.add_node(DeviceType.SENSOR, x, y)
        self.canvas.update()

    def _pair_all(self):
        steps = self.mesh.pair_all_unpaired()
        self.canvas.queue_animation_steps(steps)

    def _send_data(self):
        """Send data from all paired sensors (or selected sensor)."""
        from mesh_model import NodeState
        # If a sensor is selected, send from that one only
        sel = self.canvas.selected_node
        if sel and sel.is_sensor and sel.state == NodeState.PAIRED:
            steps = self.mesh.simulate_data_send(sel)
            self.canvas.queue_animation_steps(steps)
            return

        # Otherwise send from all paired sensors
        sensors = [n for n in self.mesh.nodes
                   if n.is_sensor and n.state == NodeState.PAIRED]
        if not sensors:
            self.mesh._log("No paired sensors to send data", "WRN")
            return
        for sensor in sensors:
            steps = self.mesh.simulate_data_send(sensor)
            self.canvas.queue_animation_steps(steps)

    def _randomize(self):
        import random
        margin = 60
        w = max(self.canvas.width() - margin * 2, 200)
        h = max(self.canvas.height() - margin * 2, 200)
        for node in self.mesh.nodes:
            node.x = random.randint(margin, margin + w)
            node.y = random.randint(margin, margin + h)
        self.mesh._log("Randomized node positions")
        self.canvas.update()
        if self.canvas.selected_node:
            self.info_panel.update_info(self.canvas.selected_node)

    def _unpair_all(self):
        self.mesh.unpair_all()
        self.canvas.selected_node = None
        self.info_panel.update_info(None)
        self.canvas.update()

    def _clear_all(self):
        self.mesh.nodes.clear()
        self.mesh.log_entries.clear()
        self.log_panel.clear()
        self.canvas.selected_node = None
        self.info_panel.update_info(None)
        self.canvas.update()

    def _on_node_selected(self, node):
        self.info_panel.update_info(node)

    def _on_topology_changed(self):
        if self.canvas.selected_node:
            self.info_panel.update_info(self.canvas.selected_node)
        self.canvas.update()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    window = MainWindow()
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
