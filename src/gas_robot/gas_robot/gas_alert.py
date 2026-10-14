
#!/usr/bin/env python3

import math
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import rclpy
from rclpy.node import Node

from gas_robot_interfaces.msg import GasSensor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point


class GasAlertNode(Node):
    def __init__(self):
        super().__init__("gas_alert_dashboard")

        self.lock = threading.Lock()

        self.data = {
            "gas_ppm": 0.0,
            "temperature": 0.0,
            "humidity": 0.0,
            "sensor_ok": False,
            "gas_received": False,
            "last_gas_time": 0.0,

            "robot_x": 0.0,
            "robot_y": 0.0,
            "robot_yaw": 0.0,
            "odom_received": False,

            "source_x": None,
            "source_y": None,
            "source_z": 0.0,
            "source_detected": False,
            "source_event_time": 0.0,
        }

        self.create_subscription(
            GasSensor,
            "/gas_sensor",
            self.gas_callback,
            10,
        )

        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )

        self.create_subscription(
            Point,
            "/danger_zone",
            self.source_callback,
            10,
        )

        self.get_logger().info(
            "Gas alert dashboard subscribed to /gas_sensor, "
            "/odom and /danger_zone"
        )

    def gas_callback(self, msg):
        with self.lock:
            self.data["gas_ppm"] = float(msg.gas_ppm)
            self.data["temperature"] = float(msg.temperature)
            self.data["humidity"] = float(msg.humidity)
            self.data["sensor_ok"] = bool(msg.sensor_ok)
            self.data["gas_received"] = True
            self.data["last_gas_time"] = time.time()

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )

        with self.lock:
            self.data["robot_x"] = float(position.x)
            self.data["robot_y"] = float(position.y)
            self.data["robot_yaw"] = float(yaw)
            self.data["odom_received"] = True

    def source_callback(self, msg):
        # Store the source coordinates and mark a new detection event.
        with self.lock:
            self.data["source_x"] = float(msg.x)
            self.data["source_y"] = float(msg.y)
            self.data["source_z"] = float(msg.z)
            self.data["source_detected"] = True
            self.data["source_event_time"] = time.time()

        self.get_logger().info(
            f"Gas source detected at "
            f"x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}"
        )

    def get_dashboard_data(self):
        with self.lock:
            return dict(self.data)


class GasAlertWindow:
    def __init__(self, node):
        self.node = node

        self.root = tk.Tk()
        self.root.title("Gas Detection and Safety Alert System")
        self.root.geometry("780x690")
        self.root.minsize(680, 600)
        self.root.configure(bg="#eef2f7")

        self.last_status = None
        self.last_source_event_time = 0.0
        self.critical_popup_active = False
        self.last_popup_time = 0.0
        self.last_history_status = None

        self.build_ui()

        self.root.protocol("WM_DELETE_WINDOW", self.close_window)
        self.root.after(250, self.refresh_dashboard)

    def build_ui(self):
        header = tk.Frame(self.root, bg="#172554", height=75)
        header.pack(fill="x")
        header.pack_propagate(False)

        tk.Label(
            header,
            text="GAS DETECTION SAFETY DASHBOARD",
            font=("Arial", 19, "bold"),
            fg="white",
            bg="#172554",
        ).pack(pady=(12, 2))

        tk.Label(
            header,
            text="Live monitoring | ROS 2 Humble | Gazebo Classic",
            font=("Arial", 10),
            fg="#dbeafe",
            bg="#172554",
        ).pack()

        self.status_frame = tk.Frame(
            self.root,
            bg="#64748b",
            height=90,
        )
        self.status_frame.pack(fill="x", padx=16, pady=12)
        self.status_frame.pack_propagate(False)

        self.status_label = tk.Label(
            self.status_frame,
            text="WAITING FOR SENSOR",
            font=("Arial", 22, "bold"),
            fg="white",
            bg="#64748b",
        )
        self.status_label.pack(pady=(12, 0))

        self.status_detail = tk.Label(
            self.status_frame,
            text="Waiting for live gas sensor data...",
            font=("Arial", 10),
            fg="white",
            bg="#64748b",
        )
        self.status_detail.pack(pady=4)

        readings = ttk.LabelFrame(
            self.root,
            text="Live Sensor Readings",
            padding=12,
        )
        readings.pack(fill="x", padx=16, pady=5)

        self.gas_label = self.make_value_row(
            readings, "Gas concentration:", "0.0 ppm", 0
        )
        self.temp_label = self.make_value_row(
            readings, "Temperature:", "0.0 °C", 1
        )
        self.humidity_label = self.make_value_row(
            readings, "Humidity:", "0.0 %", 2
        )
        self.health_label = self.make_value_row(
            readings, "Sensor health:", "Waiting", 3
        )

        positions = ttk.LabelFrame(
            self.root,
            text="Robot Position and Orientation",
            padding=12,
        )
        positions.pack(fill="x", padx=16, pady=5)

        self.robot_label = self.make_value_row(
            positions, "Robot position:", "Waiting for odometry", 0
        )
        self.heading_label = self.make_value_row(
            positions, "Robot heading:", "--", 1
        )

        source = ttk.LabelFrame(
            self.root,
            text="Detected Gas Source",
            padding=12,
        )
        source.pack(fill="x", padx=16, pady=5)

        self.source_label = self.make_value_row(
            source, "Source coordinates:", "Waiting for source", 0
        )
        self.source_state_label = self.make_value_row(
            source, "Detection status:", "Not detected", 1
        )

        history_frame = ttk.LabelFrame(
            self.root,
            text="Alert History",
            padding=8,
        )
        history_frame.pack(
            fill="both",
            expand=True,
            padx=16,
            pady=(5, 12),
        )

        self.history = tk.Listbox(
            history_frame,
            height=5,
            font=("Courier New", 9),
        )
        self.history.pack(fill="both", expand=True)

        self.add_history("Dashboard started; waiting for ROS 2 messages.")

    def make_value_row(self, parent, label_text, initial_value, row):
        tk.Label(
            parent,
            text=label_text,
            font=("Arial", 10, "bold"),
            bg="#f0f0f0",
            anchor="w",
            width=23,
        ).grid(row=row, column=0, sticky="w", padx=5, pady=4)

        value = tk.Label(
            parent,
            text=initial_value,
            font=("Arial", 10),
            anchor="w",
            bg="#f0f0f0",
        )
        value.grid(row=row, column=1, sticky="w", padx=5, pady=4)

        return value

    def add_history(self, message):
        timestamp = time.strftime("%H:%M:%S")

        self.history.insert(
            tk.END,
            f"[{timestamp}] {message}",
        )
        self.history.yview_moveto(1.0)

        while self.history.size() > 100:
            self.history.delete(0)

    def determine_status(self, data):
        if not data["gas_received"]:
            return "WAITING FOR SENSOR", "#64748b"

        if time.time() - data["last_gas_time"] > 3.0:
            return "SENSOR OFFLINE", "#475569"

        if not data["sensor_ok"]:
            return "SENSOR FAULT", "#7c3aed"

        gas = data["gas_ppm"]

        if gas >= 500:
            return "CRITICAL", "#b91c1c"
        elif gas >= 200:
            return "DANGER", "#ea580c"
        elif gas >= 100:
            return "WARNING", "#ca8a04"
        else:
            return "SAFE", "#15803d"

    def show_source_popup(self, data):
        source_x = data["source_x"]
        source_y = data["source_y"]
        source_z = data["source_z"]

        if source_x is None or source_y is None:
            coordinates = "Source coordinates are not available yet."
        else:
            coordinates = (
                f"X: {source_x:.3f} m\n"
                f"Y: {source_y:.3f} m\n"
                f"Z: {source_z:.3f} m"
            )

        gas_text = (
            f"{data['gas_ppm']:.1f} ppm"
            if data["gas_received"]
            else "Not available"
        )

        messagebox.showwarning(
            "GAS SOURCE DETECTED",
            "The gas source has been detected!\n\n"
            f"Source coordinates:\n{coordinates}\n\n"
            f"Current gas concentration: {gas_text}\n\n"
            "Please follow the safety procedure.",
            parent=self.root,
        )

    def show_critical_popup(self, data):
        source_x = data["source_x"]
        source_y = data["source_y"]

        if source_x is None or source_y is None:
            source_text = "Source coordinates not yet received."
        else:
            source_text = (
                f"Source X: {source_x:.3f} m\n"
                f"Source Y: {source_y:.3f} m"
            )

        messagebox.showerror(
            "CRITICAL GAS ALERT",
            "CRITICAL GAS CONCENTRATION DETECTED!\n\n"
            f"Gas concentration: {data['gas_ppm']:.1f} ppm\n"
            f"{source_text}\n\n"
            "Move away from the hazardous area and "
            "follow the safety procedure.",
            parent=self.root,
        )

    def refresh_dashboard(self):
        try:
            data = self.node.get_dashboard_data()
            status, color = self.determine_status(data)

            self.status_frame.configure(bg=color)
            self.status_label.configure(
                text=status,
                bg=color,
            )

            self.status_detail.configure(
                text=f"Last update: {time.strftime('%H:%M:%S')}",
                bg=color,
            )

            self.gas_label.configure(
                text=f"{data['gas_ppm']:.2f} ppm"
                if data["gas_received"]
                else "-- ppm"
            )
            self.temp_label.configure(
                text=f"{data['temperature']:.2f} °C"
                if data["gas_received"]
                else "-- °C"
            )
            self.humidity_label.configure(
                text=f"{data['humidity']:.2f} %"
                if data["gas_received"]
                else "-- %"
            )

            if not data["gas_received"]:
                health_text = "Waiting for messages"
            elif time.time() - data["last_gas_time"] > 3.0:
                health_text = "No recent messages"
            elif data["sensor_ok"]:
                health_text = "OK"
            else:
                health_text = "FAULT"

            self.health_label.configure(text=health_text)

            if data["odom_received"]:
                self.robot_label.configure(
                    text=(
                        f"X: {data['robot_x']:.3f} m, "
                        f"Y: {data['robot_y']:.3f} m"
                    )
                )
                self.heading_label.configure(
                    text=f"{math.degrees(data['robot_yaw']):.1f}°"
                )
            else:
                self.robot_label.configure(
                    text="Waiting for odometry"
                )
                self.heading_label.configure(text="--")

            if data["source_detected"]:
                self.source_label.configure(
                    text=(
                        f"X: {data['source_x']:.3f} m, "
                        f"Y: {data['source_y']:.3f} m, "
                        f"Z: {data['source_z']:.3f} m"
                    )
                )
                self.source_state_label.configure(
                    text="SOURCE DETECTED",
                    fg="#b91c1c",
                    font=("Arial", 10, "bold"),
                )
            else:
                self.source_label.configure(
                    text="Waiting for source coordinates"
                )
                self.source_state_label.configure(
                    text="Not detected",
                    fg="black",
                    font=("Arial", 10),
                )

            # Log status changes.
            if status != self.last_status:
                self.add_history(f"Status changed to {status}.")
                self.last_status = status

            # IMPORTANT:
            # Show a popup when a NEW source event is received.
            # This does not depend on the gas threshold.
            event_time = data["source_event_time"]

            if (
                data["source_detected"]
                and event_time > self.last_source_event_time
            ):
                self.last_source_event_time = event_time

                self.add_history(
                    "Gas source detected at "
                    f"X={data['source_x']:.3f}, "
                    f"Y={data['source_y']:.3f}."
                )

                self.root.bell()
                self.show_source_popup(data)

            # Show the critical popup when gas reaches 500 ppm.
            # Avoid reopening it continuously while the gas stays high.
            gas_is_critical = (
                data["gas_received"]
                and data["sensor_ok"]
                and data["gas_ppm"] >= 500
            )

            if gas_is_critical and not self.critical_popup_active:
                self.critical_popup_active = True
                self.add_history(
                    f"CRITICAL gas level: {data['gas_ppm']:.1f} ppm."
                )
                self.root.bell()
                self.show_critical_popup(data)

            elif not gas_is_critical:
                self.critical_popup_active = False

        except Exception as exc:
            self.node.get_logger().error(
                f"Dashboard refresh error: {exc}"
            )

        self.root.after(250, self.refresh_dashboard)

    def close_window(self):
        try:
            self.root.destroy()
        finally:
            if rclpy.ok():
                rclpy.shutdown()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)

    node = GasAlertNode()

    ros_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True,
    )
    ros_thread.start()

    try:
        window = GasAlertWindow(node)
        window.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

        node.destroy_node()


if __name__ == "__main__":
    main()

