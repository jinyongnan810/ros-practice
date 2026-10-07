#!/usr/bin/env python3
"""Command the example's position controllers with sliders and show joint feedback."""

import math
import time
import tkinter as tk
from tkinter import ttk
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class PositionCommandGui(Node):
    # Order must match each controller's joints parameter in robot_controllers.yaml.
    GROUPS = {
        'wheel_joint_controller': ('wheel_joint',),
        'arm_controller': ('arm_joint_1', 'arm_joint_2'),
    }

    def __init__(self):
        super().__init__('position_command_gui')
        description = self.declare_parameter('robot_description', '').value
        robot = ET.fromstring(description)
        self.limits = {}
        for names in self.GROUPS.values():
            for name in names:
                joint = robot.find(f"joint[@name='{name}']")
                limit = None if joint is None else joint.find('limit')
                if limit is None:
                    raise ValueError(f'Missing URDF limits for {name}')
                lower, upper = float(limit.get('lower')), float(limit.get('upper'))
                if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
                    raise ValueError(f'Invalid URDF limits for {name}')
                self.limits[name] = (lower, upper)

        self.root = tk.Tk()
        self.root.title('ros2_control position commands')
        self.updating_sliders = False
        self.feedback = {}
        self.initialized = set()
        self.targets = {}
        self.actual = {}
        self.scales = {}
        self.status = {}
        self.publishers_by_group = {}
        ttk.Label(self.root, text='Move sliders to send joint position targets (radians).').pack(
            padx=12, pady=10)

        for group, names in self.GROUPS.items():
            self.publishers_by_group[group] = self.create_publisher(
                Float64MultiArray, f'{group}/commands', 10)
            frame = ttk.LabelFrame(self.root, text=group)
            frame.pack(fill='x', padx=12, pady=6)
            self.status[group] = tk.StringVar(value='Waiting for feedback and controller...')
            for name in names:
                ttk.Label(frame, text=name).pack(anchor='w', padx=8)
                lower, upper = self.limits[name]
                self.targets[name] = tk.DoubleVar(value=max(lower, min(0.0, upper)))
                self.actual[name] = tk.StringVar(value='Actual: waiting for /joint_states')
                scale = ttk.Scale(frame, from_=lower, to=upper,
                                  variable=self.targets[name], state='disabled')
                scale.pack(fill='x', padx=8, pady=4)
                self.scales[name] = scale
                ttk.Label(frame, textvariable=self.actual[name]).pack(anchor='w', padx=8)
                self.targets[name].trace_add(
                    'write', lambda *args, group=group: self.publish_target(group))
            ttk.Label(frame, textvariable=self.status[group]).pack(anchor='w', padx=8, pady=6)

        self.subscription = self.create_subscription(
            JointState, 'joint_states', self.receive_feedback, 10)

    def receive_feedback(self, message):
        now = time.monotonic()
        for name, position in zip(message.name, message.position):
            if name in self.limits and math.isfinite(position):
                self.feedback[name] = (position, now)
                self.actual[name].set(f'Actual: {position:.3f} rad')

    def ready(self, group):
        now = time.monotonic()
        return (
            self.publishers_by_group[group].get_subscription_count() > 0
            and all(name in self.feedback and now - self.feedback[name][1] < 1.0
                    for name in self.GROUPS[group])
        )

    def publish_target(self, group):
        # Programmatic initialization must never send a movement command.
        if self.updating_sliders or group not in self.initialized or not self.ready(group):
            return
        values = []
        for name in self.GROUPS[group]:
            value = self.targets[name].get()
            if not math.isfinite(value):
                return
            lower, upper = self.limits[name]
            values.append(max(lower, min(value, upper)))
        self.publishers_by_group[group].publish(Float64MultiArray(data=values))
        self.status[group].set('Target: ' + ', '.join(f'{value:.3f}' for value in values) + ' rad')

    def tick(self):
        if not rclpy.ok():
            self.root.quit()
            return
        rclpy.spin_once(self, timeout_sec=0.0)
        for group, names in self.GROUPS.items():
            ready = self.ready(group)
            if ready and group not in self.initialized:
                self.updating_sliders = True
                try:
                    for name in names:
                        lower, upper = self.limits[name]
                        self.targets[name].set(max(lower, min(self.feedback[name][0], upper)))
                finally:
                    self.updating_sliders = False
                self.initialized.add(group)
                self.status[group].set('Ready — move a slider to command')
            elif not ready:
                self.initialized.discard(group)
                self.status[group].set('Waiting for fresh feedback and controller...')
            for name in names:
                self.scales[name].configure(state='normal' if ready else 'disabled')
        self.root.after(50, self.tick)

    def run(self):
        self.root.after(0, self.tick)
        self.root.mainloop()


def main():
    rclpy.init()
    node = None
    try:
        node = PositionCommandGui()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.root.destroy()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
