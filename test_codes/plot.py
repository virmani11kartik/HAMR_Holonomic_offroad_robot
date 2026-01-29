#!/usr/bin/env python3
import argparse, math, threading, sys
from collections import deque

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from PyQt5 import QtWidgets
import pyqtgraph as pg
import numpy as np

def yaw_from_quat(qx,qy,qz,qw):
    s = 2.0*(qw*qz + qx*qy)
    c = 1.0 - 2.0*(qy*qy + qz*qz)
    return math.atan2(s, c)

class XYPlotter(Node):
    def __init__(self, topic_a, topic_b, max_points=5000, every_n=1, heading=False):
        super().__init__('xy_plotter_fast')
        self.every_n = max(1, every_n)
        self.dec = 0
        self.heading = heading
        self.max_points = max_points

        self.ax_a, self.ay_a = deque(maxlen=max_points), deque(maxlen=max_points)
        self.ax_b, self.ay_b = deque(maxlen=max_points), deque(maxlen=max_points)
        self.yaw_a, self.yaw_b = deque(maxlen=max_points), deque(maxlen=max_points)

        self.create_subscription(PoseStamped, topic_a, self.cb_a, 50)
        self.create_subscription(PoseWithCovarianceStamped, topic_b, self.cb_b, 50)

        # Qt / pyqtgraph UI
        self.app = QtWidgets.QApplication(sys.argv)
        pg.setConfigOptions(antialias=False, useOpenGL=True)
        self.win = pg.GraphicsLayoutWidget(show=True, title='Fast X–Y Trajectory')
        self.plot = self.win.addPlot()
        self.plot.setLabel('bottom', 'X [m]'); self.plot.setLabel('left', 'Y [m]')
        self.plot.setAspectLocked(True)                # keep 1:1 aspect
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.curve_a = self.plot.plot(pen=pg.mkPen(width=2), name='A')
        self.curve_b = self.plot.plot(pen=pg.mkPen(width=2), name='B')

        # Faster updates via a QTimer (no blitting needed)
        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.on_timer)
        self.timer.start(50)  # ~20 FPS

    def _keep(self):
        self.dec = (self.dec + 1) % self.every_n
        return self.dec == 0

    def cb_a(self, msg: PoseStamped):
        if not self._keep(): return
        p = msg.pose.position
        self.ax_a.append(p.x); self.ay_a.append(p.y)
        if self.heading:
            o = msg.pose.orientation
            self.yaw_a.append(yaw_from_quat(o.x,o.y,o.z,o.w))

    def cb_b(self, msg: PoseWithCovarianceStamped):
        if not self._keep(): return
        p = msg.pose.pose.position
        self.ax_b.append(p.x); self.ay_b.append(p.y)
        if self.heading:
            o = msg.pose.pose.orientation
            self.yaw_b.append(yaw_from_quat(o.x,o.y,o.z,o.w))

    def on_timer(self):
        if self.ax_a:
            xa = np.fromiter(self.ax_a, float); ya = np.fromiter(self.ay_a, float)
            self.curve_a.setData(xa, ya, connect='finite')
        if self.ax_b:
            xb = np.fromiter(self.ax_b, float); yb = np.fromiter(self.ay_b, float)
            self.curve_b.setData(xb, yb, connect='finite')

    def exec(self, executor):
        # Spin ROS in background thread
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        # Run GUI
        exit_code = self.app.exec_()
        executor.shutdown()
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(exit_code)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic-a', default='/HAMR_Turret/pose')
    ap.add_argument('--topic-b', default='/robot_pose')
    ap.add_argument('--max-points', type=int, default=5000)
    ap.add_argument('--every-n', type=int, default=2)   # decimate for speed
    ap.add_argument('--heading', action='store_true')
    args = ap.parse_args()

    rclpy.init()
    node = XYPlotter(args.topic_a, args.topic_b,
                     max_points=args.max_points,
                     every_n=args.every_n,
                     heading=args.heading)
    execu = MultiThreadedExecutor()
    execu.add_node(node)
    node.exec(execu)

if __name__ == '__main__':
    main()
