#!/usr/bin/env python3
"""
Plota em tempo real a pose (geometry_msgs/PoseStamped) publicada pelo
pose_publisher.

Graficos:
    - Posicao x, y, z [m] ao longo do tempo
    - Orientacao roll, pitch, yaw [graus] ao longo do tempo
    - Trajetoria 3D do efetuador

Uso:
    python3 pose_plotter.py
    python3 pose_plotter.py --ros-args -p topic:=pose -p window:=15.0 -p trail:=3000
    python3 pose_plotter.py --ros-args -r pose:=/meu_namespace/pose

Parametros:
    topic   (string) Topico PoseStamped a assinar.            Padrao: "pose"
    window  (double) Janela de tempo dos graficos 2D, em s.   Padrao: 10.0
    trail   (int)    Maximo de pontos da trajetoria 3D.       Padrao: 2000
"""

import math
import threading
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

POS_LABELS = ['x', 'y', 'z']
RPY_LABELS = ['roll', 'pitch', 'yaw']


def quaternion_to_rpy(x, y, z, w):
    """Quaternio -> (roll, pitch, yaw) em radianos (convencao ZYX)."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def unwrap(prev, new):
    """Evita saltos de 360 graus: devolve 'new' continuo em relacao a 'prev'."""
    d = (new - prev + math.pi) % (2.0 * math.pi) - math.pi
    return prev + d


class PosePlotter(Node):
    def __init__(self):
        super().__init__('pose_plotter')

        self.declare_parameter('topic', 'pose')
        self.declare_parameter('window', 10.0)
        self.declare_parameter('trail', 2000)

        topic = self.get_parameter('topic').value
        self.window = float(self.get_parameter('window').value)
        trail = int(self.get_parameter('trail').value)

        self.lock = threading.Lock()
        self.t = deque()
        self.pos = [deque() for _ in range(3)]
        self.rpy = [deque() for _ in range(3)]
        self.trail = [deque(maxlen=trail) for _ in range(3)]
        self.t0 = None
        self.last_rpy = None

        self.create_subscription(PoseStamped, topic, self.pose_cb, 100)
        self.get_logger().info(f'Assinando "{topic}" (janela de {self.window:.1f} s)')

    def pose_cb(self, msg: PoseStamped):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp == 0.0:
            stamp = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = stamp
        t = stamp - self.t0

        p = msg.pose.position
        q = msg.pose.orientation
        rpy = quaternion_to_rpy(q.x, q.y, q.z, q.w)

        # Desenrola os angulos para a curva nao saltar em +-180 graus
        if self.last_rpy is not None:
            rpy = tuple(unwrap(a, b) for a, b in zip(self.last_rpy, rpy))
        self.last_rpy = rpy

        with self.lock:
            self.t.append(t)
            for d, v in zip(self.pos, (p.x, p.y, p.z)):
                d.append(v)
            for d, v in zip(self.rpy, rpy):
                d.append(math.degrees(v))
            for d, v in zip(self.trail, (p.x, p.y, p.z)):
                d.append(v)

            # Descarta amostras fora da janela (so dos graficos 2D)
            while self.t and self.t[0] < t - self.window:
                self.t.popleft()
                for d in self.pos + self.rpy:
                    d.popleft()

    def snapshot(self):
        """Copia os dados de forma segura para a thread do grafico."""
        with self.lock:
            return (list(self.t),
                    [list(d) for d in self.pos],
                    [list(d) for d in self.rpy],
                    [list(d) for d in self.trail])


def padded_limits(values, pad_ratio=0.1, min_span=0.1):
    lo, hi = min(values), max(values)
    span = max(hi - lo, min_span)
    mid = 0.5 * (hi + lo)
    return mid - 0.5 * span * (1 + pad_ratio), mid + 0.5 * span * (1 + pad_ratio)


def main():
    rclpy.init()
    node = PosePlotter()

    # O ROS roda em outra thread; a thread principal fica com o matplotlib
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig = plt.figure(figsize=(13, 7))
    fig.suptitle(f'Pose em "{node.get_parameter("topic").value}"')
    gs = GridSpec(2, 2, width_ratios=[1.2, 1], figure=fig)
    ax_pos = fig.add_subplot(gs[0, 0])
    ax_rpy = fig.add_subplot(gs[1, 0], sharex=ax_pos)
    ax_3d = fig.add_subplot(gs[:, 1], projection='3d')

    pos_lines = [ax_pos.plot([], [], label=l)[0] for l in POS_LABELS]
    rpy_lines = [ax_rpy.plot([], [], label=l)[0] for l in RPY_LABELS]

    ax_pos.set_ylabel('Posição [m]')
    ax_rpy.set_ylabel('Orientação [°]')
    ax_rpy.set_xlabel('Tempo [s]')
    for ax in (ax_pos, ax_rpy):
        ax.grid(True)
        ax.legend(loc='upper right')

    trail_line, = ax_3d.plot([], [], [], lw=1.2, color='tab:blue')
    tip_point, = ax_3d.plot([], [], [], 'o', color='tab:red')
    ax_3d.set_xlabel('x [m]')
    ax_3d.set_ylabel('y [m]')
    ax_3d.set_zlabel('z [m]')
    ax_3d.set_title('Trajetória do efetuador')

    def update(_frame):
        t, pos, rpy, trail = node.snapshot()
        if not t:
            return pos_lines + rpy_lines + [trail_line, tip_point]

        for line, y in zip(pos_lines, pos):
            line.set_data(t, y)
        for line, y in zip(rpy_lines, rpy):
            line.set_data(t, y)

        ax_pos.set_xlim(max(0.0, t[-1] - node.window), max(t[-1], 1e-3))
        for ax in (ax_pos, ax_rpy):
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)

        # Trajetoria 3D
        tx, ty, tz = trail
        trail_line.set_data(tx, ty)
        trail_line.set_3d_properties(tz)
        tip_point.set_data([tx[-1]], [ty[-1]])
        tip_point.set_3d_properties([tz[-1]])
        ax_3d.set_xlim(*padded_limits(tx))
        ax_3d.set_ylim(*padded_limits(ty))
        ax_3d.set_zlim(*padded_limits(tz))

        return pos_lines + rpy_lines + [trail_line, tip_point]

    # Mantém a referência para a animação não ser coletada pelo GC
    _anim = FuncAnimation(fig, update, interval=50, blit=False,
                          cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()