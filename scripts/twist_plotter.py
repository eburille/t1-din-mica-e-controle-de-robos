#!/usr/bin/env python3
"""
Plota em tempo real o Twist publicado pelo nó twist_publisher.

Uso:
    python3 twist_plotter.py
    python3 twist_plotter.py --ros-args -p topic:=twist -p window:=15.0
    python3 twist_plotter.py --ros-args -r twist:=/meu_namespace/twist

Parâmetros:
    topic   (string) Tópico TwistStamped a assinar.      Padrão: "twist"
    window  (double) Janela de tempo exibida, em s.      Padrão: 10.0
"""

import threading
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

LINEAR_LABELS = ['vx', 'vy', 'vz']
ANGULAR_LABELS = ['wx', 'wy', 'wz']


class TwistPlotter(Node):
    def __init__(self):
        super().__init__('twist_plotter')

        self.declare_parameter('topic', 'twist')
        self.declare_parameter('window', 10.0)

        topic = self.get_parameter('topic').value
        self.window = float(self.get_parameter('window').value)

        self.lock = threading.Lock()
        self.t = deque()
        self.linear = [deque() for _ in range(3)]
        self.angular = [deque() for _ in range(3)]
        self.t0 = None

        self.create_subscription(TwistStamped, topic, self.twist_cb, 100)
        self.get_logger().info(f'Assinando "{topic}" (janela de {self.window:.1f} s)')

    def twist_cb(self, msg: TwistStamped):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = stamp
        t = stamp - self.t0

        lin = msg.twist.linear
        ang = msg.twist.angular

        with self.lock:
            self.t.append(t)
            for d, v in zip(self.linear, (lin.x, lin.y, lin.z)):
                d.append(v)
            for d, v in zip(self.angular, (ang.x, ang.y, ang.z)):
                d.append(v)

            # Descarta amostras fora da janela
            while self.t and self.t[0] < t - self.window:
                self.t.popleft()
                for d in self.linear + self.angular:
                    d.popleft()

    def snapshot(self):
        """Copia os dados de forma segura para a thread do gráfico."""
        with self.lock:
            return (list(self.t),
                    [list(d) for d in self.linear],
                    [list(d) for d in self.angular])


def main():
    rclpy.init()
    node = TwistPlotter()

    # O ROS roda em outra thread; a thread principal fica com o matplotlib
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig, (ax_lin, ax_ang) = plt.subplots(2, 1, sharex=True, figsize=(10, 7))
    fig.suptitle(f'Twist em "{node.get_parameter("topic").value}"')

    lin_lines = [ax_lin.plot([], [], label=l)[0] for l in LINEAR_LABELS]
    ang_lines = [ax_ang.plot([], [], label=l)[0] for l in ANGULAR_LABELS]

    ax_lin.set_ylabel('Velocidade linear [m/s]')
    ax_ang.set_ylabel('Velocidade angular [rad/s]')
    ax_ang.set_xlabel('Tempo [s]')
    for ax in (ax_lin, ax_ang):
        ax.grid(True)
        ax.legend(loc='upper right')

    def update(_frame):
        t, lin, ang = node.snapshot()
        if not t:
            return lin_lines + ang_lines

        for line, y in zip(lin_lines, lin):
            line.set_data(t, y)
        for line, y in zip(ang_lines, ang):
            line.set_data(t, y)

        ax_lin.set_xlim(max(0.0, t[-1] - node.window), max(t[-1], 1e-3))
        for ax in (ax_lin, ax_ang):
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)

        return lin_lines + ang_lines

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