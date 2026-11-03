#!/usr/bin/env python3
"""
Interface para o gerador_trajetoria.

  - Campos para digitar a pose de referencia (x, y, z em m; roll, pitch, yaw
    em graus) e botao para publica-la em "referencia" (PoseStamped).
  - Plota a trajetoria publicada pelo gerador em "/trajeto" (PoseStamped):
      * trajetoria 3D, com ponto inicial, ponto atual e referencia
      * posicao x, y, z ao longo do tempo (tracejado = referencia)
      * orientacao roll, pitch, yaw ao longo do tempo (tracejado = referencia)
      * velocidade linear (deve mostrar o perfil trapezoidal)

Uso:
    python3 interface_referencia.py
    python3 interface_referencia.py --ros-args -p traj_topic:=/trajeto \
                                               -p ref_topic:=referencia \
                                               -p frame_id:=base_link

Parametros:
    ref_topic   (string) Topico onde a referencia e publicada.  Padrao: "referencia"
    traj_topic  (string) Topico da trajetoria a plotar.         Padrao: "/trajeto"
    frame_id    (string) frame_id da referencia publicada.      Padrao: "base_link"
    max_points  (int)    Maximo de amostras guardadas.          Padrao: 10000

Obs.: a referencia e publicada uma unica vez a cada clique. Se o
gerador_trajetoria ainda nao estava rodando, clique em "Publicar" de novo.
"""

import math
import threading
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec
from matplotlib.widgets import Button, TextBox

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

FIELDS = ['x [m]', 'y [m]', 'z [m]', 'roll [°]', 'pitch [°]', 'yaw [°]']
# Valores iniciais dos campos: x, y, z [m] e roll, pitch, yaw [°]
DEFAULT_VALUES = [-0.082, 0.0, 0.4, 81.0, 0.0, 0.0]
POS_LABELS = ['x', 'y', 'z']
RPY_LABELS = ['roll', 'pitch', 'yaw']
MAX_DRAW_POINTS = 2000   # pontos desenhados por curva (o resto e decimado)
SPEED_LAG = 5            # amostras usadas para estimar a velocidade


# --------------------------------------------------------------------------
# Utilidades
# --------------------------------------------------------------------------
def quaternion_to_rpy(x, y, z, w):
    """Quaternio -> (roll, pitch, yaw) em radianos (convencao ZYX)."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def rpy_to_quaternion(roll, pitch, yaw):
    """(roll, pitch, yaw) em radianos (ZYX) -> quaternio (x, y, z, w)."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def unwrap(prev, new):
    """Evita saltos de 2*pi: devolve 'new' continuo em relacao a 'prev'."""
    d = (new - prev + math.pi) % (2.0 * math.pi) - math.pi
    return prev + d


def wrap_near(angle_deg, around_deg):
    """Equivalente de 'angle_deg' (graus) mais proximo de 'around_deg'."""
    return around_deg + (angle_deg - around_deg + 180.0) % 360.0 - 180.0


def linear_speed(t, pos, lag=SPEED_LAG):
    """Velocidade linear estimada por diferencas com 'lag' amostras de atraso."""
    n = len(t)
    v = np.full(n, np.nan)
    if n > lag:
        dp = np.linalg.norm(pos[:, lag:] - pos[:, :-lag], axis=0)
        dt = t[lag:] - t[:-lag]
        v[lag:] = np.divide(dp, dt, out=np.zeros_like(dp), where=dt > 1e-9)
    return v


def set3d(line, x, y, z):
    line.set_data(x, y)
    line.set_3d_properties(z)


# --------------------------------------------------------------------------
# No ROS
# --------------------------------------------------------------------------
class ReferenceGui(Node):
    def __init__(self):
        super().__init__('interface_referencia')

        self.declare_parameter('ref_topic', 'referencia')
        self.declare_parameter('traj_topic', '/trajeto')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('max_points', 10000)

        self.ref_topic = self.get_parameter('ref_topic').value
        self.traj_topic = self.get_parameter('traj_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.max_points = int(self.get_parameter('max_points').value)

        self.lock = threading.Lock()
        self.ref = None          # ultima referencia publicada (x,y,z,r,p,y em graus)
        self.last_pose = None    # ultima pose recebida (x,y,z,r,p,y em graus)
        self.clear()

        self.pub = self.create_publisher(PoseStamped, self.ref_topic, 10)
        self.create_subscription(PoseStamped, self.traj_topic, self.traj_cb, 100)
        self.get_logger().info(
            f'Referencia em "{self.ref_topic}", trajetoria de "{self.traj_topic}"')

    def clear(self):
        with self.lock:
            n = self.max_points
            self.t = deque(maxlen=n)
            self.pos = [deque(maxlen=n) for _ in range(3)]
            self.rpy = [deque(maxlen=n) for _ in range(3)]   # graus, continuo
            self.t0 = None
            self.last_rpy = None

    def traj_cb(self, msg: PoseStamped):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp == 0.0:
            stamp = self.get_clock().now().nanoseconds * 1e-9

        p = msg.pose.position
        q = msg.pose.orientation
        rpy = quaternion_to_rpy(q.x, q.y, q.z, q.w)
        self.last_pose = (p.x, p.y, p.z) + tuple(math.degrees(a) for a in rpy)

        with self.lock:
            if self.t0 is None:
                self.t0 = stamp
            if self.last_rpy is not None:
                rpy = tuple(unwrap(a, b) for a, b in zip(self.last_rpy, rpy))
            self.last_rpy = rpy

            self.t.append(stamp - self.t0)
            for d, v in zip(self.pos, (p.x, p.y, p.z)):
                d.append(v)
            for d, v in zip(self.rpy, rpy):
                d.append(math.degrees(v))

    def publish_reference(self, x, y, z, roll_deg, pitch_deg, yaw_deg):
        qx, qy, qz, qw = rpy_to_quaternion(
            math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pub.publish(msg)

        self.ref = (x, y, z, roll_deg, pitch_deg, yaw_deg)

    def snapshot(self):
        """Copia os dados de forma segura para a thread do grafico."""
        with self.lock:
            t = np.array(self.t, dtype=float)
            pos = np.array([list(d) for d in self.pos], dtype=float).reshape(3, -1)
            rpy = np.array([list(d) for d in self.rpy], dtype=float).reshape(3, -1)
        return t, pos, rpy


# --------------------------------------------------------------------------
# Interface
# --------------------------------------------------------------------------
def main():
    rclpy.init()
    node = ReferenceGui()

    # O ROS roda em outra thread; a thread principal fica com o matplotlib
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig = plt.figure(figsize=(15, 8))
    try:
        fig.canvas.manager.set_window_title('Interface de referência')
    except Exception:
        pass

    gs = GridSpec(3, 2, figure=fig, left=0.28, right=0.98, top=0.94,
                  bottom=0.07, wspace=0.28, hspace=0.45, width_ratios=[1.1, 1])
    ax_3d = fig.add_subplot(gs[:, 0], projection='3d')
    ax_pos = fig.add_subplot(gs[0, 1])
    ax_rpy = fig.add_subplot(gs[1, 1], sharex=ax_pos)
    ax_vel = fig.add_subplot(gs[2, 1], sharex=ax_pos)

    # ---- Painel de controle ------------------------------------------------
    fig.text(0.04, 0.95, 'Referência', fontsize=14, weight='bold')
    boxes = []
    for i, label in enumerate(FIELDS):
        ax = fig.add_axes([0.10, 0.86 - i * 0.06, 0.13, 0.04])
        boxes.append(TextBox(ax, label + '  ', initial=str(DEFAULT_VALUES[i])))

    btn_pub = Button(fig.add_axes([0.04, 0.44, 0.19, 0.05]), 'Publicar referência')
    btn_cur = Button(fig.add_axes([0.04, 0.37, 0.19, 0.05]), 'Usar pose atual')
    btn_clr = Button(fig.add_axes([0.04, 0.30, 0.19, 0.05]), 'Limpar trajetória')
    status = fig.text(0.04, 0.25, '', fontsize=9, va='top')
    info = fig.text(0.04, 0.19, '', fontsize=9, va='top', family='monospace')

    def set_status(text, error=False):
        status.set_text(text)
        status.set_color('crimson' if error else 'darkgreen')
        fig.canvas.draw_idle()

    def on_publish(_event):
        try:
            vals = [float(b.text.replace(',', '.')) for b in boxes]
        except ValueError:
            set_status('Valor inválido: use só números (ex.: 0.35).', error=True)
            return
        node.publish_reference(*vals)
        set_status('Referência publicada.')

    def on_current(_event):
        pose = node.last_pose
        if pose is None:
            set_status('Nenhuma pose recebida ainda.', error=True)
            return
        for b, v in zip(boxes, pose):
            b.set_val(f'{v:.4f}')
        set_status('Campos preenchidos com a pose atual.')

    def on_clear(_event):
        node.clear()
        set_status('Trajetória limpa.')

    btn_pub.on_clicked(on_publish)
    btn_cur.on_clicked(on_current)
    btn_clr.on_clicked(on_clear)

    # ---- Graficos ------------------------------------------------------------
    pos_lines = [ax_pos.plot([], [], label=l)[0] for l in POS_LABELS]
    rpy_lines = [ax_rpy.plot([], [], label=l)[0] for l in RPY_LABELS]
    vel_line, = ax_vel.plot([], [], color='tab:purple')

    # Linhas tracejadas da referencia (mesma cor da curva correspondente)
    ref_pos = [ax_pos.axhline(0, ls='--', lw=0.9, color=ln.get_color(), visible=False)
               for ln in pos_lines]
    ref_rpy = [ax_rpy.axhline(0, ls='--', lw=0.9, color=ln.get_color(), visible=False)
               for ln in rpy_lines]

    ax_pos.set_ylabel('Posição [m]')
    ax_rpy.set_ylabel('Orientação [°]')
    ax_vel.set_ylabel('Vel. linear [m/s]')
    ax_vel.set_xlabel('Tempo [s]')
    for ax in (ax_pos, ax_rpy):
        ax.legend(loc='upper right', fontsize=8)
    for ax in (ax_pos, ax_rpy, ax_vel):
        ax.grid(True)

    trail_line, = ax_3d.plot([], [], [], lw=1.4, color='tab:blue', label='trajetória')
    start_pt, = ax_3d.plot([], [], [], 's', color='tab:green', label='início')
    tip_pt, = ax_3d.plot([], [], [], 'o', color='tab:orange', label='atual')
    ref_pt, = ax_3d.plot([], [], [], 'x', color='crimson', ms=11, mew=2.5,
                         label='referência')
    ax_3d.set_xlabel('x [m]')
    ax_3d.set_ylabel('y [m]')
    ax_3d.set_zlabel('z [m]')
    ax_3d.set_title('Trajetória do efetuador')
    ax_3d.set_box_aspect((1, 1, 1))
    ax_3d.legend(loc='upper left', fontsize=8)

    def update(_frame):
        t, pos, rpy = node.snapshot()
        ref = node.ref
        n = len(t)

        # Referencia
        if ref is not None:
            set3d(ref_pt, [ref[0]], [ref[1]], [ref[2]])
            for ln, v in zip(ref_pos, ref[:3]):
                ln.set_ydata([v, v])
                ln.set_visible(True)
            for i, ln in enumerate(ref_rpy):
                v = wrap_near(ref[3 + i], rpy[i, -1]) if n else ref[3 + i]
                ln.set_ydata([v, v])
                ln.set_visible(True)

        if n == 0:
            for ax in (ax_pos, ax_rpy, ax_vel):
                ax.relim()
                ax.autoscale_view()
            return []

        # Decimacao para manter o desenho leve
        step = max(1, n // MAX_DRAW_POINTS)
        idx = np.arange(0, n, step)
        if idx[-1] != n - 1:
            idx = np.append(idx, n - 1)

        speed = linear_speed(t, pos)
        for line, y in zip(pos_lines, pos):
            line.set_data(t[idx], y[idx])
        for line, y in zip(rpy_lines, rpy):
            line.set_data(t[idx], y[idx])
        vel_line.set_data(t[idx], speed[idx])

        for ax in (ax_pos, ax_rpy, ax_vel):
            ax.relim()
            ax.autoscale_view()

        # Trajetoria 3D (mesmo span nos tres eixos, para nao distorcer)
        xs, ys, zs = pos[0][idx], pos[1][idx], pos[2][idx]
        set3d(trail_line, xs, ys, zs)
        set3d(start_pt, [pos[0, 0]], [pos[1, 0]], [pos[2, 0]])
        set3d(tip_pt, [pos[0, -1]], [pos[1, -1]], [pos[2, -1]])

        mins, maxs = [], []
        for i, a in enumerate((xs, ys, zs)):
            lo, hi = float(a.min()), float(a.max())
            if ref is not None:
                lo, hi = min(lo, ref[i]), max(hi, ref[i])
            mins.append(lo)
            maxs.append(hi)
        span = max(max(h - l for l, h in zip(mins, maxs)), 0.2) * 1.15
        for setter, lo, hi in zip((ax_3d.set_xlim, ax_3d.set_ylim, ax_3d.set_zlim),
                                  mins, maxs):
            c = 0.5 * (lo + hi)
            setter(c - span / 2.0, c + span / 2.0)

        # Texto de informacao
        cur = pos[:, -1]
        vcur = np.nan_to_num(speed[-1])
        text = (f'Atual [m]\n x={cur[0]:+.3f} y={cur[1]:+.3f} z={cur[2]:+.3f}\n'
                f'Velocidade: {vcur:.3f} m/s')
        if ref is not None:
            dist = float(np.linalg.norm(cur - np.array(ref[:3])))
            text += f'\nDist. à ref.: {dist:.4f} m'
        info.set_text(text)

        return []

    # Mantém a referência para a animação não ser coletada pelo GC
    _anim = FuncAnimation(fig, update, interval=100, blit=False,
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