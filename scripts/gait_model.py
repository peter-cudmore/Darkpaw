import numpy as np
import scipy as sp
import model
import pyglet
from pyglet.gl import *

FrontLeft = 0
FrontRight = 1
BackLeft = 2
BackRight = 3

eye4 = np.eye(4, dtype=np.float)


def RotateXY(angle):
    c = np.cos(angle)
    s = np.sin(angle)
    out = np.eye(4, dtype=np.float)
    out[0, 0] = c
    out[1, 1] = c
    out[0, 1] = -s
    out[1, 0] = s
    return out


def RotateXZ(angle):
    c = np.cos(angle)
    s = np.sin(angle)
    return np.array([[c,  0, -s, 0], [0,  1, 0, 0], [s,  0, c, 0], [0,  0, 0, 1]], dtype=np.float64)


def RotateYZ(angle):
    c = np.cos(angle)
    s = np.sin(angle)
    return np.array([[1,  0,  0, 0], [0,  c, -s, 0], [0,  s, c, 0], [0,  0, 0, 1]], dtype=np.float64)


def ReflectY():
    return np.array([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=np.float64)


def ReflectX():
    return np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=np.float64)


def Translate(x, y, z):
    out = np.eye(4, dtype=np.float)
    for i, v in enumerate((x, y, z)):
        out[i, 3] = v
    return out


Origin = np.array([0, 0, 0, 1.0], dtype=np.float)


class Leg:
    Leg_Pivot = 0
    Leg_Bottom = 1
    Leg_Top = 2
    Y_inner = 3
    Y_Outer = 4
    Pivot = 5
    D_bottom = 6
    Top_Motor_Arm = 7
    Top_Motor_Base = 8
    Bottom_Motor_Arm = 9
    Bottom_Motor_Base = 10
    Arm_Anchor = 11

    def __init__(self, front=True, left=True):

        self.front = front
        self.left = left

        self.lines = [
            self.Leg_Pivot, self.Leg_Top,
            self.Leg_Top, self.Leg_Bottom,
            self.Leg_Bottom, self.Leg_Pivot,    # leg
            self.Leg_Pivot, self.Pivot,
            self.Pivot, self.D_bottom,
            self.D_bottom, self.Leg_Pivot,        # D section
            self.Y_inner, self.Y_Outer,
            self.Y_Outer, self.Pivot,
            self.Pivot, self.Y_inner,             # Y section
            self.Leg_Top, self.Y_Outer,                                       # Closure of the parallelogram
            self.Y_inner, self.Top_Motor_Arm,
            self.Top_Motor_Arm, self.Top_Motor_Base,
            self.Top_Motor_Base, self.Pivot,  # Top motor RRRR
            self.D_bottom, self.Bottom_Motor_Arm,
            self.Bottom_Motor_Arm, self.Bottom_Motor_Base,
            self.Bottom_Motor_Base, self.Pivot,
            self.Arm_Anchor, self.Pivot
        ]

        self.batch = pyglet.graphics.Batch()
        self.vertex_list = self.batch.add_indexed(12, pyglet.graphics.GL_LINES, None, self.lines, 'v3f', 'c4B')
        for idx in range(len(self.vertex_list.colors)):
            self.vertex_list.colors[idx] = 255

    def update_leg_positions(self, motor_top, motor_bottom, motor_body):

        beta_Y = model.beta_y(motor_top)
        beta_D = model.beta_d(motor_bottom)
        beta_A = np.pi / 2 - model.beta_a(motor_body)

        anchor = Translate(model.body_width / 2, model.body_width / 2, 0)

        if not self.left:
            anchor = ReflectX() @ anchor

        if not self.front:
            anchor = ReflectY() @ anchor

        pivot = anchor @ RotateXY(beta_A) @ Translate(model.arm_anchor_to_pivot, 0, 0)

        leg_pivot = RotateXZ(beta_D) @ Translate(-model.D_section_e, 0, 0) @ RotateXZ(-beta_D)
        y_outer = RotateXZ(beta_Y) @ Translate(-model.Y_o, 0, model.Y_h) @ RotateXZ(-beta_Y)
        top_motor = Translate(-model.motor_offset_x, 0, model.motor_offset_top)
        bottom_motor = Translate(-model.motor_offset_x, 0, model.motor_offset_bottom)

        transforms = {
            self.Arm_Anchor: anchor,
            self.Leg_Pivot: pivot @ leg_pivot,
            self.Leg_Bottom: pivot @ leg_pivot @ RotateXZ(beta_Y) @ Translate(-model.leg_pivot_to_foot_x, 0, model.leg_pivot_to_foot_y),
            self.Leg_Top: pivot @ leg_pivot @ RotateXZ(beta_Y) @ Translate(-model.Y_o, 0, model.Y_h) @ RotateXZ(-beta_Y),
            self.Y_inner: pivot @ RotateXZ(beta_Y) @ Translate(-model.Y_i, 0, model.Y_h) @ RotateXZ(-beta_Y),
            self.Y_Outer: pivot @ y_outer,
            self.D_bottom: pivot @ RotateXZ(beta_D) @ Translate(0, 0, model.D_section_b) @ RotateXZ(-beta_D),
            self.Top_Motor_Arm: pivot @ top_motor @ RotateXZ(motor_top) @ Translate(0, 0, model.motor_arm_top) @ RotateXZ(-motor_top),
            self.Top_Motor_Base: pivot @ top_motor,
            self.Bottom_Motor_Base: pivot @ bottom_motor,
            self.Bottom_Motor_Arm: pivot @ bottom_motor @ RotateXZ(motor_bottom) @ Translate(0, 0, model.motor_arm_bottom) @ RotateXZ(-motor_bottom),
            self.Pivot: pivot
        }

        for vertex, transform in transforms.items():
            idx = vertex*3
            self.vertex_list.vertices[idx:idx+3] = (transform @ Origin)[0: 3]

    def draw(self):
        self.batch.draw()


class DarkpawModel:
    # Points

    def __init__(self):
        self.legs = {
            FrontLeft: Leg(True, True),
            FrontRight: Leg(True, False),
            BackLeft: Leg(False, True),
            BackRight: Leg(False, False)
        }
        self.motor_values = {
            FrontLeft: {'motor_top': 0, 'motor_bottom': 0, 'motor_body': 0},
            FrontRight: {'motor_top': 0, 'motor_bottom': 0, 'motor_body': 0},
            BackLeft: {'motor_top': 0, 'motor_bottom': 0, 'motor_body': 0},
            BackRight: {'motor_top': 0, 'motor_bottom': 0, 'motor_body': 0}
        }
        self.freq = 0.001
        self.phase = 0

    def update(self, dt):
        self.phase = np.mod(self.phase + self.freq * dt, 1)

        for leg, motor in self.motor_values.items():
            motor['motor_body'] += 0.2 * (1 - 2 * self.phase)
        self.update_positions()

    def update_positions(self):
        for index, leg in self.legs.items():
            leg.update_leg_positions(**self.motor_values[index])

    def draw(self):
        for leg in self.legs.values():
            leg.draw()


window = pyglet.window.Window()

theta = 0
phi = 0
radius = 100


class Camera:
    def __init__(self):
        self.polar = np.pi
        self.azimuth = 0
        self.radius = 300
        self.acceleration = 0.1

        self.verts = [
            (-20, 0, 0, 255, 0, 0, 255),
            (100, 0, 0, 255, 0, 0, 255),
            (0, -100, 0, 0, 255, 0, 255),
            (0, 100, 0, 0, 255, 0, 255),
            (0, 0, 0, 0, 0, 255, 255),
            (0, 0, 100, 0, 0, 255, 255)
        ]
        self.batch = pyglet.graphics.Batch()
        self.vertex_list = self.batch.add(len(self.verts), GL_LINES, None, 'v3f', 'c4B')

        for i in range(len(self.verts)):
            self.vertex_list.vertices[3*i:3*(i+1)] = self.verts[i][0:3]
            self.vertex_list.colors[4*i:4*(i +1)] = self.verts[i][3:7]

    def draw(self):
        self.batch.draw()

    @property
    def x(self):
        return self.radius * np.sin(self.polar) * np.cos(self.azimuth)

    @property
    def y(self):
        return self.radius * np.sin(self.polar) * np.sin(self.azimuth)

    @property
    def z(self):
        return self.radius * np.cos(self.polar)


camera = Camera()
darkpaw = DarkpawModel()


@window.event
def on_mouse_drag(x, y, dx, dy, buttons, modifiers):
    if buttons & pyglet.window.mouse.RIGHT:
        if buttons & pyglet.window.mouse.LEFT:
            camera.radius -= dy * camera.acceleration
            camera.radius = min(max(camera.radius, 10), 1000)
        else:
            camera.polar += dy*camera.acceleration
            camera.azimuth -= dx*camera.acceleration


def set_view():
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    gluLookAt(camera.x, camera.y, camera.z, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)


@window.event
def on_resize(width, height):
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()

    gluPerspective(65, width / float(height), .1, 1000)
    return pyglet.event.EVENT_HANDLED


@window.event
def on_draw():
    gl.glClear(GL_COLOR_BUFFER_BIT)
    set_view()
    darkpaw.draw()
    camera.draw()


if __name__ == '__main__':
    darkpaw.update_positions()
    update_freq = 1/10
    pyglet.clock.schedule_interval(darkpaw.update, update_freq)
    pyglet.app.run()
