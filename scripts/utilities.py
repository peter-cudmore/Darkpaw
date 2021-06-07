import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3D
import numpy as np
from typing import List


def update_active_transform_plot(axis: Axes3D,
                                 chain: List[np.ndarray],
                                 bones: List[Line3D]):

    n, _ = chain[-1].shape
    origin = np.zeros(shape=(n,), dtype=float)
    origin[-1] = 1
    last_point = origin
    x_max = y_max = z_max = 10
    x_min = y_min = z_min = -10
    for xform, bone in list(zip(chain, bones)):
        next_point = np.array(
            [float(x_i) for x_i in xform.dot(origin)]
        )
        x, y, z = zip(last_point[0:3], next_point[0:3])

        bone.set_data_3d(x, y, z)
        last_point = next_point.copy()
        x_max = max(x_max, max(*x))
        x_min = min(x_min, min(*x))
        y_max = max(y_max, max(*y))
        y_min = min(y_min, min(*y))
        z_max = max(z_max, max(*z))
        z_min = min(z_min, min(*z))

    axis.set_xlim(x_min, x_max)
    axis.set_ylim(y_min, y_max)
    axis.set_zlim(z_min, z_max)

    return bones


def create_active_transform_plot(figure, chain: List[np.ndarray]):
    ax = Axes3D(figure)

    ax.set_xlabel('x (body fwd)')
    ax.set_ylabel('y (body left)')
    ax.set_zlabel('z (body up)')
    ax.set_title('Bone Positions')

    bones = update_active_transform_plot(
        ax,
        chain,
        [ax.plot3D([], [], [])[0] for _ in chain]
    )

    return bones


def create_active_transform_animation(animated_angle=0, period=180):
    from matplotlib.animation import FuncAnimation
    from model import get_active_kinematic_chain, body_angle_bounds, height_angle_bounds, radial_angle_bounds

    angle_bounds = [
        body_angle_bounds,
        radial_angle_bounds,
        height_angle_bounds
    ]
    angle_start = [b[0] for b in angle_bounds]
    angle_ranges = [b[1] - b[0] for b in angle_bounds]
    angle_current = [0, 0, 0]

    chain = get_active_kinematic_chain(*angle_current)
    figure = plt.figure()
    bones = create_active_transform_plot(figure, chain)

    def update(frame):
        ax = plt.gca()
        angle_current[animated_angle] = angle_start[animated_angle] + \
                                        angle_ranges[animated_angle] * (frame % period) / period

        return update_active_transform_plot(
            ax,
            get_active_kinematic_chain(*angle_current),
            bones
        )

    anim = FuncAnimation(
        figure,
        update,
        frames=period,
        blit=True,
        repeat=True
    )
    return anim
