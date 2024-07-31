import matplotlib.animation as animation
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import os

from utils.helpers import in2m

# (0, 0) at the center of the field
# +x points toward the opposing alliance station wall
# +y points left
class Field(object):
    FIELD_IMAGE = os.path.join(os.path.dirname(__file__), "../../../images/field.png")
    LENGTH = in2m(12 * 54 + 3.25)  # m
    WIDTH = in2m(12 * 26 + 11.25)  # m
    GOAL = (in2m(6.0), in2m(218.5))  # m

    OBSTACLES = [  # (x, y, radius)
        # TODO: Update for 2024
    ]

    def __init__(self, obstacles=True):
        if not obstacles:
            self.OBSTACLES = []

    def plot_field(self, ax):
        img = mpimg.imread(self.FIELD_IMAGE)
        ax.imshow(img, extent=[0.0, self.LENGTH, 0.0, self.WIDTH])
        # Plot obstacles
        for x, y, r in self.OBSTACLES:
            ax.add_artist(plt.Circle((x, y), r, color="m"))

    def plot_traj(self, robot, traj, save_file, plot_mod=10, save=False, quiet=False):
        fig, ax = plt.subplots()

        # Plot field and path
        self.plot_field(ax)
        plt.scatter(
            traj[:, 1],
            traj[:, 2],
            marker=".",
            color="r",
        )

        traj_size = traj.shape[0]
        for k in range(traj_size):
            if k % plot_mod == 0 or k == traj_size - 1:
                robot.plot(ax, traj[k, 1:4])

        fig.set_size_inches(9, 4.5)
        plt.savefig(
            os.path.join(os.path.dirname(__file__), "../plots/{}".format(save_file))
        )
        if not quiet:
            plt.show()

    def anim_traj(self, robot, traj, save_file, save_gif=False):
        fig, ax = plt.subplots()

        # Plot field and path
        self.plot_field(ax)
        plt.scatter(
            traj[:, 1],
            traj[:, 2],
            marker=".",
            color="r",
        )

        # Plot first pose
        robot.plot(ax, traj[0, 1:4])
        # Plot last pose
        robot.plot(ax, traj[-1, 1:4])

        # Animation function
        def animate(i):
            print("Rendering Frame: {}".format(i))
            # Hack to remove the old robot poses
            for item in ax.collections[7:]:
                item.remove()
            robot.plot(ax, traj[i, 1:4])
            return ax.collections

        anim = animation.FuncAnimation(
            fig, animate, frames=traj.shape[0], interval=20, blit=True, repeat=True
        )
        if save_gif:
            anim.save("out.gif", writer="pillow", fps=50)
        plt.show()
