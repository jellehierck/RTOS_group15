import matplotlib.pyplot as plt
import numpy as np

from vectormath import Vector2


class Trajectory:

    def __init__(self, point0=None, point1=None, targets: int = 2):
        if point0 is None:
            self.point0 = np.zeros(2)
        else:
            self.point0 = point0
        if point1 is None:
            self.point1 = np.zeros(2)
        else:
            self.point1 = point1

        self.target_coords = np.zeros([targets, 2])
        self.target_angles = np.zeros([targets, 2])

        self.path_vector = None
        self.path = None
        self.get_path()

    def get_path(self):
        self.path_vector = Vector2(self.point1 - self.point0)
        self.path = np.array([self.point0, self.path_vector + self.point0])
        return self.path

    def set_target_angle(self, target, angle, point_nr):
        self.target_angles[target][point_nr] = angle

    def get_target_coords(self, target):
        self.get_path()

        alpha0 = self.target_angles[target][0]
        alpha1 = np.pi - self.target_angles[target][1]

        # Determine path length using the law of sines
        target_vector_length = self.path_vector.length * np.sin(alpha1) / np.sin(np.pi - alpha0 - alpha1)

        # Create a vector using the length (rho) and the angle w.r.t. the x axis (theta)
        target_vector = Vector2(target_vector_length, self.path_vector.theta + alpha0, polar=True)

        self.target_coords[target] = np.array([target_vector.x + self.point0[0], target_vector.y + self.point0[1]])
        return self.target_coords[target]

    def calculate_remaining_trajectory(self):
        dest_vector = Vector2(self.target_coords[1][0] - self.point1[0], self.target_coords[1][1] - self.point1[1])
        t0_vector = Vector2(self.target_coords[0][0] - self.point1[0], self.target_coords[0][1] - self.point1[1])

        t0_angle = dest_vector.angle(t0_vector)
        print(np.rad2deg(t0_angle))

        dest_path = np.array([self.point1, dest_vector + self.point1])
        self.t0_path = np.array([self.point1, t0_vector + self.point1])
        return dest_path


if __name__ == '__main__':
    # Set expected points (where bottles are located)
    t0_expected = np.array([-60, 350])
    t1_expected = np.array([60, 440])

    # Create trajectory
    traj = Trajectory(point0=np.array([0, 0]), point1=np.array([-80, 200]), targets=2)

    ## Car starts at point0 [0,0]

    # Set the angles measured by the camera at point0
    traj.set_target_angle(target=0, angle=np.deg2rad(9.7 - 21.8), point_nr=0)
    traj.set_target_angle(target=1, angle=np.deg2rad(-7.8 - 21.8), point_nr=0)

    ## now the car will drive towards point1 (located at [-80, 200])

    # Set the angles measured by the camera at point1
    traj.set_target_angle(0, np.deg2rad(-7.6 - 21.8), 1)
    traj.set_target_angle(1, np.deg2rad(-30.3 - 21.8), 1)

    # The car can now calculate the exact coordinates of the target bottles
    t0 = traj.get_target_coords(0)
    t1 = traj.get_target_coords(1)

    ## The rest of the trajectory calculation still needs to happen here

    ## This is only to plot our results (only for visualization)
    fig, ax = plt.subplots()  # Create a figure containing a single axes.
    ax.plot(*t0_expected, marker='o', markersize=7, color="red")  # Actual target points
    ax.plot(*t1_expected, marker='o', markersize=7, color="red")

    ax.plot(*t0, marker='o', markersize=7, color="green")  # Calculated target points
    ax.plot(*t1, marker='o', markersize=7, color="green")

    ax.plot(*traj.point0, marker='o', markersize=7, color="blue")  # Measurement points
    ax.plot(*traj.point1, marker='o', markersize=7, color="blue")

    ax.plot(*zip(*traj.get_path()), linewidth=2)  # Path vector between point0 and poin1

    dest_path = traj.calculate_remaining_trajectory()
    ax.plot(*zip(*dest_path), linewidth=1, color="magenta", marker="o", markersize='7')
    ax.plot(*zip(*traj.t0_path), linewidth=1, color="magenta", marker="o", markersize='7')
    # plt.xlim(-100, 100)  # Set x bounds
    # plt.ylim(-20, 450)  # Set y bounds
    plt.gca().set_aspect('equal', adjustable='box')  # Make the axes have equal scales
    plt.grid(True, which='both')  # Show grid

    plt.show()  # show plot
