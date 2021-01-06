import threading

import matplotlib.pyplot as plt
import numpy as np

from easygopigo3 import EasyGoPiGo3

from vectormath import Vector2


class DriverSeat:

    def __init__(self, gopigo: EasyGoPiGo3, point0: np.ndarray = None, point1: np.ndarray = None):
        if point0 is None:  # Set point0 to the origin if none is given
            point0 = np.array([0, 0])
        if point1 is None:  # Set point1 to a specified point
            point1 = np.array([-80, 200])

        self.gopigo = gopigo
        self.trajectory = Trajectory(point0=point0, point1=point1)

        self.curr_angle = 270  # Starting angle in degrees (facing backwards, i.e. towards the negative y axis)
        self.target_a = Target()  # Instantiate the first target (left bottle)
        self.target_b = Target()  # Instantiate the second target (right bottle)

    def start_trajectory(self):
        # --------------------------------------------------------------------------------------
        # Stage 1: Spin on point0 until you find both bottles
        deg_to_turn = self.curr_angle - self.trajectory.p0_to_p1.theta_deg  # How far the car needs to turn
        self.gopigo.turn_degrees(degrees=deg_to_turn, blocking=True)  # Rotate the car

        self.curr_angle = self.curr_angle - deg_to_turn  # Update the current angle after the spin

        # Stage 1.5: Record the angle of both bottles in point0
        # TODO: Angle recording goes here somehow (replace next two lines)
        target_a_angle = np.deg2rad(9.7 - 21.8)  # Angle from the front of the car to target A
        target_b_angle = np.deg2rad(-7.8 - 21.8)  # Angle from the front of the car to target B

        # Store the angles in the internal targets
        target_a.set_angle(point_nr=0, point_coordinates=point0, angle=target_a_angle)
        target_b.set_angle(point_nr=0, point_coordinates=point0, angle=target_b_angle)

        # --------------------------------------------------------------------------------------
        # Stage 2: Drive to point1
        self.gopigo.drive_cm(dist=self.trajectory.p0_to_p1.rho)

        ## Stage 2.5: Record the angle of both bottles in point1 and calculate the rest of the trajectory
        # TODO: Angle recording goes here somehow (replace next two lines)
        target_a_angle = np.deg2rad(-7.6 - 21.8)  # Angle from the front of the car to target A
        target_b_angle = np.deg2rad(-30.3 - 21.8)  # Angle from the front of the car to target B

        # Store the angles in the internal targets
        target_a.set_angle(point_nr=1, point_coordinates=point1, angle=target_a_angle)
        target_b.set_angle(point_nr=1, point_coordinates=point1, angle=target_b_angle)

        # Calculate trajectory
        trajectory.set_targets(target_a=target_a_coordinates, target_b=target_b_coordinates)
        trajectory.calculate_remaining()

        # --------------------------------------------------------------------------------------
        ## Stage 3: Spin to the correct angle for point2 (next to the first target bottle)
        deg_to_turn = self.curr_angle - self.trajectory.p1_to_p2.theta_deg  # How far the car needs to turn
        self.gopigo.turn_degrees(degrees=deg_to_turn, blocking=True)  # Rotate the car

        self.curr_angle = self.curr_angle - deg_to_turn  # Update the current angle after the spin

        # --------------------------------------------------------------------------------------
        ## Stage 4: Drive to point2
        self.gopigo.drive_cm(dist=self.trajectory.p1_to_p2.rho)

        # --------------------------------------------------------------------------------------
        ## Stage 5: Spin to the correct angle for point3 (10 cm before the seconds target bottle)
        deg_to_turn = self.curr_angle - self.trajectory.p2_to_p3.theta_deg  # How far the car needs to turn
        self.gopigo.turn_degrees(degrees=deg_to_turn, blocking=True)  # Rotate the car

        self.curr_angle = self.curr_angle - deg_to_turn  # Update the current angle after the spin

        # --------------------------------------------------------------------------------------
        ## Stage 6: Drive to point3
        self.gopigo.drive_cm(dist=self.trajectory.p2_to_p3.rho)


class Trajectory:
    DISTANCE_FROM_TARGET_B = 10  # Stop 10 cm from target_b
    TARGET_A_ANGLE_SCALE = 1.3  # This angle offset will make the robot not collide with target_a but drive around

    def __init__(self, point0: np.ndarray, point1: np.ndarray, target_a: np.ndarray = None,
                 target_b: np.ndarray = None):
        """
        Initialize the trajectory. The trajectory needs to receive its first two points. It is optional to provide the
        target points, they can also be added later. The trajectory will calculate the remaining points on the
        trajectory when all data is provided.

        All coordinates are considered in CM

        :param point0: Origin of the robot. This point is where the first measurement of target_a and target_b locations
        is taken.
        :param point1: First destination point. This point is where the second measurement of target_a and target_b
        locations is taken.
        :param target_a: (optional) Coordinates of target_a (not the destination target).
        :param target_b: (optional) Coordinates of target_b (destination target).
        """
        self.point0 = point0
        self.point1 = point1

        self.target_a = target_a
        self.target_b = target_b

        # Next destination points
        self.point2 = None
        self.point3 = None

        # Vectors to destinations (these vectors assume <current point>_<next point>)
        self.p0_to_p1 = Vector2(self.point1[0] - self.point0[0], self.point1[1] - self.point0[1])
        self.p1_to_p2 = None
        self.p2_to_p3 = None

        # Vector paths (2D array containing the starting point and the destination point of each path segment)
        self.p0_to_p1_path = np.array([self.point0, self.p0_to_p1 + self.point0])
        self.p1_to_p2_path = None
        self.p2_to_p3_path = None

    def set_targets(self, target_a: np.ndarray, target_b: np.ndarray):
        """Set the target coordinates."""
        self.target_a = target_a
        self.target_b = target_b

    def calculate_remaining(self) -> None:
        """Calculate the remaining points and vectors based on point0, point1, target_a and target_b."""
        # target_a and target_b first need to be calculated before this function can be executed
        if self.target_a is None or self.target_b is None:
            raise ValueError("target_a and/or target_b are undefined. Use .set_targets() first.")

        # Vectors from p1 to ta and tb
        p1_tb_vector = Vector2(self.target_b[0] - self.point1[0],
                               self.target_b[1] - self.point1[1])
        p1_ta_vector = Vector2(self.target_a[0] - self.point1[0],
                               self.target_a[1] - self.point1[1])

        # Determine the vector from p1 to p2
        p1_ta_angle = p1_tb_vector.angle(p1_ta_vector)  # Calculate the angle between the path to tb and the path to ta
        p1_ta_proj = p1_ta_vector.length * np.cos(p1_ta_angle)  # Calculate at which height ta lies on the path to tb
        p1_p2_angle = p1_ta_angle * 1.3  # Scale the angle to p2 up so you do not hit ta

        self.p1_to_p2 = Vector2(p1_ta_proj / np.cos(p1_p2_angle),  # Calculate length of path from p1 to p2
                                p1_tb_vector.theta + p1_p2_angle,
                                # Calculate angle of global x axis and path from p1 to p2
                                polar=True)  # Submit as polar coordinates
        self.point2 = self.p1_to_p2 + self.point1  # Store the coordinates of point2
        self.p1_to_p2_path = np.array([self.point1, self.p1_to_p2 + self.point1])  # Store the path from p1 to p2

        # Determine the vector from p2 to tb and scale it so that the robot stops before hitting target_b
        p2_tb_vector = Vector2(self.target_b[0] - self.point2[0],
                               self.target_b[1] - self.point2[1])
        self.p2_to_p3 = p2_tb_vector.as_length(p2_tb_vector.length - self.DISTANCE_FROM_TARGET_B)
        self.point3 = self.p2_to_p3 + self.point2  # Store the coordinates of p3
        self.p2_to_p3_path = np.array([self.point2, self.p2_to_p3 + self.point2])  # Store the path from p2 to p3

    def get_next_vector(self, current_point) -> Vector2:
        """Return the vector to the next point based on which point you are currently in."""
        if current_point == 0:
            return self.p0_to_p1
        elif current_point == 1:
            return self.p1_to_p2
        elif current_point == 2:
            return self.p2_to_p3
        else:
            raise ValueError("No valid point was given.")


class Target:

    def __init__(self):
        self.point0 = None
        self.angle_in_p0 = None
        self.point1 = None
        self.angle_in_p1 = None
        self.coordinates = None

    def set_angle(self, point_nr: int, point_coordinates: np.ndarray, angle: float):
        if point_nr == 0:
            self.point0 = point_coordinates
            self.angle_in_p0 = angle
        elif point_nr == 1:
            self.point1 = point_coordinates
            self.angle_in_p1 = angle
        else:
            raise ValueError("No valid point was given.")

    def get_coordinates(self):
        if self.angle_in_p0 is None and self.angle_in_p1 is None:
            raise ValueError("One or two angles are undefined. Use .set_angle() first.")

        alpha0 = self.angle_in_p0
        alpha1 = np.pi - self.angle_in_p1

        path_vector = Vector2(point1 - point0)

        # Determine path length using the law of sines
        target_vector_length = path_vector.length * np.sin(alpha1) / np.sin(np.pi - alpha0 - alpha1)

        # Create a vector using the length (rho) and the angle w.r.t. the x axis (theta)
        target_vector = Vector2(target_vector_length, path_vector.theta + alpha0, polar=True)

        self.coordinates = np.array([target_vector.x + self.point0[0], target_vector.y + self.point0[1]])
        return self.coordinates


if __name__ == '__main__':
    # Set expected points (where bottles are located)
    t0_expected = np.array([-60, 350])
    t1_expected = np.array([60, 440])

    # Create trajectory
    point0 = np.array([0, 0])
    point1 = np.array([-80, 200])  # TODO: Determine point1 when spinning
    target_a = Target()
    target_b = Target()
    trajectory = Trajectory(point0, point1)

    ## Car starts at point0 [0,0]

    # Set the angles measured by the camera at point0
    target_a.set_angle(point_nr=0, point_coordinates=point0, angle=np.deg2rad(9.7 - 21.8))
    target_b.set_angle(point_nr=0, point_coordinates=point0, angle=np.deg2rad(-7.8 - 21.8))

    ## now the car will drive towards point1 (located at [-80, 200])

    # Set the angles measured by the camera at point1
    target_a.set_angle(point_nr=1, point_coordinates=point1, angle=np.deg2rad(-7.6 - 21.8))
    target_b.set_angle(point_nr=1, point_coordinates=point1, angle=np.deg2rad(-30.3 - 21.8))

    # The car can now calculate the exact coordinates of the target bottles
    target_a_coordinates = target_a.get_coordinates()
    target_b_coordinates = target_b.get_coordinates()

    ## The rest of the trajectory calculation happens here

    trajectory.set_targets(target_a=target_a_coordinates, target_b=target_b_coordinates)
    trajectory.calculate_remaining()

    ## This is only to plot our results (only for visualization)
    fig, ax = plt.subplots()  # Create a figure containing a single axes.
    ax.plot(*t0_expected, marker='o', markersize=7, color="red")  # Actual target points
    ax.plot(*t1_expected, marker='o', markersize=7, color="red")

    ax.plot(*target_a_coordinates, marker='o', markersize=7, color="green")  # Calculated target points
    ax.plot(*target_b_coordinates, marker='o', markersize=7, color="green")

    ax.plot(*point0, marker='o', markersize=7, color="blue")  # Measurement points
    ax.plot(*point1, marker='o', markersize=7, color="blue")

    ax.plot(*zip(*trajectory.p0_to_p1_path), linewidth=2)  # Path vector between point0 and point1
    ax.plot(*zip(*trajectory.p1_to_p2_path), linewidth=1, color="magenta", marker="o", markersize='7')
    ax.plot(*zip(*trajectory.p2_to_p3_path), linewidth=1, color="magenta", marker="o", markersize='7')
    plt.xlim(-100, 100)  # Set x bounds
    plt.ylim(-20, 450)  # Set y bounds
    plt.gca().set_aspect('equal', adjustable='box')  # Make the axes have equal scales
    plt.grid(True, which='both')  # Show grid

    plt.show()  # show plot
