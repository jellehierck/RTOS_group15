import numpy as np

from vectormath import Vector2


def calc_target_vector(angle0: float, angle1: float, path_vector: Vector2) -> Vector2:
    """
    Calculates the vector to the target w.r.t. the starting point (where angle0 is determined).

    :param angle0: Angle (radians) between the path_vector and the target at point0.
    :param angle1: Angle (radians) between the path_vector and the target at point1.
    :param path_vector: Path vector.
    :return: Target vector.
    """
    # Determine the inner angles of the triangle between point0, point1 and target
    alpha0 = angle0
    alpha1 = np.pi - angle1

    # Determine path length using the law of sines
    target_vector_length = path_vector.length * np.sin(alpha1) / np.sin(alpha0 + alpha1)
    # Create a vector using the length (rho) and the angle w.r.t. the x axis (theta)
    return Vector2(target_vector_length, path_vector.theta + angle0, polar=True)


def calc_target_point(angle0: float, angle1: float, path_vector: Vector2, point0 = None):
    """
    Calculates the absolute coordinates of the target in the world using the point0 coordinates.

    :param angle0: Angle (radians) between the path_vector and the target at point0.
    :param angle1: Angle (radians) between the path_vector and the target at point1.
    :param path_vector: Path vector.
    :param point0: Point0 where angle0 was determined (to get absolute reference coordinates in the world).
    :return: Target vector.
    """
    target_vector = calc_target_vector(angle0=angle0, angle1=angle1, path_vector=path_vector)
    if np.ndarray is None:
        point0 = np.array([0, 0])  # Assume point0 was the origin
    return np.array([target_vector.x + point0[0], target_vector.y + point0[1]])


if __name__ == '__main__':
    point0 = np.array([9, -5])  # At this point, the first camera angle was determined
    point1 = np.array([4, 4])  # At this point, the second camera angle was determined

    path_vector = Vector2(point1 - point0)
    print(path_vector.theta_deg)

    obj_angle0 = np.deg2rad(-40)  # clockwise wrt the front of the car (aka the direction it is driving)
    obj_angle1 = np.deg2rad(-112)

    print(calc_target_vector(obj_angle0, obj_angle1, path_vector))  # Expected: (2, 10)
    print(calc_target_point(obj_angle0, obj_angle1, path_vector, point0))  # Expected: (11, 5)
