import numpy as np

def find_closest_point(origin, points):
    relative_points = points - origin
    distances = np.linalg.norm(relative_points, axis=-1)
    min_id = np.argmin(distances)

    return points[min_id]

points = np.array([[1.0, 1.0, 1.0], [2.0, 2.0, 2.0], [3.0, 3.0, 3.0]])
origin = np.array([0.0, 0.0, 0.0])

closest = find_closest_point(origin, points)

print(closest)