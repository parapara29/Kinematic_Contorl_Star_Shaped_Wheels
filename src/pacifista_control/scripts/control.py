import numpy as np
import matplotlib.pyplot as plt
from stl import mesh
from scipy.spatial import ConvexHull

def calculate_contact_points(vertices):
    distances_to_ground = vertices[:, 2]
    min_distance = np.min(distances_to_ground)

    tolerance = 1e-5
    z_offset = 0.001
    contact_indices = np.abs(distances_to_ground - (min_distance)) < tolerance

    filtered_vertices = vertices[contact_indices]
    y_offset = 100
    contact_indices_y = filtered_vertices[:, 1] < (np.max(filtered_vertices[:, 1]) + y_offset)
    contact_points = filtered_vertices[contact_indices_y]

    return contact_points

def calculate_velocities_and_icr(contact_points):
    angular_speed = 10 * 2 * np.pi
    angular_velocity = np.array([0, angular_speed, 0])

    linear_velocity = np.cross(angular_velocity, contact_points)

    icr = np.mean(contact_points, axis=0)

    return linear_velocity, angular_velocity, icr

def main():
    wheel_stl = mesh.Mesh.from_file('/home/paradox/OELP_WS/src/pacifista_control/scripts/wheel.stl')
    vertices = wheel_stl.vectors.reshape(-1, 3) / 1000

    contact_points = calculate_contact_points(vertices)
    linear_velocity, angular_velocity, icr = calculate_velocities_and_icr(contact_points)

    print('Contact points:')
    print(contact_points)
    print('Linear velocity:')
    print(linear_velocity)
    print('Angular velocity:')
    print(angular_velocity)
    print('ICR:')
    print(icr)

if __name__ == '__main__':
    main()
