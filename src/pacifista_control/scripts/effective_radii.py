import numpy as np
import pandas as pd

def calculate_effective_radii(contact_points, icrs):
    return np.linalg.norm(contact_points - icrs, axis=1)

def radius_function(angle, radii, num_points=5):
    angle_normalized = angle % (2 * np.pi / num_points)
    idx = int(angle_normalized * len(radii) / (2 * np.pi / num_points))
    return radii[idx]

def main():
    # Read the contact points and ICRs from the CSV files
    contact_points = pd.read_csv('/home/paradox/OELP_WS/src/pacifista_control/scripts/contact_points_1000.csv').values
    center_of_wheel = np.array([0, 0, 0])

    effective_radii = calculate_effective_radii(contact_points, center_of_wheel)

    # Example usage of radius_function:
    wheel_rotation_angle = np.deg2rad(1) # In radians; replace this with the actual wheel rotation angle
    # current_effective_radius = radius_function(wheel_rotation_angle, effective_radii)

    # print('Effective radii:')
    # print(effective_radii)
    # print('Example effective radius for wheel rotation angle of {}:'.format(wheel_rotation_angle))
    # print(current_effective_radius)

if __name__ == '__main__':
    main()


if __name__ == '__main__':
    main()
