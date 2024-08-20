# import numpy as np
#
# def angle():
#     # 定义两个点的坐标
#     point1 = np.array([-76.01671074, -127.84804927, 613.55604607])
#     point2 = np.array([-38.53177982, -64.80434153, 364.32442822])
#     # point2 = np.array([0.0,          0.0,         108.13248452])
#     # 计算两点间的向量
#     vector = point1 - point2
#
#     # 定义z轴向量
#     z_axis = np.array([0, 0, 1])
#
#     # 计算两向量的点积
#     dot_product = np.dot(vector, z_axis)
#
#     # 计算两向量的模长
#     vector_norm = np.linalg.norm(vector)
#     z_axis_norm = np.linalg.norm(z_axis)
#
#     # 计算两向量的夹角（弧度）
#     angle_radians = np.arccos(dot_product / (vector_norm * z_axis_norm))
#
#     # 转换为度数
#     angle_degrees = np.degrees(angle_radians)
#     print(angle_degrees)
#
#
#     # 定义旋转矩阵 R
#     R = np.array([
#         [0.984952136328966, -0.012650740756177, 0.172363708185124],
#         [-0.006088375048936, -0.999238186078382, -0.038548400382477],
#         [0.172720064932451, 0.036918914408529, -0.984278808533754]
#     ])
#
#     # 计算欧拉角
#     theta_y = np.arcsin(-R[2, 0])
#     theta_z = np.arctan2(R[1, 0], R[0, 0])
#     theta_x = np.arctan2(R[2, 1], R[2, 2])
#
#     # 转换为度数
#     theta_y_deg = np.degrees(theta_y)
#     theta_z_deg = np.degrees(theta_z)
#     theta_x_deg = np.degrees(theta_x)
#
#     print("偏航角 Theta_z (Degrees):", theta_z_deg)
#     print("俯仰角 Theta_y (Degrees):", theta_y_deg)
#     print("翻滚角 Theta_x (Degrees):", theta_x_deg)

import numpy as np

def generate_lefteye_positions(eye_center, center, displacement = 101, angle = 22):
    """
    Generates position and direction for eye_left based on a specified displacement
    along the intersection line of plane A (perpendicular to the line connecting eye_center and center)
    and plane B (horizontal plane containing eye_center).

    Parameters:
    - eye_center: Coordinates of the central viewpoint as a NumPy array.
    - center: Coordinates of the target center point as a NumPy array.
    - displacement: Distance in mm from eye_center to eye_left along the intersection line.

    Returns:
    - Coordinates for eye_left and direction vector from eye_left to the apex.
    """
    # Calculate the direction vector from eye_center to center
    direction_vector = np.array(center) - np.array(eye_center)
    direction_vector_normalized = direction_vector / np.linalg.norm(direction_vector)

    # Plane B is horizontal, so its normal can be the z-axis component disregarding
    normal_B = np.array([0, 0, 1])

    # Find the intersection line direction for planes A and B
    line_direction = np.cross(direction_vector_normalized, normal_B)
    line_direction_normalized = line_direction / np.linalg.norm(line_direction)

    # Calculate eye_left by moving along the line_direction from eye_center
    eye_left = eye_center - line_direction_normalized * displacement

    # Calculate the apex of the isosceles triangle
    apex_angle_rad = np.deg2rad(angle)
    d = displacement / (2 * np.tan(apex_angle_rad / 2))  # distance from the base midpoint to the apex
    apex = eye_center + direction_vector_normalized * d

    # Direction vector from eye_left to apex
    direction_from_left_to_apex = apex - eye_left
    direction_from_left_to_apex_normalized = direction_from_left_to_apex / np.linalg.norm(direction_from_left_to_apex)
    print(f"eye_center: {eye_center}, direction_from_center: {direction_vector_normalized}")
    print(f"eye_left: {eye_left}, direction_from_left: {direction_from_left_to_apex_normalized}")
    return eye_left, direction_from_left_to_apex_normalized

# Example usage


if __name__ == "__main__":
    # angle()
    eye_center = np.array([ 8.45086976e-15, 1.33790965e+02, 4.91409916e+02])
    center = np.array([0, 0, 18.10626149])
    generate_lefteye_positions(eye_center, center)