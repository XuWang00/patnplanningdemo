import numpy as np
import open3d as o3d

def normalize_and_sum_angles(mesh, angles_list):
    # 提取所有三角形面片的顶点索引
    triangles = np.asarray(mesh.triangles)
    # 提取所有顶点坐标
    vertices = np.asarray(mesh.vertices)

    # 初始化求和变量
    angle_weighted_sum = 0.0

    # 处理每一个角度和索引
    for index, angle in angles_list:
        # 计算三角形面积
        triangle = triangles[index]
        v1, v2, v3 = vertices[triangle[0]], vertices[triangle[1]], vertices[triangle[2]]
        area = 0.5 * np.linalg.norm(np.cross(v2 - v1, v3 - v1))
        if angle > 90.0:
            normalized_angle = 0
        else:
            # 归一化角度 (0度->1, 90度->0)
            normalized_angle = 1 - (angle / 90.0)
            # 加权求和
            angle_weighted_sum += normalized_angle * area

    return angle_weighted_sum


def normalize_and_sum_distances(mesh, distance_list):
    # 提取所有三角形面片的顶点索引
    triangles = np.asarray(mesh.triangles)
    # 提取所有顶点坐标
    vertices = np.asarray(mesh.vertices)

    # 初始化求和变量
    distance_weighted_sum = 0.0

    # 处理每一个距离和索引
    for index, distance in distance_list:
        # 计算三角形面积
        triangle = triangles[index]
        v1, v2, v3 = vertices[triangle[0]], vertices[triangle[1]], vertices[triangle[2]]
        area = 0.5 * np.linalg.norm(np.cross(v2 - v1, v3 - v1))
        # 距离调整
        adjusted_distance = distance - 560
        # 归一化调整距离 (0mm -> 1, 20mm -> 0)
        if adjusted_distance < 0:
            normalized_distance = 1.0
        elif adjusted_distance > 20:
            normalized_distance = 0.0
        else:
            normalized_distance = 1 - (adjusted_distance / 20.0)
        # 加权求和
        distance_weighted_sum += normalized_distance * area

    return distance_weighted_sum


def calculate_total_area_of_triangles(mesh, hit_triangle_indices):
    # 确保提供的mesh是一个Open3D的TriangleMesh对象
    if not isinstance(mesh, o3d.geometry.TriangleMesh):
        raise TypeError("The mesh must be an Open3D TriangleMesh object.")

    # 获取三角形网格的顶点坐标
    vertices = np.asarray(mesh.vertices)

    # 初始化面积总和
    total_area = 0.0

    # 遍历所有击中的面片索引，计算每个面片的面积
    for idx in hit_triangle_indices:
        # 获取定义面片的三个顶点索引
        triangle = mesh.triangles[idx]
        # 获取顶点坐标
        v1, v2, v3 = vertices[triangle[0]], vertices[triangle[1]], vertices[triangle[2]]
        # 计算该面片的面积
        # 面片面积公式：0.5 * ||(v2-v1) x (v3-v1)||
        area = 0.5 * np.linalg.norm(np.cross(v2 - v1, v3 - v1))
        # 累加到总面积
        total_area += area

    return total_area

def calculate_value_c(a, b, e, f, mesh, angle_weighted_sum, distance_weighted_sum, hit_triangle_indices):
    """
    Calculate the value C using the formula:
    C = a + b * e * angle_weighted_sum + b * f * distance_weighted_sum

    Parameters:
    a (float): Constant term in the formula.
    b (float): Coefficient for the weighted sums in the formula.
    e (float): Coefficient for the angle weighted sum.
    f (float): Coefficient for the distance weighted sum.
    angle_weighted_sum (float): The weighted sum of angles, calculated from previous function.
    distance_weighted_sum (float): The weighted sum of distances, calculated from previous function.

    Returns:
    float: The calculated value of C.
    """
    # Calculate the value C according to the provided formula
    C = a*calculate_total_area_of_triangles(mesh, hit_triangle_indices) + b * e * angle_weighted_sum + b * f * distance_weighted_sum
    return C


def caculate_costfunction(mesh, angles_list, distance_list, hit_triangle_indices, a, b, e, f):

    angle_weighted_sum = normalize_and_sum_angles(mesh, angles_list)
    distance_weighted_sum = normalize_and_sum_distances(mesh, distance_list)
    c = calculate_value_c(a, b, e, f, mesh, angle_weighted_sum, distance_weighted_sum, hit_triangle_indices)

    return c