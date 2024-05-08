import numpy as np
import open3d as o3d


#这里需要修改，最好是以两个camera计算而不是以eye_center计算
def calculate_view_angles_and_distances(viewpoint, mesh, hit_triangle_indices):
    # 提取所有三角形面片的顶点索引
    triangles = np.asarray(mesh.triangles)
    # 提取所有顶点坐标
    vertices = np.asarray(mesh.vertices)
    # 提取所有三角形的法线
    triangle_normals = np.asarray(mesh.triangle_normals)

    # 初始化返回列表
    angles_list = []
    distance_list = []

    # 遍历每个击中的三角形索引
    for index in hit_triangle_indices:
        # 获取该三角形的顶点索引
        triangle = triangles[index]
        # 获取三个顶点的坐标
        v1, v2, v3 = vertices[triangle[0]], vertices[triangle[1]], vertices[triangle[2]]
        # 计算三角形的中心
        centroid = (v1 + v2 + v3) / 3
        # 计算从三角形中心到视点的方向向量
        direction_to_viewpoint = viewpoint - centroid
        # 计算距离
        distance = np.linalg.norm(direction_to_viewpoint)
        # 单位化方向向量
        direction_to_viewpoint_normalized = direction_to_viewpoint / np.linalg.norm(direction_to_viewpoint)
        # 获取三角形的法线向量
        normal = triangle_normals[index]
        # 计算方向向量和法线向量的夹角余弦值
        cos_angle = np.dot(direction_to_viewpoint_normalized, normal)
        # 计算角度（弧度转换为度）
        angle = np.arccos(cos_angle) * (180.0 / np.pi)
        # 存储结果
        angles_list.append((index, angle))
        distance_list.append((index, distance))

    return angles_list, distance_list


def sum_of_angles(angles_list):
    total_angle = sum(angle for _, angle in angles_list)
    return total_angle

def adjusted_sum_of_distances(distance_list):
    adjustment = 480
    total_distance = sum((distance - adjustment) for _, distance in distance_list)
    return total_distance




def calculate_coverage_ratio(all_hits, objmesh):
    """
    计算选中面片的面积占总面片面积的比例。

    参数:
    - all_hits: 包含被选中的面片索引的集合。
    - objmesh: 包含所有面片的网格对象。

    返回:
    - coverage_ratio: 被选中面片的面积占总面片面积的比例。
    """
    # 计算所有三角形的面积
    vertices = np.asarray(objmesh.vertices)
    triangles = np.asarray(objmesh.triangles)
    total_area = 0
    selected_area = 0

    for i, triangle in enumerate(triangles):
        # 获取三角形顶点
        p1, p2, p3 = vertices[triangle[0]], vertices[triangle[1]], vertices[triangle[2]]
        # 计算三角形面积 using the cross product area formula
        area = 0.5 * np.linalg.norm(np.cross(p2 - p1, p3 - p1))
        total_area += area
        if i in all_hits:
            selected_area += area

    # 计算覆盖比例
    if total_area == 0:  # 防止除以零
        return 0
    coverage_ratio = selected_area / total_area
    return coverage_ratio
