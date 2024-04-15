def highlight_cuboid_frame(cuboids, index):
    """
    创建并返回指定立方体的红色线框模型。

    参数:
    - cuboids: 一系列 AxisAlignedBoundingBox 对象的列表。
    - index: 要高亮显示的立方体的索引。

    返回:
    - line_set: 表示高亮立方体边框的线集对象。
    """
    # 获取指定立方体的AABB
    cuboid = cuboids[index]

    # 计算立方体的8个顶点
    points = []
    for i in [-1, 1]:
        for j in [-1, 1]:
            for k in [-1, 1]:
                point = cuboid.get_center() + np.array([i, j, k]) * (cuboid.get_extent() / 2)
                points.append(point)
    points = np.array(points)

    # 定义立方体的12条边
    lines = [[0,1], [1,3], [3,2], [2,0],
             [4,5], [5,7], [7,6], [6,4],
             [0,4], [1,5], [3,7], [2,6]]

    # 创建LineSet对象
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines),
    )

    # 设置线框颜色为红色
    colors = [[1, 0, 0] for i in range(len(lines))]  # 红色
    line_set.colors = o3d.utility.Vector3dVector(colors)

    return line_set


import numpy as np
import open3d as o3d

def is_line_of_sight_blocked(mesh, viewpoint, target_centroid):
    """
    检查从视点到目标三角面片中心的视线是否被遮挡。

    参数:
    - mesh: Open3D TriangleMesh 对象。
    - viewpoint: 视点的坐标。
    - target_centroid: 目标三角面片的质心坐标。

    返回:
    - blocked: 布尔值，如果视线被遮挡则为True，否则为False。
    """
    # 计算从视点到目标质心的方向向量
    direction = target_centroid - viewpoint
    direction /= np.linalg.norm(direction)  # 归一化方向向量

    # 创建光线
    ray_origins = np.array([viewpoint])
    ray_directions = np.array([direction])

    # 进行光线-网格相交测试
    mesh_rtree = o3d.geometry.RayMeshIntersection(mesh)
    hits = mesh_rtree.query_ray_intersection(ray_origins, ray_directions)

    # 如果有相交且相交点不是目标质心，则认为视线被遮挡
    blocked = False
    for hit in hits:
        hit_point = hit[0]
        distance_to_hit = np.linalg.norm(hit_point - viewpoint)
        distance_to_target = np.linalg.norm(target_centroid - viewpoint)
        if distance_to_hit < distance_to_target:
            blocked = True
            break

    return blocked

# 示例使用
mesh = o3d.io.read_triangle_mesh("path_to_your_mesh.obj")
viewpoint = np.array([x, y, z])  # 视点坐标
target_centroid = np.array([tx, ty, tz])  # 目标三角面片质心坐标

blocked = is_line_of_sight_blocked(mesh, viewpoint, target_centroid)
if blocked:
    print("Line of sight is blocked.")
else:
    print("Line of sight is clear.")


from typing import Union, Tuple
import numpy as np

def ray_triangle_intersection(
    vertices: np.ndarray,
    ray_origin: np.ndarray,
    ray_direction: np.ndarray,
    culling: bool = False,
    epsilon: float = 1e-6,
) -> Union[bool, Tuple[float, float, float]]:
    """
    实现Möller-Trumbore射线-三角形相交算法。

    参数:
    vertices : np.ndarray
        一个2D数组，包含三角形三个顶点的x, y, z坐标。
    ray_origin : np.ndarray
        射线的起点坐标。
    ray_direction : np.ndarray
        射线的方向向量。
    culling : bool, optional
        如果为True，忽略从三角形背面射入的射线。默认为False。
    epsilon : float, optional
        用于处理数值计算中的容差阈值，默认值为1e-6。

    返回值:
    如果射线与三角形不相交，返回False。否则，返回距离t和重心坐标u, v的元组。

    """

    # 提取三角形的三个顶点
    vertex_0 = vertices[0]
    vertex_1 = vertices[1]
    vertex_2 = vertices[2]

    # 计算三角形两边的向量
    edge_1 = vertex_1 - vertex_0
    edge_2 = vertex_2 - vertex_0

    # 计算射线方向向量和edge_2的外积
    p_vec = np.cross(ray_direction, edge_2)

    # 计算行列式，判断射线与三角形平面的关系
    determinant = np.dot(p_vec, edge_1)

    # 如果启用了剔除背面三角形的选项
    if culling:
        # 行列式小于等于0表示射线与三角形的法线同向或平行，即不相交
        if determinant < epsilon:
            return False

        # 计算从射线起点到三角形第一个顶点的向量
        t_vec = ray_origin - vertex_0
        # 计算重心坐标u
        u_ = np.dot(p_vec, t_vec)
        if u_ < 0.0 or u_ > determinant:
            return False

        # 计算t_vec和edge_1的外积
        q_vec = np.cross(t_vec, edge_1)
        # 计算重心坐标v
        v_ = np.dot(q_vec, ray_direction)
        if v_ < 0.0 or (u_ + v_) > determinant:
            return False

        # 计算交点到射线起点的距离t，并调整u和v的值
        inv_determinant = 1.0 / determinant
        t = np.dot(q_vec, edge_2) * inv_determinant
        u = u_ * inv_determinant
        v = v_ * inv_determinant

        return t, u, v

    # 如果不剔除背面三角形
    else:
        # 行列式接近0表示射线与三角形平行，不相交
        if np.abs(determinant) < epsilon:
            return False

        inv_determinant = 1.0 / determinant

        t_vec = ray_origin - vertex_0
        # 计算重心坐标u
        u = np.dot(p_vec, t_vec) * inv_determinant
        if u < 0.0 or u > 1.0:
            return False

        q_vec = np.cross(t_vec, edge_1)
        # 计算重心坐标v
        v = np.dot(q_vec, ray_direction) * inv_determinant
        if v < 0.0 or (u + v) > 1.0:
            return False

        # 计算交点到射线起点的距离t
        t = np.dot(q_vec, edge_2) * inv_determinant
        # 如果t小于epsilon，表示交点在射线起点之前，因此不相交
        if t < epsilon:
            return False
        return t, u, v


def count_visible_faces_for_viewpoint(viewpoint, highlighted_mesh, objmesh):
    """
    计算视点可见的目标区域内的面片个数和面积。

    参数:
    - viewpoint: 视点，形如 (point coordinates, view direction, view category)。
    - highlighted_mesh: 目标区域的mesh。
    - objmesh: 整个模型的mesh。

    返回:
    - count: 可见的面片个数。
    - visible_area: 可见的面片总面积。
    """
    # 解包视点信息
    ray_origin, view_direction, _ = viewpoint

    count = 0
    visible_area = 0.0

    # 遍历目标区域内的每个面片
    for face in highlighted_mesh.triangles:
        # 获取当前面片的顶点坐标
        triangle_vertices = np.asarray(face)

        # 检查射线与目标区域内的面片是否相交
        intersection_point = ray_triangle_intersection(ray_origin, view_direction, triangle_vertices)
        if intersection_point is None:
            # 如果没有相交点，则面片可见，更新计数和面积
            count += 1
            visible_area += compute_triangle_area(triangle_vertices)

    return count, visible_area

def compute_triangle_area(vertices):
    """
    计算三角形的面积。

    参数:
    - vertices: 三角形的顶点坐标。

    返回:
    - area: 三角形的面积。
    """
    # 使用海伦公式计算三角形面积
    a = np.linalg.norm(vertices[0] - vertices[1])
    b = np.linalg.norm(vertices[1] - vertices[2])
    c = np.linalg.norm(vertices[2] - vertices[0])
    s = (a + b + c) / 2
    area = np.sqrt(s * (s - a) * (s - b) * (s - c))
    return area
