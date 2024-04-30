import open3d as o3d
import numpy as np




def ray_casting_for_visualization1(eye, cuboid_center,scene):

    # 创建射线
    rays = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
        fov_deg=60,
        center=cuboid_center,
        eye=eye,
        up=[0, 0, 1],
        width_px=5000,
        height_px=5000
    )


    # 计算射线与场景中的交点
    ans = scene.cast_rays(rays)
    hit = ans['t_hit'].isfinite()
    points = rays[hit][:,:3] + rays[hit][:,3:]*ans['t_hit'][hit].reshape((-1,1))
    #pcd = o3d.t.geometry.PointCloud(points)

    return rays, ans

def compute_intrinsic_matrix(fov_deg, width_px, height_px):
    # 水平视场角转换为弧度
    fov_rad = np.deg2rad(fov_deg)
    # 计算焦距（focal length）
    f = (width_px / 2) / np.tan(fov_rad / 2)
    cx = width_px / 2
    cy = height_px / 2
    intrinsic_matrix = np.array([[f, 0, cx],
                                 [0, f, cy],
                                 [0, 0, 1]])
    return intrinsic_matrix

# def compute_extrinsic_matrix(eye, center, up):
#     # 相机的z轴方向是从相机位置指向目标点的方向（相反）
#     z_axis = np.array(eye) - np.array(center)
#     z_axis /= np.linalg.norm(z_axis)
#     x_axis = np.cross(up, z_axis)
#     x_axis /= np.linalg.norm(x_axis)
#     y_axis = np.cross(z_axis, x_axis)
#     extrinsic_matrix = np.eye(4)
#     extrinsic_matrix[:3, :3] = np.stack([x_axis, y_axis, -z_axis], axis=1)
#     extrinsic_matrix[:3, 3] = -np.dot(extrinsic_matrix[:3, :3], np.array(eye))
#     return extrinsic_matrix



def compute_extrinsic_matrix(eye, center, up):
    """
    Compute the camera extrinsic matrix from eye point to center point using up vector.

    Parameters:
    eye : array_like
        The coordinates of the camera's position.
    center : array_like
        The point in space the camera is looking at.
    up : array_like
        The up direction vector for the camera.

    Returns:
    numpy.ndarray
        The 4x4 extrinsic matrix.
    """

    eye = np.array(eye).reshape(3)  # Reshape eye to ensure it is a 1D array of shape (3,)
    center = np.array(center).reshape(3)
    up = np.array(up).reshape(3)

    # Calculate camera's z-axis which points in the direction from the scene point to the camera
    z_axis = eye - center
    z_axis /= np.linalg.norm(z_axis)  # Normalize z-axis

    # Calculate x-axis as cross product of up vector and z-axis
    x_axis = np.cross(up, z_axis)
    x_axis /= np.linalg.norm(x_axis)  # Normalize x-axis

    # Calculate y-axis as cross product of z-axis and x-axis
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)  # Normalize y-axis

    # Create rotation matrix from world coordinates to camera coordinates
    rotation_matrix = np.stack([x_axis, y_axis, -z_axis], axis=1)

    # Create translation vector
    translation_vector = -np.dot(rotation_matrix, eye)

    # Combine rotation matrix and translation vector into a 4x4 extrinsic matrix
    extrinsic_matrix = np.eye(4)
    extrinsic_matrix[:3, :3] = rotation_matrix
    extrinsic_matrix[:3, 3] = translation_vector

    return extrinsic_matrix


def ray_casting_for_visualization(eye, cuboid_center, scene):
    width_px = 5000
    height_px = 5000
    fov_deg = 60
    up = [0, 0, 1]

    # 计算内参矩阵和外参矩阵
    intrinsic_matrix = compute_intrinsic_matrix(fov_deg, width_px, height_px)
    extrinsic_matrix = compute_extrinsic_matrix(eye, cuboid_center, up)

    # 创建射线
    rays = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
        o3d.core.Tensor(intrinsic_matrix, dtype=o3d.core.float64),
        o3d.core.Tensor(extrinsic_matrix, dtype=o3d.core.float64),
        width_px,
        height_px
    )

    # 计算射线与场景中的交点
    ans = scene.cast_rays(rays)
    hit = ans['t_hit'].isfinite()
    points = rays[hit][:,:3] + rays[hit][:,3:]*ans['t_hit'][hit].reshape((-1,1))

    return rays, ans





# def compute_extrinsic_matrix(eye, target, up, angle=0):
#     # 计算视点到目标点的方向向量
#     direction = np.array(target) - np.array(eye)
#     direction_normalized = direction / np.linalg.norm(direction)
#
#     # 如果指定了角度偏转，修改方向向量
#     if angle != 0:
#         # 计算旋转轴，即方向向量和up向量的叉积
#         rotation_axis = np.cross(direction_normalized, up)
#         rotation_axis_normalized = rotation_axis / np.linalg.norm(rotation_axis)
#         # 根据给定角度创建旋转矩阵
#         rotation_matrix = np.array(
#             [[np.cos(np.radians(angle)) + rotation_axis_normalized[0] ** 2 * (1 - np.cos(np.radians(angle))),
#               rotation_axis_normalized[0] * rotation_axis_normalized[1] * (1 - np.cos(np.radians(angle))) -
#               rotation_axis_normalized[2] * np.sin(np.radians(angle)),
#               rotation_axis_normalized[0] * rotation_axis_normalized[2] * (1 - np.cos(np.radians(angle))) +
#               rotation_axis_normalized[1] * np.sin(np.radians(angle))],
#              [rotation_axis_normalized[1] * rotation_axis_normalized[0] * (1 - np.cos(np.radians(angle))) +
#               rotation_axis_normalized[2] * np.sin(np.radians(angle)),
#               np.cos(np.radians(angle)) + rotation_axis_normalized[1] ** 2 * (1 - np.cos(np.radians(angle))),
#               rotation_axis_normalized[1] * rotation_axis_normalized[2] * (1 - np.cos(np.radians(angle))) -
#               rotation_axis_normalized[0] * np.sin(np.radians(angle))],
#              [rotation_axis_normalized[2] * rotation_axis_normalized[0] * (1 - np.cos(np.radians(angle))) -
#               rotation_axis_normalized[1] * np.sin(np.radians(angle)),
#               rotation_axis_normalized[2] * rotation_axis_normalized[1] * (1 - np.cos(np.radians(angle))) +
#               rotation_axis_normalized[0] * np.sin(np.radians(angle)),
#               np.cos(np.radians(angle)) + rotation_axis_normalized[2] ** 2 * (1 - np.cos(np.radians(angle)))]])
#         # 旋转方向向量
#         direction_normalized = np.dot(rotation_matrix, direction_normalized)
#
#     # 构建摄像机坐标系
#     z_axis = -direction_normalized
#     x_axis = np.cross(up, z_axis)
#     x_axis /= np.linalg.norm(x_axis)
#     y_axis = np.cross(z_axis, x_axis)
#
#     # 构建外参矩阵
#     extrinsic_matrix = np.eye(4)
#     extrinsic_matrix[0:3, 0] = x_axis
#     extrinsic_matrix[0:3, 1] = y_axis
#     extrinsic_matrix[0:3, 2] = z_axis
#     extrinsic_matrix[0:3, 3] = eye
#
#     return extrinsic_matrix


#
# def compute_extrinsic_matrix(eye, target, up, tilt_deg=0):
#     # Calculate directional vectors
#     forward = np.array(target) - np.array(eye)
#     forward /= np.linalg.norm(forward)  # Normalize
#
#     # Apply tilt if needed
#     if tilt_deg != 0:
#         # Rotate forward vector around the up vector by the tilt angle
#         tilt_rad = np.radians(tilt_deg)
#         axis = np.cross(forward, up)
#         axis /= np.linalg.norm(axis)  # Normalize rotation axis
#         forward = forward * np.cos(tilt_rad) + np.cross(axis, forward) * np.sin(tilt_rad) + axis * np.dot(axis, forward) * (1 - np.cos(tilt_rad))
#
#     # Re-normalize in case of numerical instability
#     forward /= np.linalg.norm(forward)
#
#     # Recalculate right and up vectors
#     right = np.cross(up, forward)
#     right /= np.linalg.norm(right)
#     actual_up = np.cross(forward, right)
#     actual_up /= np.linalg.norm(actual_up)
#
#     # Construct the extrinsic matrix
#     extrinsic_matrix = np.eye(4)
#     extrinsic_matrix[:3, 0] = right
#     extrinsic_matrix[:3, 1] = actual_up
#     extrinsic_matrix[:3, 2] = -forward
#     extrinsic_matrix[:3, 3] = eye
#
#     return extrinsic_matrix

def compute_plane_normal(p1, p2, p3):
    """
    Compute the normal vector of the plane defined by three points.
    Parameters:
    - p1, p2, p3 (array-like): Coordinates of the three points.

    Returns:
    - numpy.ndarray: The normal vector of the plane.
    """
    # Vector from p1 to p2
    v1 = np.array(p2) - np.array(p1)
    # Vector from p1 to p3
    v2 = np.array(p3) - np.array(p1)
    # Cross product of v1 and v2 gives the normal to the plane
    normal = np.cross(v1, v2)
    # Normalize the vector
    normal_normalized = normal / np.linalg.norm(normal)

    return normal_normalized

def ray_casting_for_visualization_3eyes(eye_center, eye_left, eye_right, center, scene):
    width_px = 5000
    height_px = 5000
    fov_deg = 60
    up = compute_plane_normal(eye_left, center, eye_right)  # 使用之前定义的函数计算up向量

    # 存储射线和交点结果
    rays = []
    ans = []
    cameras = []  # 存储相机视锥的可视化对象

    # 处理三个视点：中心，左侧，右侧
    for eye in [eye_center, eye_left, eye_right]:
        # 转换为Open3D Tensor
        eye_tensor = o3d.core.Tensor(eye, dtype=o3d.core.float64)
        center_tensor = o3d.core.Tensor(center, dtype=o3d.core.float64)
        up_tensor = o3d.core.Tensor(up, dtype=o3d.core.float64)

        # 直接使用 create_rays_pinhole 函数
        rays_current = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
            fov_deg=fov_deg,
            center=center_tensor,
            eye=eye_tensor,
            up=up_tensor,
            width_px=width_px,
            height_px=height_px
        )

        # 计算射线与场景中的交点
        ans_current = scene.cast_rays(rays_current)
        rays.append(rays_current)
        ans.append(ans_current)

    #     # 创建相机可视化，此处假设我们仍然需要extrinsic和intrinsic矩阵，仅用于可视化
    #     intrinsic_matrix = compute_intrinsic_matrix(fov_deg, width_px, height_px)  # 假设此函数仍然可用
    #     extrinsic_matrix = compute_extrinsic_matrix(eye, center, up)  # 假设此函数仍然可用
    #     camera = o3d.geometry.LineSet.create_camera_visualization(width_px, height_px, intrinsic_matrix,
    #                                                               extrinsic_matrix, scale=100.0)
    #     cameras.append(camera)
    #
    # # 设置不同颜色以区分不同的相机
    # cameras[0].paint_uniform_color([1, 0, 0])  # Red
    # cameras[1].paint_uniform_color([0, 1, 0])  # Green
    # cameras[2].paint_uniform_color([0, 0, 1])  # Blue

    return rays, ans, cameras

def visualize_rays_from_viewpoints(eye_center, eye_left, eye_right, center, scene, ray_length=150, colors=[[0, 1, 1], [0, 1, 1], [0, 1, 1]]):
    """
    Visualize rays from multiple viewpoints with specified colors and lengths.

    Parameters:
    - eye_center, eye_left, eye_right: Lists or numpy arrays containing the coordinates of the viewpoints.
    - center: The target point where the cameras look at.
    - scene: The Open3D scene for raycasting. Here assumed None for visualization only.
    - ray_length: The length to extend the rays for visualization.
    - colors: List of colors corresponding to each viewpoint's rays.

    Returns:
    - List of o3d.geometry.LineSet objects for visualization.
    """
    width_px = 5  # Small number of pixels
    height_px = 5
    fov_deg = 60
    # up = np.array([0, 0, 1])  # Assuming up vector
    up = compute_plane_normal(eye_left, center, eye_right)  # 使用之前定义的函数计算up向量

    # Store ray visualization objects
    rays_visualizations = []

    # Process each viewpoint
    viewpoints = [eye_center, eye_left, eye_right]
    for idx, eye in enumerate(viewpoints):
        # Create rays
        rays = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
            fov_deg=fov_deg,
            center=o3d.core.Tensor(center, dtype=o3d.core.float64),
            eye=o3d.core.Tensor(eye, dtype=o3d.core.float64),
            up=o3d.core.Tensor(up, dtype=o3d.core.float64),
            width_px=width_px,
            height_px=height_px
        )

        # Visualize each ray
        for i in range(rays.shape[0]):
            for j in range(rays.shape[1]):
                ray_origin = np.array(eye)
                ray_direction = rays[i, j, 3:6].numpy() * ray_length  # Extend ray direction for visualization
                line_points = [ray_origin, ray_origin + ray_direction]
                line = o3d.geometry.LineSet(
                    points=o3d.utility.Vector3dVector(line_points),
                    lines=o3d.utility.Vector2iVector([[0, 1]])
                )
                line.paint_uniform_color(colors[idx])  # Set color for this set of rays
                rays_visualizations.append(line)

    return rays_visualizations

# def ray_casting_for_visualization_3eyes(eye_center, eye_left, eye_right, center, scene):
#     width_px = 5000
#     height_px = 5000
#     fov_deg = 60
#     up = compute_plane_normal(eye_left, center, eye_right)
#     angle = 0  # 左右视锥偏转角度
#
#     # 存储射线和交点结果
#     rays = []
#     ans = []
#     cameras = []  # 存储相机视锥的可视化对象
#
#     # 创建三个视锥的射线和交点
#     for eye in zip([eye_center, eye_left, eye_right]):
#         intrinsic_matrix = compute_intrinsic_matrix(fov_deg, width_px, height_px)
#         extrinsic_matrix = compute_extrinsic_matrix(eye, center, up)
#
#         # 创建射线
#         rays_current = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
#             o3d.core.Tensor(intrinsic_matrix, dtype=o3d.core.float64),
#             o3d.core.Tensor(extrinsic_matrix, dtype=o3d.core.float64),
#             width_px,
#             height_px
#         )
#
#         # 计算交点
#         ans_current = scene.cast_rays(rays_current)
#         rays.append(rays_current)
#         ans.append(ans_current)
#
#         # 创建相机可视化
#         camera = o3d.geometry.LineSet.create_camera_visualization(width_px, height_px, intrinsic_matrix,
#                                                                   extrinsic_matrix, scale=100.0)
#         cameras.append(camera)
#
#     # 设置不同颜色以区分不同的相机
#     cameras[0].paint_uniform_color([1, 0, 0])  # Red
#     cameras[1].paint_uniform_color([0, 1, 0])  # Green
#     cameras[2].paint_uniform_color([0, 0, 1])  # Blue
#
#     return rays, ans, cameras





#
#
# def ray_casting_for_visualization_3eyes(eye, center, scene):
#     width_px = 5000
#     height_px = 5000
#     fov_deg = 60
#     up = [0, 0, 1]
#     displacement = 95  # 80 mm
#
#     rays = []
#     ans = []
#     # hit = []
#
#     # 计算内参矩阵
#     intrinsic_matrix = compute_intrinsic_matrix(fov_deg, width_px, height_px)
#     intrinsic_tensor = o3d.core.Tensor(intrinsic_matrix, dtype=o3d.core.float64)
#
#     # 创建三个外参矩阵，对应中心、左侧和右侧视点
#     eye_center = np.array(eye)
#     # eye_left = eye_center - np.array([displacement, 0, 0])
#     # eye_right = eye_center + np.array([displacement, 0, 0])
#
#     eye_left, eye_right = generate_eye_positions(eye_center, center, displacement)
#
#     extrinsic_matrix_center = compute_extrinsic_matrix(eye_center, center, up)
#     extrinsic_matrix_left = compute_extrinsic_matrix(eye_left, center, up, angle_deg=-10)
#     extrinsic_matrix_right = compute_extrinsic_matrix(eye_right, center, up, angle_deg=10)
#
#
#     rays_center = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
#         o3d.core.Tensor(intrinsic_matrix, dtype=o3d.core.float64),
#         o3d.core.Tensor(extrinsic_matrix_center, dtype=o3d.core.float64),
#         width_px,
#         height_px
#     )
#
#     rays_left = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
#         o3d.core.Tensor(intrinsic_matrix, dtype=o3d.core.float64),
#         o3d.core.Tensor(extrinsic_matrix_left, dtype=o3d.core.float64),
#         width_px,
#         height_px
#     )
#
#     rays_right = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
#         o3d.core.Tensor(intrinsic_matrix, dtype=o3d.core.float64),
#         o3d.core.Tensor(extrinsic_matrix_right, dtype=o3d.core.float64),
#         width_px,
#         height_px
#     )
#
#
#     ans_center = scene.cast_rays(rays_center )
#     # hit_center = ans_center['t_hit'].isfinite()
#     rays.append(rays_center)
#     ans.append(ans_center)
#
#     ans_left = scene.cast_rays(rays_left )
#     # hit_left = ans_left['t_hit'].isfinite()
#     rays.append(rays_left)
#     ans.append(ans_left)
#
#     ans_right = scene.cast_rays(rays_right )
#     # hit_right = ans_right['t_hit'].isfinite()
#     rays.append(rays_right)
#     ans.append(ans_right)
#
#     camera_center = o3d.geometry.LineSet.create_camera_visualization(width_px, height_px, intrinsic_matrix,
#                                                                      extrinsic_matrix_center, scale=100.0)
#     camera_left = o3d.geometry.LineSet.create_camera_visualization(width_px, height_px, intrinsic_matrix,
#                                                                    extrinsic_matrix_left, scale=100.0)
#     camera_right = o3d.geometry.LineSet.create_camera_visualization(width_px, height_px, intrinsic_matrix,
#                                                                     extrinsic_matrix_right, scale=100.0)
#     # Set colors to differentiate cameras
#     camera_center.paint_uniform_color([1, 0, 0])  # Red
#     camera_left.paint_uniform_color([0, 1, 0])    # Green
#     camera_right.paint_uniform_color([0, 0, 1])   # Blue
#
#     return rays, ans, camera_center, camera_left, camera_right


def generate_eye_positions(eye_center, center, displacement):
    """
    Generates positions for eye_left and eye_right based on a specified displacement
    along the intersection line of plane A (perpendicular to the line connecting eye_center and center)
    and plane B (horizontal plane containing eye_center).

    Parameters:
    - eye_center: Coordinates of the central viewpoint as a NumPy array.
    - center: Coordinates of the target center point as a NumPy array.
    - displacement: Distance in mm from eye_center to both eye_left and eye_right along the intersection line.

    Returns:
    - Tuple containing coordinates for eye_left and eye_right.
    """
    # Special case handling for zenith and nadir points
    if np.isclose(eye_center[0], center[0], atol=0.1) and np.isclose(eye_center[1], center[1], atol=0.1):
        # Directly above or below - handle by simple displacement along the x-axis
        eye_left = eye_center - np.array([displacement, 0, 0])
        eye_right = eye_center + np.array([displacement, 0, 0])
        print("It's the zenith point.")
    else:
        # Calculate the direction vector from eye_center to center
        direction_vector = np.array(center) - np.array(eye_center)
        direction_vector_normalized = direction_vector / np.linalg.norm(direction_vector)

        # Plane B is horizontal, so its normal can be the z-axis component disregarding
        normal_B = np.array([0, 0, 1])

        # Find the intersection line direction for planes A and B
        # It's the cross product of the direction_vector_normalized and the z-axis (normal_B)
        line_direction = np.cross(direction_vector_normalized, normal_B)
        line_direction_normalized = line_direction / np.linalg.norm(line_direction)

        # Calculate eye_left and eye_right by moving along the line_direction from eye_center
        eye_left = eye_center - line_direction_normalized * displacement
        eye_right = eye_center + line_direction_normalized * displacement
    print("eye_center, eye_left, eye_right:",eye_center, eye_left, eye_right)
    return eye_left, eye_right

def count_hits(ans):
    hits = []
    for result in ans:
        # primitive_ids 会记录击中的原始体索引，我们需要检查它们是否有效（非 INVALID_ID）
        valid_hits = result['primitive_ids'][result['t_hit'].isfinite()]
        # 计算每个结果中有效击中的数量
        hit_count = len(np.unique(valid_hits.numpy()))
        hits.append(hit_count)
    return hits


def count_unique_hits_per_view(ans_list):
    # 初始化一个列表用于存储每个视点的唯一面片索引集合
    unique_primitive_ids_per_view = []

    # 遍历每个视角的结果
    for ans in ans_list:
        primitive_ids = ans['primitive_ids']

        # 将primitive_ids转换为numpy数组以便处理
        primitive_ids_np = primitive_ids.numpy()

        # 筛选出有效击中的索引，假设4294967295代表无效击中
        valid_ids = primitive_ids_np[primitive_ids_np != 4294967295]

        # 从有效索引中提取唯一索引
        unique_primitive_ids = set(valid_ids)

        # 将唯一索引集合添加到列表中
        unique_primitive_ids_per_view.append(unique_primitive_ids)

    # 返回每个视点的唯一面片索引集合列表
    return unique_primitive_ids_per_view




def filter_hits_by_angle_for_three_views(mesh, rays_list, ans_list, min_angle_deg=0,
    projector_max_angle_deg=80, camera_max_angle_deg=70):


    valid_hits_per_view = []

    for i, (rays, ans) in enumerate(zip(rays_list, ans_list)):
        if i == 0:  # 特殊处理projector
            min_cos_angle = np.cos(np.radians(min_angle_deg))
            max_cos_angle = np.cos(np.radians(projector_max_angle_deg))
        else:  # 其他视锥使用一般条件
            min_cos_angle = np.cos(np.radians(min_angle_deg))
            max_cos_angle = np.cos(np.radians(camera_max_angle_deg))
        hit_indices = ans['primitive_ids'][ans['t_hit'].isfinite()].numpy()  # 获取有效击中的面片索引
        hit_normals = np.asarray(mesh.triangle_normals)[hit_indices]  # 获取被击中的面片的法线向量
        ray_directions = rays[ans['t_hit'].isfinite()][:, 3:6].numpy()  # 获取有效击中的射线方向，仅取方向部分

        valid_hits = []
        for idx, (normal, direction) in enumerate(zip(hit_normals, ray_directions)):
            # 计算法线和射线方向的余弦值
            cos_angle = np.dot(normal, -direction) / (np.linalg.norm(normal) * np.linalg.norm(direction))
            # 检查夹角是否在允许的范围内
            if min_cos_angle >= cos_angle >= max_cos_angle:
                valid_hits.append(hit_indices[idx])

        valid_hits_per_view.append(np.array(valid_hits))

    return valid_hits_per_view


def get_unique_valid_hits(valid_hit_triangle_indices_per_view):
    unique_valid_hit_triangle_indices_per_view = []

    for hit_indices in valid_hit_triangle_indices_per_view:
        # 使用set来自动去除重复的面片索引
        unique_hits = set(hit_indices)
        # 将唯一击中面片索引的集合添加到结果列表中
        unique_valid_hit_triangle_indices_per_view.append(unique_hits)

    return unique_valid_hit_triangle_indices_per_view


def compute_intersection_of_hits(unique_primitive_ids_per_view):
    if len(unique_primitive_ids_per_view) < 3:
        raise ValueError("Expected three sets of hit indices, but got fewer.")

    # 获取三个相机的击中面片索引集合
    set1, set2, set3 = unique_primitive_ids_per_view

    # 计算三个集合的交集
    hit_triangle_indices = set1.intersection(set2).intersection(set3)

    return hit_triangle_indices

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



def visualize_hit_faces(mesh, hit_triangles_indices):
    # 创建一个新的TriangleMesh实例
    visualized_mesh = o3d.geometry.TriangleMesh()

    # 复制顶点和面片数据
    visualized_mesh.vertices = o3d.utility.Vector3dVector(np.asarray(mesh.vertices))
    visualized_mesh.triangles = o3d.utility.Vector3iVector(np.asarray(mesh.triangles))

    # 确保每个顶点都有颜色，初始为白色
    vertex_colors = np.ones((len(mesh.vertices), 3))  # 白色
    visualized_mesh.vertex_colors = o3d.utility.Vector3dVector(vertex_colors)

    # 根据面片索引设置顶点颜色为红色
    for tri_idx in hit_triangles_indices:
        for vertex_idx in mesh.triangles[tri_idx]:
            visualized_mesh.vertex_colors[vertex_idx] = [1, 0, 0]  # 红色

    return visualized_mesh



def check_and_adjust_up_vector(eye, cuboid_center, default_up=[0, 0, 1]):
    # 计算 z_axis
    z_axis = np.array(cuboid_center) - np.array(eye)
    z_axis_normalized = z_axis / np.linalg.norm(z_axis)

    # 检查 up 向量是否与 z_axis 平行
    up_vector = np.array(default_up)
    if np.isclose(np.abs(np.dot(up_vector, z_axis_normalized)), 1.0, atol=1e-6):
        # 如果平行，改变 up 向量
        print("Up vector is parallel to view direction. Adjusting up vector.")
        if np.allclose(z_axis_normalized, [1, 0, 0]) or np.allclose(z_axis_normalized, [-1, 0, 0]):
            # 如果 z_axis 接近 x 轴，改用 y 轴作为 up 向量
            up_vector = np.array([0, 1, 0])
        else:
            # 否则使用 x 轴作为 up 向量
            up_vector = np.array([1, 0, 0])
    return up_vector


def visualize_camera_frustums(width_px, height_px, intrinsic_matrix, extrinsic_matrix_center, extrinsic_matrix_left, extrinsic_matrix_right):
    """
    Visualize camera frustums using Open3D's create_camera_visualization method.

    Parameters:
    - width_px: int, width of the view in pixels.
    - height_px: int, height of the view in pixels.
    - intrinsic_matrix: numpy.ndarray, the camera's intrinsic matrix.
    - extrinsic_matrix_center: numpy.ndarray, the extrinsic matrix for the center viewpoint.
    - extrinsic_matrix_left: numpy.ndarray, the extrinsic matrix for the left viewpoint.
    - extrinsic_matrix_right: numpy.ndarray, the extrinsic matrix for the right viewpoint.
    """
    # Create LineSet objects for each camera visualization
    camera_center = o3d.geometry.LineSet.create_camera_visualization(width_px, height_px, intrinsic_matrix, extrinsic_matrix_center, scale=1.0)
    camera_left = o3d.geometry.LineSet.create_camera_visualization(width_px, height_px, intrinsic_matrix, extrinsic_matrix_left, scale=1.0)
    camera_right = o3d.geometry.LineSet.create_camera_visualization(width_px, height_px, intrinsic_matrix, extrinsic_matrix_right, scale=1.0)

    return camera_center, camera_left, camera_right