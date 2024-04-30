# import necessary libs
import open3d as o3d
import numpy as np
# import point_cloud_viewer
import copy


# main function
def main():
    ############################################
    ############ 3D visulization window
    ############################################

    # 创建可视化窗口并添加模型
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    # 在窗口中显示坐标轴
    coord_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=50, origin=[0, 0, 0])
    vis.add_geometry(coord_axes)

    ############################################
    ############ object to be 3D inspected
    ############################################

    # OBJ file path
    obj_file_path = "D:\\pathplanning\\pathplanningcodezhan\\1.obj"
    model = o3d.io.read_triangle_mesh(obj_file_path)
    current_center = model.get_center()
    # translation target
    target_position = [60, 0, 60]

    # calculation of vector
    translation_vector = np.array(target_position) - np.array(current_center)
    # intermediate check
    # print("translation vector is:",translation_vector)

    # Tranlating
    model.translate(translation_vector)
    # scaling
    scale_factor = 0.1
    model.scale(scale_factor, center=model.get_center())
    # orientation
    model_copy = copy.deepcopy(model)
    R = model_copy.get_rotation_matrix_from_xyz((-np.pi / 2, 0, 0))
    model_copy.rotate(R, center=(target_position))

    # calculate the normals
    model.compute_vertex_normals()
    model_copy.compute_vertex_normals()

    # add the obeject in 3D scene
    # vis.add_geometry(model)
    vis.add_geometry(model_copy)

    ############################################
    ############ plot a circular trajectory
    ############################################

    # XOZ plane circular trajectory
    radius = 50.0  # radius
    num_points = 100  # point on the circle
    theta = np.linspace(0, 2 * np.pi, num_points)  # angle
    x = radius * np.cos(theta)
    y = np.zeros_like(x)
    z = radius * np.sin(theta)

    # create a point cloud
    cloud = o3d.geometry.PointCloud()
    cloud.scale(500.0, center=cloud.get_center())
    points = np.column_stack((x, y, z)) + [0, 0, 0]
    cloud.points = o3d.utility.Vector3dVector(points)

    center_trajectory = model.get_center()
    print("tra", center_trajectory)

    cloud.translate(center_trajectory)
    points_new = np.column_stack((x, y, z))
    # add the point cloud
    vis.add_geometry(cloud)

    ############################################
    ############ import 5M 3D model
    ############################################

    obj_file_path_5m = "D:\\pathplanning\\pathplanningcodezhan\\5m.obj"
    model_5m = o3d.io.read_triangle_mesh(obj_file_path_5m)
    current_center_5m = model_5m.get_center()
    # translation
    target_position_5m = points[20]
    translation_vector_5m = np.array(target_position_5m) - np.array(current_center_5m)
    model_5m.translate(translation_vector_5m)

    ## orientation 5m, calculating the rotation matrix
    v2 = [1, 0, 0]
    # v1 = np.array(model_5m.get_center()) - np.array(model.get_center())
    v1 = np.array(model_5m.get_center()) - np.array(model.get_center())

    unit_v1 = v1 / np.linalg.norm(v1)
    unit_v2 = v2 / np.linalg.norm(v2)

    rotation_axis = np.cross(unit_v2, unit_v1)

    cosine_theta = np.dot(unit_v2, unit_v1)
    sine_theta = np.linalg.norm(rotation_axis)
    rotation_angle = np.arctan2(sine_theta, cosine_theta)

    if rotation_angle != 0:
        rotation_matrix = np.array([
            [cosine_theta + (1 - cosine_theta) * rotation_axis[0] ** 2,
             (1 - cosine_theta) * rotation_axis[0] * rotation_axis[1] - rotation_axis[2] * np.sin(rotation_angle),
             (1 - cosine_theta) * rotation_axis[0] * rotation_axis[2] + rotation_axis[1] * np.sin(rotation_angle)],
            [(1 - cosine_theta) * rotation_axis[1] * rotation_axis[0] + rotation_axis[2] * np.sin(rotation_angle),
             cosine_theta + (1 - cosine_theta) * rotation_axis[1] ** 2,
             (1 - cosine_theta) * rotation_axis[1] * rotation_axis[2] - rotation_axis[0] * np.sin(rotation_angle)],
            [(1 - cosine_theta) * rotation_axis[2] * rotation_axis[0] - rotation_axis[1] * np.sin(rotation_angle),
             (1 - cosine_theta) * rotation_axis[2] * rotation_axis[1] + rotation_axis[0] * np.sin(rotation_angle),
             cosine_theta + (1 - cosine_theta) * rotation_axis[2] ** 2]
        ])
    else:

        rotation_matrix = np.identity(3)

    # print("Translation matrix：")
    # print(rotation_matrix)
    model_5m.rotate(rotation_matrix, center=(model_5m.get_center()))
    scale_factor = 0.06  #

    model_5m.scale(scale_factor, center=model_5m.get_center())
    model_5m.compute_vertex_normals()
    vis.add_geometry(model_5m)

    ############################################
    ############ ray casting related
    ############################################
    scene = o3d.t.geometry.RaycastingScene()
    mesh_scene = o3d.t.io.read_triangle_mesh(obj_file_path)
    # translate
    mesh_scene.translate(translation_vector)
    # rotate
    mesh_scene.rotate(R, center=target_position)
    mesh_scene.scale(0.1, center=mesh_scene.get_center())
    # vis.add_geometry(mesh_scene.to_legacy())

    mesh_id = scene.add_triangles(mesh_scene)

    # here need to be extended, something to do with the parameters of 5M scanner, now I am waiting for it
    rays = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
        fov_deg=90,
        center=[10, 0, 40],
        eye=[2, 3, 0],
        up=[0, 1, 0],
        width_px=640,
        height_px=480,
    )
    ans = scene.cast_rays(rays)
    hit = ans['t_hit'].isfinite()
    points = rays[hit][:, :3] + rays[hit][:, 3:] * ans['t_hit'][hit].reshape((-1, 1))
    # pcd = o3d.t.geometry.PointCloud(points)

    # color the point cloud
    num_points = points.shape[0]
    colors = np.zeros((num_points, 3))
    colors[:, 2] = 1  # give all the points blue

    tensor_colors = o3d.core.Tensor(colors, dtype=o3d.core.Dtype.Float32)

    # color
    pcd = o3d.t.geometry.PointCloud(points)
    pcd.point['colors'] = tensor_colors

    #   tensor claas transformation to a normal one
    pcd = pcd.to_legacy()
    # pcd.points = o3d.utility.Vector3dVector(points)
    vis.add_geometry(pcd)

    vis.run()
    vis.destroy_window()


# 3D scanner 3D model


############################################
############ point cloud saving
############################################


############################################
############ point cloud visulization
############################################


############################################
############
############################################


############################################
############
############################################

if __name__ == "__main__":
    main()
