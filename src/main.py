import open3d as o3d
import os
import numpy as np
from uniform_viewpoint_generator import *
from geometry import *
from window_visualizer import *
from model_loader import *
from partitioner import *
from postprocess import *
from dilation import *
from viewpoints_generator import *
from ray_triangle_intersection import *

#main workflow
def main():
    # Set the model file path
    model_directory = "D:\PATH_PLANNING\pp01\models"
    model_name = "1upprocessed01.obj"
    model_path = os.path.join(model_directory, model_name)


    app = o3d.visualization.gui.Application.instance
    app.initialize()

    # Create a visualization window
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Model Visualization", width=800, height=600)


    # Load the model
    # objmesh, wireframe = load_obj_and_create_wireframe(model_path)
    objmesh = load_obj(model_path)
    # # 移除孤立的三角面片
    # objmesh = remove_isolated_triangles(objmesh)

    rotation_angles = [0 , 0, 0]  # x-axis 90 degree
    objmesh = rotate_model(objmesh, rotation_angles)
    # Adjust model position
    objmesh = adjust_model_position(objmesh)

    aabb, obb = get_bbox(objmesh)
    max_dimension,length, width, height = bbox_dimensions(aabb)
    center, base_center = compute_object_center(aabb)

    #划分物体为立方体区块
    aabb_cuboids = generate_aabb_grid_base_centered(objmesh, cuboid_size = 41)
    aabb_cuboids = filter_cuboids_containing_surface(objmesh, aabb_cuboids)

    average_normals = visualize_average_normals(objmesh, aabb_cuboids)

    centers_with_ids = get_cuboids_centers(aabb_cuboids)
    # 迭代 centers_with_ids 来分别获取中心点和编号
    for center, idx in centers_with_ids:
        print(f"Cuboid {idx} center: {center}")
    # centers, idx = get_cuboids_centers(aabb_cuboids)
    # print(f"Center: {centers}, id:{idx}")

    cuboid_idx = 3
    highlight_line_set = highlight_cuboid_frame(aabb_cuboids, cuboid_idx)
    highlighted_mesh, complement_mesh = highlight_cuboid_vertices(objmesh, aabb_cuboids, cuboid_idx)

    view_stats = analyze_mesh_by_view(highlighted_mesh)


    # Create wireframe after transformations
    wireframe = create_wireframe(objmesh)
    # normals = visualize_mesh_normals(objmesh)

    # # Generate equally distributed viewpoints on a sphere
    # if max_dimension <= 500:
    #     radius = 500 + max_dimension
    #     viewpoints = generate_viewpoints_on_hemisphere(radius, base_center)
    #     arrow_list = visualize_viewpoints_as_arrows(viewpoints)
    # else:
    #     print("not  designed yet")


    # # Generate equally distributed viewpoints on a cylinder, use angle when max_dimension <= 300
    # if max_dimension <= 500:
    #     radius = 500 + length
    #     height = 500 + height
    #     viewpoints = generate_viewpoints_on_cylinder2(radius, height, base_center)
    #     arrow_list = visualize_viewpoints_as_arrows(viewpoints)
    # else:
    #     print("not  designed yet")

    # viewpoints = generate_viewpoints_on_hemiellipsoid(a = 500 + length, b = 500 + width, c = 500 + height, base_center = base_center, angle_step=20)
    # arrow_list = visualize_viewpoints_as_arrows(viewpoints)


    # viewpoints = generate_viewpoints_on_elliptical_cylinder(a = 500 + length, b = 500 + width, height = 500 + height, base_center = base_center, angle_step=30, height_step=300)
    # arrow_list = visualize_viewpoints_as_arrows(viewpoints)


    # # Print viewpoints and view directions for verification
    # for viewpoint in viewpoints:
    #     point, direction = viewpoint
    #     print(f"Viewpoint: {point}, Direction: {direction}")

    cuboid_center = get_cuboid_center(aabb_cuboids, cuboid_idx)
    radius = 500 + length
    viewpoints = generate_viewpoints_on_sphere1(radius, cuboid_center)

    filtered_viewpoints = filter_viewpoints_by_z(viewpoints)
    filtered_viewpoints = filter_viewpoints_by_area(filtered_viewpoints, view_stats, area_threshold=50.0)
    # filtered_viewpoints = filter_viewpoints_by_name(filtered_viewpoints, view = 'Frontview')
    arrow_list = visualize_viewpoints_as_arrows(filtered_viewpoints)


    # Print viewpoints and view directions for verification
    for viewpoint in viewpoints:
        point, direction, view= viewpoint
        print(f"Viewpoint: {point}, Direction: {direction}, View: {view}")
    count_viewpoints_by_view(viewpoints)


    process_viewpoints(filtered_viewpoints, highlighted_mesh, objmesh)


    # Add elements to the visualization window


    # vis.add_geometry(objmesh)
    vis.add_geometry(wireframe)
    ##法线可视化
    vis.add_geometry(average_normals)

    vis.add_geometry(aabb)
    vis.add_geometry(obb)



    for arrow in arrow_list:
        vis.add_geometry(arrow)

    # # 添加每个 AABB 区块到可视化
    # for aabb_box in aabb_cuboids:
    #     vis.add_geometry(aabb_box)

    vis.add_geometry(highlight_line_set)
    vis.add_geometry(highlighted_mesh)


    # Set rendering options
    render_option = vis.get_render_option()
    render_option.light_on = True
    render_option.background_color = np.array([0.05, 0.05, 0.05])  # Set background color to dark grey
    render_option.point_size = 5  # If it is a point cloud, you can set the size of the points


    # Add XYZ coordinate axes
    coordinate_frame = coordinate()
    vis.add_geometry(coordinate_frame)

    #Add 底平面
    ground_plane = create_ground_plane()
    vis.add_geometry(ground_plane)

    #Add 辅助线
    line_set = axis(base_center)
    vis.add_geometry(line_set)
    ray = generate_ray(np.pi / 2, np.pi / 2)
    vis.add_geometry(ray)

    #dialation部分，暂不可用
    # voxel_size = 10  # 可以根据需要调整体素尺寸
    # voxel_grid = voxelization(objmesh, voxel_size)
    # # voxel_coords = generate_voxel_dilate_xyz2(voxel_grid, expan_dis = 50 , limit_z = 1.0)
    # voxel_coords = voxel_expansion(voxel_grid, expan_dis = 500)
    # # voxel_coords = extract_outer_layer(filtered_coords, voxel_size, dilate_iter)
    # # mesh = create_surface_from_voxels(voxel_coords, voxel_size = 10)
    # point_cloud = visualize_dilated_voxels(voxel_coords)
    # vis.add_geometry(point_cloud)

    #offset部分，暂不可用
    # offset_mesh = offset_mesh_surface(objmesh, offset_distance=500)
    # vis.add_geometry(offset_mesh)




    # Run the visualization
    vis.run()


if __name__ == "__main__":
    main()
