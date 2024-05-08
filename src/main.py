import open3d as o3d
import os
import numpy as np
import time
from uniform_viewpoint_generator import *
from geometry import *
from window_visualizer import *
from model_loader import *
from partitioner import *
# from dilation import *
from viewpoints_generator import *
from ray_casting import *
from vis_quality import *
from objective_function import *
from diagram import *
from log import *

#main workflow
def main():
    ##################################################
    #####             Initialization           #######
    ##################################################
    logger = setup_logging()
    start_time = time.time()
    # Set the model file path
    model_directory = "D:\\PATH_PLANNING\\pp01\\models"
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


    rotation_angles = [0 , 0, 0]  # x-axis 90 degree
    objmesh = rotate_model(objmesh, rotation_angles)
    # Adjust model position
    objmesh = adjust_model_position(objmesh)



    aabb, obb = get_bbox(objmesh)
    max_dimension,length, width, height = bbox_dimensions(aabb)
    center, base_center = compute_object_center(aabb)
    print(f"base_center: {base_center} center: {center}")

    # #划分物体为立方体区块
    # aabb_cuboids = generate_aabb_grid_base_centered(objmesh, cuboid_size = 41)
    # aabb_cuboids = filter_cuboids_containing_surface(objmesh, aabb_cuboids)
    #
    # # average_normals = visualize_average_normals(objmesh, aabb_cuboids)
    #
    # centers_with_ids = get_cuboids_centers(aabb_cuboids)
    # # 迭代 centers_with_ids 来分别获取中心点和编号
    # for center, idx in centers_with_ids:
    #     print(f"Cuboid {idx} center: {center}")
    # # centers, idx = get_cuboids_centers(aabb_cuboids)
    # # print(f"Center: {centers}, id:{idx}")
    #
    # cuboid_idx = 3
    # highlight_line_set = highlight_cuboid_frame(aabb_cuboids, cuboid_idx)
    # highlighted_mesh = highlight_cuboid_vertices(objmesh, aabb_cuboids, cuboid_idx)
    #
    # view_stats = analyze_mesh_by_view(highlighted_mesh)


    # Create wireframe after transformations
    wireframe = create_wireframe(objmesh)
    # normals = visualize_mesh_normals(objmesh)

    print("Initialization completed in {:.2f} seconds".format(time.time() - start_time))
    ################################################
    ##Uniformlly distributed viewpoints generation##
    ################################################

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


    #############################################
    ## viewpoints generation based on cuboids  ##
    #############################################
    # cuboid_center = get_cuboid_center(aabb_cuboids, cuboid_idx)
    # radius = 500 + length / 2
    # viewpoints = generate_viewpoints_on_sphere1(radius, cuboid_center)
    #
    # filtered_viewpoints = filter_viewpoints_by_z(viewpoints)
    # filtered_viewpoints = filter_viewpoints_by_area(filtered_viewpoints, view_stats, area_threshold=50.0)
    # filtered_viewpoints = filter_viewpoints_by_name(filtered_viewpoints)
    # arrow_list = visualize_viewpoints_as_arrows(filtered_viewpoints)


    # # Print viewpoints and view directions for verification
    # for viewpoint in viewpoints:
    #     point, direction, view= viewpoint
    #     print(f"Viewpoint: {point}, Direction: {direction}, View: {view}")
    # count_viewpoints_by_view(viewpoints)

    # print("viewpoints generation based on cuboids completed in {:.2f} seconds".format(time.time() - start_time))
#######################################################################################################
    ############################################
    #               ray casting               ##
    ############################################
    viewpoints = generate_viewpoints_on_ellipsoid1(a = 480 + length / 2, b = 480 + width/ 2, c = 480 + height/ 2,
                                                       center = center)
    filtered_viewpoints = filter_viewpoints_by_z(viewpoints, z_min=100)
    # filtered_viewpoints = filter_viewpoints_by_area(filtered_viewpoints, view_stats, area_threshold=50.0)
    # filtered_viewpoints = filter_viewpoints_by_name(filtered_viewpoints, 'Topview')

    arrow_list = visualize_viewpoints_as_arrows(filtered_viewpoints)

    # Print viewpoints and view directions for verification
    for viewpoint in filtered_viewpoints:
        point, direction, view= viewpoint
        print(f"Viewpoint: {point}, Direction: {direction}, View: {view}")

    # Create RaycastingScene
    scene = o3d.t.geometry.RaycastingScene()
    objmesh_t = o3d.t.geometry.TriangleMesh.from_legacy(objmesh)
    scene.add_triangles(objmesh_t )


    # 检查视点列表是否不为空
    if filtered_viewpoints:
        # 提取第一个视点的坐标
        first_viewpoint_coordinates = filtered_viewpoints[0][0]  # 第一个元素的第一个元组项是坐标
        # 将坐标转换为NumPy数组（如果还未是数组）
        eye_center = np.array(first_viewpoint_coordinates)
        print("Coordinates of the first viewpoint:", eye_center)
    else:
        print("No viewpoints generated.")


    eye_left, eye_right, apex = generate_eye_positions(eye_center, center, displacement =230)
    rays, ans = ray_casting_for_visualization_3eyes(eye_center, eye_left, eye_right, apex, scene)


    ####可视化视锥射线，目前最优解决方案########
    rays_viz = visualize_rays_from_viewpoints(eye_center, eye_left, eye_right, apex, scene)
    for line_set in rays_viz:
        vis.add_geometry(line_set)

    valid_hit_triangle_indices = filter_hits_by_angle_for_three_views(objmesh, rays, ans)
    # 打印结果，查看每个视锥有效击中的面片数量
    for idx, hits in enumerate(valid_hit_triangle_indices):
        print(f"Number of valid hits for view {idx + 1}: {len(hits)}")
    # 获取每个视锥中不重复的有效击中面片索引
    unique_valid_hits_per_view = get_unique_valid_hits(valid_hit_triangle_indices)
    # 打印结果，查看每个视锥的不重复有效击中面片数量
    for idx, hits in enumerate(unique_valid_hits_per_view):
        print(f"Number of unique valid hits for view {idx + 1}: {len(hits)}")
    # 获取三个视锥的面片索引交集
    hits_intersection = compute_intersection_of_hits(unique_valid_hits_per_view)
    print("Number of intersecting hits:", len(hits_intersection))
    # # 计算并打印所选面片的总面积
    # total_area = calculate_total_area_of_triangles(objmesh, hits_intersection)
    # print(f"Total area of the selected triangles: {total_area} square units")

    # # # 可视化被击中的面片
    visualized_mesh = visualize_hit_faces(objmesh, hits_intersection)
    vis.add_geometry(visualized_mesh)

    line_set3 = create_line_set3(eye_center, eye_left, eye_right)
    line_set2 = create_line_set2(eye_center, eye_left, eye_right, apex)
    vis.add_geometry(line_set3)
    vis.add_geometry(line_set2)

    coverage_ratio = calculate_coverage_ratio(hits_intersection, objmesh)
    # print("Number of objmesh.triangles:", len(objmesh.triangles))
    # coverage_ratio = len(hits_intersection) / len(objmesh.triangles)
    logger.info(f"Current coverage ratio: {coverage_ratio:.2%}")

    logger.info("Ray casting completed in {:.2f} seconds".format(time.time() - start_time))




    #############################################
    ##            Optical-quality              ##
    #############################################

    #这里需要修改，最好是以两个camera计算而不是以eye_center计算
    angles_list, distance_list = calculate_view_angles_and_distances(eye_center, objmesh, hits_intersection)



    c = calculate_costfunction(objmesh, angles_list, distance_list, hits_intersection,
                              a=2, b=3, e=2, f=3)

    print("Calculated value of Objective Function:", c)






    print("Optical-quality caculation completed in {:.2f} seconds".format(time.time() - start_time))

#################################################################################################################

    #############################################
    ##          TEST  Viewposition             ##
    #############################################
    # viewpoints = generate_viewpoints_on_ellipsoid1(a=560 + length / 2, b=560 + width / 2, c=560 + height / 2,
    #                                                center=center)
    # filtered_viewpoints = filter_viewpoints_by_z(viewpoints, z_min=100)
    # # filtered_viewpoints = filter_viewpoints_by_area(filtered_viewpoints, view_stats, area_threshold=50.0)
    #
    # arrow_list = visualize_viewpoints_as_arrows(filtered_viewpoints)
    #
    # # Create RaycastingScene
    # scene = o3d.t.geometry.RaycastingScene()
    # objmesh_t = o3d.t.geometry.TriangleMesh.from_legacy(objmesh)
    # scene.add_triangles(objmesh_t)
    #
    # views = ['Topview', 'Frontview', 'Leftview', 'Backview', 'Rightview']
    # best_viewpoints = []
    #
    # for view in views:
    #     current_viewpoints = filter_viewpoints_by_name(filtered_viewpoints, view)
    #     best_score = -np.inf
    #     best_viewpoint_data = None
    #     print("Now is", view)
    #
    #     for idx, viewpoint in enumerate(current_viewpoints):
    #         eye_center = np.array(viewpoint[0])  # 转换视点坐标为NumPy数组
    #         eye_left, eye_right = generate_eye_positions(eye_center, center, displacement=90)
    #         rays, ans, cameras = ray_casting_for_visualization_3eyes(eye_center, eye_left, eye_right, center, scene)
    #
    #         valid_hit_triangle_indices = filter_hits_by_angle_for_three_views(objmesh, rays, ans)
    #         unique_valid_hits_per_view = get_unique_valid_hits(valid_hit_triangle_indices)
    #         hits_intersection = compute_intersection_of_hits(unique_valid_hits_per_view)
    #
    #         angles_list, distance_list = calculate_view_angles_and_distances(eye_center, objmesh, hits_intersection)
    #         c = calculate_costfunction(objmesh, angles_list, distance_list, hits_intersection, a=2, b=3, e=2, f=3)
    #
    #         if c > best_score:
    #             best_score = c
    #             best_viewpoint_data = (idx, viewpoint[0], viewpoint[1], view, c)
    #
    #     if best_viewpoint_data:
    #         best_viewpoints.append(best_viewpoint_data)
    #         print(f"Best viewpoint for {view}: Index {best_viewpoint_data[0] + 1}, Score: {best_viewpoint_data[4]}")
    #
    #     print("Optical-quality calculation completed in {:.2f} seconds".format(time.time() - start_time))
    #
    # # 打印所有视角的最佳视点
    # for vp in best_viewpoints:
    #     print(f"Viewpoint {vp[0] + 1}: Position {vp[1]}, Direction {vp[2]}, View {vp[3]}, Score {vp[4]}")
    #
    # print("Optical-quality calculation completed in {:.2f} seconds".format(time.time() - start_time))
    #

    # #############################################
    # ##          TEST  All Viewpoints           ##
    # #############################################
    # viewpoints = generate_viewpoints_on_longitude_line(a=480 + length / 2, b=480 + width / 2, c=480 + height / 2,
    #                                                center=center)
    # filtered_viewpoints = filter_viewpoints_by_z(viewpoints, z_min=0)
    #
    # arrow_list = visualize_viewpoints_as_arrows(filtered_viewpoints)
    #
    # # Create RaycastingScene
    # scene = o3d.t.geometry.RaycastingScene()
    # objmesh_t = o3d.t.geometry.TriangleMesh.from_legacy(objmesh)
    # scene.add_triangles(objmesh_t)
    #
    # viewpoint_details = []  # 用于存储每个视点的详细信息
    #
    # # 检查视点列表是否不为空
    # if filtered_viewpoints:
    #     for idx, viewpoint in enumerate(filtered_viewpoints):
    #         eye_center = np.array(viewpoint[0])  # 转换视点坐标为NumPy数组
    #         eye_left, eye_right, apex = generate_eye_positions(eye_center, center, displacement=230)
    #         rays, ans, cameras = ray_casting_for_visualization_3eyes(eye_center, eye_left, eye_right, apex, scene)
    #
    #         valid_hit_triangle_indices = filter_hits_by_angle_for_three_views(objmesh, rays, ans)
    #         unique_valid_hits_per_view = get_unique_valid_hits(valid_hit_triangle_indices)
    #         hits_intersection = compute_intersection_of_hits(unique_valid_hits_per_view)
    #
    #         angles_list, distance_list = calculate_view_angles_and_distances(eye_center, objmesh, hits_intersection)
    #         c = calculate_costfunction(objmesh, angles_list, distance_list, hits_intersection, a=0, b=5, e=2, f=0)
    #
    #         # 存储视点详细信息
    #         viewpoint_details.append({
    #             "index": idx,
    #             "position": viewpoint[0],
    #             "direction": viewpoint[1],
    #             "view": viewpoint[2],
    #             "score": c
    #         })
    #
    #         print(f"Processed viewpoint {idx + 1}: Score {c}")
    #
    # else:
    #     print("No viewpoints generated.")
    #
    # # 打印所有视点的详细信息
    # for vp in viewpoint_details:
    #     print(
    #         f"Viewpoint {vp['index'] + 1}: Position {vp['position']}, Direction {vp['direction']}, View {vp['view']}, Score {vp['score']}")
    #
    # # 调用函数，绘制图表
    # plot_viewpoint_scores(viewpoint_details)
    #
    # print("Optical-quality calculation completed in {:.2f} seconds".format(time.time() - start_time))

    #############################################
    ## Add elements to the visualization window##
    #############################################

    # vis.add_geometry(objmesh)
    vis.add_geometry(wireframe)
    # ##法线可视化
    # vis.add_geometry(average_normals)

    vis.add_geometry(aabb)
    # vis.add_geometry(obb)



    for arrow in arrow_list:
        vis.add_geometry(arrow)

    # # 添加每个 AABB 区块到可视化
    # for aabb_box in aabb_cuboids:
    #     vis.add_geometry(aabb_box)

    # vis.add_geometry(highlight_line_set)
    # vis.add_geometry(highlighted_mesh)


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
