import open3d as o3d
import os
import numpy as np
import time
from uniform_viewpoint_generator import *
from geometry import *
from window_visualizer import *
from model_loader import *
from partitioner import *
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


    # Create wireframe after transformations
    wireframe = create_wireframe(objmesh)
    # normals = visualize_mesh_normals(objmesh)

    print("Initialization completed in {:.2f} seconds".format(time.time() - start_time))

    #############################################
    ##              Iteration                  ##
    #############################################
    viewpoints = generate_viewpoints_on_ellipsoid1(a=480 + length / 2, b=480 + width / 2, c=480 + height / 2,
                                                   center=center)
    filtered_viewpoints = filter_viewpoints_by_z(viewpoints, z_min=100)

    arrow_list = visualize_viewpoints_as_arrows(filtered_viewpoints)

    # Create RaycastingScene
    scene = o3d.t.geometry.RaycastingScene()
    objmesh_t = o3d.t.geometry.TriangleMesh.from_legacy(objmesh)
    scene.add_triangles(objmesh_t)

    best_viewpoints = []
    all_hits = set()
    # Pair each viewpoint with its original index
    remaining_viewpoints = [(idx, vp) for idx, vp in enumerate(filtered_viewpoints)]

    while remaining_viewpoints:
        current_best = None
        current_best_score = -np.inf
        best_idx = -1

        for idx, (original_idx, viewpoint) in enumerate(remaining_viewpoints):
            eye_center = np.array(viewpoint[0])
            eye_left, eye_right, apex = generate_eye_positions(eye_center, center, displacement=230)
            rays, ans = ray_casting_for_visualization_3eyes(eye_center, eye_left, eye_right, apex, scene)

            valid_hit_triangle_indices = filter_hits_by_angle_for_three_views(objmesh, rays, ans)
            unique_valid_hits_per_view = get_unique_valid_hits(valid_hit_triangle_indices)
            hits_intersection = compute_intersection_of_hits(unique_valid_hits_per_view)

            # Remove already selected hits
            hits_intersection = hits_intersection.difference(all_hits)

            angles_list, distance_list = calculate_view_angles_and_distances(eye_center, objmesh, hits_intersection)
            c = calculate_costfunction(objmesh, angles_list, distance_list, hits_intersection, a=2, b=3, e=3, f=2)

            if c > current_best_score:
                current_best = {
                    "index": original_idx,  # Use the original index
                    "position": viewpoint[0],
                    "direction": viewpoint[1],
                    "view": viewpoint[2],
                    "score": c,
                    "hits_intersection": hits_intersection
                }
                current_best_score = c
                best_idx = idx  # Track the best index to remove

        if current_best:
            best_viewpoints.append(current_best)
            print(f"Added best viewpoint: Index {current_best['index']}, Position {current_best['position']}, Direction {current_best['direction']}, View {current_best['view']}, Score {current_best['score']}")
            all_hits.update(current_best["hits_intersection"])
            del remaining_viewpoints[best_idx]  # Remove the best viewpoint from the list

            # Check if the cumulative hits have reached 95% of the mesh's faces
            # coverage_ratio = len(all_hits) / len(objmesh.triangles)
            coverage_ratio = calculate_coverage_ratio(all_hits, objmesh)
            if coverage_ratio >= 0.95:
                print("Reached 95% coverage of mesh faces.")
                break
            else:
                print(f"Current coverage ratio: {coverage_ratio:.2%}")
        else:
            break

    # Print all best viewpoints
    for vp in best_viewpoints:
        print(
            f"Viewpoint {vp['index'] + 1}: Position {vp['position']}, Direction {vp['direction']}, View {vp['view']}, Score {vp['score']}")

    print("Optical-quality calculation completed in {:.2f} seconds".format(time.time() - start_time))

    #############################################
    ## Add elements to the visualization window##
    #############################################

    vis.add_geometry(objmesh)
    vis.add_geometry(wireframe)
    # ##法线可视化
    # vis.add_geometry(average_normals)

    vis.add_geometry(aabb)
    # vis.add_geometry(obb)


    for arrow in arrow_list:
        vis.add_geometry(arrow)


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


    # Run the visualization
    vis.run()


if __name__ == "__main__":
    main()
