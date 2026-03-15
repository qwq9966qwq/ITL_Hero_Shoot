#!/usr/bin/env python3
"""
STL-PCD 配准选点工具
STL变换对齐到PCD，PCD坐标全程不变。
Shift+左键选点，关闭窗口后输出PCD原始坐标。
"""
import open3d as o3d
import numpy as np
import copy

PCD_PATH = "/home/guo/Hero_Shoot/Real/src/bringup/pcd/Hero.pcd"
STL_PATH = "/home/guo/Hero_Shoot/Simulation/src/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/models/rmuc_2026/meshes/rmuc_2026.stl"

def main():
    # 加载
    print("[1/5] 加载文件...")
    pcd = o3d.io.read_point_cloud(PCD_PATH)
    mesh = o3d.io.read_triangle_mesh(STL_PATH)
    mesh.compute_vertex_normals()
    print(f"  PCD: {len(pcd.points)} 点")
    print(f"  STL: {len(mesh.vertices)} 顶点, {len(mesh.triangles)} 面")

    pcd_bbox = pcd.get_axis_aligned_bounding_box()
    stl_bbox = mesh.get_axis_aligned_bounding_box()
    print(f"  PCD 范围: {np.array(pcd_bbox.min_bound).round(3)} ~ {np.array(pcd_bbox.max_bound).round(3)}")
    print(f"  STL 范围: {np.array(stl_bbox.min_bound).round(3)} ~ {np.array(stl_bbox.max_bound).round(3)}")

    # STL采样为点云
    print("[2/5] STL采样为点云...")
    stl_pcd = mesh.sample_points_uniformly(number_of_points=200000)

    # 粗对齐：中心对齐
    print("[3/4] 中心对齐 + ICP配准...")
    pcd_center = pcd_bbox.get_center()
    stl_center = stl_bbox.get_center()
    init_translate = pcd_center - stl_center
    stl_pcd_shifted = copy.deepcopy(stl_pcd)
    stl_pcd_shifted.translate(init_translate)
    print(f"  中心偏移: {init_translate.round(3)}")

    # 降采样后直接ICP（跳过RANSAC，两者尺度一致中心对齐后已经很近）
    voxel_size = 0.15
    src_down = stl_pcd_shifted.voxel_down_sample(voxel_size)
    tgt_down = pcd.voxel_down_sample(voxel_size)
    src_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    tgt_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    print(f"  降采样: STL {len(src_down.points)} 点, PCD {len(tgt_down.points)} 点")

    # 多轮ICP：先粗后细
    max_dist_list = [2.0, 1.0, 0.5, 0.2]
    T_current = np.eye(4)
    for i, max_dist in enumerate(max_dist_list):
        result = o3d.pipelines.registration.registration_icp(
            src_down, tgt_down,
            max_correspondence_distance=max_dist,
            init=T_current,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane())
        T_current = result.transformation
        print(f"  ICP轮{i+1} (dist={max_dist}): fitness={result.fitness:.4f}, rmse={result.inlier_rmse:.4f}")

    # 总变换 = ICP变换 × 初始平移
    T_init = np.eye(4)
    T_init[:3, 3] = init_translate
    T_total = T_current @ T_init
    print(f"  总变换矩阵:\n{T_total.round(6)}")

    # 变换STL网格用于显示（保留原始mesh，不用采样点云）
    mesh_aligned = copy.deepcopy(mesh)
    mesh_aligned.transform(T_total)
    mesh_aligned.paint_uniform_color([0.6, 0.7, 1.0])  # STL 浅蓝色

    # PCD染色
    pcd.paint_uniform_color([0.0, 0.8, 0.0])  # PCD 绿色

    # 预览对齐效果
    print("[4/4] 预览对齐效果，确认无误后关闭窗口进入选点模式...")
    o3d.visualization.draw_geometries(
        [pcd, mesh_aligned],
        window_name="对齐预览 - 绿=PCD 蓝=STL网格（关闭后进入选点）",
        width=1280, height=720)

    # 选点模式
    print()
    print("=" * 40)
    print("  选点模式")
    print("  Shift + 左键：选取点")
    print("  关闭窗口后输出坐标")
    print("=" * 40)
    print()

    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name="Shift+左键选点（绿=PCD 蓝=STL网格）", width=1280, height=720)
    vis.add_geometry(pcd)
    vis.add_geometry(mesh_aligned)
    vis.run()
    vis.destroy_window()

    picked = vis.get_picked_points()
    if picked:
        pts = np.asarray(pcd.points)
        print(f"\n选中 {len(picked)} 个点（PCD原始坐标）：")
        for i, idx in enumerate(picked):
            p = pts[idx]
            print(f"  点{i+1}: index={idx}, 坐标=[{p[0]:.6f}, {p[1]:.6f}, {p[2]:.6f}]")
    else:
        print("\n未选中任何点。")


if __name__ == "__main__":
    main()
