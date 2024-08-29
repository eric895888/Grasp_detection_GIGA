import argparse
from pathlib import Path

import numpy as np
import open3d as o3d
import scipy.signal as signal
from tqdm import tqdm
import multiprocessing as mp

from vgn.grasp import Grasp, Label
from vgn.io import *
from vgn.perception import *
from vgn.simulation import ClutterRemovalSim
from vgn.utils.transform import Rotation, Transform
from vgn.utils.implicit import get_mesh_pose_list_from_world
#原版的

OBJECT_COUNT_LAMBDA = 4
MAX_VIEWPOINT_COUNT = 6


def main(args, rank): #rank＝0 表示主要process

    print("test_main")
    GRASPS_PER_SCENE = args.grasps_per_scene
    np.random.seed()
    seed = np.random.randint(0, 1000) + rank
    np.random.seed(seed)
    sim = ClutterRemovalSim(args.scene, args.object_set, gui=args.sim_gui)
    finger_depth = sim.gripper.finger_depth
    grasps_per_worker = args.num_grasps // args.num_proc
    pbar = tqdm(total=grasps_per_worker, disable=rank != 0)

    if rank == 0: #只有主process可以創立資料夾
        print("main process creat folder")
        (args.root / "scenes").mkdir(parents=True,exist_ok=True)
        write_setup(
            args.root,
            sim.size,
            sim.camera.intrinsic,
            sim.gripper.max_opening_width,
            sim.gripper.finger_depth,
        )
        if args.save_scene:
            (args.root / "mesh_pose_list").mkdir(parents=True,exist_ok=True)
    
    for _ in range(grasps_per_worker // GRASPS_PER_SCENE):
        # generate heap
        object_count = np.random.randint(low=5,high=8,size=1)
        sim.reset(object_count)
        sim.save_state()
        print(str(rank)+"sim_reset_end")
        # render synthetic depth images
        n = MAX_VIEWPOINT_COUNT
        depth_imgs, extrinsics = render_images(sim, n)  #存六個角度 n=6
        depth_imgs_side, extrinsics_side = render_side_images(sim, 1, args.random) #存一個角度 n=1
        print(str(rank)+"save_depth_imgs_side_end")
        # reconstrct point cloud using a subset of the images
        tsdf = create_tsdf(sim.size, 120, depth_imgs, sim.camera.intrinsic, extrinsics) #這裡卡住！！
        import time
        print(str(rank)+"create_tsdf_end") #這裡卡住！！
        pc = tsdf.get_cloud()
        print(str(rank)+"get_cloud_end")
        # crop surface and borders from point cloud
        bounding_box = o3d.geometry.AxisAlignedBoundingBox(sim.lower, sim.upper)
        pc = pc.crop(bounding_box)
        # o3d.visualization.draw_geometries([pc])
        print(str(rank)+"pc_crop+end")
        if pc.is_empty():
            print("Point cloud empty, skipping scene")
            continue

        # store the raw data
        scene_id = write_sensor_data(args.root, depth_imgs_side, extrinsics_side)
        print(str(rank)+"write_sensor_data_end")
        if args.save_scene:
            mesh_pose_list = get_mesh_pose_list_from_world(sim.world, args.object_set)
            print(str(rank)+"mesh_pose_list_end")
            write_point_cloud(args.root, scene_id, mesh_pose_list, name="mesh_pose_list") 
             #要注意處理 VisibleDeprecationWarning: Creating an ndarray from ragged nested sequences 
             #(which is a list-or-tuple of lists-or-tuples-or ndarrays with different lengths or shapes) is deprecated. If you meant to do this, 
             #you must specify 'dtype=object' when creating the ndarray.

        print(str(rank)+"write_mesh_pose_list_end")
        for _ in range(GRASPS_PER_SCENE):
            # sample and evaluate a grasp point
            point, normal = sample_grasp_point(pc, finger_depth)
            grasp, label = evaluate_grasp_point(sim, point, normal)

            # store the sample
            write_grasp(args.root, scene_id, grasp, label)
            pbar.update()
    print(str(rank)+"test_main_end")

    pbar.close()
    print('Process %d finished!' % rank)


def render_images(sim, n):
    height, width = sim.camera.intrinsic.height, sim.camera.intrinsic.width
    origin = Transform(Rotation.identity(), np.r_[sim.size / 2, sim.size / 2, 0.0])

    extrinsics = np.empty((n, 7), np.float32)
    depth_imgs = np.empty((n, height, width), np.float32)

    for i in range(n):
        r = np.random.uniform(1.6, 2.4) * sim.size
        theta = np.random.uniform(0.0, np.pi / 4.0)
        phi = np.random.uniform(0.0, 2.0 * np.pi)

        extrinsic = camera_on_sphere(origin, r, theta, phi)
        depth_img = sim.camera.render(extrinsic)[1]

        extrinsics[i] = extrinsic.to_list()
        depth_imgs[i] = depth_img

    return depth_imgs, extrinsics

def render_side_images(sim, n=1, random=False): #跟vgn的差別是只用一個視角 n=1固定視角
    height, width = sim.camera.intrinsic.height, sim.camera.intrinsic.width
    origin = Transform(Rotation.identity(), np.r_[sim.size / 2, sim.size / 2, sim.size / 3])

    extrinsics = np.empty((n, 7), np.float32)
    depth_imgs = np.empty((n, height, width), np.float32)

    for i in range(n):
        if random:
            r = np.random.uniform(1.6, 2.4) * sim.size
            theta = np.random.uniform(np.pi / 4.0, 5.0 * np.pi / 12.0)
            phi = np.random.uniform(- 5.0 * np.pi / 5, - 3.0 * np.pi / 8.0)
        else:   #目前是固定視角
            r = 2 * sim.size
            theta = np.pi / 3.0
            phi = - np.pi / 2.0

        extrinsic = camera_on_sphere(origin, r, theta, phi)
        depth_img = sim.camera.render(extrinsic)[1]

        extrinsics[i] = extrinsic.to_list()
        depth_imgs[i] = depth_img

    return depth_imgs, extrinsics


def sample_grasp_point(point_cloud, finger_depth, eps=0.1):
    points = np.asarray(point_cloud.points)
    normals = np.asarray(point_cloud.normals)
    ok = False
    while not ok:
        # TODO this could result in an infinite loop, though very unlikely
        idx = np.random.randint(len(points))
        point, normal = points[idx], normals[idx]
        ok = normal[2] > -0.1  # make sure the normal is poitning upwards
    grasp_depth = np.random.uniform(-eps * finger_depth, (1.0 + eps) * finger_depth)
    point = point + normal * grasp_depth
    return point, normal


def evaluate_grasp_point(sim, pos, normal, num_rotations=6):
    # define initial grasp frame on object surface
    z_axis = -normal
    x_axis = np.r_[1.0, 0.0, 0.0]
    if np.isclose(np.abs(np.dot(x_axis, z_axis)), 1.0, 1e-4):
        x_axis = np.r_[0.0, 1.0, 0.0]
    y_axis = np.cross(z_axis, x_axis)
    x_axis = np.cross(y_axis, z_axis)
    R = Rotation.from_matrix(np.vstack((x_axis, y_axis, z_axis)).T)

    # try to grasp with different yaw angles
    yaws = np.linspace(0.0, np.pi, num_rotations)
    outcomes, widths = [], []
    for yaw in yaws:
        ori = R * Rotation.from_euler("z", yaw)
        sim.restore_state()
        candidate = Grasp(Transform(ori, pos), width=sim.gripper.max_opening_width)
        outcome, width = sim.execute_grasp(candidate, remove=False)
        outcomes.append(outcome)
        widths.append(width)

    # detect mid-point of widest peak of successful yaw angles
    # TODO currently this does not properly handle periodicity
    successes = (np.asarray(outcomes) == Label.SUCCESS).astype(float)
    if np.sum(successes):
        peaks, properties = signal.find_peaks(
            x=np.r_[0, successes, 0], height=1, width=1
        )
        idx_of_widest_peak = peaks[np.argmax(properties["widths"])] - 1
        ori = R * Rotation.from_euler("z", yaws[idx_of_widest_peak])
        width = widths[idx_of_widest_peak]

    return Grasp(Transform(ori, pos), width), int(np.max(outcomes))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path,default="/home/eric/Grasp_detection_GIGA/scripts")
    parser.add_argument("--scene", type=str, choices=["pile", "packed"], default="pile")
    parser.add_argument("--object-set", type=str, default="blocks")
    parser.add_argument("--num-grasps", type=int, default=10000)
    parser.add_argument("--grasps-per-scene", type=int, default=120)
    parser.add_argument("--num-proc", type=int, default=1)
    parser.add_argument("--save-scene", action="store_true")
    parser.add_argument("--random", action="store_true", help="Add distrubation to camera pose")
    parser.add_argument("--sim-gui", action="store_true")
    args = parser.parse_args()
    args.save_scene = True
    mp.set_start_method('spawn') #可以平行化的關鍵
    if args.num_proc > 1:
        pool = mp.Pool(processes=args.num_proc)
        for i in range(args.num_proc):
            pool.apply_async(func=main, args=(args, i))
            import time
            time.sleep(5)
        pool.close()
        pool.join()
    else:
        main(args, 0)
