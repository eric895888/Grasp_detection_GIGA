import argparse
from pathlib import Path

import numpy as np
import rospy

from vgn import vis
from vgn.dataset import Dataset
from vgn.grasp import Grasp
from vgn.utils.transform import Rotation, Transform


def main(args):
    rospy.init_node("vgn_vis", anonymous=True)

    dataset = Dataset(args.dataset, augment=True)
    i = np.random.randint(len(dataset))

    voxel_grid, (label, rotations, width), index = dataset[i]
    grasp = Grasp(Transform(Rotation.from_quat(rotations[0]), index), width)

    vis.clear()
    vis.draw_workspace(40)
    vis.draw_tsdf(voxel_grid, 1.0)
    vis.draw_grasp(grasp, float(label), 40.0 / 6.0)

    rospy.sleep(1.0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path,default="/media/eric/Disk/vgn/scripts/data/raw/foo")
    args = parser.parse_args()
    main(args)
