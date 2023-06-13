from itertools import chain
from typing import Dict, Set

import numpy as np
import numpy.typing as npt
import open3d as o3d

from Eval.LabeledPcd import LabeledPcd

EXTENSIONS_SCAN = [".bin"]
EXTENSIONS_LABEL = [".label"]


def read_pcd_from_bin(bin_file) -> o3d.geometry.PointCloud:
    np_points = np.fromfile(bin_file, dtype=np.float32)
    pcd_points = np_points.reshape(-1, 4)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pcd_points[:, :3])
    return pcd


def read_labels(label_file):
    # open label
    print(label_file)
    label = np.fromfile(label_file, dtype=np.uint32)
    label = label.reshape((-1))
    upper_half = label >> 16  # get upper half for instances
    lower_half = label & 0xFFFF  # get lower half for semantics
    label = (upper_half << 16) + lower_half  # reconstruct full label
    return label.astype(np.uint32), lower_half


def get_AoI_indices(cur_pcd: o3d.data.PCDPointCloud):
    points_to_filter = np.asarray(cur_pcd.points)[:, 0]
    indices = np.where(points_to_filter > 0)[0].tolist()
    return indices
