from typing import Dict

import numpy as np
import numpy.typing as npt
import open3d as o3d
from LabeledPcd import LabeledPcd


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
    return label.astype(np.uint32)


def get_semantic_from_indices(pcd: LabeledPcd, indices: npt.NDArray):
    gt_labels = pcd.labels[indices]
    label = np.bincount(gt_labels).argmax()
    semantic = np.where(pcd.labels == label)[0]
    return label, semantic


def evaluate_IoU(gt_pcd: LabeledPcd, alg_pcd: LabeledPcd, map_raw_true: Dict):
    overall_IoU = 0
    for label in alg_pcd.unique_labels:
        alg_label_indices = np.where(alg_pcd.labels == label)[0]
        gt_label, semantic_indices = get_semantic_from_indices(
            gt_pcd, alg_label_indices
        )

        map_raw_true[label] = gt_label  # may be several to one error

        intersection = len(np.intersect1d(alg_label_indices, semantic_indices))
        union = len(alg_label_indices) + len(semantic_indices)
        overall_IoU += intersection / union
    return overall_IoU / max(
        len(alg_pcd.unique_labels), len(gt_pcd.unique_labels)
    )  # if pcds have different ammount of unique labels, coefficient is calculated over the max
