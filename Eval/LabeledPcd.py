import numpy as np
import numpy.typing as npt
import open3d as o3d


class LabeledPcd:
    def __init__(
        self, pcd: o3d.geometry.PointCloud, gt_labels: npt.NDArray, raw_labels: npt.NDArray = None,
            sem_labels: npt.NDArray = None, sem_colors: npt.NDArray = None,
            raw_colors: npt.NDArray = None, true_colors: npt.NDArray = None
    ):
        if pcd:
            self.pcd = pcd

            if gt_labels is not None and len(list(pcd.points)) != len(gt_labels):
                print("Points` shape: ", self.pcd.points.shape)
                print("gt Labels` shape: ", gt_labels.shape)
                raise ValueError("Lengths don`t match")

            if raw_labels is not None and len(list(pcd.points)) != len(raw_labels):
                print("Points` shape: ", self.pcd.points.shape)
                print("raw Labels` shape: ", raw_labels.shape)
                raise ValueError("Lengths don`t match")

            self.gt_labels = gt_labels
            self.gt_colors = true_colors

            if gt_labels is not None:
                self.unique_gt_labels = set(gt_labels)
            else:
                self.unique_gt_labels = set()

            self.raw_labels = raw_labels
            self.raw_colors = raw_colors
            if raw_labels is not None:
                self.unique_raw_labels = set(raw_labels)
            else:
                self.unique_gt_labels = set()

            self.sem_labels = sem_labels
            self.sem_colors = sem_colors
            if sem_labels is not None:
                self.unique_sem_labels = set(sem_labels)
