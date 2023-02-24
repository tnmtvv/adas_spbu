from typing import Optional

import numpy as np
import numpy.typing as npt
import open3d as o3d


class LabeledPcd:
    def __init__(
        self, pcd: Optional[o3d.geometry.PointCloud], labels: Optional[npt.NDArray]
    ):
        if pcd:
            self.pcd = pcd

            if len(pcd.points) != len(labels):
                print("Points shape: ", self.pcd.points.shape)
                print("Label shape: ", labels.shape)
                raise ValueError("Lengths don`t match")
            self.labels = labels
            self.unique_labels = set(labels)
