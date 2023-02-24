import numpy.typing as npt
import open3d as o3d


class LabeledPcd:
    def __init__(
        self, pcd: o3d.geometry.PointCloud, labels: npt.NDArray
    ):
        if pcd:
            self.pcd = pcd

            if len(pcd.points) != len(labels):
                print("Points shape: ", self.pcd.points.shape)
                print("Label shape: ", labels.shape)
                raise ValueError("Lengths don`t match")
            self.labels = labels
            self.unique_labels = set(labels)
