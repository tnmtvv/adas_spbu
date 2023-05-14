import argparse

from Utils.DatasetFunctions.SemanticKitti_methods import get_AoI_indices
from Utils.MyUtils import *
from Utils.loader import *

from sklearn import cluster as skc

from IoU import *


def main(
    path_to_lidar: str,
    path_to_images: str,
    start_indx: int,
    num_shots_to_optimise: int,
    best_params_str: str,
    dataset: int,
    clusterisation_method: str,
    visualize=True,
    verbose=True,
):

    algos_params_types = read_algos_params()
    algos_functions = {'dbscan': skc.DBSCAN, 'kmeans': skc.KMeans, 'optics': skc.OPTICS,
                       'AgglomerativeClustering': skc.AgglomerativeClustering, 'Bisecting_K-Means': skc.BisectingKMeans}

    algo = algos_functions[clusterisation_method]
    params_types, params_names = parse_parameters_info(algos_params_types[clusterisation_method])


    best_params = parse_parameters(best_params_str)

    list_main, list_sub = create_data_lists(
        path_to_lidar,
        path_to_images,
        start_indx,
        start_indx + num_shots_to_optimise,
        separator="\.|_",
    )

    if num_shots_to_optimise == -1:
        num_shots_to_optimise = len(list_main)
    if dataset == 1:
        map_label_color: Dict[int, npt.NDArray] = {}
        pcds, lidars = build_point_clouds_and_lidars(
            list_main, num_shots_to_optimise, list_sub
        )
        gt_labeled_pcds = build_audi_labeled_pcds(pcds)
        (
            pcds_labeled_outliers,
            pcds_inliers,
            inliers_indx,
        ) = plane_segmentation(gt_labeled_pcds)
    else:
        map_label_color: Dict[int, npt.NDArray] = {}
        pcds = extract_sem_kitti_pcds(
            list_main, list_sub, get_AoI_indices
        )
        gt_labeled_pcds, necessary_labels = build_sem_kitti_labeled_pcds(pcds, list_sub, map_label_color, get_AoI_indices)

        (
            pcds_labeled_outliers,
            pcds_inliers,
            inliers_indx,
        ) = plane_segmentation(gt_labeled_pcds)
        # getting pcds without road, road, above road area and road points indices

    IoU = []
    best_params_dict = dict(zip(params_names, best_params))
    necessary_labels = set()

    for i, pcd in enumerate(
        zip(pcds_labeled_outliers, pcds_inliers)
    ):  # clustering with chosen parameters
        pcd_outlier, pcd_inlier = pcd

        flatten_indices_of_interest = extract_necessary_indices(pcd[0], necessary_labels)
        clusterer = algo()
        clusterer.set_params(**best_params_dict)
        pcd_outlier.raw_labels = clusterer.fit_predict(pcd_outlier.pcd.points)

        pcd_outlier.unique_raw_labels = set(pcd_outlier.raw_labels)

        map_raw_true: Dict[int, int] = {}
        pcd_outlier.gt_colors = np.asarray(pcd_outlier.pcd.colors)
        evaluated_IoU, map_true_raw = evaluate_IoU(
            pcd_outlier, map_raw_true, flatten_indices_of_interest
        )
        IoU.append(evaluated_IoU)

        colors_cloud_raw = np.zeros(np.shape(pcd_outlier.pcd.points))

        one_more_set = set()
        for label in pcd_outlier.unique_raw_labels:
            cur_color = align_color(label, one_more_set)
            cur_indices = np.where(pcd_outlier.raw_labels == label)[0].tolist()
            # label_indices_of_interest = list(set(cur_indices).intersection(set(flatten_indices_of_interest)))
            colors_cloud_raw[cur_indices] = cur_color

        if visualize:
            pcd_outlier.pcd.colors = o3d.utility.Vector3dVector(pcd_outlier.gt_colors)
            o3d.visualization.draw_geometries([pcd_outlier.pcd])

            pcd_outlier.pcd.colors = o3d.utility.Vector3dVector(colors_cloud_raw)
            o3d.visualization.draw_geometries([pcd_outlier.pcd])

        if verbose:
            print(str(i) + ': ' + str(evaluated_IoU))

    print(np.mean(np.asarray(IoU)))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Finds parameters for clusterization algorithms"
    )
    parser.add_argument(
        "lidar_path", type=str, help="Directory where lidar files are stored"
    )
    parser.add_argument(
        "images_path", type=str, help="Directory where images are stored"
    )
    parser.add_argument("start_indx", type=int, help="Start index")
    parser.add_argument("num_of_shots", type=int, help="Number of shots to optimise")
    parser.add_argument("algorithm", type=int, choices=[1, 2], help="Choose algorithm: 1 -- DBSCAN, 2 -- Kmeans")
    parser.add_argument("best_parameters", type=str, help="Parameters to evaluate in (param_1, param_2, ... , param_n) format")
    parser.add_argument(
        "dataset",
        type=int,
        choices=[1, 2],
        help="Choose dataset: 1 -- AUDI," " 2 -- SemanticKitti",
    )
    parser.add_argument("algo", type=str, help="Your algorithm")
    parser.add_argument("--no-verbose", dest="verbose", action="store_false")
    parser.add_argument("--no-visualize", dest="visualize", action="store_false")

    args = parser.parse_args()

    main(
        args.lidar_path,
        args.images_path,
        args.start_indx,
        args.num_of_shots,
        args.best_parameters,
        args.dataset,
        args.algo,
        args.verbose,
        args.visualize
    )
