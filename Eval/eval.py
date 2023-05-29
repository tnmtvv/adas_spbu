import argparse
import EvalToGt
from Eval import IoU
from Utils import loader
import matplotlib.pylab as pt

from Utils.DatasetFunctions.SemanticKitti_methods import get_AoI_indices
from Utils.MyUtils import *
from Utils.loader import *

from sklearn import cluster as skc

from IoU import *


def cluster_with_params(labeled_pcd, params_dict, clusterer, flatten_indices_of_interest):
    clusterer.set_params(**params_dict)
    labeled_pcd.raw_labels = clusterer.fit_predict(labeled_pcd.pcd.points)
    labeled_pcd.unique_raw_labels = set(labeled_pcd.raw_labels)
    map_raw_true: Dict[int, int] = {}
    labeled_pcd.gt_colors = np.asarray(labeled_pcd.pcd.colors)
    evaluated_IoU, map_true_raw = evaluate_IoU(
        labeled_pcd, map_raw_true, flatten_indices_of_interest
    )
    return evaluated_IoU, map_true_raw


def main(
    path_to_lidar: str,
    path_to_labels: str,
    start_indx: int,
    num_shots_to_optimise: int,
    best_params_str: str,
    dataset: int,
    clusterisation_method: str,
    visualize=True,
    verbose=True,
    best_possible_result=True,
    with_images=False,
):

    algos_params_types, algos_str_domains, map_params_domains = read_algos_params()

    algos_functions = {'dbscan': skc.DBSCAN, 'kmeans': skc.KMeans, 'optics': skc.OPTICS,
                       'AgglomerativeClustering': skc.AgglomerativeClustering, 'Bisecting_K-Means': skc.BisectingKMeans}

    algo = algos_functions[clusterisation_method]
    params_types, params_names = parse_parameters_info(algos_params_types[clusterisation_method])

    cur_ga_params, best_params = parse_parameters(best_params_str, params_types)

    # изменить для семантик (добавить флаг, читать вмсте с изображениями или нет)
    list_main, list_sub = create_data_lists(
        path_to_lidar,
        path_to_labels,
        start_indx,
        start_indx + num_shots_to_optimise,
        separator="\.|_",
        audi_mode=True
    )
    necessary_labels = set()

    if num_shots_to_optimise == -1:
        num_shots_to_optimise = len(list_main)
    if dataset == 1:
        map_label_color: Dict[int, npt.NDArray] = {}
        pcds, lidars = build_point_clouds_and_lidars(
            list_main, num_shots_to_optimise, list_sub
        )
        images = list(map(lambda x: cv2.imread(x), list_sub))
        gt_labeled_pcds = build_audi_labeled_pcds(pcds)
        (
            pcds_labeled_outliers,
            pcds_inliers,
            inliers_indx,
        ) = plane_segmentation(gt_labeled_pcds, if_eval=True)
    else:
        map_label_color: Dict[int, npt.NDArray] = {}
        gt_labeled_pcds, necessary_labels = extract_sem_kitti_pcds_labeled(list_main, list_sub, map_label_color, get_AoI_indices)
        (
            pcds_labeled_outliers,
            pcds_inliers,
            inliers_indx,
        ) = plane_segmentation(gt_labeled_pcds, if_eval=True)
        # getting pcds without road, road, above road area and road points indices

    IoU_algo = []
    IoU_gt = []

    best_params_dict = dict(zip(params_names, best_params))
    algos_params_types, string_params, map_params_domains = loader.read_algos_params()

    possible_str_params = None
    if clusterisation_method in string_params:
        possible_str_params = string_params[clusterisation_method]
    if clusterisation_method in map_params_domains:
        cur_params_domain = map_params_domains[clusterisation_method]

    best_params_dict_gt = {}
    # if not best_possible_result:
    #     ga = EvalToGt.GAToCheck(num_shots_to_optimise, cur_ga_params[0], cur_ga_params[1],
    #                         len(params_names), 0.3, params_types, params_names, cur_params_domain,
    #                         algo, pcds_labeled_outliers, necessary_labels,
    #                         possible_string_params=possible_str_params)
    #
    #     best_params_gt = ga.ga_run(verbose)
    #     best_params_dict_gt = dict(zip(params_names, best_params_gt))

    for i, pcd in enumerate(
        zip(pcds_labeled_outliers, pcds_inliers)
    ):  # clustering with chosen parameters
        pcd_outlier, pcd_inlier = pcd

        merged_pcd = pcd_inlier + pcd_outlier.pcd
        if not necessary_labels:
            flatten_indices_of_interest = IoU.extract_necessary_indices(pcd_outlier, set(pcd_outlier.gt_labels))
        else:
            flatten_indices_of_interest = IoU.extract_necessary_indices(pcd_outlier, necessary_labels)
        clusterer = algo()

        evaluated_IoU, map_true_raw=cluster_with_params(pcd_outlier, best_params_dict, clusterer, flatten_indices_of_interest)
        IoU_algo.append(evaluated_IoU)

        # if not best_possible_result:
        #     evaluated_IoU_gt, map_true_raw_gt = cluster_with_params(pcd_outlier, best_params_dict_gt, clusterer, flatten_indices_of_interest)
        #     IoU_gt.append(evaluated_IoU_gt)

        colors_cloud_raw = np.zeros(np.shape(pcd_outlier.pcd.points))

        one_more_set = set()
        for label in pcd_outlier.unique_raw_labels:
            cur_color = align_color(label, one_more_set)
            cur_indices = np.where(pcd_outlier.raw_labels == label)[0].tolist()
            # label_indices_of_interest = list(set(cur_indices).intersection(set(flatten_indices_of_interest)))
            colors_cloud_raw[cur_indices] = cur_color

        colors = np.zeros(np.shape(merged_pcd.colors))
        mask = np.ones(len(merged_pcd.colors), dtype=bool)
        mask[inliers_indx[i]] = 0
        colors[mask] = colors_cloud_raw

        cur_image=map_lidar_points_onto_image(images[i], lidars[i], colors)
        pt.fig = pt.figure(figsize=(20, 20))
        pt.imshow(cur_image)
        pt.axis('off')
        # if visualize:
        #     pcd_outlier.pcd.colors = o3d.utility.Vector3dVector(pcd_outlier.gt_colors)
        #     o3d.visualization.draw_geometries([pcd_outlier.pcd])
        #
        #     pcd_outlier.pcd.colors = o3d.utility.Vector3dVector(colors_cloud_raw)
        #     o3d.visualization.draw_geometries([pcd_outlier.pcd])
        #
        # if verbose:
        #     print(str(i) + ': ' + str(evaluated_IoU))

    print(np.mean(np.asarray(IoU_algo)))
    if best_possible_result:
        print('best possible result', np.mean(np.asarray(IoU_gt)))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Finds parameters for clusterization algorithms"
    )
    parser.add_argument(
        "lidar_path", type=str, help="Directory where lidar files are stored"
    )
    parser.add_argument(
        "labels_path", type=str, help="Directory where images are stored"
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
    parser.add_argument("best_possible_result", action="store_true")

    args = parser.parse_args()

    params_list = [[2, '(10,15;0.57,1)', 'dbscan'], [100, '(10,15;20,216,230)', 'kmeans'],
                   [100, '(10,15;25,377,512)', 'kmeans'], [100, '(10,15;21,221,492)', 'kmeans']]

    for params in params_list:
        main(
            args.lidar_path,
            args.labels_path,
            args.start_indx,
            params[0],
            params[1],
            args.dataset,
            params[2],
            args.verbose,
            args.visualize,
            args.best_possible_result
        )