#!/usr/bin/python
# -*- coding: utf-8 -*-
import json

import cv2
import numpy as np
import open3d as o3d

# Adopted from https://www.a2d2.audi/a2d2/en/tutorial.html


def colours_from_reflectances(reflectances):
    return np.stack([reflectances, reflectances, reflectances], axis=1)


def make_colours(lidar, cam_image):
    rows = (lidar["row"] + 0.5).astype(np.int)
    cols = (lidar["col"] + 0.5).astype(np.int)
    colours = cam_image[rows, cols, :] / 255.0
    return colours


def create_open3d_pc(lidar, cam_image=None):
    # create open3d point cloud

    pcd = o3d.geometry.PointCloud()

    # assign point coordinates
    pcd.points = o3d.utility.Vector3dVector(lidar["points"])

    # assign colours
    if cam_image is None:
        median_reflectance = np.median(lidar["reflectance"])
        colours = colours_from_reflectances(lidar["reflectance"]) / (
            median_reflectance * 5
        )

        # clip colours for visualisation on a white background

        colours = np.clip(colours, 0, 0.75)
    else:
        colours = make_colours(lidar, cam_image)
    pcd.colors = o3d.utility.Vector3dVector(colours)
    return pcd


def skew_sym_matrix(u):
    return np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]])


def axis_angle_to_rotation_mat(axis, angle):
    return (
        np.cos(angle) * np.eye(3)
        + np.sin(angle) * skew_sym_matrix(axis)
        + (1 - np.cos(angle)) * np.outer(axis, axis)
    )


def read_bounding_boxes(file_name_bboxes):
    # open the file
    with open(file_name_bboxes, "r") as f:
        bboxes = json.load(f)
    boxes = []  # a list for containing bounding boxes
    print(bboxes.keys())

    for bbox in bboxes.keys():
        bbox_read = {}  # a dictionary for a given bounding box
        bbox_read["class"] = bboxes[bbox]["class"]
        bbox_read["truncation"] = bboxes[bbox]["truncation"]
        bbox_read["occlusion"] = bboxes[bbox]["occlusion"]
        bbox_read["alpha"] = bboxes[bbox]["alpha"]
        bbox_read["top"] = bboxes[bbox]["2d_bbox"][0]
        bbox_read["left"] = bboxes[bbox]["2d_bbox"][1]
        bbox_read["bottom"] = bboxes[bbox]["2d_bbox"][2]
        bbox_read["right"] = bboxes[bbox]["2d_bbox"][3]
        bbox_read["center"] = np.array(bboxes[bbox]["center"])
        bbox_read["size"] = np.array(bboxes[bbox]["size"])
        angle = bboxes[bbox]["rot_angle"]
        axis = np.array(bboxes[bbox]["axis"])
        bbox_read["rotation"] = axis_angle_to_rotation_mat(axis, angle)
        boxes.append(bbox_read)
    return boxes


def get_points(bbox):
    half_size = bbox["size"] / 2.0

    if half_size[0] > 0:
        # calculate unrotated corner point offsets relative to center
        brl = np.asarray([-half_size[0], +half_size[1], -half_size[2]])
        bfl = np.asarray([+half_size[0], +half_size[1], -half_size[2]])
        bfr = np.asarray([+half_size[0], -half_size[1], -half_size[2]])
        brr = np.asarray([-half_size[0], -half_size[1], -half_size[2]])
        trl = np.asarray([-half_size[0], +half_size[1], +half_size[2]])
        tfl = np.asarray([+half_size[0], +half_size[1], +half_size[2]])
        tfr = np.asarray([+half_size[0], -half_size[1], +half_size[2]])
        trr = np.asarray([-half_size[0], -half_size[1], +half_size[2]])

        # rotate points
        points = np.asarray([brl, bfl, bfr, brr, trl, tfl, tfr, trr])
        points = np.dot(points, bbox["rotation"].T)

        # add center position
        points = points + bbox["center"]
    return points


def extract_images(images_path_list):
    images_list = []

    for file_name in images_path_list:
        images_list.append(cv2.imread(file_name))
    return images_list


def map_lidar_points_onto_image(
    image_orig, lidar, colors, pixel_size=3, pixel_opacity=1
):
    image = np.copy(image_orig)

    # get rows and cols
    rows = (lidar["row"] + 0.5).astype(np.int)
    cols = (lidar["col"] + 0.5).astype(np.int)

    # get distances
    distances = lidar["distance"]
    # determine point colours from distance
    colours = colors.flatten(order="C")
    colours = np.asarray(
        [np.asarray(hsv_to_rgb(0.75 * c, np.sqrt(pixel_opacity), 1.0)) for c in colours]
    )
    pixel_rowoffs = np.indices([pixel_size, pixel_size])[0] - pixel_size // 2
    pixel_coloffs = np.indices([pixel_size, pixel_size])[1] - pixel_size // 2
    canvas_rows = image.shape[0]
    canvas_cols = image.shape[1]
    for i in range(len(colours)):
        pixel_rows = np.clip(rows[i] + pixel_rowoffs, 0, canvas_rows - 1)
        pixel_cols = np.clip(cols[i] + pixel_coloffs, 0, canvas_cols - 1)
        image[pixel_rows, pixel_cols, :] = (1.0 - pixel_opacity) * np.multiply(
            image[pixel_rows, pixel_cols, :], colours[i]
        ) + pixel_opacity * 255 * colours[i]
    return image.astype(np.uint8)


def hsv_to_rgb(h, s, v):
    if s == 0.0:
        return v, v, v
    i = int(h * 6.0)
    f = h * 6.0 - i
    p = v * (1.0 - s)
    q = v * (1.0 - s * f)
    t = v * (1.0 - s * (1.0 - f))
    i = i % 6

    if i == 0:
        return v, t, p
    if i == 1:
        return q, v, p
    if i == 2:
        return p, v, t
    if i == 3:
        return p, q, v
    if i == 4:
        return t, p, v
    if i == 5:
        return v, p, q


def undistort_image(image, cam_name):
    with open("../Utils/DatasetFunctions/config_json.json", "r") as f:
        config = json.load(f)
    if cam_name in [
        "front_left",
        "front_center",
        "front_right",
        "side_left",
        "side_right",
        "rear_center",
    ]:
        # get parameters from config file
        intr_mat_undist = np.asarray(config["cameras"][cam_name]["CamMatrix"])
        intr_mat_dist = np.asarray(config["cameras"][cam_name]["CamMatrixOriginal"])
        dist_parms = np.asarray(config["cameras"][cam_name]["Distortion"])
        lens = config["cameras"][cam_name]["Lens"]

        if lens == "Fisheye":
            return cv2.fisheye.undistortImage(
                image, intr_mat_dist, D=dist_parms, Knew=intr_mat_undist
            )
        elif lens == "Telecam":
            return cv2.undistort(
                image,
                intr_mat_dist,
                distCoeffs=dist_parms,
                newCameraMatrix=intr_mat_undist,
            )
        else:
            return image
    else:
        return image


def build_colored_pcd(lidar_file, file_name):
    semantic_image_front_center = cv2.imread(file_name)
    semantic_image_front_center_undistorted = undistort_image(
        semantic_image_front_center, "front_center"
    )
    pcd_lidar_colored = create_open3d_pc(
        lidar_file, semantic_image_front_center_undistorted
    )
    return pcd_lidar_colored
