#!/usr/bin/python
# -*- coding: utf-8 -*-
import numpy as np
import open3d as o3d

# Adopted from https://www.a2d2.audi/a2d2/en/tutorial.html


class AUDIMethods:
    @staticmethod
    def colours_from_reflectances(reflectances):
        return np.stack([reflectances, reflectances, reflectances], axis=1)

    @staticmethod
    def create_open3d_pc(lidar, cam_image=None):
        # create open3d point cloud

        pcd = o3d.geometry.PointCloud()

        # assign point coordinates

        pcd.points = o3d.utility.Vector3dVector(lidar["points"])

        # assign colours

        if cam_image is None:
            median_reflectance = np.median(lidar["reflectance"])
            colours = AUDIMethods.colours_from_reflectances(lidar["reflectance"]) / (
                median_reflectance * 5
            )

            # clip colours for visualisation on a white background

            colours = np.clip(colours, 0, 0.75)
        else:
            rows = (lidar["row"] + 0.5).astype(np.int)
            cols = (lidar["col"] + 0.5).astype(np.int)
            colours = cam_image[rows, cols, :] / 255.0
        pcd.colors = o3d.utility.Vector3dVector(colours)

        return pcd

    @staticmethod
    def map_lidar_points_onto_image(
        image_orig,
        lidar,
        local_colors,
        pixel_size=3,
        pixel_opacity=1,
    ):
        image = np.copy(image_orig)

        # get rows and cols

        rows = (lidar["row"] + 0.5).astype(np.int)
        cols = (lidar["col"] + 0.5).astype(np.int)

        # determine point colours from distance

        colours = local_colors

        pixel_rowoffs = np.indices([pixel_size, pixel_size])[0] - pixel_size // 2
        pixel_coloffs = np.indices([pixel_size, pixel_size])[1] - pixel_size // 2
        canvas_rows = image.shape[0]
        canvas_cols = image.shape[1]
        for i in range(len(rows)):
            pixel_rows = np.clip(rows[i] + pixel_rowoffs, 0, canvas_rows - 1)
            pixel_cols = np.clip(cols[i] + pixel_coloffs, 0, canvas_cols - 1)
            image[pixel_rows, pixel_cols, :] = (1.0 - pixel_opacity) * np.multiply(
                image[pixel_rows, pixel_cols, :], colours[i]
            ) + pixel_opacity * 255 * colours[i]
        return image.astype(np.uint8)

    @staticmethod
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
