/* ----------------------------------------------------------------------------
 * Copyright 2026, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once

// This file is a mofified version of
// https://github.com/sikang/motion_primitive_library/blob/master/test/read_map.hpp

#include <fstream>
#include <iostream>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "hgp/data_utils.hpp"
#include <yaml-cpp/yaml.h>

/** @brief Reads a point cloud into a voxelized occupancy grid with obstacle inflation.
 *
 *  @tparam Ti Integer vector type for grid dimensions.
 *  @tparam Tf Float vector type for origin coordinates.
 */
template <class Ti, class Tf>
class MapReader {
 public:
  /** @brief Construct a voxel map from a point cloud.
   *  @param pclptr Input point cloud of obstacle points.
   *  @param cells_x Number of cells in the x dimension.
   *  @param cells_y Number of cells in the y dimension.
   *  @param cells_z Number of cells in the z dimension.
   *  @param res Voxel resolution in meters.
   *  @param center_map Center of the map in global coordinates.
   *  @param z_min Minimum z value to include.
   *  @param z_max Maximum z value to include.
   *  @param inflation Obstacle inflation radius in meters.
   */
  MapReader(
      pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr,
      int cells_x,
      int cells_y,
      int cells_z,
      double res,
      const Vec3f center_map,
      double z_min,
      double z_max,
      double inflation) {
    // Box of the map: the box with which the map moves.
    // Center_map: The center of the box of the map, expressed in global float coordinates.
    // origin_: The corner of the box with minX, minY, minZ, expressed in global float coordinates.
    // Cell coordinates: relative to origin_, always positive.
    // Occupied cell = 100, Free cell = 0, Unknown cell = -1.
    // z_min [m] limits the map size to exclude points below the ground.
    // inflation [m] inflates obstacles by that amount; map is also inflated in x and y.

    Vec3i dim(cells_x, cells_y, cells_z);

    dim[0] = dim[0] + (int)(5 * (inflation * 1.0) / res);
    dim[1] = dim[1] + (int)(5 * (inflation * 1.0) / res);

    int dim2_down = dim[2] / 2.0;
    int dim2_up = dim[2] / 2.0;

    if (center_map[2] - res * dim[2] / 2.0 < 0) {
      dim2_down = (int)((center_map[2] - z_min) / res) + 1;  //+1 to avoid problems when taking off
    }

    if (center_map[2] + res * dim[2] / 2.0 > z_max) {
      dim2_up = (int)((z_max - center_map[2]) / res);
      dim2_up = (dim2_up > 0) ? dim2_up : 1;  // Force it to be >= 1
    }
    dim[2] = dim2_down + dim2_up;
    origin_(0) = center_map[0] - res * dim[0] / 2.0;
    origin_(1) = center_map[1] - res * dim[1] / 2.0;
    origin_(2) = center_map[2] - res * dim2_down;

    for (unsigned int i = 0; i < 3; i++) {
      dim_(i) = dim[i];
    }

    resolution_ = res;
    data_.resize(dim[0] * dim[1] * dim[2], 0);
    int total_size = dim[0] * dim[1] * dim[2];
    for (size_t i = 0; i < pclptr->points.size(); ++i) {
      // Let's find the cell coordinates of the point expresed in a system of coordinates that has
      // as origin the (minX, minY, minZ) point of the map
      int x = std::round((pclptr->points[i].x - origin_(0)) / res - 0.5);
      int y = std::round((pclptr->points[i].y - origin_(1)) / res - 0.5);
      int z = std::round((pclptr->points[i].z - origin_(2)) / res - 0.5);

      // Force them to be positive:
      x = (x > 0) ? x : 0;
      y = (y > 0) ? y : 0;
      z = (z > 0) ? z : 0;
      // this next formula works only when x, y, z are in cell coordinates (relative to the origin
      // of the map)
      int id = x + dim_(0) * y + dim_(0) * dim_(1) * z;

      if (id < 0) {
        printf("JPS Reader: There is sth wrong, id= %d\n", id);
      }
      if (id >= 0 && id < total_size) {
        data_[id] = 100;
      }

      // now let's inflate the voxels around that point
      int m = (int)floor((inflation / res));
      // m is the amount of cells to inflate in each direction

      for (int ix = x - m; ix <= x + m; ix++) {
        for (int iy = y - m; iy <= y + m; iy++) {
          for (int iz = z - m; iz <= z + m; iz++) {
            int id_infl = ix + dim_(0) * iy + dim_(0) * dim_(1) * iz;
            if (id_infl >= 0 && id_infl < total_size)  // Ensure we are inside the map
            {
              data_[id_infl] = 100;
            }
          }
        }
      }
    }
  }

  /** @brief Get the origin (minimum corner) of the map in global coordinates.
   *  @return Origin position vector.
   */
  Tf origin() { return origin_; }

  /** @brief Get the grid dimensions in cells.
   *  @return Dimension vector (cells_x, cells_y, cells_z).
   */
  Ti dim() { return dim_; }

  /** @brief Get the voxel resolution.
   *  @return Resolution in meters.
   */
  double resolution() { return resolution_; }

  /** @brief Get the raw occupancy data (100 = occupied, 0 = free).
   *  @return Flat vector of occupancy values.
   */
  std::vector<signed char> data() { return data_; }

 private:
  Tf origin_;
  Ti dim_;

  double resolution_;
  std::vector<signed char> data_;
};
