/* ----------------------------------------------------------------------------
 * Copyright 2025, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file data_utils.hpp
 * @brief Provide a few widely used function for basic type
 */
// NOTE: Intentionally uses DATA_UTILS_H (not #pragma once) to share the guard
// with decomp_util's data_utils.h and prevent template redefinition.
#ifndef DATA_UTILS_H
#define DATA_UTILS_H

#include "hgp/data_type.hpp"

/// Template for transforming a vector
template <class T, class TF>
vec_E<T> transform_vec(const vec_E<T>& t, const TF& tf) {
  vec_E<T> new_t;
  for (const auto& it : t) new_t.push_back(tf * it);
  return new_t;
}

/// Template for calculating distance
template <class T>
decimal_t total_distance(const vec_E<T>& vs) {
  decimal_t dist = 0;
  for (unsigned int i = 1; i < vs.size(); i++) dist += (vs[i] - vs[i - 1]).norm();

  return dist;
}

/// Transform all entries in a vector using given TF
#define transform_vec3 transform_vec<Vec3f, Aff3f>
/// Sum up total distance for vec_Vec2f
#define total_distance2f total_distance<Vec2f>
/// Sum up total distance for vec_Vec3f
#define total_distance3f total_distance<Vec3f>
/// Sum up total distance for vec_Vec2i
#define total_distance2i total_distance<Vec2i>
/// Sum up total distance for vec_Vec3i
#define total_distance3i total_distance<Vec3i>

#endif  // DATA_UTILS_H
