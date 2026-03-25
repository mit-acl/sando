/* ----------------------------------------------------------------------------
 * Copyright 2025, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <deque>
#include <hgp/utils.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sando/sando_type.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "dynus_interfaces/msg/coeff_poly3.hpp"
#include "dynus_interfaces/msg/dyn_traj.hpp"
#include "dynus_interfaces/msg/pwp_traj.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"

namespace sando_utils {

// Define colors
static constexpr int red_normal = 1;
static constexpr int red_trans = 2;
static constexpr int red_trans_trans = 3;
static constexpr int green_normal = 4;
static constexpr int blue_normal = 5;
static constexpr int blue_trans = 6;
static constexpr int blue_trans_trans = 7;
static constexpr int blue_light = 8;
static constexpr int yellow_normal = 9;
static constexpr int orange_trans = 10;
static constexpr int black_trans = 11;
static constexpr int teal_normal = 12;
static constexpr int green_trans_trans = 13;

/** @brief Convert a PWPTraj ROS message to a PieceWisePol struct.
 *  @param pwp_msg ROS message to convert.
 *  @return Equivalent PieceWisePol.
 */
PieceWisePol convertPwpMsg2Pwp(const dynus_interfaces::msg::PWPTraj& pwp_msg);

/** @brief Convert a PieceWisePol struct to a PWPTraj ROS message.
 *  @param pwp PieceWisePol to convert.
 *  @return Equivalent ROS message.
 */
dynus_interfaces::msg::PWPTraj convertPwp2PwpMsg(const PieceWisePol& pwp);

/** @brief Convert a PieceWisePol trajectory to a colored marker array for visualization.
 *  @param pwp Piecewise polynomial trajectory.
 *  @param samples Number of sample points per segment.
 *  @return MarkerArray colored by velocity magnitude.
 */
visualization_msgs::msg::MarkerArray convertPwp2ColoredMarkerArray(PieceWisePol& pwp, int samples);

/** @brief Convert an Eigen 3D vector to a geometry_msgs Point.
 *  @param vector Eigen vector to convert.
 *  @return Equivalent geometry_msgs Point.
 */
geometry_msgs::msg::Point convertEigen2Point(Eigen::Vector3d vector);

/** @brief Convert a float covariance message to an Eigen 3D vector.
 *  @param msg_cov Float vector containing diagonal covariance values.
 *  @return Eigen vector with covariance values.
 */
Eigen::Vector3d convertCovMsg2Cov(const std::vector<float>& msg_cov);

/** @brief Convert an Eigen covariance vector to a float message vector.
 *  @param cov Eigen vector with covariance values.
 *  @return Float vector for ROS message.
 */
std::vector<float> convertCov2CovMsg(const Eigen::Vector3d& cov);

/** @brief Convert a float coefficient message to a 6x1 Eigen vector.
 *  @param msg_coeff Float vector containing polynomial coefficients.
 *  @return 6x1 Eigen vector of coefficients.
 */
Eigen::Matrix<double, 6, 1> convertCoeffMsg2Coeff(const std::vector<float>& msg_coeff);

/** @brief Get an RGBA color corresponding to a predefined color ID.
 *  @param id Color identifier constant (e.g., red_normal, blue_trans).
 *  @return ColorRGBA message with the requested color.
 */
std_msgs::msg::ColorRGBA getColor(int id);

/** @brief Convert polynomial coefficients to Bezier-like control points.
 *  @param pwp Piecewise polynomial trajectory.
 *  @param A_rest_pos_basis_inverse Inverse of the basis matrix for conversion.
 *  @return Vector of 3x4 control point matrices, one per segment.
 */
std::vector<Eigen::Matrix<double, 3, 4>> convertCoefficients2ControlPoints(
    const PieceWisePol& pwp, const Eigen::Matrix<double, 4, 4>& A_rest_pos_basis_inverse);

/** @brief Compute minimum time for a 1D double-integrator point-to-point maneuver.
 *  @param p0 Initial position.
 *  @param v0 Initial velocity.
 *  @param pf Final position.
 *  @param vf Final velocity.
 *  @param v_max Maximum velocity.
 *  @param a_max Maximum acceleration.
 *  @return Minimum transfer time.
 */
double getMinTimeDoubleIntegrator1D(const double p0, const double v0, const double pf,
                                    const double vf, const double v_max, const double a_max);

/** @brief Compute minimum time for a 3D double-integrator point-to-point maneuver.
 *  @param p0 Initial position.
 *  @param v0 Initial velocity.
 *  @param pf Final position.
 *  @param vf Final velocity.
 *  @param v_max Per-axis maximum velocity.
 *  @param a_max Per-axis maximum acceleration.
 *  @return Minimum transfer time (maximum across axes).
 */
double getMinTimeDoubleIntegrator3D(const Eigen::Vector3d& p0, const Eigen::Vector3d& v0,
                                    const Eigen::Vector3d& pf, const Eigen::Vector3d& vf,
                                    const Eigen::Vector3d& v_max, const Eigen::Vector3d& a_max);

/** @brief Convert a quaternion to Euler angles (roll, pitch, yaw).
 *  @param q Input quaternion.
 *  @param roll Output roll angle in radians.
 *  @param pitch Output pitch angle in radians.
 *  @param yaw Output yaw angle in radians.
 */
void quaternion2Euler(tf2::Quaternion q, double& roll, double& pitch, double& yaw);

/** @brief Convert a quaternion to Euler angles (roll, pitch, yaw).
 *  @param q Input quaternion.
 *  @param roll Output roll angle in radians.
 *  @param pitch Output pitch angle in radians.
 *  @param yaw Output yaw angle in radians.
 */
void quaternion2Euler(Eigen::Quaterniond q, double& roll, double& pitch, double& yaw);

/** @brief Convert a quaternion to Euler angles (roll, pitch, yaw).
 *  @param q Input quaternion.
 *  @param roll Output roll angle in radians.
 *  @param pitch Output pitch angle in radians.
 *  @param yaw Output yaw angle in radians.
 */
void quaternion2Euler(geometry_msgs::msg::Quaternion q, double& roll, double& pitch, double& yaw);

/** @brief Clamp a value to the range [min, max].
 *  @param var Value to saturate (modified in place).
 *  @param min Lower bound.
 *  @param max Upper bound.
 */
void saturate(double& var, double min, double max);

/** @brief Wrap an angle difference to the range [-pi, pi].
 *  @param diff Angle difference in radians (modified in place).
 */
void angle_wrap(double& diff);

/** @brief Project a point onto the surface of an axis-aligned box.
 *  @param P1 Center of the box.
 *  @param P2 Point to project.
 *  @param wdx Half-width of the box along x.
 *  @param wdy Half-width of the box along y.
 *  @param wdz Half-width of the box along z.
 *  @return Projected point on the box surface.
 */
Eigen::Vector3d projectPointToBox(Eigen::Vector3d& P1, Eigen::Vector3d& P2, double wdx, double wdy,
                                  double wdz);

/** @brief Project a point onto the surface of a sphere.
 *  @param P1 Center of the sphere.
 *  @param P2 Point to project.
 *  @param radius Radius of the sphere.
 *  @return Projected point on the sphere surface.
 */
Eigen::Vector3d projectPointToSphere(const Eigen::Vector3d& P1, const Eigen::Vector3d& P2,
                                     double radius);

/** @brief Convert a MarkerArray of obstacles to vectors of polygon vertices and scales.
 *  @param marker_array Input MarkerArray containing obstacle markers.
 *  @param vec Output vector of polygon vertex lists.
 *  @param scale_vec Output vector of obstacle scales.
 */
void convertMarkerArray2Vec_Vec_Vecf3(const visualization_msgs::msg::MarkerArray& marker_array,
                                      std::vector<vec_Vecf<3>>& vec,
                                      std::vector<double>& scale_vec);

/** @brief Subdivide path segments so that no segment exceeds a given length.
 *  @param path Path to subdivide (modified in place).
 *  @param d Maximum allowed segment length.
 */
void createMoreVertexes(vec_Vecf<3>& path, double d);

/** @brief Compute the Euclidean distance between two 3D points.
 *  @param p1 First point.
 *  @param p2 Second point.
 *  @return Distance between p1 and p2.
 */
double euclideanDistance(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);

/** @brief Convert a TransformStamped message to a 4x4 homogeneous transformation matrix.
 *  @param transform_stamped Input transform message.
 *  @return 4x4 Eigen transformation matrix.
 */
Eigen::Matrix4d transformStampedToMatrix(
    const geometry_msgs::msg::TransformStamped& transform_stamped);

/** @brief Estimate velocity vectors at each waypoint along a path.
 *  @param path Input waypoint path.
 *  @param velocities Output velocity vectors at each waypoint.
 *  @param A Current vehicle state providing initial velocity.
 *  @param v_max_3d Per-axis maximum velocity limits.
 *  @param verbose Enable debug output.
 */
void findVelocitiesInPath(const vec_Vecf<3>& path, vec_Vecf<3>& velocities, const RobotState& A,
                          const Eigen::Vector3d& v_max_3d, bool verbose);

/** @brief Compute segment travel times along a path using double-integrator dynamics.
 *  @param path Input waypoint path.
 *  @param A Current vehicle state.
 *  @param debug_verbose Enable debug output.
 *  @param v_max_3d Per-axis maximum velocity limits.
 *  @param a_max_3d Per-axis maximum acceleration limits.
 *  @return Vector of travel times for each segment.
 */
std::vector<double> getTravelTimes(const vec_Vecf<3>& path, const RobotState& A, bool debug_verbose,
                                   const Eigen::Vector3d& v_max_3d,
                                   const Eigen::Vector3d& a_max_3d);

/** @brief Create an identity geometry_msgs Pose (position zero, orientation identity).
 *  @return Identity Pose message.
 */
geometry_msgs::msg::Pose identityGeometryMsgsPose();

/** @brief Compute the sign of a value (-1, 0, or +1).
 *  @tparam T Numeric type.
 *  @param val Input value.
 *  @return -1 if negative, 0 if zero, +1 if positive.
 */
template <typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

}  // namespace sando_utils
