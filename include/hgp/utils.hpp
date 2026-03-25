/* ----------------------------------------------------------------------------
 * Copyright 2025, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <deque>
#include <hgp/data_utils.hpp>
#include <hgp/termcolor.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "sando/sando_type.hpp"

#define RED 1
#define RED_TRANS 2
#define RED_TRANS_TRANS 3
#define GREEN 4
#define BLUE 5
#define BLUE_TRANS 6
#define BLUE_TRANS_TRANS 7
#define BLUE_LIGHT 8
#define YELLOW 9
#define ORANGE_TRANS 10
#define BLACK_TRANS 11
#define ORANGE 12
#define GREEN_TRANS_TRANS 13

#define STATE 0
#define INPUT 1

#define WHOLE_TRAJ 0
#define RESCUE_PATH 1

#define OCCUPIED_SPACE 1
#define UNKOWN_AND_OCCUPIED_SPACE 2

/** @brief Convert a vector of 3D waypoints to a MarkerArray for RViz visualization.
 *  @param traj Vector of 3D waypoints.
 *  @param m_array Output MarkerArray pointer.
 *  @param color Color for the markers.
 *  @param type Marker type (default: ARROW).
 *  @param radii Optional per-waypoint radii for marker scaling.
 */
void vectorOfVectors2MarkerArray(vec_Vecf<3> traj, visualization_msgs::msg::MarkerArray* m_array,
                                 std_msgs::msg::ColorRGBA color,
                                 int type = visualization_msgs::msg::Marker::ARROW,
                                 std::vector<double> radii = std::vector<double>());

/** @brief Convert a path to a MarkerArray with line strips and dot markers.
 *  @param traj Vector of 3D waypoints.
 *  @param m_array Output MarkerArray pointer.
 *  @param color Color for the markers.
 *  @param line_width Width of the line strip.
 *  @param dot_diameter Diameter of the dot markers.
 *  @param base_id Starting marker ID.
 *  @param frame_id TF frame for the markers.
 *  @param lifetime_sec Marker lifetime in seconds.
 */
void pathLineDotsToMarkerArray(const vec_Vecf<3>& traj,
                               visualization_msgs::msg::MarkerArray* m_array,
                               const std_msgs::msg::ColorRGBA& color, double line_width = 0.03,
                               double dot_diameter = 0.06, int base_id = 50000,
                               const std::string& frame_id = "map", double lifetime_sec = 1.0);

/** @brief Map a scalar value to a jet colormap color.
 *  @param v Value to map.
 *  @param vmin Minimum value of the range.
 *  @param vmax Maximum value of the range.
 *  @return ColorRGBA from the jet colormap.
 */
std_msgs::msg::ColorRGBA getColorJet(double v, double vmin, double vmax);

/** @brief Get a predefined RGBA color by ID.
 *  @param id Color identifier constant (e.g., RED, BLUE, YELLOW).
 *  @return ColorRGBA message.
 */
std_msgs::msg::ColorRGBA color(int id);

/** @brief Convert a tf2 quaternion to Euler angles (roll, pitch, yaw).
 *  @param q Input quaternion.
 *  @param roll Output roll angle in radians.
 *  @param pitch Output pitch angle in radians.
 *  @param yaw Output yaw angle in radians.
 */
void quaternion2Euler(tf2::Quaternion q, double& roll, double& pitch, double& yaw);

/** @brief Convert an Eigen quaternion to Euler angles (roll, pitch, yaw).
 *  @param q Input quaternion.
 *  @param roll Output roll angle in radians.
 *  @param pitch Output pitch angle in radians.
 *  @param yaw Output yaw angle in radians.
 */
void quaternion2Euler(Eigen::Quaterniond q, double& roll, double& pitch, double& yaw);

/** @brief Convert a geometry_msgs quaternion to Euler angles (roll, pitch, yaw).
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

/** @brief Compute the angle in radians between two 3D vectors.
 *  @param a First vector.
 *  @param b Second vector.
 *  @return Angle between a and b in radians.
 */
double angleBetVectors(const Eigen::Vector3d& a, const Eigen::Vector3d& b);

/** @brief Sample points on a sphere surface near a given point.
 *  @param B Reference point on the sphere.
 *  @param r Radius of the sphere.
 *  @param center Center of the sphere.
 *  @return Vector of sampled 3D points on the sphere.
 */
std::vector<Eigen::Vector3d> samplePointsSphere(Eigen::Vector3d& B, double r,
                                                Eigen::Vector3d& center);

/** @brief Sample points on a sphere surface guided by a JPS path.
 *  @param B First intersection of the JPS path with the sphere.
 *  @param r Radius of the sphere.
 *  @param center_sent Center of the sphere.
 *  @param path_sent JPS path used to guide sampling.
 *  @param last_index_inside_sphere Index of the last path point inside the sphere.
 *  @return Vector of sampled 3D points on the sphere.
 */
std::vector<Eigen::Vector3d> samplePointsSphereWithJPS(Eigen::Vector3d& B, double r,
                                                       Eigen::Vector3d& center_sent,
                                                       vec_Vecf<3>& path_sent,
                                                       int last_index_inside_sphere);

/** @brief Wrap an angle difference to the range [-pi, pi].
 *  @param diff Angle difference in radians (modified in place).
 */
void angle_wrap(double& diff);

/** @brief Convert a PCL point cloud pointer to a vector of 3D float points.
 *  @param ptr_cloud Input PCL point cloud pointer.
 *  @return Vector of 3D float points.
 */
vec_Vec3f pclptr_to_vec(const pcl::KdTreeFLANN<pcl::PointXYZ>::PointCloudConstPtr ptr_cloud);

/** @brief Merge two PCL point cloud pointers into a single vector of 3D float points.
 *  @param ptr_cloud1 First point cloud pointer.
 *  @param ptr_cloud2 Second point cloud pointer.
 *  @return Combined vector of 3D float points.
 */
vec_Vec3f pclptr_to_vec(const pcl::KdTreeFLANN<pcl::PointXYZ>::PointCloudConstPtr ptr_cloud1,
                        const pcl::KdTreeFLANN<pcl::PointXYZ>::PointCloudConstPtr ptr_cloud2);

/** @brief Create a geometry_msgs Point at the origin (0, 0, 0).
 *  @return Point message at origin.
 */
geometry_msgs::msg::Point pointOrigin();

/** @brief Convert a geometry_msgs Vector3 to an Eigen 3D vector.
 *  @param vector Input ROS vector.
 *  @return Equivalent Eigen vector.
 */
Eigen::Vector3d vec2eigen(geometry_msgs::msg::Vector3 vector);

/** @brief Convert an Eigen 3D vector to a geometry_msgs Vector3.
 *  @param vector Input Eigen vector.
 *  @return Equivalent ROS vector.
 */
geometry_msgs::msg::Vector3 eigen2rosvector(Eigen::Vector3d vector);

/** @brief Convert an Eigen 3D vector to a geometry_msgs Point.
 *  @param vector Input Eigen vector.
 *  @return Equivalent ROS point.
 */
geometry_msgs::msg::Point eigen2point(Eigen::Vector3d vector);

/** @brief Create a geometry_msgs Vector3 with all components zero.
 *  @return Zero vector message.
 */
geometry_msgs::msg::Vector3 vectorNull();

/** @brief Create a geometry_msgs Vector3 with all components set to a uniform value.
 *  @param a Value for all components.
 *  @return Uniform vector message.
 */
geometry_msgs::msg::Vector3 vectorUniform(double a);

template <typename T>
using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;

template <int N>
using Vecf = Eigen::Matrix<decimal_t, N, 1>;  // Be CAREFUL, because this is with doubles!

template <int N>
using vec_Vecf = vec_E<Vecf<N>>;

/** @brief Check intersection of a line segment with a plane Ax+By+Cz+D=0.
 *  @param P1 First endpoint of the segment.
 *  @param P2 Second endpoint of the segment.
 *  @param coeff Plane coefficients [A, B, C, D].
 *  @param intersection Output intersection point (if found).
 *  @return True if the segment intersects the plane.
 */
bool getIntersectionWithPlane(const Eigen::Vector3d& P1, const Eigen::Vector3d& P2,
                              const Eigen::Vector4d& coeff, Eigen::Vector3d& intersection);

/** @brief Compute the cumulative path length from a given index to the end.
 *  @param path Input path as a vector of 3D points.
 *  @param index_start Starting index.
 *  @return Total path length from index_start to the end.
 */
double normJPS(vec_Vecf<3>& path, int index_start);

/** @brief Crop the end of a path by removing a given distance from the tail.
 *  @param path Path to crop (modified in place).
 *  @param d Distance to remove from the end.
 */
void reduceJPSbyDistance(vec_Vecf<3>& path, double d);

/** @brief Compute the intersection of a line segment with a sphere.
 *  @param A Point inside the sphere.
 *  @param B Point outside the sphere.
 *  @param r Radius of the sphere.
 *  @param center Center of the sphere.
 *  @return Intersection point on the sphere surface.
 */
Eigen::Vector3d getIntersectionWithSphere(Eigen::Vector3d& A, Eigen::Vector3d& B, double r,
                                          Eigen::Vector3d& center);

/** @brief Find the first intersection of a path with a sphere.
 *  @param path Path starting inside the sphere.
 *  @param r Radius of the sphere.
 *  @param center Center of the sphere (prepended to path internally).
 *  @param last_index_inside_sphere Optional output: index of the last point inside the sphere.
 *  @param noPointsOutsideSphere Optional output: true if the entire path is inside the sphere.
 *  @return First intersection point on the sphere surface.
 */
Eigen::Vector3d getFirstIntersectionWithSphere(vec_Vecf<3>& path, double r, Eigen::Vector3d& center,
                                               int* last_index_inside_sphere = NULL,
                                               bool* noPointsOutsideSphere = NULL);

/** @brief Find the last intersection of a path with a sphere.
 *  @param path Path starting inside the sphere.
 *  @param r Radius of the sphere.
 *  @param center Center of the sphere.
 *  @return Last intersection point on the sphere surface.
 */
Eigen::Vector3d getLastIntersectionWithSphere(vec_Vecf<3> path, double r, Eigen::Vector3d center);

/** @brief Compute the total length of a path.
 *  @param path Input path as a vector of 3D points.
 *  @return Total path length.
 */
double getDistancePath(vec_Vecf<3>& path);

/** @brief Find the last intersection of a path with a sphere, also returning remaining distance.
 *  @param path Path starting inside the sphere.
 *  @param r Radius of the sphere.
 *  @param center Center of the sphere.
 *  @param Jdist Output: distance from the last intersection to the path end.
 *  @return Last intersection point on the sphere surface.
 */
Eigen::Vector3d getLastIntersectionWithSphere(vec_Vecf<3> path, double r, Eigen::Vector3d center,
                                              double* Jdist);

/** @brief Extract path points lying between two concentric spheres.
 *  @param path Input path.
 *  @param ra Radius of the inner sphere.
 *  @param rb Radius of the outer sphere.
 *  @param center Common center of the spheres.
 *  @return Points between the two spheres.
 */
vec_Vecf<3> getPointsBw2Spheres(vec_Vecf<3> path, double ra, double rb, Eigen::Vector3d center);

/** @brief Stream insertion operator for printing std::vectors.
 *  @tparam T Element type.
 *  @param out Output stream.
 *  @param v Vector to print.
 *  @return Reference to the output stream.
 */
template <typename T>
std::ostream& operator<<(std::ostream& out, const std::vector<T>& v) {
  if (!v.empty()) {
    out << '[';
    std::copy(v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
    out << "\b\b]";
  }
  return out;
}

/** @brief Convert a state vector to a jet-colored MarkerArray based on velocity.
 *  @param data Vector of states to visualize.
 *  @param type Whether to visualize STATE or INPUT.
 *  @param max_value Maximum value for the jet colormap scaling.
 *  @param stamp Timestamp for the markers.
 *  @return MarkerArray with jet-colored markers.
 */
visualization_msgs::msg::MarkerArray stateVector2ColoredMarkerArray(const std::vector<RobotState>& data,
                                                                    int type, double max_value,
                                                                    const rclcpp::Time& stamp);

/** @brief Convert a state vector to a jet-colored line strip MarkerArray.
 *  @param data Vector of states to visualize.
 *  @param id Marker ID.
 *  @param ns Marker namespace.
 *  @param max_value Maximum value for the jet colormap scaling.
 *  @param stamp Timestamp for the markers.
 *  @param line_width Width of the line strip.
 *  @param max_points_vis Maximum number of points to visualize.
 *  @return MarkerArray with a jet-colored line strip.
 */
visualization_msgs::msg::MarkerArray stateVector2ColoredLineStripMarkerArray(
    const std::vector<RobotState>& data, int id, const std::string& ns, double max_value,
    const rclcpp::Time& stamp, double line_width, size_t max_points_vis);

/** @brief Remove vertices from a JPS path, keeping at most max_value points.
 *  @param JPS_path Path to prune (modified in place).
 *  @param max_value Maximum number of vertices to keep.
 */
void deleteVertexes(vec_Vecf<3>& JPS_path, int max_value);
