/* ----------------------------------------------------------------------------
 * Copyright 2025, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <Eigen/Core>
#include <algorithm>  // for std::clamp
#include <iostream>
#include <memory>
#include <sim/exprtk.hpp>
#include <vector>

#include "hgp/data_type.hpp"

/** @brief Holds position, velocity, acceleration, and jerk vectors for a state derivative. */
struct StateDeriv {
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Vector3d accel;
  Eigen::Vector3d jerk;
};

/** @brief Represents a convex polytope defined by the half-space inequalities Ax <= b. */
struct Polytope {
  Eigen::MatrixXd A;
  Eigen::MatrixXd b;
};

/** @brief Configuration parameters for the SANDO planner, loaded from YAML at startup. */
struct Parameters {
  // Sim enviroment
  std::string sim_env;
  bool use_global_pc;

  // UAV or Ground robot
  std::string vehicle_type;
  bool provide_goal_in_global_frame;
  bool state_already_in_global_frame;
  bool use_hardware;

  // Flight mode
  std::string flight_mode;

  // Visual level
  int visual_level;

  // Global planner parameters
  std::string global_planner;
  bool global_planner_verbose;
  double global_planner_heuristic_weight;
  double factor_hgp;
  double inflation_hgp;
  double x_min;
  double x_max;
  double y_min;
  double y_max;
  double z_min;
  double z_max;
  double drone_radius;
  int hgp_timeout_duration_ms;
  int max_num_expansion;
  bool use_free_start;
  double free_start_factor;
  bool use_free_goal;
  double free_goal_factor;

  // LOS post processing parameters
  int los_cells;
  double min_len;   // [m] minimum length between two waypoints after post processing
  double min_turn;  // [deg] minimum turn angle after post processing

  // Path push visualization parameters
  bool use_state_update;
  bool use_random_color_for_global_path;
  bool use_path_push_for_visualization;

  // Decomposition parameters
  std::string environment_assumption;
  std::vector<double> sfc_size;
  double min_dist_from_agent_to_traj;
  bool use_shrinked_box;
  double shrinked_box_size;

  // Map parameters
  double map_buffer;
  double center_shift_factor;
  double initial_wdx;
  double initial_wdy;
  double initial_wdz;
  double min_wdx;
  double min_wdy;
  double min_wdz;
  double res;

  // Communication delay parameters
  bool use_comm_delay_inflation;
  double comm_delay_inflation_alpha;
  double comm_delay_inflation_max;
  double comm_delay_filter_alpha;

  // Simulation parameters
  double depth_camera_depth_max;
  double fov_visual_depth;
  double fov_visual_x_deg;
  double fov_visual_y_deg;

  // number of segments parameters
  double max_dist_vertexes;  // [m] Maximum distance between two consecutive vertexes
  double w_unknown;          // [-] Weight for the unknown cells in the global planner
  double w_align;            // strength of alignment penalty (cells)
  double decay_len_cells;    // e-folding distance from the start (cells)
  double w_side;             // side (handedness) tie-break strength (cells)
  double heat_weight;        // weight for heat in edge cost

  // Heat map parameters
  bool use_heat_map;

  // Dynamic heat
  bool dynamic_heat_enabled;
  bool dynamic_as_occupied_current;
  bool dynamic_as_occupied_future;
  bool use_only_curr_pos_for_dynamic_obst;
  double heat_alpha0, heat_alpha1;
  int heat_p, heat_q;
  double heat_tau_ratio, heat_gamma, heat_Hmax;
  double dyn_base_inflation_m, dyn_heat_tube_radius_m;
  int heat_num_samples;

  // Static heat
  bool static_heat_enabled;
  double static_heat_alpha;
  int static_heat_p;
  double static_heat_Hmax, static_heat_rmax_m, static_heat_default_radius_m;
  bool static_heat_boundary_only, static_heat_apply_on_unknown, static_heat_exclude_dynamic;

  // Soft-cost
  bool use_soft_cost_obstacles;
  double obstacle_soft_cost;

  // Optimiztion parameters
  double horizon;
  double dc;
  std::string dynamic_constraint_type;  // "Linf", "L1", or "L2"
  double v_max;
  double a_max;
  double j_max;
  std::vector<double> drone_bbox;
  double goal_radius;
  double goal_seen_radius;

  // SANDO specific parameters
  int num_P;                           // number of polytopes
  int num_N;                           // number of trajectory pieces
  bool use_dynamic_factor;             // use dynamic factor for the trajectory time allocation
  double dynamic_factor_k_radius;      // the factors are [prev_successful_factor - k
                                       // radius, prev_successful_factor - k_radius +
                                       // factor_constant_step_size, ..., prev_successful_factor,
                                       // ... prev_successful_factor + k_radius]
  double dynamic_factor_initial_mean;  // initial factor for the dynamic factor search (only used
                                       // when use_dynamic_factor is true)
  double factor_initial = 1.0;         // initial factor for the trajectory time allocation
  double factor_final = 5.0;           // final factor for the trajectory time allocation
  double factor_constant_step_size = 0.1;  // step size for the constant factor increase
  double obst_max_vel;                     // maximum velocity of dynamic obstacles
  double obst_position_error = 0.0;      // bounded position estimation error for dynamic obstacles
  bool inflate_unknown_boundary = true;  // inflate unknown-space boundary in corridor decomposition
  double max_gurobi_comp_time_sec;       // maximum Gurobi computation time per replanning
  double jerk_smooth_weight;             // weight for the jerk smoothness
  bool using_variable_elimination = true;

  // Dynamic obstacles parameters
  double traj_lifetime;

  // Dynamic k_value parameters
  int num_replanning_before_adapt;
  int default_k_value;
  double alpha_k_value_filtering;
  double k_value_factor;

  // Yaw-related parameters
  double alpha_filter_dyaw;
  double w_max;
  double w_max_yawing;
  bool skip_initial_yawing;
  int yaw_spinning_threshold;
  double yaw_spinning_dyaw;

  // Simulation env parameters
  bool force_goal_z;
  double default_goal_z;

  // Debug flag
  bool debug_verbose;

  // Hover avoidance parameters
  bool ignore_other_trajs = false;
  bool hover_avoidance_enabled = false;
  bool hover_avoidance_2d = true;
  double hover_avoidance_d_trigger = 4.0;
  double hover_avoidance_h = 3.0;
  double hover_avoidance_min_repulsion_norm = 0.01;
};

/** @brief Precomputed basis conversion matrices for transforming between B-spline, Bezier, and MINVO representations. */
struct BasisConverter {
  Eigen::Matrix<double, 4, 4> A_pos_mv_rest;
  Eigen::Matrix<double, 4, 4> A_pos_mv_rest_inv;
  Eigen::Matrix<double, 3, 3> A_vel_mv_rest;
  Eigen::Matrix<double, 3, 3> A_vel_mv_rest_inv;
  Eigen::Matrix<double, 2, 2> A_accel_mv_rest;
  Eigen::Matrix<double, 2, 2> A_accel_mv_rest_inv;
  Eigen::Matrix<double, 4, 4> A_pos_be_rest;
  Eigen::Matrix<double, 4, 4> A_pos_bs_seg0, A_pos_bs_seg1, A_pos_bs_rest, A_pos_bs_seg_last2,
      A_pos_bs_seg_last;

  Eigen::Matrix<double, 4, 4> M_pos_bs2mv_seg0, M_pos_bs2mv_seg1, M_pos_bs2mv_rest,
      M_pos_bs2mv_seg_last2, M_pos_bs2mv_seg_last;

  Eigen::Matrix<double, 4, 4> M_pos_bs2be_seg0, M_pos_bs2be_seg1, M_pos_bs2be_rest,
      M_pos_bs2be_seg_last2, M_pos_bs2be_seg_last;

  Eigen::Matrix<double, 3, 3> M_vel_bs2mv_seg0, M_vel_bs2mv_rest, M_vel_bs2mv_seg_last;
  Eigen::Matrix<double, 3, 3> M_vel_bs2be_seg0, M_vel_bs2be_rest, M_vel_bs2be_seg_last;

  BasisConverter() {
    // See matlab.
    // This is for t \in [0 1];

    //////MATRICES A FOR MINVO POSITION///////// (there is only one)
    A_pos_mv_rest << -3.4416308968564117698463178385282, 6.9895481477801393310755884158425,
        -4.4622887507045296828778191411402, 0.91437149978080234369315348885721,
        6.6792587327074839365081970754545, -11.845989901556746914934592496138,
        5.2523596690684613008670567069203, 0, -6.6792587327074839365081970754545,
        8.1917862965657040064115790301003, -1.5981560640774179482548333908198,
        0.085628500219197656306846511142794, 3.4416308968564117698463178385282,
        -3.3353445427890959784633650997421, 0.80808514571348655231020075007109,
        -0.0000000000000000084567769453869345852581318467855;

    //////INVERSE OF A_pos_mv_rest
    A_pos_mv_rest_inv = A_pos_mv_rest.inverse();

    //////MATRICES A FOR MINVO VELOCITY///////// (there is only one)
    A_vel_mv_rest << 1.4999999992328318931811281800037, -2.3660254034601951866889635311964,
        0.9330127021136816189983420599674, -2.9999999984656637863622563600074,
        2.9999999984656637863622563600074, 0, 1.4999999992328318931811281800037,
        -0.6339745950054685996732928288111, 0.066987297886318325490506708774774;

    //////INVERSE OF A_vel_mv_rest
    A_vel_mv_rest_inv = A_vel_mv_rest.inverse();

    //////MATRICES A FOR MINVO ACCELERATION///////// (there is only one)
    A_accel_mv_rest << -1.0, 1.0, 1.0, 0.0;

    /////INVERSE OF A_accel_mv_rest
    A_accel_mv_rest_inv = A_accel_mv_rest.inverse();

    //////MATRICES A FOR Bezier POSITION///////// (there is only one)
    A_pos_be_rest <<

        -1.0,
        3.0, -3.0, 1.0, 3.0, -6.0, 3.0, 0, -3.0, 3.0, 0, 0, 1.0, 0, 0, 0;

    //////MATRICES A FOR BSPLINE POSITION/////////
    A_pos_bs_seg0 <<

        -1.0000,
        3.0000, -3.0000, 1.0000, 1.7500, -4.5000, 3.0000, 0, -0.9167, 1.5000, 0, 0, 0.1667, 0, 0, 0;

    A_pos_bs_seg1 <<

        -0.2500,
        0.7500, -0.7500, 0.2500, 0.5833, -1.2500, 0.2500, 0.5833, -0.5000, 0.5000, 0.5000, 0.1667,
        0.1667, 0, 0, 0;

    A_pos_bs_rest <<

        -0.1667,
        0.5000, -0.5000, 0.1667, 0.5000, -1.0000, 0, 0.6667, -0.5000, 0.5000, 0.5000, 0.1667,
        0.1667, 0, 0, 0;

    A_pos_bs_seg_last2 <<

        -0.1667,
        0.5000, -0.5000, 0.1667, 0.5000, -1.0000, 0.0000, 0.6667, -0.5833, 0.5000, 0.5000, 0.1667,
        0.2500, 0, 0, 0;

    A_pos_bs_seg_last <<

        -0.1667,
        0.5000, -0.5000, 0.1667, 0.9167, -1.2500, -0.2500, 0.5833, -1.7500, 0.7500, 0.7500, 0.2500,
        1.0000, 0, 0, 0;

    //////BSPLINE to MINVO POSITION/////////

    M_pos_bs2mv_seg0 <<

        1.1023313949144333268037598827505,
        0.34205724556666972091534262290224, -0.092730934245582874453361910127569,
        -0.032032766697130621302846975595457, -0.049683556253749178166501110354147,
        0.65780347324677179710050722860615, 0.53053863760186903419935333658941,
        0.21181027098212013015654520131648, -0.047309044211162346038612724896666,
        0.015594436894155586093013710069499, 0.5051827557159349613158383363043,
        0.63650059656260427054519368539331, -0.0053387944495217444854096022766043,
        -0.015455155707597083292181849856206, 0.057009540927778303009976212933907,
        0.18372189915240558222286892942066;

    M_pos_bs2mv_seg1 <<

        0.27558284872860833170093997068761,
        0.085514311391667430228835655725561, -0.023182733561395718613340477531892,
        -0.0080081916742826553257117438988644, 0.6099042761975865811763242163579,
        0.63806904207840509091198555324809, 0.29959938009132258684985572472215,
        0.12252106674808682651445224109921, 0.11985166952332682033244282138185,
        0.29187180223752445806795208227413, 0.66657381254229419731416328431806,
        0.70176522577378930289881964199594, -0.0053387944495217444854096022766043,
        -0.015455155707597083292181849856206, 0.057009540927778303009976212933907,
        0.18372189915240558222286892942066;

    M_pos_bs2mv_rest <<

        0.18372189915240555446729331379174,
        0.057009540927778309948870116841135, -0.015455155707597117986651369392348,
        -0.0053387944495218164764338553140988, 0.70176522577378919187651717948029,
        0.66657381254229419731416328431806, 0.29187180223752384744528853843804,
        0.11985166952332582113172065874096, 0.11985166952332682033244282138185,
        0.29187180223752445806795208227413, 0.66657381254229419731416328431806,
        0.70176522577378930289881964199594, -0.0053387944495217444854096022766043,
        -0.015455155707597083292181849856206, 0.057009540927778303009976212933907,
        0.18372189915240558222286892942066;

    M_pos_bs2mv_seg_last2 <<

        0.18372189915240569324517139193631,
        0.057009540927778309948870116841135, -0.015455155707597145742226985021261,
        -0.0053387944495218164764338553140988, 0.70176522577378952494342456702725,
        0.66657381254229453038107067186502, 0.29187180223752412500104469472717,
        0.11985166952332593215402312125661, 0.1225210667480875342816304396365,
        0.29959938009132280889446064975346, 0.63806904207840497988968309073243,
        0.60990427619758624810941682881094, -0.0080081916742826154270717964323012,
        -0.023182733561395621468825822830695, 0.085514311391667444106623463540018,
        0.27558284872860833170093997068761;

    M_pos_bs2mv_seg_last <<

        0.18372189915240555446729331379174,
        0.057009540927778309948870116841135, -0.015455155707597117986651369392348,
        -0.0053387944495218164764338553140988, 0.63650059656260415952289122287766,
        0.5051827557159349613158383363043, 0.015594436894155294659469745965907,
        -0.047309044211162887272337229660479, 0.21181027098212068526805751389475,
        0.53053863760186914522165579910506, 0.65780347324677146403359984105919,
        -0.049683556253749622255710960416764, -0.032032766697130461708287185729205,
        -0.09273093424558248587530329132278, 0.34205724556666977642649385416007,
        1.1023313949144333268037598827505;

    //////BSPLINE to BEZIER POSITION/////////

    M_pos_bs2be_seg0 <<

        1.0000,
        0.0000, -0.0000, 0, 0, 1.0000, 0.5000, 0.2500, 0, -0.0000, 0.5000, 0.5833, 0, 0, 0, 0.1667;

    M_pos_bs2be_seg1 <<

        0.2500,
        0.0000, -0.0000, 0, 0.5833, 0.6667, 0.3333, 0.1667, 0.1667, 0.3333, 0.6667, 0.6667, 0, 0, 0,
        0.1667;

    M_pos_bs2be_rest <<

        0.1667,
        0.0000, 0, 0, 0.6667, 0.6667, 0.3333, 0.1667, 0.1667, 0.3333, 0.6667, 0.6667, 0, 0, 0,
        0.1667;

    M_pos_bs2be_seg_last2 <<

        0.1667,
        0, -0.0000, 0, 0.6667, 0.6667, 0.3333, 0.1667, 0.1667, 0.3333, 0.6667, 0.5833, 0, 0, 0,
        0.2500;

    M_pos_bs2be_seg_last <<

        0.1667,
        0.0000, 0, 0, 0.5833, 0.5000, 0, 0, 0.2500, 0.5000, 1.0000, 0, 0, 0, 0, 1.0000;

    /////BSPLINE to MINVO VELOCITY
    M_vel_bs2mv_seg0 <<

        1.077349059083916,
        0.1666702138890985, -0.07735049175615138, -0.03867488648729411, 0.7499977187062712,
        0.5386802643920123, -0.03867417280506149, 0.08333206631563977, 0.538670227146185;

    M_vel_bs2mv_rest <<

        0.538674529541958,
        0.08333510694454926, -0.03867524587807569, 0.4999996430546639, 0.8333328256508203,
        0.5000050185139366, -0.03867417280506149, 0.08333206631563977, 0.538670227146185;

    M_vel_bs2mv_seg_last <<

        0.538674529541958,
        0.08333510694454926, -0.03867524587807569, 0.5386738158597254, 0.7500007593351806,
        -0.03866520863224832, -0.07734834561012298, 0.1666641326312795, 1.07734045429237;

    /////BSPLINE to BEZIER VELOCITY
    M_vel_bs2be_seg0 <<

        1.0000,
        0, 0, 0, 1.0000, 0.5000, 0, 0, 0.5000;

    M_vel_bs2be_rest <<

        0.5000,
        0, 0, 0.5000, 1.0000, 0.5000, 0, 0, 0.5000;

    M_vel_bs2be_seg_last <<

        0.5000,
        0, 0, 0.5000, 1.0000, 0, 0, 0, 1.0000;
  }

  //////MATRIX A FOR MINVO POSITION/////////
  Eigen::Matrix<double, 4, 4> getArestMinvo() { return A_pos_mv_rest; }
  //////MATRIX A FOR Bezier POSITION/////////
  Eigen::Matrix<double, 4, 4> getArestBezier() { return A_pos_be_rest; }

  //////MATRIX A FOR BSPLINE POSITION/////////
  Eigen::Matrix<double, 4, 4> getArestBSpline() { return A_pos_bs_rest; }

  //////MATRICES A FOR MINVO POSITION/////////
  std::vector<Eigen::Matrix<double, 4, 4>> getAMinvo(int num_pol) {
    std::vector<Eigen::Matrix<double, 4, 4>> A_pos_mv;  // will have as many elements as num_pol
    for (int i = 0; i < num_pol; i++) {
      A_pos_mv.push_back(A_pos_mv_rest);
    }
    return A_pos_mv;
  }

  //////MATRICES A FOR Bezier POSITION/////////
  std::vector<Eigen::Matrix<double, 4, 4>> getABezier(int num_pol) {
    std::vector<Eigen::Matrix<double, 4, 4>> A_pos_be;  // will have as many elements as num_pol
    for (int i = 0; i < num_pol; i++) {
      A_pos_be.push_back(A_pos_be_rest);
    }
    return A_pos_be;
  }

  //////MATRICES A FOR BSPLINE POSITION/////////
  std::vector<Eigen::Matrix<double, 4, 4>> getABSpline(int num_pol) {
    std::vector<Eigen::Matrix<double, 4, 4>> A_pos_bs;  // will have as many elements as num_pol
    A_pos_bs.push_back(A_pos_bs_seg0);
    A_pos_bs.push_back(A_pos_bs_seg1);
    for (int i = 0; i < (num_pol - 4); i++) {
      A_pos_bs.push_back(A_pos_bs_rest);
    }
    A_pos_bs.push_back(A_pos_bs_seg_last2);
    A_pos_bs.push_back(A_pos_bs_seg_last);
    return A_pos_bs;
  }

  //////BSPLINE to MINVO POSITION/////////
  std::vector<Eigen::Matrix<double, 4, 4>> getMinvoPosConverters(int num_pol) {
    std::vector<Eigen::Matrix<double, 4, 4>> M_pos_bs2mv;  // will have as many elements as num_pol
    M_pos_bs2mv.push_back(M_pos_bs2mv_seg0);
    M_pos_bs2mv.push_back(M_pos_bs2mv_seg1);
    for (int i = 0; i < (num_pol - 4); i++) {
      M_pos_bs2mv.push_back(M_pos_bs2mv_rest);
    }
    M_pos_bs2mv.push_back(M_pos_bs2mv_seg_last2);
    M_pos_bs2mv.push_back(M_pos_bs2mv_seg_last);
    return M_pos_bs2mv;
  }

  //////BEZIER to MINVO POSITION/////////
  //////Q_{MINVO} = M_{BEZIER2MINVO} * Q_{BEZIER}
  Eigen::Matrix<double, 4, 4> getMinvoPosConverterFromBezier() {
    // Compute the conversion matrix for one segment
    Eigen::Matrix<double, 4, 4> M_be2mv = A_pos_mv_rest_inv * A_pos_be_rest;

    return M_be2mv;
  }

  //////BSPLINE to BEZIER POSITION/////////
  std::vector<Eigen::Matrix<double, 4, 4>> getBezierPosConverters(int num_pol) {
    std::vector<Eigen::Matrix<double, 4, 4>> M_pos_bs2be;  // will have as many elements as num_pol
    M_pos_bs2be.push_back(M_pos_bs2be_seg0);
    M_pos_bs2be.push_back(M_pos_bs2be_seg1);
    for (int i = 0; i < (num_pol - 4); i++) {
      M_pos_bs2be.push_back(M_pos_bs2be_rest);
    }
    M_pos_bs2be.push_back(M_pos_bs2be_seg_last2);
    M_pos_bs2be.push_back(M_pos_bs2be_seg_last);
    return M_pos_bs2be;
  }

  //////BSPLINE to BSPLINE POSITION/////////
  std::vector<Eigen::Matrix<double, 4, 4>> getBSplinePosConverters(int num_pol) {
    std::vector<Eigen::Matrix<double, 4, 4>> M_pos_bs2bs;  // will have as many elements as num_pol
    for (int i = 0; i < num_pol; i++) {
      M_pos_bs2bs.push_back(Eigen::Matrix<double, 4, 4>::Identity());
    }
    return M_pos_bs2bs;
  }

  //////BSPLINE to MINVO Velocity/////////
  std::vector<Eigen::Matrix<double, 3, 3>> getMinvoVelConverters(int num_pol) {
    std::vector<Eigen::Matrix<double, 3, 3>> M_vel_bs2mv;  // will have as many elements as num_pol
    M_vel_bs2mv.push_back(M_vel_bs2mv_seg0);
    for (int i = 0; i < (num_pol - 2 - 1); i++) {
      M_vel_bs2mv.push_back(M_vel_bs2mv_rest);
    }
    M_vel_bs2mv.push_back(M_vel_bs2mv_seg_last);
    return M_vel_bs2mv;
  }

  //////BSPLINE to BEZIER Velocity/////////
  std::vector<Eigen::Matrix<double, 3, 3>> getBezierVelConverters(int num_pol) {
    std::vector<Eigen::Matrix<double, 3, 3>> M_vel_bs2be;  // will have as many elements as segments
    M_vel_bs2be.push_back(M_vel_bs2be_seg0);
    for (int i = 0; i < (num_pol - 2 - 1); i++) {
      M_vel_bs2be.push_back(M_vel_bs2be_rest);
    }
    M_vel_bs2be.push_back(M_vel_bs2be_seg_last);
    return M_vel_bs2be;
  }

  //////BSPLINE to BSPLINE Velocity/////////
  std::vector<Eigen::Matrix<double, 3, 3>> getBSplineVelConverters(int num_pol) {
    std::vector<Eigen::Matrix<double, 3, 3>> M_vel_bs2bs;  // will have as many elements as num_pol
    for (int i = 0; i < num_pol; i++) {
      M_vel_bs2bs.push_back(Eigen::Matrix<double, 3, 3>::Identity());
    }
    return M_vel_bs2bs;
  }
};

/** @brief Piecewise cubic polynomial trajectory, with per-interval coefficients in x, y, z.
 *
 *  Each interval i spans [times[i], times[i+1]) and is parameterized by
 *  u = t - times[i], so pol(t) = coeff * [u^3, u^2, u, 1]^T.
 */
struct PieceWisePol {
  // Interval 0: t\in[t0, t1)
  // Interval 1: t\in[t1, t2)
  // Interval 2: t\in[t2, t3)
  //...
  // Interval n-1: t\in[tn, tn+1)

  // n intervals in total

  // times has n+1 elements
  std::vector<double> times;  // [t0,t1,t2,...,tn+1]

  // coefficients has n elements
  // The coeffients are such that pol(t)=coeff_of_that_interval*[u^3 u^2 u 1]
  // with u=(t-t_min_that_interval)/(t_max_that_interval- t_min_that_interval)
  std::vector<Eigen::Matrix<double, 4, 1>> coeff_x;  // [a b c d]' of Int0 , [a b c d]' of Int1,...
  std::vector<Eigen::Matrix<double, 4, 1>> coeff_y;  // [a b c d]' of Int0 , [a b c d]' of Int1,...
  std::vector<Eigen::Matrix<double, 4, 1>> coeff_z;  // [a b c d]' of Int0 , [a b c d]' of Int1,...

  /** @brief Return the total duration of the trajectory (last time minus first time). */
  double getDuration() const { return times.back() - times.front(); }

  /** @brief Clear all time knots and polynomial coefficients. */
  void clear() {
    times.clear();
    coeff_x.clear();
    coeff_y.clear();
    coeff_z.clear();
  }

  /** @brief Return the end time of the trajectory (last time knot). */
  double getEndTime() const { return times.back(); }

  /** @brief Evaluate the position at time t by finding the containing interval.
   *  @param t Absolute time at which to evaluate.
   *  @return 3D position vector.
   */
  inline Eigen::Vector3d eval(double t) const {
    Eigen::Vector3d result;

    // return the last value of the polynomial in the last interval
    if (t >= times.back()) {
      Eigen::Matrix<double, 4, 1> tmp;
      double u = times.back() - times[times.size() - 2];
      tmp << u * u * u, u * u, u, 1.0;
      result.x() = coeff_x.back().transpose() * tmp;
      result.y() = coeff_y.back().transpose() * tmp;
      result.z() = coeff_z.back().transpose() * tmp;
      return result;
    }

    // return the first value of the polynomial in the first interval
    if (t < times.front()) {
      Eigen::Matrix<double, 4, 1> tmp;
      double u = 0;
      tmp << u * u * u, u * u, u, 1.0;
      result.x() = coeff_x.front().transpose() * tmp;
      result.y() = coeff_y.front().transpose() * tmp;
      result.z() = coeff_z.front().transpose() * tmp;
      return result;
    }

    // Find the interval where t is
    //(times - 1) is the number of intervals
    for (int i = 0; i < (times.size() - 1); i++) {
      if (times[i] <= t && t < times[i + 1]) {
        double u = t - times[i];

        // TODO: This is hand-coded for a third-degree polynomial
        Eigen::Matrix<double, 4, 1> tmp;
        tmp << u * u * u, u * u, u, 1.0;

        result.x() = coeff_x[i].transpose() * tmp;
        result.y() = coeff_y[i].transpose() * tmp;
        result.z() = coeff_z[i].transpose() * tmp;

        break;
      }
    }
    return result;
  }

  /** @brief Evaluate the velocity (first derivative) at time t.
   *  @param t Absolute time at which to evaluate.
   *  @return 3D velocity vector.
   */
  inline Eigen::Vector3d velocity(double t) const {
    Eigen::Vector3d vel;

    // Handle the case where t is after the last interval
    if (t >= times.back()) {
      double u = times.back() - times[times.size() - 2];
      vel.x() = 3 * coeff_x.back()(0) * u * u + 2 * coeff_x.back()(1) * u + coeff_x.back()(2);
      vel.y() = 3 * coeff_y.back()(0) * u * u + 2 * coeff_y.back()(1) * u + coeff_y.back()(2);
      vel.z() = 3 * coeff_z.back()(0) * u * u + 2 * coeff_z.back()(1) * u + coeff_z.back()(2);
      return vel;
    }

    // Handle the case where t is before the first interval
    if (t < times.front()) {
      vel.x() = coeff_x.front()(2);
      vel.y() = coeff_y.front()(2);
      vel.z() = coeff_z.front()(2);
      return vel;
    }

    // Find the interval where t lies and calculate velocity
    for (int i = 0; i < (times.size() - 1); i++) {
      if (times[i] <= t && t < times[i + 1]) {
        double u = t - times[i];
        vel.x() = 3 * coeff_x[i](0) * u * u + 2 * coeff_x[i](1) * u + coeff_x[i](2);
        vel.y() = 3 * coeff_y[i](0) * u * u + 2 * coeff_y[i](1) * u + coeff_y[i](2);
        vel.z() = 3 * coeff_z[i](0) * u * u + 2 * coeff_z[i](1) * u + coeff_z[i](2);
        break;
      }
    }

    return vel;
  }

  /// Evaluate acceleration (second derivative) at time t
  inline Eigen::Vector3d acceleration(double t) const {
    Eigen::Vector3d a{0, 0, 0};

    // 1) If t is after the last interval:
    if (t >= times.back()) {
      double u = times.back() - times[times.size() - 2];
      // coeff_* .back()(0) is the cubic a, .back()(1) is the quadratic b
      a.x() = 6.0 * coeff_x.back()(0) * u + 2.0 * coeff_x.back()(1);
      a.y() = 6.0 * coeff_y.back()(0) * u + 2.0 * coeff_y.back()(1);
      a.z() = 6.0 * coeff_z.back()(0) * u + 2.0 * coeff_z.back()(1);
      return a;
    }

    // 2) If t is before the first interval:
    if (t < times.front()) {
      a.x() = 2.0 * coeff_x.front()(1);
      a.y() = 2.0 * coeff_y.front()(1);
      a.z() = 2.0 * coeff_z.front()(1);
      return a;
    }

    // 3) Otherwise, find the correct interval i
    for (size_t i = 0; i + 1 < times.size(); ++i) {
      if (times[i] <= t && t < times[i + 1]) {
        double u = t - times[i];
        a.x() = 6.0 * coeff_x[i](0) * u + 2.0 * coeff_x[i](1);
        a.y() = 6.0 * coeff_y[i](0) * u + 2.0 * coeff_y[i](1);
        a.z() = 6.0 * coeff_z[i](0) * u + 2.0 * coeff_z[i](1);
        break;
      }
    }
    return a;
  }

  /** @brief Print all time knots and polynomial coefficients to stdout. */
  void print() const {
    std::cout << "coeff_x.size()= " << coeff_x.size() << std::endl;
    std::cout << "times.size()= " << times.size() << std::endl;
    std::cout << "Note that coeff_x.size() == times.size()-1" << std::endl;

    for (int i = 0; i < times.size(); i++) {
      printf("Time: %f\n", times[i]);
    }

    for (int i = 0; i < (times.size() - 1); i++) {
      std::cout << "From " << times[i] << " to " << times[i + 1] << std::endl;
      std::cout << "  Coeff_x= " << coeff_x[i].transpose() << std::endl;
      std::cout << "  Coeff_y= " << coeff_y[i].transpose() << std::endl;
      std::cout << "  Coeff_z= " << coeff_z[i].transpose() << std::endl;
    }
  }
};

/** @brief Dynamic trajectory representation supporting both piecewise polynomial and analytic (ExprTk) modes. */
struct DynTraj {
  /// Which representation to use
  enum class Mode { Piecewise, Analytic } mode{Mode::Analytic};

  // --- piecewise cubic branch ---
  PieceWisePol pwp;

  // --- analytic expression branch ---
  std::string traj_x, traj_y, traj_z;
  std::string traj_vx, traj_vy, traj_vz;  // optional (velocity expressions)
  double t_var{0.0};
  exprtk::symbol_table<double> symbol_table;
  exprtk::expression<double> expr_x, expr_y, expr_z;
  exprtk::expression<double> expr_vx, expr_vy, expr_vz;
  bool analytic_compiled{false};

  // shared metadata
  Eigen::Vector3d ekf_cov_p;
  Eigen::Vector3d ekf_cov_q;
  Eigen::Vector3d poly_cov;
  std::vector<Eigen::Matrix<double, 3, 4>> control_points;
  Eigen::Vector3d bbox;
  Eigen::Vector3d goal;
  Eigen::Vector3d current_pos{Eigen::Vector3d::Zero()};  // actual position at time of last msg
  bool is_agent = false;
  int id = -1;
  double time_received = 0.0;
  double tracking_utility = 0.0;
  double communication_delay = 0.0;

  DynTraj() = default;

  /// Switch to a piecewise cubic representation
  inline void setPiecewise(const PieceWisePol& poly) {
    mode = Mode::Piecewise;
    pwp = poly;
  }

  /** @brief Compile the analytic trajectory expressions (traj_x/y/z and optionally traj_vx/vy/vz) via ExprTk.
   *  @return True if all required expressions compiled successfully.
   */
  bool compileAnalytic() {
    symbol_table.clear();
    symbol_table.add_variable("t", t_var);
    symbol_table.add_constants();

    auto reg = [&](exprtk::expression<double>& e) { e.register_symbol_table(symbol_table); };
    reg(expr_x);
    reg(expr_y);
    reg(expr_z);
    reg(expr_vx);
    reg(expr_vy);
    reg(expr_vz);

    exprtk::parser<double> parser;

    auto compile_one = [&](const std::string& label, const std::string& src,
                           exprtk::expression<double>& expr) -> bool {
      if (src.empty()) {
        if (label == "traj_x" || label == "traj_y" || label == "traj_z") {
          std::cerr << "Missing required analytic expression " << label << "\n";
          return false;
        }
        // otherwise it was a velocity string → OK to skip
        return true;
      }

      if (!parser.compile(src, expr)) {
        std::ostringstream oss;
        oss << "ExprTk compile failure (" << label << "): '" << src << "' errors:";
        for (std::size_t i = 0; i < parser.error_count(); ++i) {
          auto e = parser.get_error(i);
          oss << " [pos " << e.token.position << " type " << exprtk::parser_error::to_str(e.mode)
              << " msg '" << e.diagnostic << "']";
        }
        std::cerr << oss.str() << std::endl;
        return false;
      }
      return true;
    };

    bool ok = true;
    ok &= compile_one("traj_x", traj_x, expr_x);
    ok &= compile_one("traj_y", traj_y, expr_y);
    ok &= compile_one("traj_z", traj_z, expr_z);
    ok &= compile_one("traj_vx", traj_vx, expr_vx);
    ok &= compile_one("traj_vy", traj_vy, expr_vy);
    ok &= compile_one("traj_vz", traj_vz, expr_vz);

    analytic_compiled = ok;
    return ok;
  }

  /// Evaluate position at time t
  inline Eigen::Vector3d eval(double t) const {
    switch (mode) {
      case Mode::Piecewise:
        return pwp.eval(t);
      case Mode::Analytic:
        return evalAnalyticPos(t);
    }
    return Eigen::Vector3d::Zero();
  }

  /** @brief Evaluate a 5th-degree polynomial p(tau) = c0 + c1*tau + ... + c5*tau^5.
   *  @param c Coefficient vector [c0, c1, ..., c5].
   *  @param tau Evaluation point.
   *  @return Polynomial value.
   */
  static inline double poly5_abs(const Eigen::Matrix<double, 6, 1>& c, double tau) {
    double v = c(5);
    v = v * tau + c(4);
    v = v * tau + c(3);
    v = v * tau + c(2);
    v = v * tau + c(1);
    v = v * tau + c(0);
    return v;
  }

  /** @brief Evaluate the first derivative of a 5th-degree polynomial.
   *  @param c Coefficient vector [c0, c1, ..., c5].
   *  @param tau Evaluation point.
   *  @return First derivative value.
   */
  static inline double dpoly5_abs(const Eigen::Matrix<double, 6, 1>& c, double tau) {
    double v = 5 * c(5);
    v = v * tau + 4 * c(4);
    v = v * tau + 3 * c(3);
    v = v * tau + 2 * c(2);
    v = v * tau + c(1);
    return v;
  }

  /** @brief Evaluate the second derivative of a 5th-degree polynomial.
   *  @param c Coefficient vector [c0, c1, ..., c5].
   *  @param tau Evaluation point.
   *  @return Second derivative value.
   */
  static inline double ddpoly5_abs(const Eigen::Matrix<double, 6, 1>& c, double tau) {
    double v = 20 * c(5);
    v = v * tau + 12 * c(4);
    v = v * tau + 6 * c(3);
    v = v * tau + 2 * c(2);
    return v;
  }

  /** @brief Evaluate position using the compiled analytic expressions.
   *  @param t Absolute time at which to evaluate.
   *  @return 3D position vector.
   */
  inline Eigen::Vector3d evalAnalyticPos(double t) const {
    if (!analytic_compiled) {
      // this should never happen if steps 1+2 are correct
      std::cerr << "[DynTraj] evalAnalyticPos called but analytic_compiled==false\n";
      return Eigen::Vector3d::Zero();
    }

    const_cast<DynTraj*>(this)->t_var = t;
    return {expr_x.value(), expr_y.value(), expr_z.value()};
  }

  /// Evaluate velocity at time t
  inline Eigen::Vector3d velocity(double t) const {
    switch (mode) {
      case Mode::Piecewise:
        return pwp.velocity(t);
      case Mode::Analytic:
        return velocityAnalytic(t);
    }
    return Eigen::Vector3d::Zero();
  }

  /** @brief Evaluate velocity using analytic expressions, falling back to numerical differentiation.
   *  @param t Absolute time at which to evaluate.
   *  @return 3D velocity vector.
   */
  inline Eigen::Vector3d velocityAnalytic(double t) const {
    if (!analytic_compiled) return Eigen::Vector3d::Zero();
    const_cast<DynTraj*>(this)->t_var = t;
    // If velocity expressions provided
    if (!traj_vx.empty() && !traj_vy.empty() && !traj_vz.empty())
      return {expr_vx.value(), expr_vy.value(), expr_vz.value()};

    // Fallback numerical diff (dt small):
    double dt = 1e-3;
    const_cast<DynTraj*>(this)->t_var = t;
    double x0 = expr_x.value(), y0 = expr_y.value(), z0 = expr_z.value();
    const_cast<DynTraj*>(this)->t_var = t + dt;
    double x1 = expr_x.value(), y1 = expr_y.value(), z1 = expr_z.value();
    return {(x1 - x0) / dt, (y1 - y0) / dt, (z1 - z0) / dt};
  }

  /// Evaluate acceleration at time t
  inline Eigen::Vector3d accel(double t) const {
    switch (mode) {
      case Mode::Piecewise:
        return pwp.acceleration(t);
      case Mode::Analytic:
        return accelAnalytic(t);
    }
    return Eigen::Vector3d::Zero();
  }

  /** @brief Evaluate acceleration using numerical second derivative of the analytic velocity.
   *  @param t Absolute time at which to evaluate.
   *  @return 3D acceleration vector.
   */
  inline Eigen::Vector3d accelAnalytic(double t) const {
    // If you add analytic second derivatives later, evaluate them here.
    // For now numeric second derivative:
    if (!analytic_compiled) return Eigen::Vector3d::Zero();
    double dt = 1e-3;
    Eigen::Vector3d v1 = velocity(t - dt);
    Eigen::Vector3d v2 = velocity(t + dt);
    return (v2 - v1) / (2 * dt);
  }

  /** @brief Return a human-readable string for the given trajectory mode.
   *  @param m The trajectory mode enum value.
   *  @return C-string name of the mode.
   */
  static const char* modeName(DynTraj::Mode m) {
    switch (m) {
      case DynTraj::Mode::Piecewise:
        return "Piecewise";
      case DynTraj::Mode::Analytic:
        return "Analytic";
      default:
        return "Unknown";
    }
  }

  /// Print debug info
  inline void print() const {
    std::cout << "DynTraj id=" << id << " mode=" << modeName(mode) << "\n";

    if (mode == Mode::Piecewise) {
      pwp.print();
    } else if (mode == Mode::Analytic) {
      std::cout << "  traj_x='" << traj_x << "'\n";
      std::cout << "  traj_y='" << traj_y << "'\n";
      std::cout << "  traj_z='" << traj_z << "'\n";
      if (!traj_vx.empty() || !traj_vy.empty() || !traj_vz.empty()) {
        std::cout << "  traj_vx='" << traj_vx << "'\n";
        std::cout << "  traj_vy='" << traj_vy << "'\n";
        std::cout << "  traj_vz='" << traj_vz << "'\n";
      }
      std::cout << "  analytic_compiled=" << analytic_compiled << "\n";
    }
  }
};

/** @brief Full robot state including position, velocity, acceleration, jerk, and yaw. */
struct RobotState {
  // time stamp
  double t = 0.0;

  // pos, vel, accel, jerk, yaw, dyaw
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel = Eigen::Vector3d::Zero();
  Eigen::Vector3d jerk = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double dyaw = 0.0;

  // flag for tracking
  bool use_tracking_yaw = false;

  /** @brief Set the timestamp. */
  void setTimeStamp(const double data) { t = data; }

  /** @brief Set position from x, y, z components. */
  void setPos(const double x, const double y, const double z) { pos << x, y, z; }
  /** @brief Set velocity from x, y, z components. */
  void setVel(const double x, const double y, const double z) { vel << x, y, z; }
  /** @brief Set acceleration from x, y, z components. */
  void setAccel(const double x, const double y, const double z) { accel << x, y, z; }

  /** @brief Set jerk from x, y, z components. */
  void setJerk(const double x, const double y, const double z) { jerk << x, y, z; }

  /** @brief Set position from an Eigen vector. */
  void setPos(const Eigen::Vector3d& data) { pos << data.x(), data.y(), data.z(); }

  /** @brief Set velocity from an Eigen vector. */
  void setVel(const Eigen::Vector3d& data) { vel << data.x(), data.y(), data.z(); }

  /** @brief Set acceleration from an Eigen vector. */
  void setAccel(const Eigen::Vector3d& data) { accel << data.x(), data.y(), data.z(); }

  /** @brief Set jerk from an Eigen vector. */
  void setJerk(const Eigen::Vector3d& data) { jerk << data.x(), data.y(), data.z(); }

  /** @brief Set all motion state vectors at once.
   *  @param pos Position vector.
   *  @param vel Velocity vector.
   *  @param accel Acceleration vector.
   *  @param jerk Jerk vector.
   */
  void setState(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel,
                const Eigen::Vector3d& accel, const Eigen::Vector3d& jerk) {
    this->pos = pos;
    this->vel = vel;
    this->accel = accel;
    this->jerk = jerk;
  }

  /** @brief Set the yaw angle. */
  void setYaw(const double data) { yaw = data; }

  /** @brief Set the yaw rate. */
  void setDYaw(const double data) { dyaw = data; }

  /** @brief Reset all state fields to zero. */
  void setZero() {
    pos = Eigen::Vector3d::Zero();
    vel = Eigen::Vector3d::Zero();
    accel = Eigen::Vector3d::Zero();
    jerk = Eigen::Vector3d::Zero();
    yaw = 0;
    dyaw = 0;
  }

  /** @brief Print the position vector to stdout. */
  void printPos() { std::cout << "Pos= " << pos.transpose() << std::endl; }

  /** @brief Print time, position, velocity, and acceleration to stdout. */
  void print() {
    std::cout << "Time= " << t << std::endl;
    std::cout << "Pos= " << pos.transpose() << std::endl;
    std::cout << "Vel= " << vel.transpose() << std::endl;
    std::cout << "Accel= " << accel.transpose() << std::endl;
  }

  /** @brief Print position, velocity, acceleration, and jerk on a single line. */
  void printHorizontal() {
    std::cout << "Pos, Vel, Accel, Jerk= " << pos.transpose() << " " << vel.transpose() << " "
              << accel.transpose() << " " << jerk.transpose() << std::endl;
  }
};