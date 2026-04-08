/* ----------------------------------------------------------------------------
 * Copyright 2026, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <Eigen/Dense>
#include <decomp_util/seed_decomp.h>               // Polyhedron / Hyperplane
#include <decomp_rviz_plugins/data_ros_utils.hpp>  // DecompROS::polyhedron_array_to_ros
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cctype>
#include <decomp_ros_msgs/msg/polyhedron_array.hpp>
#include <filesystem>
#include <optional>
#include <rclcpp/rclcpp.hpp>

namespace fs = std::filesystem;

// ------------------------ types ------------------------

using Vec3d = Eigen::Vector3d;
using Vec3f = Eigen::Matrix<double, 3, 1>;

struct TrajPoint {
  double t{0.0};
  double x{0.0}, y{0.0}, z{0.0};
};

struct TrajCsv {
  std::string planner_name;  // NOTE: we use this as the "planner variant key" (e.g., sando_N4)
  std::string case_file;
  std::string frame_id;
  std::vector<TrajPoint> pts;
};

struct CaseBundle {
  std::string case_file;  // basename: "sfc_g000.mysco2"
  fs::path mysco2_path;

  // from .mysco2 (preferred for start/goal labels)
  Vec3d mysco2_start{0, 0, 0};
  Vec3d mysco2_goal{0, 0, 0};
  bool have_mysco2_start_goal{false};

  decomp_ros_msgs::msg::PolyhedronArray poly_msg;
  visualization_msgs::msg::MarkerArray guide_path_ma;

  // planner_variant_key -> traj
  std::map<std::string, TrajCsv> planner_to_traj;
};

// ------------------------ small helpers ------------------------

static inline std::string trim(std::string s) {
  auto notSpace = [](int ch) { return !std::isspace(ch); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), notSpace));
  s.erase(std::find_if(s.rbegin(), s.rend(), notSpace).base(), s.end());
  return s;
}

static inline bool startsWith(const std::string& s, const std::string& pref) {
  return s.size() >= pref.size() && s.compare(0, pref.size(), pref) == 0;
}

static inline bool endsWith(const std::string& s, const std::string& suf) {
  return s.size() >= suf.size() && s.compare(s.size() - suf.size(), suf.size(), suf) == 0;
}

static inline std::string toLower(std::string s) {
  for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

static inline std::string zeroPadInt(int v, int width) {
  std::ostringstream oss;
  oss << std::setw(width) << std::setfill('0') << v;
  return oss.str();
}

static inline std::vector<std::string> splitCsvLine(const std::string& line) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string tok;
  while (std::getline(ss, tok, ',')) out.push_back(trim(tok));
  return out;
}

static inline bool isFinite3(double x, double y, double z) {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

static inline std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a) {
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

// --- Fixed palette (7 planners) ---------------------------------------------

static inline std_msgs::msg::ColorRGBA makeColorHex(std::string hex, float a) {
  // Accept "#RRGGBB" or "RRGGBB"
  if (!hex.empty() && hex[0] == '#') hex = hex.substr(1);

  if (hex.size() != 6) return makeColor(1.0f, 1.0f, 1.0f, a);  // fallback: white

  auto byte01 = [&](int pos) -> float {
    unsigned int v = 255;
    std::stringstream ss;
    ss << std::hex << hex.substr(pos, 2);
    ss >> v;
    return static_cast<float>(v) / 255.0f;
  };

  return makeColor(byte01(0), byte01(2), byte01(4), a);
}

static const std::vector<std::string> kPlannerPaletteHex = {
    "#e51010",  // red
    "#ff6600",  // orange
    "#fc4589",  // pink
    "#fffc5d",  // yellow
    "#4adeaf",  // mint
    "#00b2ff",  // cyan
    "#420e87",  // purple
    "#00ff00"   // green
};

// ------------------------ overlap-visibility controls ------------------------
// Improves visibility when trajectories overlap by:
//  (1) Drawing a translucent "halo" behind each trajectory line
//  (2) Optionally stacking lines in Z by a tiny offset per planner slot
//  (3) Optionally highlighting a specific planner
struct TrajVizConfig {
  // Base visibility
  float alpha_line_base = 0.10f;
  float alpha_pts_base = 0.25f;

  // Halo/outline behind each line
  bool enable_halo = true;
  float halo_alpha = 0.10f;
  double halo_scale = 2.5;                            // multiplier on line width
  float halo_r = 0.0f, halo_g = 0.0f, halo_b = 0.0f;  // black halo by default

  // Optional Z stacking
  bool enable_z_offset = false;
  double z_step = 0.02;  // meters per planner slot

  // Optional highlight
  std::string highlight_planner = "";  // e.g. "super" or "sando_N4"
  bool highlight_only = false;         // if true, only draw highlight_planner
  float alpha_line_highlight = 1.0f;
  float alpha_pts_highlight = 0.8f;
  double highlight_scale = 3.0;  // multiplier on line width for highlight
};

static TrajVizConfig g_traj_viz;

// Deterministic hash -> [0,1)
static inline double hash01(const std::string& s) {
  std::uint64_t h = 1469598103934665603ull;
  for (unsigned char c : s) {
    h ^= c;
    h *= 1099511628211ull;
  }
  return (h % 1000000) / 1000000.0;
}

// HSV in [0,1] -> RGB [0,1]
static inline void hsv2rgb(double h, double s, double v, double& r, double& g, double& b) {
  if (s <= 1e-9) {
    r = g = b = v;
    return;
  }
  h = std::fmod(h, 1.0);
  if (h < 0) h += 1.0;

  const double i = std::floor(h * 6.0);
  const double f = h * 6.0 - i;
  const double p = v * (1.0 - s);
  const double q = v * (1.0 - s * f);
  const double t = v * (1.0 - s * (1.0 - f));

  switch (static_cast<int>(i) % 6) {
    case 0:
      r = v;
      g = t;
      b = p;
      break;
    case 1:
      r = q;
      g = v;
      b = p;
      break;
    case 2:
      r = p;
      g = v;
      b = t;
      break;
    case 3:
      r = p;
      g = q;
      b = v;
      break;
    case 4:
      r = t;
      g = p;
      b = v;
      break;
    case 5:
      r = v;
      g = p;
      b = q;
      break;
  }
}

static inline geometry_msgs::msg::Point toPoint(double x, double y, double z) {
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

static inline visualization_msgs::msg::Marker deleteAllMarker(
    const std::string& frame_id, const rclcpp::Time& stamp) {
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = stamp;
  m.action = visualization_msgs::msg::Marker::DELETEALL;
  return m;
}

// ------------------------ .mysco2 reader (corridor + guide path) ------------------------

static uint32_t readU32(std::ifstream& ifs) {
  uint32_t v;
  ifs.read(reinterpret_cast<char*>(&v), sizeof(v));
  if (!ifs) throw std::runtime_error("Corrupt .mysco2 (u32).");
  return v;
}
static double readD(std::ifstream& ifs) {
  double v;
  ifs.read(reinterpret_cast<char*>(&v), sizeof(v));
  if (!ifs) throw std::runtime_error("Corrupt .mysco2 (double).");
  return v;
}

// Build Polyhedron<3> from A x <= b, ensuring seed is inside by flipping violating planes.
static Polyhedron<3> polyFromHalfspacesSeeded(
    Eigen::MatrixXd A, Eigen::VectorXd b, const Vec3f& seed, double eps) {
  for (int r = 0; r < A.rows(); ++r) {
    const double v = A.row(r).dot(seed) - b(r);
    if (v > eps) {
      A.row(r) *= -1.0;
      b(r) *= -1.0;
    }
  }

  Polyhedron<3> poly;
  for (int r = 0; r < A.rows(); ++r) {
    Vec3f a = A.row(r).transpose();
    const double norm = a.norm();
    if (norm < 1e-12) continue;

    const Vec3f n = a / norm;
    const double d = b(r) / norm;
    const Vec3f p0 = n * d;  // point on plane

    poly.add(Hyperplane<3>(p0, n));
  }
  return poly;
}

static void loadMysco2CorridorAndPath(
    const fs::path& file,
    Vec3d& start,
    Vec3d& goal,
    std::vector<Vec3f>& path_pts,
    vec_E<Polyhedron<3>>& polys,
    double poly_seed_eps) {
  std::ifstream ifs(file, std::ios::binary);
  if (!ifs) throw std::runtime_error("Failed to open: " + file.string());

  char magic[8];
  ifs.read(magic, 8);
  if (!ifs) throw std::runtime_error("Corrupt .mysco2 (magic): " + file.string());
  const std::string m(magic, magic + 8);
  if (m.rfind("MYSCO2", 0) != 0) throw std::runtime_error("Bad magic in: " + file.string());

  const uint32_t version = readU32(ifs);
  if (version != 1) throw std::runtime_error("Unsupported .mysco2 version: " + file.string());

  start.x() = readD(ifs);
  start.y() = readD(ifs);
  start.z() = readD(ifs);
  goal.x() = readD(ifs);
  goal.y() = readD(ifs);
  goal.z() = readD(ifs);

  const uint32_t num_path_pts = readU32(ifs);
  path_pts.clear();
  path_pts.reserve(num_path_pts);
  for (uint32_t i = 0; i < num_path_pts; ++i) {
    Vec3f p;
    p.x() = readD(ifs);
    p.y() = readD(ifs);
    p.z() = readD(ifs);
    path_pts.push_back(p);
  }

  const uint32_t num_seg = readU32(ifs);
  // seg_end_times not needed here; consume it
  for (uint32_t i = 0; i < num_seg; ++i) (void)readD(ifs);

  if (path_pts.size() < 2 || (path_pts.size() - 1) != num_seg)
    throw std::runtime_error("File inconsistent: path.size()-1 != num_seg in " + file.string());

  polys.clear();
  polys.resize(num_seg);

  for (uint32_t si = 0; si < num_seg; ++si) {
    const uint32_t mplanes = readU32(ifs);

    Eigen::MatrixXd A(mplanes, 3);
    Eigen::VectorXd b(mplanes);

    for (uint32_t r = 0; r < mplanes; ++r)
      for (int c = 0; c < 3; ++c) A(r, c) = readD(ifs);

    for (uint32_t r = 0; r < mplanes; ++r) b(r) = readD(ifs);

    const Vec3f seed = 0.5 * (path_pts[si] + path_pts[si + 1]);
    polys[si] = polyFromHalfspacesSeeded(A, b, seed, poly_seed_eps);
  }
}

static visualization_msgs::msg::MarkerArray makeGuidePathMarkers(
    const std::vector<Vec3f>& path,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    const std_msgs::msg::ColorRGBA& color,
    double line_width,
    double point_diam) {
  visualization_msgs::msg::MarkerArray arr;
  arr.markers.push_back(deleteAllMarker(frame_id, stamp));

  visualization_msgs::msg::Marker line;
  line.header.frame_id = frame_id;
  line.header.stamp = stamp;
  line.ns = "guide_path";
  line.id = 1;
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.pose.orientation.w = 1.0;
  line.scale.x = line_width;
  line.color = color;

  visualization_msgs::msg::Marker pts = line;
  pts.ns = "guide_path_pts";
  pts.id = 2;
  pts.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  pts.scale.x = point_diam;
  pts.scale.y = point_diam;
  pts.scale.z = point_diam;

  for (const auto& p : path) {
    line.points.push_back(toPoint(p.x(), p.y(), p.z()));
    pts.points.push_back(toPoint(p.x(), p.y(), p.z()));
  }

  arr.markers.push_back(line);
  arr.markers.push_back(pts);
  return arr;
}

// Screenshot-mode helper: append guide path markers WITHOUT DELETEALL, with unique ns/id.
static void appendGuidePathMarkersNoDelete(
    visualization_msgs::msg::MarkerArray& arr,
    int& id,
    const std::vector<Vec3f>& path,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    const std_msgs::msg::ColorRGBA& color,
    double line_width,
    double point_diam,
    const std::string& ns_prefix) {
  visualization_msgs::msg::Marker line;
  line.header.frame_id = frame_id;
  line.header.stamp = stamp;
  line.ns = ns_prefix + "/guide_path";
  line.id = id++;
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.pose.orientation.w = 1.0;
  line.scale.x = line_width;
  line.color = color;

  visualization_msgs::msg::Marker pts = line;
  pts.ns = ns_prefix + "/guide_path_pts";
  pts.id = id++;
  pts.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  pts.scale.x = point_diam;
  pts.scale.y = point_diam;
  pts.scale.z = point_diam;

  line.points.reserve(path.size());
  pts.points.reserve(path.size());
  for (const auto& p : path) {
    const auto gp = toPoint(p.x(), p.y(), p.z());
    line.points.push_back(gp);
    pts.points.push_back(gp);
  }

  arr.markers.push_back(line);
  arr.markers.push_back(pts);
}

// ------------------------ trajectory CSV scan + parse ------------------------

static bool parseTrajCsv(const fs::path& csv_path, TrajCsv& out) {
  std::ifstream ifs(csv_path);
  if (!ifs) return false;

  out = TrajCsv{};
  std::string line;
  bool header_seen = false;
  std::unordered_map<std::string, int> col;

  while (std::getline(ifs, line)) {
    line = trim(line);
    if (line.empty()) continue;

    if (startsWith(line, "#")) {
      const auto pos = line.find(':');
      if (pos != std::string::npos) {
        const std::string key = trim(line.substr(1, pos - 1));
        const std::string val = trim(line.substr(pos + 1));
        if (key == "planner_name")
          out.planner_name = val;
        else if (key == "case_file" || key == "source_file" || key == "file")
          out.case_file = val;
        else if (key == "frame_id")
          out.frame_id = val;
      }
      continue;
    }

    if (!header_seen) {
      header_seen = true;
      const auto toks = splitCsvLine(line);
      for (int i = 0; i < (int)toks.size(); ++i) col[toks[i]] = i;

      // Support both "x,y,z" and "px,py,pz" column names
      if (col.find("x") == col.end() && col.find("px") != col.end()) col["x"] = col["px"];
      if (col.find("y") == col.end() && col.find("py") != col.end()) col["y"] = col["py"];
      if (col.find("z") == col.end() && col.find("pz") != col.end()) col["z"] = col["pz"];

      if (col.find("x") == col.end() || col.find("y") == col.end() || col.find("z") == col.end())
        return false;

      continue;
    }

    const auto toks = splitCsvLine(line);
    auto getD = [&](const std::string& name, double def) -> double {
      auto it = col.find(name);
      if (it == col.end()) return def;
      const int i = it->second;
      if (i < 0 || i >= (int)toks.size()) return def;
      try {
        return std::stod(toks[(size_t)i]);
      } catch (...) {
        return def;
      }
    };

    TrajPoint p;
    p.t = getD("t", out.pts.empty() ? 0.0 : out.pts.back().t);
    p.x = getD("x", std::numeric_limits<double>::quiet_NaN());
    p.y = getD("y", std::numeric_limits<double>::quiet_NaN());
    p.z = getD("z", std::numeric_limits<double>::quiet_NaN());

    if (isFinite3(p.x, p.y, p.z)) out.pts.push_back(p);
  }

  const std::string fname = csv_path.filename().string();
  const std::string parent_dir = csv_path.parent_path().filename().string();

  auto stripCsvExt = [](std::string s) -> std::string {
    if (endsWith(s, ".csv")) s = s.substr(0, s.size() - 4);
    return s;
  };

  auto inferVariantFromFilename = [&]() -> std::string {
    std::string s = fname;
    if (startsWith(s, "traj_")) s = s.substr(5);
    const auto pos2 = s.find("__");
    if (pos2 == std::string::npos) return std::string();
    return stripCsvExt(s.substr(0, pos2));
  };

  auto inferCaseFromFilename = [&]() -> std::string {
    const auto pos = fname.find("__");
    if (pos == std::string::npos) return std::string();
    return stripCsvExt(fname.substr(pos + 2));
  };

  auto inferVariantFromDir = [&]() -> std::string {
    const auto pos = parent_dir.find("_N");
    if (pos == std::string::npos) return std::string();

    bool has_digit = false;
    for (size_t i = pos + 2; i < parent_dir.size(); ++i) {
      if (std::isdigit(static_cast<unsigned char>(parent_dir[i]))) {
        has_digit = true;
        break;
      }
    }
    return has_digit ? parent_dir : std::string();
  };

  if (out.case_file.empty()) {
    out.case_file = inferCaseFromFilename();

    // Fallback: if filename is like "sfc_g000.csv" (no traj_ prefix / no __ separator),
    // treat the bare stem as the case file (e.g. "sfc_g000").
    if (out.case_file.empty() && startsWith(fname, "sfc_")) {
      out.case_file = stripCsvExt(fname);
    }
  } else {
    out.case_file = fs::path(out.case_file).filename().string();
  }

  const std::string variant_fname = inferVariantFromFilename();
  const std::string variant_dir = inferVariantFromDir();
  const std::string variant = !variant_fname.empty() ? variant_fname : variant_dir;

  if (!variant.empty()) {
    out.planner_name = variant;
  } else if (out.planner_name.empty()) {
    std::string s = fname;
    if (startsWith(s, "traj_")) s = s.substr(5);
    const auto pos2 = s.find("__");
    if (pos2 != std::string::npos)
      out.planner_name = stripCsvExt(s.substr(0, pos2));
    else {
      // Fallback: use parent directory name as planner variant
      // e.g. parent_dir = "traj_l2" -> planner_name = "super_l2"
      if (startsWith(parent_dir, "traj_"))
        out.planner_name = "super_" + parent_dir.substr(5);
      else
        out.planner_name = "unknown";
    }
  }

  if (out.frame_id.empty()) out.frame_id = "map";

  return !out.pts.empty() && !out.case_file.empty();
}

// ------------------------ marker building for trajectories ------------------------

static inline std::string prettyPlannerName(const std::string& planner_key) {
  // SANDO2
  if (planner_key == "sando_N4" || planner_key == "sando_N4") return "SANDO2(N=4)";
  if (planner_key == "sando_N5" || planner_key == "sando_N5") return "SANDO2(N=5)";
  if (planner_key == "sando_N6" || planner_key == "sando_N6") return "SANDO2(N=6)";

  // Orig. FASTER
  if (planner_key == "original_faster_N4") return "Orig.FASTER(N=4)";
  if (planner_key == "original_faster_N5") return "Orig.FASTER(N=5)";
  if (planner_key == "original_faster_N6") return "Orig.FASTER(N=6)";

  // SUPER
  if (planner_key == "super_l2") return "SUPER(L2)";
  if (planner_key == "super_linf") return "SUPER";

  // CP FASTER
  if (planner_key == "safe_faster_N4") return "CP-FASTER(N=4)";
  if (planner_key == "safe_faster_N5") return "CP-FASTER(N=5)";
  if (planner_key == "safe_faster_N6") return "CP-FASTER(N=6)";

  return planner_key;
}

static void appendStartOnce(
    visualization_msgs::msg::MarkerArray& arr,
    int& id,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    const geometry_msgs::msg::Point& start_pt,
    double point_diam,
    double label_height,
    double label_z_offset) {
  const double start_goal_point_scale = 3.0;
  const double start_goal_text_scale = 2.5;
  const double start_text_dy = 0.6;
  const double start_text_dz = 0.3;

  auto init = [&](visualization_msgs::msg::Marker& m, const std::string& ns) {
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.ns = ns;
    m.id = id++;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.lifetime = rclcpp::Duration(0, 0);
  };

  // Start sphere
  {
    visualization_msgs::msg::Marker mk;
    init(mk, "global_start");
    mk.type = visualization_msgs::msg::Marker::SPHERE;
    mk.pose.position = start_pt;
    mk.scale.x = start_goal_point_scale * point_diam;
    mk.scale.y = start_goal_point_scale * point_diam;
    mk.scale.z = start_goal_point_scale * point_diam;
    mk.color = makeColor(0.0f, 0.0f, 0.0f, 0.85f);
    arr.markers.push_back(mk);
  }
  // Start text
  {
    visualization_msgs::msg::Marker mk;
    init(mk, "global_start_text");
    mk.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    mk.pose.position.x = start_pt.x;
    mk.pose.position.y = start_pt.y + start_text_dy;
    mk.pose.position.z = start_pt.z + start_text_dz + label_z_offset * 0.0;  // keep same convention
    mk.scale.z = start_goal_text_scale * label_height;
    mk.color = makeColor(0.0f, 0.0f, 0.0f, 1.0f);
    mk.text = "start";
    arr.markers.push_back(mk);
  }
}

static void appendPlannerLegendOnce(
    visualization_msgs::msg::MarkerArray& arr,
    int& id,
    const std::vector<std::string>& planners_sorted,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    const geometry_msgs::msg::Point& anchor,
    double label_height,
    double label_z_offset) {
  if (planners_sorted.empty()) return;

  const float alpha_text = 1.0f;
  const double text_scale = 2.0;
  const double label_spacing_m = 1.0;

  const int N = (int)planners_sorted.size();
  const double mid = (N - 1) / 2.0;

  auto init = [&](visualization_msgs::msg::Marker& m, const std::string& ns) {
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.ns = ns;
    m.id = id++;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.lifetime = rclcpp::Duration(0, 0);
  };

  for (int i = 0; i < N; ++i) {
    const std::string& planner = planners_sorted[i];

    const double t = (N <= 1) ? 0.0 : (double)i / (double)(N - 1);
    const double h = 0.85 * t;

    double rr, gg, bb;
    hsv2rgb(h, 1.0, 1.0, rr, gg, bb);

    visualization_msgs::msg::Marker text;
    init(text, "planner_legend");
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

    text.pose.position.x = anchor.x;
    text.pose.position.y = anchor.y + (mid - (double)i) * label_spacing_m;
    text.pose.position.z = anchor.z + label_z_offset;

    text.scale.z = text_scale * label_height;

    {
      const std::string& hex = kPlannerPaletteHex[(size_t)i % kPlannerPaletteHex.size()];
      text.color = makeColorHex(hex, alpha_text);
    }
    text.text = prettyPlannerName(planner);

    arr.markers.push_back(text);
  }
}

// Core builder that APPENDS markers (no DELETEALL). Used by both loop and screenshot modes.
static void appendTrajOverlayMarkers(
    visualization_msgs::msg::MarkerArray& arr,
    int& id,
    const std::map<std::string, TrajCsv>& planner_to_traj,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    bool show_points,
    bool show_labels,
    double line_width,
    double point_diam,
    double label_height,
    double label_z_offset,
    const std::string& ns_prefix,
    const std::string& goal_text_override,
    const std::optional<geometry_msgs::msg::Point>& start_opt,
    const std::optional<geometry_msgs::msg::Point>& goal_opt,
    bool publish_start,                // loop: true, screenshot: false
    bool publish_goal_and_case_label,  // loop: true (goal), screenshot: true (case label at goal)
    bool publish_planner_labels,       // loop: true, screenshot: false
    const std::unordered_map<std::string, int>* global_planner_index,  // screenshot: non-null
    int global_planner_count,                                          // screenshot: >0
    double goal_label_dx,  // screenshot: +1.0, loop: 0.0
    double goal_label_dy)  // screenshot: 0.0, loop: +0.6
{
  // Visual tuning knobs (configurable via params)
  const float alpha_line_base = g_traj_viz.alpha_line_base;
  const float alpha_pts_base = g_traj_viz.alpha_pts_base;
  const float alpha_text = 1.0f;

  const double line_scale = 1.5;
  const double text_scale = 2.0;

  const double start_goal_text_scale = 2.5;
  const double start_goal_point_scale = 3.0;
  const double start_goal_text_dy = 0.6;
  const double start_goal_text_dz = 0.3;

  // Choose which planner index map to use
  std::unordered_map<std::string, int> local_index;
  const std::unordered_map<std::string, int>* index_map = nullptr;
  int num_planners = 0;

  if (global_planner_index && !global_planner_index->empty() && global_planner_count > 0) {
    index_map = global_planner_index;
    num_planners = global_planner_count;
  } else {
    num_planners = static_cast<int>(planner_to_traj.size());
    local_index.reserve(static_cast<size_t>(num_planners));
    int idx = 0;
    for (const auto& kv : planner_to_traj) local_index[kv.first] = idx++;
    index_map = &local_index;
  }

  const double label_spacing_m = 1.0;
  const double mid = (num_planners - 1) / 2.0;

  auto initMarkerCommon = [&](visualization_msgs::msg::Marker& m, const std::string& ns) {
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.ns = ns;
    m.id = id++;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.lifetime = rclcpp::Duration(0, 0);
  };

  // Determine start/goal
  bool have_start_goal = false;
  geometry_msgs::msg::Point start_pt, goal_pt;

  if (start_opt && goal_opt) {
    start_pt = *start_opt;
    goal_pt = *goal_opt;
    have_start_goal = true;
  } else {
    for (const auto& kv : planner_to_traj) {
      if (!kv.second.pts.empty()) {
        start_pt =
            toPoint(kv.second.pts.front().x, kv.second.pts.front().y, kv.second.pts.front().z);
        goal_pt = toPoint(kv.second.pts.back().x, kv.second.pts.back().y, kv.second.pts.back().z);
        have_start_goal = true;
        break;
      }
    }
  }

  // Start marker/text (only if requested)
  if (publish_start && have_start_goal) {
    const std::string base_ns = ns_prefix.empty() ? "start_goal" : (ns_prefix + "/start_goal");
    const std::string base_ns_text =
        ns_prefix.empty() ? "start_goal_text" : (ns_prefix + "/start_goal_text");

    const auto col_sg_pt = makeColor(0.0f, 0.0f, 0.0f, 0.85f);
    const auto col_sg_text = makeColor(0.0f, 0.0f, 0.0f, 1.0f);

    // Start sphere
    {
      visualization_msgs::msg::Marker mk;
      initMarkerCommon(mk, base_ns);
      mk.type = visualization_msgs::msg::Marker::SPHERE;
      mk.scale.x = start_goal_point_scale * point_diam;
      mk.scale.y = start_goal_point_scale * point_diam;
      mk.scale.z = start_goal_point_scale * point_diam;
      mk.pose.position = start_pt;
      mk.color = col_sg_pt;
      arr.markers.push_back(mk);
    }
    // Start text
    {
      visualization_msgs::msg::Marker mk;
      initMarkerCommon(mk, base_ns_text);
      mk.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      mk.pose.position.x = start_pt.x;
      mk.pose.position.y = start_pt.y + start_goal_text_dy;
      mk.pose.position.z = start_pt.z + start_goal_text_dz;
      mk.scale.z = start_goal_text_scale * label_height;
      mk.color = col_sg_text;
      mk.text = "start";
      arr.markers.push_back(mk);
    }
  }

  // Goal / Case label (only if requested)
  if (publish_goal_and_case_label && have_start_goal) {
    const std::string base_ns = ns_prefix.empty() ? "goal" : (ns_prefix + "/goal");
    const std::string base_ns_text = ns_prefix.empty() ? "goal_text" : (ns_prefix + "/goal_text");

    const auto col_pt = makeColor(0.0f, 0.0f, 0.0f, 0.85f);
    const auto col_text = makeColor(0.0f, 0.0f, 0.0f, 1.0f);

    // Goal sphere
    {
      visualization_msgs::msg::Marker mk;
      initMarkerCommon(mk, base_ns);
      mk.type = visualization_msgs::msg::Marker::SPHERE;
      mk.scale.x = start_goal_point_scale * point_diam;
      mk.scale.y = start_goal_point_scale * point_diam;
      mk.scale.z = start_goal_point_scale * point_diam;
      mk.pose.position = goal_pt;
      mk.color = col_pt;
      arr.markers.push_back(mk);
    }
    // Goal/case text (requirements: screenshot => +1m in x, no y increment, no space in "caseXX")
    {
      visualization_msgs::msg::Marker mk;
      initMarkerCommon(mk, base_ns_text);
      mk.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      mk.pose.position.x = goal_pt.x + goal_label_dx;
      mk.pose.position.y = goal_pt.y + goal_label_dy;
      mk.pose.position.z = goal_pt.z + start_goal_text_dz;
      mk.scale.z = start_goal_text_scale * label_height;
      mk.color = col_text;
      mk.text = goal_text_override;
      arr.markers.push_back(mk);
    }
  }

  // Trajectory overlays per planner
  for (const auto& kv : planner_to_traj) {
    const std::string& planner = kv.first;
    const TrajCsv& tr = kv.second;

    auto it_idx = index_map->find(planner);
    const int i = (it_idx != index_map->end())
                      ? it_idx->second
                      : (int)std::floor(hash01(planner) * std::max(1, num_planners));

    // Keep i/t for label y-positioning only (do NOT use it for color in loop mode).
    const double t = (num_planners <= 1) ? 0.0 : (double)i / (double)(num_planners - 1);

    // Color hue: stable per planner in loop mode; keep index-based color in screenshot mode.
    double h_color = 0.0;
    if (global_planner_index && !global_planner_index->empty() && global_planner_count > 0) {
      // screenshot mode (global stable palette by global index)
      h_color = 0.85 * t;
    } else {
      // loop mode (stable per planner across cases even if planners are missing)
      h_color = 0.85 * hash01(planner);
    }

    double rr, gg, bb;
    hsv2rgb(h_color, 1.0, 1.0, rr, gg, bb);

    // Highlight logic
    const bool has_highlight = !g_traj_viz.highlight_planner.empty();
    const bool is_highlight = has_highlight && (planner == g_traj_viz.highlight_planner);
    if (g_traj_viz.highlight_only && has_highlight && !is_highlight) continue;

    const float a_line = is_highlight ? g_traj_viz.alpha_line_highlight : alpha_line_base;
    const float a_pts = is_highlight ? g_traj_viz.alpha_pts_highlight : alpha_pts_base;
    const double width_scale =
        is_highlight ? (line_scale * g_traj_viz.highlight_scale) : line_scale;

    // Optional Z stacking offset (helps when overlapping)
    const double z_off = g_traj_viz.enable_z_offset ? (((double)i - mid) * g_traj_viz.z_step) : 0.0;

    // Fixed palette color by planner index (stable across cases when global_planner_index is used)
    const int idx = (i < 0) ? 0 : i;
    const std::string& hex = kPlannerPaletteHex[(size_t)idx % kPlannerPaletteHex.size()];

    const auto col_line = makeColorHex(hex, a_line);
    const auto col_pts = makeColorHex(hex, a_pts);
    const auto col_text = makeColorHex(hex, alpha_text);

    const std::string ns_traj = (ns_prefix.empty() ? "" : (ns_prefix + "/")) + "traj/" + planner;
    const std::string ns_pts = (ns_prefix.empty() ? "" : (ns_prefix + "/")) + "traj_pts/" + planner;
    const std::string ns_label = (ns_prefix.empty() ? "" : (ns_prefix + "/")) + "traj_label";

    // Build points once (with optional z offset)
    std::vector<geometry_msgs::msg::Point> pts_vec;
    pts_vec.reserve(tr.pts.size());
    for (const auto& p : tr.pts) pts_vec.push_back(toPoint(p.x, p.y, p.z + z_off));

    // Halo behind the main line (improves separability under overlap)
    if (g_traj_viz.enable_halo) {
      visualization_msgs::msg::Marker halo;
      initMarkerCommon(halo, ns_traj + "/halo");
      halo.type = visualization_msgs::msg::Marker::LINE_STRIP;
      halo.scale.x = g_traj_viz.halo_scale * width_scale * line_width;
      halo.color =
          makeColor(g_traj_viz.halo_r, g_traj_viz.halo_g, g_traj_viz.halo_b, g_traj_viz.halo_alpha);
      halo.points = pts_vec;
      arr.markers.push_back(halo);
    }

    // Main colored line
    visualization_msgs::msg::Marker line;
    initMarkerCommon(line, ns_traj);
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.scale.x = width_scale * line_width;
    line.color = col_line;
    line.points = pts_vec;
    arr.markers.push_back(line);

    if (show_points) {
      visualization_msgs::msg::Marker pts;
      initMarkerCommon(pts, ns_pts);
      pts.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      pts.scale.x = point_diam;
      pts.scale.y = point_diam;
      pts.scale.z = point_diam;
      pts.color = col_pts;
      pts.points = pts_vec;
      arr.markers.push_back(pts);
    }

    // Planner labels (screenshot mode disables this; loop mode enables)
    if (publish_planner_labels && show_labels && !tr.pts.empty()) {
      const double y_label = (mid - (double)i) * label_spacing_m;

      visualization_msgs::msg::Marker text;
      initMarkerCommon(text, ns_label);
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

      text.pose.position.x = tr.pts.front().x - 2.5;
      text.pose.position.y = y_label;
      text.pose.position.z = tr.pts.front().z + z_off + label_z_offset;

      text.scale.z = text_scale * label_height;
      text.color = col_text;
      text.text = prettyPlannerName(planner);

      arr.markers.push_back(text);
    }
  }
}

// Loop-mode wrapper: returns a standalone MarkerArray (with DELETEALL).
// Loop-mode wrapper: returns a standalone MarkerArray (with DELETEALL).
static visualization_msgs::msg::MarkerArray makeTrajOverlayMarkers(
    const std::map<std::string, TrajCsv>& planner_to_traj,
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    bool show_points,
    bool show_labels,
    double line_width,
    double point_diam,
    double label_height,
    double label_z_offset,
    const std::string& goal_text_override,
    const std::optional<geometry_msgs::msg::Point>& start_opt,
    const std::optional<geometry_msgs::msg::Point>& goal_opt,
    const std::unordered_map<std::string, int>* global_planner_index,
    int global_planner_count) {
  visualization_msgs::msg::MarkerArray arr;
  arr.markers.push_back(deleteAllMarker(frame_id, stamp));
  int id = 1;

  // IMPORTANT: forward the global palette into appendTrajOverlayMarkers()
  appendTrajOverlayMarkers(
      arr, id, planner_to_traj, frame_id, stamp, show_points, show_labels, line_width, point_diam,
      label_height, label_z_offset,
      /*ns_prefix=*/"", goal_text_override, start_opt, goal_opt,
      /*publish_start=*/true,
      /*publish_goal_and_case_label=*/true,
      /*publish_planner_labels=*/true,
      /*global_planner_index=*/global_planner_index,
      /*global_planner_count=*/global_planner_count,
      /*goal_label_dx=*/0.0,
      /*goal_label_dy=*/0.6);

  return arr;
}

// ------------------------ node ------------------------

class VisualizeLocalTrajsNode final : public rclcpp::Node {
 public:
  VisualizeLocalTrajsNode() : Node("visualize_local_trajs") {
    // Inputs
    sfc_dir_ = declare_parameter<std::string>("sfc_dir", "");
    file_ext_ = declare_parameter<std::string>("file_ext", ".mysco2");
    traj_dump_root_dir_ = declare_parameter<std::string>("traj_dump_root_dir", "");

    // New: multiple roots
    traj_dump_root_dirs_ = declare_parameter<std::vector<std::string>>(
        "traj_dump_root_dirs", std::vector<std::string>{traj_dump_root_dir_});

    // Topics
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    poly_topic_ = declare_parameter<std::string>("poly_topic", "/NX01/poly_safe");
    traj_topic_ = declare_parameter<std::string>("traj_overlay_topic", "/local_traj_overlay");
    guide_path_topic_ = declare_parameter<std::string>("guide_path_topic", "/hgp_path_marker");

    // Playback
    playback_period_sec_ = declare_parameter<double>("playback_period_sec", 0.5);
    visualize_ = declare_parameter<bool>("visualize", true);

    // Mode
    mode_ = toLower(
        declare_parameter<std::string>("mode", "loop"));  // "loop", "screenshot", or "single"
    screenshot_stride_ = declare_parameter<int>("screenshot_stride", 10);
    screenshot_max_index_ = declare_parameter<int>("screenshot_max_index", 100);
    single_case_index_ = declare_parameter<int>("single_case_index", 0);

    // What to show
    show_guide_path_ = declare_parameter<bool>("show_guide_path", true);
    show_points_ = declare_parameter<bool>("show_points", true);
    show_labels_ = declare_parameter<bool>("show_labels", true);

    // Planner whitelist: if non-empty, only load these planner keys
    planner_whitelist_vec_ = declare_parameter<std::vector<std::string>>(
        "planner_whitelist", std::vector<std::string>{});
    planner_whitelist_.insert(planner_whitelist_vec_.begin(), planner_whitelist_vec_.end());

    // Styling
    traj_line_width_ = declare_parameter<double>("traj_line_width", 0.06);
    traj_point_diam_ = declare_parameter<double>("traj_point_diam", 0.10);
    label_height_ = declare_parameter<double>("label_height", 0.25);
    label_z_offset_ = declare_parameter<double>("label_z_offset", 0.25);

    // Overlap visibility controls
    g_traj_viz.alpha_line_base = (float)declare_parameter<double>("viz_alpha_line_base", 0.10);
    g_traj_viz.alpha_pts_base = (float)declare_parameter<double>("viz_alpha_pts_base", 0.25);

    g_traj_viz.enable_halo = declare_parameter<bool>("viz_enable_halo", true);
    g_traj_viz.halo_alpha = (float)declare_parameter<double>("viz_halo_alpha", 0.10);
    g_traj_viz.halo_scale = declare_parameter<double>("viz_halo_scale", 2.5);
    g_traj_viz.halo_r = (float)declare_parameter<double>("viz_halo_r", 0.0);
    g_traj_viz.halo_g = (float)declare_parameter<double>("viz_halo_g", 0.0);
    g_traj_viz.halo_b = (float)declare_parameter<double>("viz_halo_b", 0.0);

    g_traj_viz.enable_z_offset = declare_parameter<bool>("viz_enable_z_offset", false);
    g_traj_viz.z_step = declare_parameter<double>("viz_z_step", 0.02);

    g_traj_viz.highlight_planner = declare_parameter<std::string>("viz_highlight_planner", "");
    g_traj_viz.highlight_only = declare_parameter<bool>("viz_highlight_only", false);
    g_traj_viz.alpha_line_highlight =
        (float)declare_parameter<double>("viz_alpha_line_highlight", 1.0);
    g_traj_viz.alpha_pts_highlight =
        (float)declare_parameter<double>("viz_alpha_pts_highlight", 0.8);
    g_traj_viz.highlight_scale = declare_parameter<double>("viz_highlight_scale", 3.0);

    // Corridor load
    poly_seed_eps_ = declare_parameter<double>("poly_seed_eps", 1e-6);

    // QoS
    latched_ = declare_parameter<bool>("latched", true);

    if (sfc_dir_.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter sfc_dir is empty.");
      return;
    }
    if (traj_dump_root_dir_.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter traj_dump_root_dir is empty.");
      return;
    }

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();
    if (latched_) qos.transient_local();

    pub_poly_ = create_publisher<decomp_ros_msgs::msg::PolyhedronArray>(poly_topic_, qos);
    pub_traj_ = create_publisher<visualization_msgs::msg::MarkerArray>(traj_topic_, qos);
    pub_guide_path_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(guide_path_topic_, qos);

    loadAllCasesAndTrajs();

    if (cases_.empty()) {
      RCLCPP_WARN(get_logger(), "No cases loaded. Check sfc_dir and file_ext.");
      return;
    }

    if (mode_ == "loop") {
      RCLCPP_INFO(get_logger(), "Mode=loop. Loaded %zu cases. Starting playback.", cases_.size());

      if (visualize_) {
        const double period = std::max(0.05, playback_period_sec_);
        timer_ = create_wall_timer(
            std::chrono::duration<double>(period),
            std::bind(&VisualizeLocalTrajsNode::publishNext, this));
      } else {
        publishCase(0);
      }
    } else if (mode_ == "screenshot") {
      RCLCPP_INFO(get_logger(), "Mode=screenshot. Publishing sampled cases overlay.");
      // Publish once after a short delay to let subscribers connect,
      // then republish periodically so late-joining RViz still sees it.
      screenshot_timer_ = create_wall_timer(std::chrono::seconds(2), [this]() {
        publishScreenshotOverlay();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "Screenshot overlay re-published.");
      });
    } else if (mode_ == "single") {
      const size_t idx = static_cast<size_t>(std::max(0, single_case_index_));
      if (idx >= cases_.size()) {
        RCLCPP_ERROR(
            get_logger(), "single_case_index=%d but only %zu cases loaded.", single_case_index_,
            cases_.size());
        return;
      }
      RCLCPP_INFO(
          get_logger(), "Mode=single. Publishing case %zu (%s).", idx,
          cases_[idx].case_file.c_str());
      // Republish periodically so late-joining RViz sees it
      screenshot_timer_ = create_wall_timer(std::chrono::seconds(2), [this, idx]() {
        publishCase(idx);
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 10000, "Single case %zu re-published.", idx);
      });
    } else {
      RCLCPP_ERROR(
          get_logger(), "Unknown mode: '%s'. Use mode:=loop, screenshot, or single", mode_.c_str());
    }
  }

 private:
  void loadAllCasesAndTrajs() {
    // 1) Load cases (.mysco2)
    std::vector<fs::path> mysco2_files;
    for (const auto& ent : fs::directory_iterator(sfc_dir_)) {
      if (!ent.is_regular_file()) continue;
      const auto p = ent.path();
      if (p.extension() == file_ext_) mysco2_files.push_back(p);
    }
    std::sort(mysco2_files.begin(), mysco2_files.end());

    std::unordered_map<std::string, size_t> case_index;  // case_basename -> idx
    cases_.clear();
    cases_.reserve(mysco2_files.size());

    for (const auto& p : mysco2_files) {
      CaseBundle cb;
      cb.case_file = p.filename().string();
      cb.mysco2_path = p;
      cases_.push_back(std::move(cb));
      case_index[cases_.back().case_file] = cases_.size() - 1;
    }

    // 2) Load trajectories by scanning traj_dump_root_dirs recursively
    size_t traj_count = 0;

    for (const auto& root_str : traj_dump_root_dirs_) {
      if (root_str.empty()) continue;

      const fs::path root(root_str);
      std::error_code ec;
      if (!fs::exists(root, ec)) {
        RCLCPP_WARN(get_logger(), "traj_dump_root_dirs entry does not exist: %s", root_str.c_str());
        continue;
      }

      for (auto it = fs::recursive_directory_iterator(root, ec);
           it != fs::recursive_directory_iterator(); ++it) {
        if (ec) break;
        if (!it->is_regular_file()) continue;

        const auto p = it->path();
        if (p.extension() != ".csv") continue;

        TrajCsv tr;
        if (!parseTrajCsv(p, tr)) continue;

        // Skip planners not in whitelist (if whitelist is non-empty)
        if (!planner_whitelist_.empty() &&
            planner_whitelist_.find(tr.planner_name) == planner_whitelist_.end())
          continue;

        tr.case_file = fs::path(tr.case_file).filename().string();

        auto itc = case_index.find(tr.case_file);
        if (itc == case_index.end()) {
          if (!endsWith(tr.case_file, file_ext_) &&
              case_index.find(tr.case_file + file_ext_) != case_index.end()) {
            tr.case_file = tr.case_file + file_ext_;
            itc = case_index.find(tr.case_file);
          }
        }
        if (itc == case_index.end()) continue;

        tr.frame_id = frame_id_;

        cases_[itc->second].planner_to_traj[tr.planner_name] = std::move(tr);
        traj_count++;
      }
    }

    // 3) For each case, load corridor polytope and guide path markers
    std::vector<CaseBundle> filtered;
    filtered.reserve(cases_.size());

    for (auto& cb : cases_) {
      if (cb.planner_to_traj.empty()) continue;

      try {
        Vec3d start, goal;
        std::vector<Vec3f> path_pts;
        vec_E<Polyhedron<3>> polys;

        loadMysco2CorridorAndPath(cb.mysco2_path, start, goal, path_pts, polys, poly_seed_eps_);

        cb.mysco2_start = start;
        cb.mysco2_goal = goal;
        cb.have_mysco2_start_goal = true;

        auto msg = DecompROS::polyhedron_array_to_ros(polys);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = now();
        msg.lifetime = rclcpp::Duration::from_seconds(1000.0);
        cb.poly_msg = msg;

        cb.guide_path_ma = makeGuidePathMarkers(
            path_pts, frame_id_, now(), makeColor(0.6f, 0.6f, 0.6f, 1.0f), 0.04, 0.08);
      } catch (const std::exception& e) {
        RCLCPP_WARN(
            get_logger(), "Failed loading mysco2 %s: %s", cb.mysco2_path.string().c_str(),
            e.what());
        continue;
      }

      filtered.push_back(std::move(cb));
    }

    cases_.swap(filtered);

    // Build global planner ordering (union across all cases with non-empty trajectory)
    global_planner_index_.clear();
    global_planners_sorted_.clear();

    std::unordered_set<std::string> set;
    for (const auto& cb : cases_) {
      for (const auto& kv : cb.planner_to_traj) {
        if (!kv.second.pts.empty()) set.insert(kv.first);
      }
    }

    global_planners_sorted_.reserve(set.size());
    for (const auto& p : set) global_planners_sorted_.push_back(p);

    std::sort(global_planners_sorted_.begin(), global_planners_sorted_.end());

    global_planner_index_.reserve(global_planners_sorted_.size());
    for (int i = 0; i < (int)global_planners_sorted_.size(); ++i)
      global_planner_index_[global_planners_sorted_[i]] = i;

    global_planner_count_ = (int)global_planners_sorted_.size();

    RCLCPP_INFO(get_logger(), "Global planner palette: %d planners.", global_planner_count_);

    RCLCPP_INFO(
        get_logger(),
        "Scan complete: %zu cases in sfc_dir, %zu cases with trajectories, %zu trajectory "
        "CSVs loaded.",
        mysco2_files.size(), cases_.size(), traj_count);
  }

  void publishNext() {
    if (cases_.empty()) return;
    publishCase(play_idx_);
    play_idx_ = (play_idx_ + 1) % cases_.size();
  }

  void publishCase(size_t idx) {
    if (idx >= cases_.size()) return;

    const auto stamp = now();
    auto& cb = cases_[idx];

    // Clear
    {
      visualization_msgs::msg::MarkerArray clear;
      clear.markers.push_back(deleteAllMarker(frame_id_, stamp));
      pub_traj_->publish(clear);
      pub_guide_path_->publish(clear);
    }

    // Corridor
    cb.poly_msg.header.stamp = stamp;
    pub_poly_->publish(cb.poly_msg);

    // Guide path
    if (show_guide_path_) {
      for (auto& mk : cb.guide_path_ma.markers) {
        mk.header.frame_id = frame_id_;
        mk.header.stamp = stamp;
      }
      pub_guide_path_->publish(cb.guide_path_ma);
    }

    std::optional<geometry_msgs::msg::Point> start_opt;
    std::optional<geometry_msgs::msg::Point> goal_opt;
    if (cb.have_mysco2_start_goal) {
      start_opt = toPoint(cb.mysco2_start.x(), cb.mysco2_start.y(), cb.mysco2_start.z());
      goal_opt = toPoint(cb.mysco2_goal.x(), cb.mysco2_goal.y(), cb.mysco2_goal.z());
    }

    const auto traj_ma = makeTrajOverlayMarkers(
        cb.planner_to_traj, frame_id_, stamp, show_points_, show_labels_, traj_line_width_,
        traj_point_diam_, label_height_, label_z_offset_,
        /*goal_text_override=*/"goal(case-" + std::to_string(idx) + ")", start_opt, goal_opt,
        /*global_planner_index=*/&global_planner_index_,
        /*global_planner_count=*/global_planner_count_);

    pub_traj_->publish(traj_ma);

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000, "Visualizing case %zu/%zu: %s (variants=%zu)", idx + 1,
        cases_.size(), cb.case_file.c_str(), cb.planner_to_traj.size());
  }

  void publishScreenshotOverlay() {
    if (cases_.empty()) return;

    const auto stamp = now();

    // Select indices: 0, stride, 2*stride, ... up to screenshot_max_index_ (inclusive), if
    // available.
    const int stride = std::max(1, screenshot_stride_);
    const int max_i = std::max(0, screenshot_max_index_);

    std::vector<size_t> sel;
    for (int i = 0; i <= max_i; i += stride) {
      if ((size_t)i < cases_.size()) sel.push_back((size_t)i);
    }
    if (sel.empty()) sel.push_back(0);

    // Pre-scan: union of planners that have non-empty trajectory in ANY selected case.
    std::unordered_set<std::string> planner_set;
    for (size_t ci : sel) {
      for (const auto& kv : cases_[ci].planner_to_traj) {
        if (!kv.second.pts.empty()) planner_set.insert(kv.first);
      }
    }

    std::vector<std::string> planners_sorted;
    planners_sorted.reserve(planner_set.size());
    for (const auto& p : planner_set) planners_sorted.push_back(p);
    std::sort(planners_sorted.begin(), planners_sorted.end());

    std::unordered_map<std::string, int> global_planner_index;
    global_planner_index.reserve(planners_sorted.size());
    for (int i = 0; i < (int)planners_sorted.size(); ++i)
      global_planner_index[planners_sorted[i]] = i;
    const int global_planner_count = (int)planners_sorted.size();

    // Determine global start (publish only once)
    std::optional<geometry_msgs::msg::Point> global_start_opt;
    if (!sel.empty() && cases_[sel.front()].have_mysco2_start_goal) {
      const auto& cb0 = cases_[sel.front()];
      global_start_opt = toPoint(cb0.mysco2_start.x(), cb0.mysco2_start.y(), cb0.mysco2_start.z());
    }

    // Clear once
    visualization_msgs::msg::MarkerArray traj_all;
    visualization_msgs::msg::MarkerArray guide_all;
    traj_all.markers.push_back(deleteAllMarker(frame_id_, stamp));
    guide_all.markers.push_back(deleteAllMarker(frame_id_, stamp));

    // Combine corridor polys into one message
    decomp_ros_msgs::msg::PolyhedronArray poly_all;
    bool poly_init = false;

    int traj_id = 1;
    int guide_id = 1;

    for (size_t k = 0; k < sel.size(); ++k) {
      const size_t ci = sel[k];
      auto& cb = cases_[ci];

      const std::string ns_prefix = "case_" + zeroPadInt((int)ci, 3);

      // Corridor accumulation
      if (!poly_init) {
        poly_all = cb.poly_msg;
        poly_all.header.frame_id = frame_id_;
        poly_all.header.stamp = stamp;
        poly_init = true;
      } else {
        poly_all.polyhedrons.insert(
            poly_all.polyhedrons.end(), cb.poly_msg.polyhedrons.begin(),
            cb.poly_msg.polyhedrons.end());
      }

      // Guide path accumulation
      if (show_guide_path_) {
        const auto& ma = cb.guide_path_ma;
        if (ma.markers.size() >= 3 &&
            ma.markers[1].type == visualization_msgs::msg::Marker::LINE_STRIP) {
          std::vector<Vec3f> path_pts;
          path_pts.reserve(ma.markers[1].points.size());
          for (const auto& p : ma.markers[1].points) {
            Vec3f v;
            v.x() = p.x;
            v.y() = p.y;
            v.z() = p.z;
            path_pts.push_back(v);
          }
          appendGuidePathMarkersNoDelete(
              guide_all, guide_id, path_pts, frame_id_, stamp, makeColor(0.6f, 0.6f, 0.6f, 1.0f),
              0.04, 0.08, ns_prefix);
        }
      }

      // Start/goal from .mysco2
      std::optional<geometry_msgs::msg::Point> start_opt;
      std::optional<geometry_msgs::msg::Point> goal_opt;
      if (cb.have_mysco2_start_goal) {
        start_opt = toPoint(cb.mysco2_start.x(), cb.mysco2_start.y(), cb.mysco2_start.z());
        goal_opt = toPoint(cb.mysco2_goal.x(), cb.mysco2_goal.y(), cb.mysco2_goal.z());
      }

      // Case label: requirements
      // - text: "caseXX" (no space)
      // - placed at goal.x + 1.0, goal.y + 0.0
      const std::string case_text = std::string("case-") + std::to_string((int)ci);

      appendTrajOverlayMarkers(
          traj_all, traj_id, cb.planner_to_traj, frame_id_, stamp, show_points_, show_labels_,
          traj_line_width_, traj_point_diam_, label_height_, label_z_offset_, ns_prefix, case_text,
          start_opt, goal_opt,
          /*publish_start=*/false,               // start only once globally
          /*publish_goal_and_case_label=*/true,  // case label per case
          /*publish_planner_labels=*/false,      // planner legend only once globally
          /*global_planner_index=*/&global_planner_index,
          /*global_planner_count=*/global_planner_count,
          /*goal_label_dx=*/1.3,
          /*goal_label_dy=*/0.0);
    }

    // Publish global start once
    if (global_start_opt) {
      appendStartOnce(
          traj_all, traj_id, frame_id_, stamp, *global_start_opt, traj_point_diam_, label_height_,
          label_z_offset_);
    }

    // Publish planner legend once (use global planner list; stable colors)
    if (!planners_sorted.empty()) {
      geometry_msgs::msg::Point anchor;
      if (global_start_opt) {
        anchor = *global_start_opt;
        anchor.x -= 2.0;  // place legend left of start
      } else {
        anchor = toPoint(0.0, 0.0, 0.0);
      }

      appendPlannerLegendOnce(
          traj_all, traj_id, planners_sorted, frame_id_, stamp, anchor, label_height_,
          label_z_offset_);
    }

    // Publish
    if (poly_init) {
      poly_all.header.frame_id = frame_id_;
      poly_all.header.stamp = stamp;
      pub_poly_->publish(poly_all);
    }

    pub_guide_path_->publish(guide_all);
    pub_traj_->publish(traj_all);

    RCLCPP_INFO(
        get_logger(),
        "Screenshot overlay published: %zu cases sampled (stride=%d, max_index=%d). planners=%zu",
        sel.size(), stride, max_i, planners_sorted.size());
  }

 private:
  // params
  std::string sfc_dir_;
  std::string file_ext_;
  std::vector<std::string> traj_dump_root_dirs_;
  std::string traj_dump_root_dir_;

  std::string frame_id_;
  std::string poly_topic_;
  std::string traj_topic_;
  std::string guide_path_topic_;

  bool visualize_{true};
  double playback_period_sec_{0.5};
  bool latched_{true};

  std::string mode_{"loop"};
  int screenshot_stride_{10};
  int screenshot_max_index_{100};
  int single_case_index_{0};

  bool show_guide_path_{true};
  bool show_points_{true};
  bool show_labels_{true};

  std::vector<std::string> planner_whitelist_vec_;
  std::set<std::string> planner_whitelist_;

  double traj_line_width_{0.06};
  double traj_point_diam_{0.10};
  double label_height_{0.25};
  double label_z_offset_{0.25};

  double poly_seed_eps_{1e-6};

  // state
  std::vector<CaseBundle> cases_;
  size_t play_idx_{0};

  // global planner palette for loop mode (and can also be used elsewhere)
  std::vector<std::string> global_planners_sorted_;
  std::unordered_map<std::string, int> global_planner_index_;
  int global_planner_count_{0};

  // ros
  rclcpp::Publisher<decomp_ros_msgs::msg::PolyhedronArray>::SharedPtr pub_poly_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_traj_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_guide_path_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr screenshot_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualizeLocalTrajsNode>());
  rclcpp::shutdown();
  return 0;
}
