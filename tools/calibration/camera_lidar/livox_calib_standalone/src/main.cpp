/**
 * livox_calib_standalone — Targetless Camera-LiDAR calibration without ROS
 *
 * Based on hku-mars/livox_camera_calib algorithm:
 *   1. Canny edge detection on camera image
 *   2. Voxel-based plane fitting + line extraction from LiDAR
 *   3. Edge-line matching with iterative Ceres optimization
 *
 * Usage:
 *   livox_calib --image <path.png> --pcd <path.pcd> \
 *               --camera <camera.yaml> --calib <calib.yaml> \
 *               [--output <extrinsic.txt>] [--rough]
 */
#include "lidar_camera_calib.hpp"
#include "ceres/ceres.h"
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

// DEG2RAD / RAD2DEG from original common.h
#ifndef DEG2RAD
#define DEG2RAD(x) ((x) * M_PI / 180.0)
#endif
#ifndef RAD2DEG
#define RAD2DEG(x) ((x) * 180.0 / M_PI)
#endif

static void printUsage(const char* prog) {
  std::cout << "Usage:\n"
            << "  " << prog << " --image <path.png> --pcd <path.pcd>\n"
            << "               --camera <camera.yaml> --calib <calib.yaml>\n"
            << "               [--output <extrinsic.txt>] [--rough]\n"
            << "\nRequired arguments:\n"
            << "  --image   Camera image file (PNG/JPG)\n"
            << "  --pcd     LiDAR point cloud file (PCD)\n"
            << "  --camera  Camera intrinsics YAML (OpenCV FileStorage)\n"
            << "  --calib   Calibration config YAML (OpenCV FileStorage)\n"
            << "\nOptional:\n"
            << "  --output  Result file path (default: extrinsic.txt)\n"
            << "  --rough   Enable rough calibration search\n"
            << std::endl;
}

// PnP cost function (same as original)
static Eigen::Matrix3d inner;
static Eigen::Vector4d distor;

class pnp_calib {
public:
  pnp_calib(PnPData p) : pd(p) {}
  template <typename T>
  bool operator()(const T *_q, const T *_t, T *residuals) const {
    Eigen::Matrix<T, 3, 3> innerT = inner.cast<T>();
    Eigen::Matrix<T, 4, 1> distorT = distor.cast<T>();
    Eigen::Quaternion<T> q_incre{_q[3], _q[0], _q[1], _q[2]};
    Eigen::Matrix<T, 3, 1> t_incre{_t[0], _t[1], _t[2]};
    Eigen::Matrix<T, 3, 1> p_l(T(pd.x), T(pd.y), T(pd.z));
    Eigen::Matrix<T, 3, 1> p_c = q_incre.toRotationMatrix() * p_l + t_incre;
    Eigen::Matrix<T, 3, 1> p_2 = innerT * p_c;
    T uo = p_2[0] / p_2[2];
    T vo = p_2[1] / p_2[2];
    const T &fx = innerT.coeffRef(0, 0);
    const T &cx = innerT.coeffRef(0, 2);
    const T &fy = innerT.coeffRef(1, 1);
    const T &cy = innerT.coeffRef(1, 2);
    T xo = (uo - cx) / fx;
    T yo = (vo - cy) / fy;
    T r2 = xo * xo + yo * yo;
    T r4 = r2 * r2;
    T distortion = 1.0 + distorT[0] * r2 + distorT[1] * r4;
    T xd = xo * distortion + (distorT[2] * xo * yo + distorT[2] * xo * yo) +
           distorT[3] * (r2 + xo * xo + xo * xo);
    T yd = yo * distortion + distorT[3] * xo * yo + distorT[3] * xo * yo +
           distorT[2] * (r2 + yo * yo + yo * yo);
    T ud = fx * xd + cx;
    T vd = fy * yd + cy;
    residuals[0] = ud - T(pd.u);
    residuals[1] = vd - T(pd.v);
    return true;
  }
  static ceres::CostFunction *Create(PnPData p) {
    return new ceres::AutoDiffCostFunction<pnp_calib, 2, 4, 3>(new pnp_calib(p));
  }
private:
  PnPData pd;
};

class vpnp_calib {
public:
  vpnp_calib(VPnPData p) : pd(p) {}
  template <typename T>
  bool operator()(const T *_q, const T *_t, T *residuals) const {
    Eigen::Matrix<T, 3, 3> innerT = inner.cast<T>();
    Eigen::Matrix<T, 4, 1> distorT = distor.cast<T>();
    Eigen::Quaternion<T> q_incre{_q[3], _q[0], _q[1], _q[2]};
    Eigen::Matrix<T, 3, 1> t_incre{_t[0], _t[1], _t[2]};
    Eigen::Matrix<T, 3, 1> p_l(T(pd.x), T(pd.y), T(pd.z));
    Eigen::Matrix<T, 3, 1> p_c = q_incre.toRotationMatrix() * p_l + t_incre;
    Eigen::Matrix<T, 3, 1> p_2 = innerT * p_c;
    T uo = p_2[0] / p_2[2];
    T vo = p_2[1] / p_2[2];
    const T &fx = innerT.coeffRef(0, 0);
    const T &cx = innerT.coeffRef(0, 2);
    const T &fy = innerT.coeffRef(1, 1);
    const T &cy = innerT.coeffRef(1, 2);
    T xo = (uo - cx) / fx;
    T yo = (vo - cy) / fy;
    T r2 = xo * xo + yo * yo;
    T r4 = r2 * r2;
    T distortion = 1.0 + distorT[0] * r2 + distorT[1] * r4;
    T xd = xo * distortion + (distorT[2] * xo * yo + distorT[2] * xo * yo) +
           distorT[3] * (r2 + xo * xo + xo * xo);
    T yd = yo * distortion + distorT[3] * xo * yo + distorT[3] * xo * yo +
           distorT[2] * (r2 + yo * yo + yo * yo);
    T ud = fx * xd + cx;
    T vd = fy * yd + cy;
    if (T(pd.direction(0)) == T(0.0) && T(pd.direction(1)) == T(0.0)) {
      residuals[0] = ud - T(pd.u);
      residuals[1] = vd - T(pd.v);
    } else {
      residuals[0] = ud - T(pd.u);
      residuals[1] = vd - T(pd.v);
      Eigen::Matrix<T, 2, 2> I = Eigen::Matrix<float, 2, 2>::Identity().cast<T>();
      Eigen::Matrix<T, 2, 1> n = pd.direction.cast<T>();
      Eigen::Matrix<T, 1, 2> nt = pd.direction.transpose().cast<T>();
      Eigen::Matrix<T, 2, 2> V = n * nt;
      V = I - V;
      Eigen::Matrix<T, 2, 1> R;
      R.coeffRef(0, 0) = residuals[0];
      R.coeffRef(1, 0) = residuals[1];
      R = V * R;
      residuals[0] = R.coeffRef(0, 0);
      residuals[1] = R.coeffRef(1, 0);
    }
    return true;
  }
  static ceres::CostFunction *Create(VPnPData p) {
    return new ceres::AutoDiffCostFunction<vpnp_calib, 2, 4, 3>(new vpnp_calib(p));
  }
private:
  VPnPData pd;
};

int main(int argc, char **argv) {
  std::string image_file, pcd_file, camera_file, calib_file;
  std::string output_file = "extrinsic.txt";
  bool use_rough_calib = false;

  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--image" && i + 1 < argc) image_file = argv[++i];
    else if (arg == "--pcd" && i + 1 < argc) pcd_file = argv[++i];
    else if (arg == "--camera" && i + 1 < argc) camera_file = argv[++i];
    else if (arg == "--calib" && i + 1 < argc) calib_file = argv[++i];
    else if (arg == "--output" && i + 1 < argc) output_file = argv[++i];
    else if (arg == "--rough") use_rough_calib = true;
    else if (arg == "--help" || arg == "-h") { printUsage(argv[0]); return 0; }
  }

  if (image_file.empty() || pcd_file.empty() || camera_file.empty() || calib_file.empty()) {
    printUsage(argv[0]);
    return 1;
  }

  // Load camera intrinsics
  cv::FileStorage camFS(camera_file, cv::FileStorage::READ);
  if (!camFS.isOpened()) {
    std::cerr << "Failed to open camera file: " << camera_file << std::endl;
    return 1;
  }
  cv::Mat camera_matrix, dist_coeffs_mat;
  camFS["camera_matrix"] >> camera_matrix;
  camFS["dist_coeffs"] >> dist_coeffs_mat;
  camFS.release();

  float fx = camera_matrix.at<double>(0, 0);
  float cx = camera_matrix.at<double>(0, 2);
  float fy = camera_matrix.at<double>(1, 1);
  float cy = camera_matrix.at<double>(1, 2);
  float k1 = dist_coeffs_mat.at<double>(0, 0);
  float k2 = dist_coeffs_mat.at<double>(0, 1);
  float p1 = dist_coeffs_mat.at<double>(0, 2);
  float p2 = dist_coeffs_mat.at<double>(0, 3);
  float k3 = dist_coeffs_mat.at<double>(0, 4);

  std::cout << "Camera intrinsics: fx=" << fx << " fy=" << fy
            << " cx=" << cx << " cy=" << cy << std::endl;

  // Create calibration object (loads image + PCD + calib config internally)
  Calibration calibra(image_file, pcd_file, calib_file);
  calibra.fx_ = fx;
  calibra.cx_ = cx;
  calibra.fy_ = fy;
  calibra.cy_ = cy;
  calibra.k1_ = k1;
  calibra.k2_ = k2;
  calibra.p1_ = p1;
  calibra.p2_ = p2;
  calibra.k3_ = k3;

  // Setup inner/distortion matrices
  inner << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
  distor << k1, k2, p1, p2;

  Eigen::Matrix3d R = calibra.init_rotation_matrix_;
  Eigen::Vector3d T = calibra.init_translation_vector_;
  Eigen::Vector3d euler = R.eulerAngles(2, 1, 0);
  Vector6d calib_params;
  calib_params << euler(0), euler(1), euler(2), T(0), T(1), T(2);

  std::cout << "Initial rotation:\n" << R << std::endl;
  std::cout << "Initial translation: " << T.transpose() << std::endl;

  // Save initial projection image
  cv::Mat init_img = calibra.getProjectionImg(calib_params);
  cv::imwrite("calib_initial.png", init_img);
  std::cout << "Saved initial projection: calib_initial.png" << std::endl;

  // Rough calibration (optional)
  if (use_rough_calib) {
    std::cout << "Running rough calibration..." << std::endl;
    // Use same roughCalib logic from original
    float match_dis = 25;
    Eigen::Vector3d fix_adjust_euler(0, 0, 0);
    double search_resolution = DEG2RAD(0.1);
    int max_iter = 50;
    for (int n = 0; n < 2; n++) {
      for (int round = 0; round < 3; round++) {
        Eigen::Matrix3d rot = Eigen::AngleAxisd(calib_params[0], Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(calib_params[1], Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(calib_params[2], Eigen::Vector3d::UnitX());
        float min_cost = 1000;
        for (int iter = 0; iter < max_iter; iter++) {
          Eigen::Vector3d adjust_euler = fix_adjust_euler;
          adjust_euler[round] = fix_adjust_euler[round] +
                                pow(-1, iter) * int(iter / 2) * search_resolution;
          Eigen::Matrix3d adjust_rotation_matrix =
              Eigen::AngleAxisd(adjust_euler[0], Eigen::Vector3d::UnitZ()) *
              Eigen::AngleAxisd(adjust_euler[1], Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(adjust_euler[2], Eigen::Vector3d::UnitX());
          Eigen::Matrix3d test_rot = rot * adjust_rotation_matrix;
          Eigen::Vector3d test_euler = test_rot.eulerAngles(2, 1, 0);
          Vector6d test_params;
          test_params << test_euler[0], test_euler[1], test_euler[2],
              calib_params[3], calib_params[4], calib_params[5];
          std::vector<VPnPData> pnp_list;
          calibra.buildVPnp(test_params, match_dis, false,
                            calibra.rgb_egde_cloud_, calibra.plane_line_cloud_, pnp_list);
          float cost = (calibra.plane_line_cloud_->size() - pnp_list.size()) *
                       1.0 / calibra.plane_line_cloud_->size();
          if (cost < min_cost) {
            std::cout << "  Rough calib min cost: " << cost << std::endl;
            min_cost = cost;
            calib_params[0] = test_params[0];
            calib_params[1] = test_params[1];
            calib_params[2] = test_params[2];
          }
        }
      }
    }
    cv::Mat rough_img = calibra.getProjectionImg(calib_params);
    cv::imwrite("calib_rough.png", rough_img);
    std::cout << "Saved rough calibration: calib_rough.png" << std::endl;
  }

  // Fine optimization — iteratively reduce matching distance
  int iter = 0;
  bool use_vpnp = true;
  for (int dis_threshold = 30; dis_threshold > 10; dis_threshold -= 1) {
    for (int cnt = 0; cnt < 2; cnt++) {
      std::vector<VPnPData> vpnp_list;
      std::vector<PnPData> pnp_list;

      if (use_vpnp) {
        calibra.buildVPnp(calib_params, dis_threshold, true,
                          calibra.rgb_egde_cloud_, calibra.plane_line_cloud_, vpnp_list);
      } else {
        calibra.buildPnp(calib_params, dis_threshold, true,
                         calibra.rgb_egde_cloud_, calibra.plane_line_cloud_, pnp_list);
      }
      std::cout << "Iter:" << iter++ << " Dis:" << dis_threshold
                << " pnp_size:" << vpnp_list.size() << std::endl;

      Eigen::Vector3d euler_angle(calib_params[0], calib_params[1], calib_params[2]);
      Eigen::Matrix3d opt_init_R =
          Eigen::AngleAxisd(euler_angle[0], Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(euler_angle[1], Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(euler_angle[2], Eigen::Vector3d::UnitX());
      Eigen::Quaterniond q(opt_init_R);
      Eigen::Vector3d ori_t = T;

      double ext[7];
      ext[0] = q.x(); ext[1] = q.y(); ext[2] = q.z(); ext[3] = q.w();
      ext[4] = T[0]; ext[5] = T[1]; ext[6] = T[2];
      Eigen::Map<Eigen::Quaterniond> m_q = Eigen::Map<Eigen::Quaterniond>(ext);
      Eigen::Map<Eigen::Vector3d> m_t = Eigen::Map<Eigen::Vector3d>(ext + 4);

      ceres::LocalParameterization *q_parameterization =
          new ceres::EigenQuaternionParameterization();
      ceres::Problem problem;
      problem.AddParameterBlock(ext, 4, q_parameterization);
      problem.AddParameterBlock(ext + 4, 3);

      if (use_vpnp) {
        for (auto val : vpnp_list) {
          problem.AddResidualBlock(vpnp_calib::Create(val), NULL, ext, ext + 4);
        }
      } else {
        for (auto val : pnp_list) {
          problem.AddResidualBlock(pnp_calib::Create(val), NULL, ext, ext + 4);
        }
      }

      ceres::Solver::Options options;
      options.preconditioner_type = ceres::JACOBI;
      options.linear_solver_type = ceres::SPARSE_SCHUR;
      options.minimizer_progress_to_stdout = true;
      options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);
      std::cout << summary.BriefReport() << std::endl;

      Eigen::Matrix3d rot = m_q.toRotationMatrix();
      euler_angle = rot.eulerAngles(2, 1, 0);
      calib_params[0] = euler_angle[0];
      calib_params[1] = euler_angle[1];
      calib_params[2] = euler_angle[2];
      calib_params[3] = m_t(0);
      calib_params[4] = m_t(1);
      calib_params[5] = m_t(2);
      R = rot;
      T[0] = m_t(0);
      T[1] = m_t(1);
      T[2] = m_t(2);
      Eigen::Quaterniond opt_q(R);
      std::cout << "q_dis:" << RAD2DEG(opt_q.angularDistance(q))
                << " ,t_dis:" << (T - ori_t).norm() << std::endl;
    }
  }

  // Write result
  R = Eigen::AngleAxisd(calib_params[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(calib_params[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(calib_params[2], Eigen::Vector3d::UnitX());

  std::ofstream outfile(output_file);
  for (int i = 0; i < 3; i++) {
    outfile << R(i, 0) << "," << R(i, 1) << "," << R(i, 2) << "," << T[i] << std::endl;
  }
  outfile << 0 << "," << 0 << "," << 0 << "," << 1 << std::endl;
  outfile.close();

  std::cout << "\n=== Calibration Result ===" << std::endl;
  std::cout << "Rotation matrix:\n" << R << std::endl;
  std::cout << "Translation: " << T.transpose() << std::endl;
  std::cout << "Euler (ZYX, deg): " << RAD2DEG(calib_params[0]) << ", "
            << RAD2DEG(calib_params[1]) << ", " << RAD2DEG(calib_params[2]) << std::endl;
  std::cout << "Result saved to: " << output_file << std::endl;

  // Save final projection image
  cv::Mat opt_img = calibra.getProjectionImg(calib_params);
  cv::imwrite("calib_result.png", opt_img);
  std::cout << "Saved result projection: calib_result.png" << std::endl;

  return 0;
}
