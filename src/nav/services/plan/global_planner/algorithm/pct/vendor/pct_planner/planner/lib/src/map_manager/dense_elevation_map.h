#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include <Eigen/Dense>

class DenseElevationMap {
 public:
  DenseElevationMap() = default;
  ~DenseElevationMap() = default;

  void Init(const double resolution, const int num_layers,
            const Eigen::MatrixXd& cost_map, const Eigen::MatrixXd& ele_mask,
            const Eigen::MatrixXd& height, const Eigen::MatrixXd& ceiling,
            const Eigen::MatrixXd& grad_x, const Eigen::MatrixXd& grad_y);

  bool IsValid() const { return valid_; }

  double GetValueBilinear(const int layer, const double x, const double y,
                          Eigen::Vector2d* grad = nullptr);

  double GetValueBilinearSafe(const int layer, const double x, const double y,
                              const double height_hint,
                              Eigen::Vector2d* grad = nullptr);

  int UpdateLayer(const int layer, const double x, const double y);

  int UpdateLayerSafe(const int layer, const double x, const double y,
                      const double height_hint);

  double GetHeight(const int layer, const double x, const double y);

  double GetHeightSafe(const int layer, const double x, const double y,
                       const double height_hint);

  double GetCeiling(const int layer, const double x, const double y);

  double inline GetNominalCost(int layer, double x, double y) {
    if (!valid_) {
      return safe_cost_threshold_;
    }
    auto idx = CoordsToIndex(layer, x, y);
    return cost_(idx[0], idx[1]);
  };

  std::array<int, 2> inline CoordsToIndex(int layer, double x, double y) {
    int col = index_x_safe(x);
    int row = index_y_safe(y) + layer_safe(layer) * max_y_;
    return {row, col};
  }

  void SetDebug(const bool flag) { debug_ = flag; }

 private:
  int inline index(double coord) {
    if (!std::isfinite(coord)) {
      return 0;
    }
    return static_cast<int>(coord);
  }

  int inline layer_safe(int layer) {
    if (max_layers_ <= 0) {
      return 0;
    }
    return std::min(std::max(layer, 0), max_layers_ - 1);
  }

  int inline index_x_safe(double coord) {
    if (max_x_ <= 0) {
      return 0;
    }
    return std::min(std::max(index(coord), 0), max_x_ - 1);
  }

  int inline index_y_safe(double coord) {
    if (max_y_ <= 0) {
      return 0;
    }
    return std::min(std::max(index(coord), 0), max_y_ - 1);
  }

  double GetRealCost(int layer, double x, double y,
                     Eigen::Vector2d* grad = nullptr,
                     int* real_layer = nullptr);

  double GetRealCostSafe(int layer, double x, double y,
                         const double height_hint);

 private:
  bool debug_ = false;
  bool valid_ = false;
  double resolution_ = 0.0;
  double resolution_inv_ = 0.0;
  int max_layers_ = 0;
  int max_x_ = 0;
  int max_y_ = 0;
  int xy_size_ = 0;
  double offset_ = 0;

  double safe_cost_threshold_ = 10;

  Eigen::MatrixXd cost_;
  Eigen::MatrixXd ele_mask_;
  Eigen::MatrixXd height_;
  Eigen::MatrixXd ceiling_;
  Eigen::MatrixXd grad_x_;
  Eigen::MatrixXd grad_y_;
};
