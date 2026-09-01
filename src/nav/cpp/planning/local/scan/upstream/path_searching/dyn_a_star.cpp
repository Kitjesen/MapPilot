#include "planning/local/scan/upstream/path_searching/dyn_a_star.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace nav_kernel::local::scan::upstream {

AStar::~AStar() {
  releaseGridNodes();
}

void AStar::releaseGridNodes() noexcept {
  if (gridNodeMap_ == nullptr) return;
  for (int x = 0; x < poolSize_.x(); ++x) {
    for (int y = 0; y < poolSize_.y(); ++y) {
      for (int z = 0; z < poolSize_.z(); ++z) {
        delete gridNodeMap_[x][y][z];
      }
      delete[] gridNodeMap_[x][y];
    }
    delete[] gridNodeMap_[x];
  }
  delete[] gridNodeMap_;
  gridNodeMap_ = nullptr;
}

void AStar::initGridMap(GridMap::Ptr occupancyMap,
                        const Eigen::Vector3i &poolSize) {
  if (gridNodeMap_ != nullptr && poolSize == poolSize_) {
    gridMap_ = std::move(occupancyMap);
    return;
  }

  releaseGridNodes();
  poolSize_ = poolSize;
  centerIndex_ = poolSize / 2;
  gridNodeMap_ = new GridNodePtr **[poolSize_.x()];
  for (int x = 0; x < poolSize_.x(); ++x) {
    gridNodeMap_[x] = new GridNodePtr *[poolSize_.y()];
    for (int y = 0; y < poolSize_.y(); ++y) {
      gridNodeMap_[x][y] = new GridNodePtr[poolSize_.z()];
      for (int z = 0; z < poolSize_.z(); ++z) {
        gridNodeMap_[x][y][z] = new GridNode;
      }
    }
  }
  gridMap_ = std::move(occupancyMap);
}

void AStar::setGridMap(GridMap::Ptr occupancyMap) noexcept {
  gridMap_ = std::move(occupancyMap);
}

double AStar::getDiagHeuristic(GridNodePtr node1,
                               GridNodePtr node2) const {
  double dx = std::abs(node1->index.x() - node2->index.x());
  double dy = std::abs(node1->index.y() - node2->index.y());
  double dz = std::abs(node1->index.z() - node2->index.z());
  const double diagonal = std::min({dx, dy, dz});
  dx -= diagonal;
  dy -= diagonal;
  dz -= diagonal;

  double result = 0.0;
  if (dx == 0.0) {
    result = std::sqrt(3.0) * diagonal +
             std::sqrt(2.0) * std::min(dy, dz) + std::abs(dy - dz);
  }
  if (dy == 0.0) {
    result = std::sqrt(3.0) * diagonal +
             std::sqrt(2.0) * std::min(dx, dz) + std::abs(dx - dz);
  }
  if (dz == 0.0) {
    result = std::sqrt(3.0) * diagonal +
             std::sqrt(2.0) * std::min(dx, dy) + std::abs(dx - dy);
  }
  return result;
}

double AStar::getHeuristic(GridNodePtr node1, GridNodePtr node2) const {
  return kTieBreaker * getDiagHeuristic(node1, node2);
}

Eigen::Vector3d AStar::indexToCoordinate(
    const Eigen::Vector3i &index) const {
  return ((index - centerIndex_).cast<double>() * stepSize_) + center_;
}

bool AStar::coordinateToIndex(const Eigen::Vector3d &point,
                              Eigen::Vector3i &index) const {
  index = ((point - center_) * inverseStepSize_ +
           Eigen::Vector3d(0.5, 0.5, 0.5))
              .cast<int>() +
          centerIndex_;
  return index.x() >= 0 && index.x() < poolSize_.x() && index.y() >= 0 &&
         index.y() < poolSize_.y() && index.z() >= 0 &&
         index.z() < poolSize_.z();
}

int AStar::occupancy(const Eigen::Vector3d &position, double yaw) const {
  return gridMap_ != nullptr ? gridMap_->getInflateOccupancy(position, yaw)
                             : -1;
}

bool AStar::convertAndAdjust(Eigen::Vector3d start, Eigen::Vector3d end,
                             Eigen::Vector3i &startIndex,
                             Eigen::Vector3i &endIndex) const {
  if (!coordinateToIndex(start, startIndex) ||
      !coordinateToIndex(end, endIndex)) {
    return false;
  }

  Eigen::Vector3d direction = end - start;
  if (direction.norm() < 1e-6) return false;
  const double pathYaw = std::atan2(direction.y(), direction.x());
  direction.normalize();

  int occupied = occupancy(indexToCoordinate(startIndex), pathYaw);
  if (occupied != 0) {
    do {
      start -= direction * stepSize_;
      if (!coordinateToIndex(start, startIndex)) return false;
      occupied = occupancy(indexToCoordinate(startIndex), pathYaw);
      if (occupied == -1) return false;
    } while (occupied != 0);
  }

  occupied = occupancy(indexToCoordinate(endIndex), pathYaw);
  if (occupied != 0) {
    do {
      end += direction * stepSize_;
      if (!coordinateToIndex(end, endIndex)) return false;
      occupied = occupancy(indexToCoordinate(endIndex), pathYaw);
      if (occupied == -1) return false;
    } while (occupied != 0);
  }
  return true;
}

std::vector<GridNodePtr> AStar::retrievePath(GridNodePtr current) const {
  std::vector<GridNodePtr> result;
  result.push_back(current);
  while (current->cameFrom != nullptr) {
    current = current->cameFrom;
    result.push_back(current);
  }
  return result;
}

AStarResult AStar::search(double stepSize, Eigen::Vector3d start,
                          Eigen::Vector3d end) {
  const auto started = std::chrono::steady_clock::now();
  ++rounds_;
  expandedNodes_ = 0;
  gridPath_.clear();
  stepSize_ = stepSize;
  inverseStepSize_ = 1.0 / stepSize;
  center_ = (start + end) / 2.0;

  Eigen::Vector3i startIndex;
  Eigen::Vector3i endIndex;
  if (gridNodeMap_ == nullptr || gridMap_ == nullptr || stepSize <= 0.0 ||
      !convertAndAdjust(start, end, startIndex, endIndex)) {
    return AStarResult::InitError;
  }

  const Eigen::Vector3d searchStart = indexToCoordinate(startIndex);
  const Eigen::Vector3d searchEnd = indexToCoordinate(endIndex);
  const Eigen::Vector2d searchStartXY = searchStart.head<2>();
  const Eigen::Vector2d searchDeltaXY = searchEnd.head<2>() - searchStartXY;
  const double searchLengthSquared = searchDeltaXY.squaredNorm();

  const auto interpolateZIndex = [&](int xIndex, int yIndex) {
    if (searchLengthSquared < 1e-8) return startIndex.z();
    const Eigen::Vector3i sampleIndex(xIndex, yIndex, startIndex.z());
    const Eigen::Vector2d sampleXY =
        indexToCoordinate(sampleIndex).head<2>();
    double ratio =
        (sampleXY - searchStartXY).dot(searchDeltaXY) / searchLengthSquared;
    ratio = std::clamp(ratio, 0.0, 1.0);
    const double z = searchStart.z() + ratio * (searchEnd.z() - searchStart.z());
    return static_cast<int>((z - center_.z()) * inverseStepSize_ + 0.5) +
           centerIndex_.z();
  };

  GridNodePtr startNode =
      gridNodeMap_[startIndex.x()][startIndex.y()][startIndex.z()];
  GridNodePtr endNode = gridNodeMap_[endIndex.x()][endIndex.y()][endIndex.z()];
  std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator>
      empty;
  openSet_.swap(empty);

  endNode->index = endIndex;
  startNode->index = startIndex;
  startNode->rounds = rounds_;
  startNode->gScore = 0.0;
  startNode->fScore = getHeuristic(startNode, endNode);
  startNode->state = GridNode::State::OpenSet;
  startNode->cameFrom = nullptr;
  openSet_.push(startNode);

  while (!openSet_.empty()) {
    GridNodePtr current = openSet_.top();
    openSet_.pop();
    ++expandedNodes_;

    if (current->index == endNode->index) {
      gridPath_ = retrievePath(current);
      return AStarResult::Success;
    }
    current->state = GridNode::State::ClosedSet;

    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) continue;
        Eigen::Vector3i neighborIndex;
        neighborIndex.x() = current->index.x() + dx;
        neighborIndex.y() = current->index.y() + dy;
        neighborIndex.z() =
            interpolateZIndex(neighborIndex.x(), neighborIndex.y());
        if (neighborIndex.x() < 1 || neighborIndex.x() >= poolSize_.x() - 1 ||
            neighborIndex.y() < 1 || neighborIndex.y() >= poolSize_.y() - 1 ||
            neighborIndex.z() < 1 || neighborIndex.z() >= poolSize_.z() - 1) {
          continue;
        }

        GridNodePtr neighbor = gridNodeMap_[neighborIndex.x()][neighborIndex.y()]
                                           [neighborIndex.z()];
        neighbor->index = neighborIndex;
        const bool explored = neighbor->rounds == rounds_;
        if (explored && neighbor->state == GridNode::State::ClosedSet) continue;
        neighbor->rounds = rounds_;

        const double yaw = std::atan2(static_cast<double>(dy),
                                      static_cast<double>(dx));
        if (occupancy(indexToCoordinate(neighborIndex), yaw) != 0) continue;

        const int dz = neighborIndex.z() - current->index.z();
        const double stepCost =
            std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz));
        const double tentativeScore = current->gScore + stepCost;
        if (!explored) {
          neighbor->state = GridNode::State::OpenSet;
          neighbor->cameFrom = current;
          neighbor->gScore = tentativeScore;
          neighbor->fScore = tentativeScore + getHeuristic(neighbor, endNode);
          openSet_.push(neighbor);
        } else if (tentativeScore < neighbor->gScore) {
          neighbor->cameFrom = current;
          neighbor->gScore = tentativeScore;
          neighbor->fScore = tentativeScore + getHeuristic(neighbor, endNode);
        }
      }
    }

    const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started);
    if (elapsed.count() > 0.2) return AStarResult::SearchError;
  }
  return AStarResult::SearchError;
}

std::vector<Eigen::Vector3d> AStar::path() const {
  std::vector<Eigen::Vector3d> result;
  result.reserve(gridPath_.size());
  for (GridNodePtr node : gridPath_) result.push_back(indexToCoordinate(node->index));
  std::reverse(result.begin(), result.end());
  return result;
}

int AStar::expandedNodes() const noexcept {
  return expandedNodes_;
}

}  // namespace nav_kernel::local::scan::upstream
