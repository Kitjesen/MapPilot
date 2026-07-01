// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss.
// Modified by Daehan Lee, Hyungtae Lim, and Soohee Han, 2024
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "GenZICP.hpp"

#include <Eigen/Core>
#include <tuple>
#include <vector>

#include "genz_icp/core/Deskew.hpp"
#include "genz_icp/core/Preprocessing.hpp"
#include "genz_icp/core/Registration.hpp"
#include "genz_icp/core/VoxelHashMap.hpp"

namespace genz_icp::pipeline {

GenZICP::Vector3dVectorTuple GenZICP::RegisterFrame(const std::vector<Eigen::Vector3d> &frame,
                                                    const std::vector<double> &timestamps) {
    const auto &deskew_frame = [&]() -> std::vector<Eigen::Vector3d> {
        if (!config_.deskew || timestamps.empty()) return frame;
        // TODO(Nacho) Add some asserts here to sanitize the timestamps

        //  If not enough poses for the estimation, do not de-skew
        const size_t N = poses().size();
        if (N <= 2) return frame;

        // Estimate linear and angular velocities
        const auto &start_pose = poses_[N - 2];
        const auto &finish_pose = poses_[N - 1];

        return DeSkewScan(frame, timestamps, start_pose, finish_pose);
        
    }();
    return RegisterFrame(deskew_frame);
}

GenZICP::Vector3dVectorTuple GenZICP::RegisterFrame(const std::vector<Eigen::Vector3d> &frame) {
    // Preprocess the input cloud
    const auto &cropped_frame = Preprocess(frame, config_.max_range, config_.min_range);

    // Adapt voxel size based on LOCUS 2.0's adaptive voxel grid filter
    static double voxel_size = config_.voxel_size; // Initial voxel size
    const auto source_tmp = genz_icp::VoxelDownsample(cropped_frame, voxel_size);
    double adaptive_voxel_size = genz_icp::Clamp(voxel_size * static_cast<double>(source_tmp.size()) / static_cast<double>(config_.desired_num_voxelized_points), 0.02, 2.0);

    // Re-voxelize using the adaptive voxel size
    const auto &[source, frame_downsample] = Voxelize(cropped_frame, adaptive_voxel_size);
    voxel_size = adaptive_voxel_size; // Save for the next frame

    // Get motion prediction and adaptive_threshold
    const double sigma = GetAdaptiveThreshold();

    // Compute initial_guess for ICP
    const auto prediction = GetPredictionModel();
    const auto last_pose = !poses_.empty() ? poses_.back() : Sophus::SE3d();
    const auto initial_guess = last_pose * prediction;

    // Run GenZ-ICP
    const auto &[new_pose, planar_points, non_planar_points] = registration_.RegisterFrame(source,         //
                                                          local_map_,     //
                                                          initial_guess,  //
                                                          3.0 * sigma,    //
                                                          sigma / 3.0);
    const auto model_deviation = initial_guess.inverse() * new_pose;
    adaptive_threshold_.UpdateModelDeviation(model_deviation);
    local_map_.Update(frame_downsample, new_pose);
    poses_.push_back(new_pose);
    return {planar_points, non_planar_points};
}

GenZICP::Vector3dVectorTuple GenZICP::Voxelize(const std::vector<Eigen::Vector3d> &frame, double adaptive_voxel_size) const {
    const auto frame_downsample = genz_icp::VoxelDownsample(frame, std::max(adaptive_voxel_size * 0.5, 0.02)); // localmap update
    const auto source = genz_icp::VoxelDownsample(frame_downsample, adaptive_voxel_size * 1.0); // registration
    return {source, frame_downsample};
}

double GenZICP::GetAdaptiveThreshold() {
    if (!HasMoved()) {
        return config_.initial_threshold;
    }
    return adaptive_threshold_.ComputeThreshold();
}

Sophus::SE3d GenZICP::GetPredictionModel() const {
    Sophus::SE3d pred = Sophus::SE3d();
    const size_t N = poses_.size();
    if (N < 2) return pred;
    return poses_[N - 2].inverse() * poses_[N - 1];
}

bool GenZICP::HasMoved() {
    if (poses_.empty()) return false;
    const double motion = (poses_.front().inverse() * poses_.back()).translation().norm();
    return motion > 5.0 * config_.min_motion_th;
}

}  // namespace genz_icp::pipeline
