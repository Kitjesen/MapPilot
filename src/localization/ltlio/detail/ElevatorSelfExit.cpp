
#include "ElevatorSelfExit.h"
#include "ElevatorProcess.h"
#include "ESEKF.h"
#include "IMUProcess.h"
namespace lingtu::ltlio::detail {
/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: MIT
 *
 * Elevator self-exit state machine implementation.
 */

double WindowStdDev::update(double value) {
    hist.push_back(value);
    sum += value;
    sq_sum += value * value;
    if (hist.size() > kWinSize) {
        double oldest = hist.front();
        sum -= oldest;
        sq_sum -= oldest * oldest;
        hist.pop_front();
    }
    if (hist.size() == kWinSize) {
        double mean = sum / kWinSize;
        double variance = (sq_sum / kWinSize) - (mean * mean);
        return std::max(0.0, variance);
    }
    return 0.0;
}

void WindowStdDev::clear() {
    sum = 0.0;
    sq_sum = 0.0;
    hist.clear();
}

ElevatorSelfExit::ElevatorSelfExit(Context& context) : ctx_(context) {}

ElevatorSelfExit::ProcessResult ElevatorSelfExit::process(const ImuFrame& frame, State& state) {
    ProcessResult result;
    if (!ctx_.elevator_enable) {
        reset();
        return result;
    }

    // A duplicate or stale sample is not a new detector observation.  Keep
    // diagnostics, the rolling window, phase and hold timers unchanged.
    if (last_exit_eval_time_ >= 0.0 && frame.timestamp <= last_exit_eval_time_)
        return result;

    const double eval_dt = (last_exit_eval_time_ > 0.0)
        ? frame.timestamp - last_exit_eval_time_
        : 0.0;
    last_exit_eval_time_ = frame.timestamp;

    const bool variance_window_was_full = rolling_std_.full();
    debug_.acc_z = frame.acc_z;
    debug_.vel_z = frame.vel_z;
    debug_.std_dev = rolling_std_.update(frame.vel_z);

    if (!frame.in_elevator) {
        ele_state_ = IDLE;
        stable_tick_ = 0;
        low_var_hold_s_ = 0.0;
        stop_check_lidar_zupt_count_ = 0;
        stop_check_confirmed_ = false;
        last_waiting_zupt_time_ = -std::numeric_limits<double>::infinity();
        debug_.state = static_cast<int>(ele_state_);
        return result;
    }
    const double kWaitingVzAbsThresh = ctx_.elevator_waiting_zupt_vz_abs_thresh;
    const double kWaitingAzAbsThresh = ctx_.elevator_waiting_zupt_az_abs_thresh;
    const double kWaitingZuptPeriodS = ctx_.elevator_waiting_zupt_period_s;

    const bool waiting_like =
        std::abs(frame.vel_z) < kWaitingVzAbsThresh &&
        std::abs(state.az) < kWaitingAzAbsThresh;

    if (ctx_.elevator_zupt_enable &&
        ctx_.elevator_waiting_zupt_enable &&
        ele_state_ == IDLE &&
        waiting_like &&
        (frame.timestamp - last_waiting_zupt_time_) >= kWaitingZuptPeriodS) {
        result.apply_waiting_zupt = true;
        last_waiting_zupt_time_ = frame.timestamp;
    }
    if (!ctx_.self_exit_detector) return result;

    // A cleared window has no variance evidence. In particular, update()
    // returns zero while refilling, which must not be mistaken for a stopped
    // elevator after an IMU gap.
    if (!rolling_std_.full()) {
        debug_.state = static_cast<int>(ele_state_);
        return result;
    }

    const double kVelThresh = 0.2;
    const double kStopVelProtect = 0.5;
    const double kStdHigh = 0.01;
    const double kStdLow = 0.01;
    const double kConfirmHoldS = 1.5;

    const double abs_vel = std::abs(frame.vel_z);
    const double current_stdDev = debug_.std_dev;

    // A gap can hide the complete deceleration interval. If an active ride's
    // first rebuilt window is already strictly waiting-like, resume at stop
    // verification instead of requiring the elevator to move again. The
    // fresh 1.5 s hold still starts only after this completed window.
    if (!variance_window_was_full && ele_state_ != IDLE &&
        current_stdDev < kStdLow && waiting_like) {
        ele_state_ = STOP_CHECK;
        stable_tick_ = 0;
        low_var_hold_s_ = 0.0;
        stop_check_lidar_zupt_count_ = 0;
        stop_check_confirmed_ = false;
    }

    switch (ele_state_) {
    case IDLE:
        if (abs_vel > kVelThresh && current_stdDev > kStdHigh) {
            ele_state_ = ACCELERATING;
            ;
        }
        break;

    case ACCELERATING:
        if (abs_vel > kVelThresh && current_stdDev < kStdLow) {
            ele_state_ = CONSTANT_SPEED;
            ;
        }
        break;

    case CONSTANT_SPEED:
        if (current_stdDev > kStdHigh) {
            ele_state_ = DECELERATING;
            ;
        }
        break;

    case DECELERATING:
        if (current_stdDev < kStdLow && abs_vel < kStopVelProtect) {
            ele_state_ = STOP_CHECK;
            stable_tick_ = 0;
            low_var_hold_s_ = 0.0;
            stop_check_lidar_zupt_count_ = 0;
            stop_check_confirmed_ = false;
            ;
        }
        break;

    case STOP_CHECK:
        if (current_stdDev < kStdLow && abs_vel < kStopVelProtect) {
            stable_tick_++;
            // The sample which first completes a fresh variance window starts
            // the hold. Only later contiguous intervals count toward 1.5 s.
            if (variance_window_was_full) low_var_hold_s_ += eval_dt;
            if (low_var_hold_s_ >= kConfirmHoldS && !stop_check_confirmed_) {
                stop_check_confirmed_ = true;
                stop_check_lidar_zupt_count_ = 0;
                result.stop_confirmed = true;
                ;
            }
        } else {
            stable_tick_ = 0;
            low_var_hold_s_ = 0.0;
            stop_check_confirmed_ = false;
            stop_check_lidar_zupt_count_ = 0;
            if (current_stdDev > kStdHigh) {
                ele_state_ = DECELERATING;
                ;
            }
        }
        break;
    }

    debug_.state = static_cast<int>(ele_state_);

    return result;
}

void ElevatorSelfExit::invalidateImuContinuity() {
    stable_tick_ = 0;
    low_var_hold_s_ = 0.0;
    stop_check_lidar_zupt_count_ = 0;
    stop_check_confirmed_ = false;
    last_waiting_zupt_time_ = -std::numeric_limits<double>::infinity();
    last_exit_eval_time_ = -1.0;
    rolling_std_.clear();
    debug_.std_dev = 0.0;
    debug_.state = static_cast<int>(ele_state_);
}

void ElevatorSelfExit::reset() {
    ele_state_ = IDLE;
    stable_tick_ = 0;
    low_var_hold_s_ = 0.0;
    stop_check_lidar_zupt_count_ = 0;
    stop_check_confirmed_ = false;
    last_waiting_zupt_time_ = -std::numeric_limits<double>::infinity();
    last_exit_eval_time_ = -1.0;
    rolling_std_.clear();
    debug_ = DebugInfo{};
}

} // namespace lingtu::ltlio::detail
