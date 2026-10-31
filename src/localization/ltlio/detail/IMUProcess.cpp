// SPDX-License-Identifier: GPL-2.0-or-later
// Numerical body from Elevator-LIO 1d79af77; original source creator: yczhang.
#include "IMUProcess.h"
#include "Context.h"
namespace lingtu::ltlio::detail {
void IMUProcess::integration(Vector3d IMU_acc, Vector3d IMU_gyr, double time_stamp) {

    /* ---------- 1. 写入环形缓冲 ---------- */
    imu_acc_buffer[head_]  = std::move(IMU_acc);
    imu_gyr_buffer[head_]  = std::move(IMU_gyr);
    time_buffer[head_] = time_stamp;

    /* ---------- 2. 第一帧：初始化 ---------- */
    if (count_ == 0) {
        if (!initialized)initialize(time_stamp);
    }
    else {
        /* 2.1 上一帧索引与 Δt */
        int    prev = (head_ - 1 + MAX_LEN) % MAX_LEN;
        double dt   = time_stamp - time_buffer[prev];

        /* 2.2 梯形法求平均量 */
        Eigen::Vector3d acc_avg = 0.5 * (imu_acc_buffer[prev] + imu_acc_buffer[head_]);
        Eigen::Vector3d gyr_avg = 0.5 * (imu_gyr_buffer[prev] + imu_gyr_buffer[head_]);

        /* 2.3 预测下一状态 */
        State state_next = predictOnce(states_imu[prev] , acc_avg , gyr_avg, dt, time_stamp);

        if (!ctx_.elevator_enable) {
            ctx_.ELEVATOR_TRIGGER = false;
            ctx_.EXIT_FROM_ELEVATOR = false;
        }
        // A stop requests exit; the scan commit performs the coordinate change.
        // Until then, keep propagating the complete cabin + transport state.
        state_next.in_elevator = ctx_.elevator_enable &&
            (ctx_.ELEVATOR_TRIGGER || states_imu[prev].in_elevator);

        if (!states_imu[prev].in_elevator && state_next.in_elevator) {
            elev_process.ElevatorModeEnter(state_next, acc_avg);
        }

        if (states_imu[prev].in_elevator && !ctx_.ELEVATOR_TRIGGER) {
            ctx_.EXIT_FROM_ELEVATOR = true;
        }

        states_imu[head_] = state_next;

        /* 2.4 调试日志  */
    }

    /* ---------- 3. 更新环形指针 ---------- */
    head_  = (head_ + 1) % MAX_LEN;
    count_ = std::min(count_ + 1, MAX_LEN);
}

void IMUProcess::set_init_state(State state_init ,double time_stamp) {
    /* ---------- 填充环形缓冲 ---------- */
    std::fill(states_imu.begin(), states_imu.end(), state_init);
    std::fill(time_buffer.begin(), time_buffer.end(), time_stamp);
    std::fill(imu_acc_buffer.begin(),  imu_acc_buffer.end(),  Eigen::Vector3d::Zero());
    std::fill(imu_gyr_buffer.begin(),  imu_gyr_buffer.end(),  Eigen::Vector3d::Zero());

    /* ---------- 过程噪声 Q (12×12) ---------- */
    Q_.setZero(StateNoiseIndex::NOISE_TOTAL,StateNoiseIndex::NOISE_TOTAL);
    Q_.block<3,3>(StateNoiseIndex::ACC_NOISE,        StateNoiseIndex::ACC_NOISE)         = ctx_.acc_noise_variance       * Eigen::Matrix3d::Identity();
    Q_.block<3,3>(StateNoiseIndex::GYRO_NOISE,       StateNoiseIndex::GYRO_NOISE)        = ctx_.gyro_noise_variance      * Eigen::Matrix3d::Identity();
    Q_.block<3,3>(StateNoiseIndex::ACC_RANDOM_WALK,  StateNoiseIndex::ACC_RANDOM_WALK)   = ctx_.acc_bias_noise_variance  * Eigen::Matrix3d::Identity();
    Q_.block<3,3>(StateNoiseIndex::GYRO_RANDOM_WALK, StateNoiseIndex::GYRO_RANDOM_WALK)  = ctx_.gyro_bias_noise_variance * Eigen::Matrix3d::Identity();
    Q_.block<3,3>(StateNoiseIndex::GRAVITY_NOISE, StateNoiseIndex::GRAVITY_NOISE)  = GRAVITY_NOISE_VAR * Eigen::Matrix3d::Identity();
    Q_(StateNoiseIndex::ELEVATOR_ACC_NOISE, StateNoiseIndex::ELEVATOR_ACC_NOISE)  = ELEVATOR_ACC_NOISE_VAR;

    initialized = true;
}

/**
 * @brief  根据上一状态 + 平均 IMU（body 系） + dt 预测下一状态并传播协方差
 * @param  prev     上一时刻状态（均值 & 协方差）
 * @param  acc_b    本区间平均线加速度  [m/s²]  (body frame, 未去 bias)
 * @param  gyr_b    本区间平均角速度    [rad/s] (body frame, 未去 bias)
 * @param  dt       区间长度           [s]
 * @param  stamp    输出状态的时间戳    [s]
 * @return          预测后的 State
 */
State IMUProcess::predictOnce(const State            &prev,
                              const Eigen::Vector3d  &acc_b,
                              const Eigen::Vector3d  &gyr_b,
                              double                  dt,
                              double                  stamp) const
{
    /* === 0. 复制旧状态，准备写新量 === */
    State nxt = prev;      // bias、g 等直接拷贝
    nxt.time = stamp;      // 最后统一写入

    /* === 1. 姿态更新（四元数左乘） === */
    Eigen::Vector3d dtheta = (gyr_b - prev.bw) * dt;     // 去 gyro-bias
    double th = dtheta.norm();
    Eigen::Quaterniond dq(1.0, 0.0, 0.0, 0.0);
    if (th > 1e-12) {
        Eigen::Vector3d axis = dtheta / th;
        double half = 0.5 * th;
        dq.w()   = std::cos(half);
        dq.vec() = axis * std::sin(half);
    }
    nxt.q = (prev.q * dq).normalized();

    /* === 2. 世界系线加速度 === */
    Eigen::Vector3d acc_w = nxt.q * (acc_b - prev.ba) + prev.g;
    double elevator_acc = prev.az;

    if (prev.in_elevator) {
        const Eigen::Vector3d ezu = ElevatorProcess::ezu_from_g(prev.g);
        if (ctx_.elevator_strong_prior_enable) {
            Eigen::Matrix<double, 1, StateIndex::STATE_TOTAL> H;
            H.setZero();
            H(0, StateIndex::AZ) = 1.0;

            Eigen::Matrix<double, 1, 1> r;
            r(0) = ezu.dot(acc_w) - nxt.az;

            Eigen::Matrix<double, 1, 1> R;
            R(0, 0) = ctx_.elevator_strong_prior_acc_var;
            ekfUpdate<1>(nxt, H, r, R);

            acc_w = nxt.q * (acc_b - nxt.ba) + nxt.g;
            elevator_acc = nxt.az;
        }
        acc_w -= elevator_acc * ezu;
    }

    /* === 3. 位置 & 速度 === */
    nxt.p = prev.p + prev.v * dt + 0.5 * acc_w * dt * dt;
    nxt.v = prev.v + acc_w * dt;

    if (prev.in_elevator) {
        nxt.z  = prev.z + prev.vz * dt + 0.5 * elevator_acc * dt * dt;
        nxt.vz = prev.vz + elevator_acc * dt;
    }

    /* ----------------------------------------------------------------
     *                       协  方  差  传  播
     * ---------------------------------------------------------------- */
    /* === 4. 构建连续时间雅可比 A 和 噪声耦合 U === */
    Eigen::Matrix<double, STATE_TOTAL, STATE_TOTAL> A;
    A.setZero();
    A.block<3,3>(R, R) = -skew3d(gyr_b - prev.bw);
    A.block<3,3>(R, BW) = -Eigen::Matrix3d::Identity();
    A.block<3,3>(P, V) = Eigen::Matrix3d::Identity();
    A.block<3,3>(V, R) = nxt.q.toRotationMatrix() * -skew3d(acc_b - prev.ba);
    A.block<3,3>(V, BA) = -nxt.q.toRotationMatrix();
    if (ctx_.online_gravity_estimation_enable) {
        A.block<3,3>(V, G) = Eigen::Matrix3d::Identity();
    }

    /* --- U (18×15) --- */
    Eigen::Matrix<double, STATE_TOTAL, NOISE_TOTAL> U;
    U.setZero();
    U.block<3,3>(R, GYRO_NOISE)   = -Eigen::Matrix3d::Identity();           // gyro-noise
    U.block<3,3>(V, ACC_NOISE)   = -nxt.q.toRotationMatrix();              // acc-noise
    U.block<3,3>(BW, GYRO_RANDOM_WALK) =  Eigen::Matrix3d::Identity();           // gyro-bias random walk
    U.block<3,3>(BA, ACC_RANDOM_WALK) =  Eigen::Matrix3d::Identity();           // acc-bias random walk
    if (ctx_.online_gravity_estimation_enable) {
        U.block<3,3>(G, GRAVITY_NOISE) = Eigen::Matrix3d::Identity();
    }

    /* === 5. 离散化：F ≈ I + A·dt === */
    Eigen::Matrix<double, STATE_TOTAL, STATE_TOTAL> F =
        Eigen::Matrix<double, STATE_TOTAL, STATE_TOTAL>::Identity() + dt * A;

    /* === 6. 根据是否在电梯中填充 F 和 U 矩阵 === */
    if (prev.in_elevator) {
        const double dt2 = dt * dt;
        // Match the nominal position's acceleration sensitivity for both
        // sensor bias and transport acceleration. Otherwise their cancelling
        // errors create spurious cabin-position information at varying dt.
        F.block<3,3>(P, R) = 0.5 * dt2 * A.block<3,3>(V, R);
        F.block<3,3>(P, BA) = 0.5 * dt2 * A.block<3,3>(V, BA);
        if (ctx_.online_gravity_estimation_enable)
            F.block<3,3>(P, G) = 0.5 * dt2 * A.block<3,3>(V, G);
        F.block<3,1>(P, AZ) = -Eigen::Vector3d(0,0,0.5 * dt2);
        F.block<3,1>(V, AZ) = -Eigen::Vector3d(0,0,dt);
        F(Z, VZ) = dt;
        F(Z, AZ) = 0.5 * dt2;
        F(VZ, AZ) = dt;
        U(AZ, ELEVATOR_ACC_NOISE) = 1.0;
    }

    /* === 7. 协方差传播 === */
    nxt.P = F * prev.P * F.transpose() + U * Q_ * U.transpose() * dt;

    return nxt;
}

/* --------------------------------------------------------------------
 *  IMUProcess::initialize
 *  初始化环形缓冲、初始状态、协方差、过程噪声
 * ------------------------------------------------------------------*/
void IMUProcess::initialize(double t0 )
{
    initialized = true;
    /* ---------- 0. 初始 State ---------- */
    State init;
    init.q  = Eigen::Quaterniond::Identity();
    init.p  = Eigen::Vector3d::Zero();
    init.v  = Eigen::Vector3d::Zero();
    init.bw = Eigen::Vector3d::Zero();
    init.ba = Eigen::Vector3d::Zero();
    init.g  = Eigen::Vector3d(0, 0, -G_m_s2);
    init.z = 0.0;
    init.vz = 0.0;
    init.az = 0.0;
    init.time = t0;

    /* 协方差 P (18×18) */
    init.P.setZero(StateIndex::STATE_TOTAL,StateIndex::STATE_TOTAL);
    init.P.block<3,3>(StateIndex::R,  StateIndex::R)  = Eigen::Matrix3d::Identity() * 1e-4;
    init.P.block<3,3>(StateIndex::P,  StateIndex::P)  = Eigen::Matrix3d::Identity() * 1e-2;
    init.P.block<3,3>(StateIndex::V,  StateIndex::V)  = Eigen::Matrix3d::Identity() * 1e-2;
    init.P.block<3,3>(StateIndex::BW, StateIndex::BW) = Eigen::Matrix3d::Identity() * 1e-6;
    init.P.block<3,3>(StateIndex::BA, StateIndex::BA) = Eigen::Matrix3d::Identity() * 1e-6;
    init.P.block<3,3>(StateIndex::G,  StateIndex::G)  = Eigen::Matrix3d::Identity() * 1e-4;
    init.P(StateIndex::Z, StateIndex::Z) = 1e-3;
    init.P(StateIndex::VZ, StateIndex::VZ) = 1e-2;
    init.P(StateIndex::AZ, StateIndex::AZ) = 1e-1;

    /* ---------- 1. 填充环形缓冲 ---------- */
    std::fill(states_imu.begin(), states_imu.end(), init);
    std::fill(time_buffer.begin(), time_buffer.end(), t0);
    std::fill(imu_acc_buffer.begin(),  imu_acc_buffer.end(),  Eigen::Vector3d::Zero());
    std::fill(imu_gyr_buffer.begin(),  imu_gyr_buffer.end(),  Eigen::Vector3d::Zero());

    /* ---------- 2. 过程噪声 Q (12×12) ---------- */
    Q_.setZero(StateNoiseIndex::NOISE_TOTAL,StateNoiseIndex::NOISE_TOTAL);
    Q_.block<3,3>(StateNoiseIndex::ACC_NOISE,        StateNoiseIndex::ACC_NOISE)         = ctx_.acc_noise_variance       * Eigen::Matrix3d::Identity();
    Q_.block<3,3>(StateNoiseIndex::GYRO_NOISE,       StateNoiseIndex::GYRO_NOISE)        = ctx_.gyro_noise_variance      * Eigen::Matrix3d::Identity();
    Q_.block<3,3>(StateNoiseIndex::ACC_RANDOM_WALK,  StateNoiseIndex::ACC_RANDOM_WALK)   = ctx_.acc_bias_noise_variance  * Eigen::Matrix3d::Identity();
    Q_.block<3,3>(StateNoiseIndex::GYRO_RANDOM_WALK, StateNoiseIndex::GYRO_RANDOM_WALK)  = ctx_.gyro_bias_noise_variance * Eigen::Matrix3d::Identity();
    Q_.block<3,3>(StateNoiseIndex::GRAVITY_NOISE, StateNoiseIndex::GRAVITY_NOISE)  = GRAVITY_NOISE_VAR * Eigen::Matrix3d::Identity();
    Q_(StateNoiseIndex::ELEVATOR_ACC_NOISE, StateNoiseIndex::ELEVATOR_ACC_NOISE) = ELEVATOR_ACC_NOISE_VAR;

}

/* ========= 静态 ofstream 定义 ========= */
inline int IMUProcess::wrapIndex(int idx) const {
    idx %= MAX_LEN;
    return idx < 0 ? idx + MAX_LEN : idx;
}

void IMUProcess::set_state_at_t(State &state_in, double time_stamp) {
    if (!containsTime(time_stamp)) return;

    int index = -1;
    int lower_index = -1;
    for (int i = 0; i < count_; ++i) {
        int time_index = wrapIndex(head_ - 1 - i);
        if (time_buffer[time_index] < time_stamp - 1E-6 ) {
            lower_index = time_index;
            break;
        }
    }
    if (lower_index == -1) {
        // The oldest retained sample need not be at slot zero after wrapping.
        index = wrapIndex(head_ - count_);
    } else {
        const int next_index = wrapIndex(lower_index + 1);
        const bool replace = std::abs(time_buffer[next_index] - time_stamp) < 1E-6;
        index = replace ? next_index : lower_index;
        if (!replace) {
            // The posterior moves this slot to an interpolated scan time.
            // Its IMU readings must move to that time too before reintegration.
            const double ratio = (time_stamp - time_buffer[lower_index]) /
                (time_buffer[next_index] - time_buffer[lower_index]);
            imu_acc_buffer[index] = (1.0 - ratio) * imu_acc_buffer[lower_index] +
                ratio * imu_acc_buffer[next_index];
            imu_gyr_buffer[index] = (1.0 - ratio) * imu_gyr_buffer[lower_index] +
                ratio * imu_gyr_buffer[next_index];
        }
    }
    bool previous_was_in_elevator = states_imu[index].in_elevator;
    states_imu[index] = state_in;
    time_buffer[index] = time_stamp;
    for (int i = 1; wrapIndex(index + i) != head_; ++i) {
        int index_ = wrapIndex(index + i);
        int pre_index_ = wrapIndex(index + i - 1);
        double dt = time_buffer[index_] - time_buffer[pre_index_];

        /* 2.2 梯形法求平均量 */
        Eigen::Vector3d acc_avg = 0.5 * (imu_acc_buffer[pre_index_] + imu_acc_buffer[index_]);
        Eigen::Vector3d gyr_avg = 0.5 * (imu_gyr_buffer[pre_index_] + imu_gyr_buffer[index_]);
        const bool was_in_elevator = states_imu[index_].in_elevator;
        const bool entered_at_sample = !previous_was_in_elevator && was_in_elevator;
        previous_was_in_elevator = was_in_elevator;
        states_imu[index_] = predictOnce(states_imu[pre_index_], acc_avg, gyr_avg, dt, time_buffer[index_]);
        // A correction before entry must not erase an already observed future
        // transition. An exit posterior must not re-enter an old active tail.
        if (entered_at_sample && !states_imu[index_].in_elevator)
            elev_process.ElevatorModeEnter(states_imu[index_], acc_avg);
    }
}

bool IMUProcess::containsTime(double time_stamp) const {
    if (count_ <= 0 || !std::isfinite(time_stamp)) return false;
    const double oldest = time_buffer[wrapIndex(head_ - count_)];
    const double latest = time_buffer[wrapIndex(head_ - 1)];
    return time_stamp >= oldest - 1E-6 && time_stamp <= latest + 1E-6;
}

void IMUProcess::get_state_at_t(State &state_out, double time_stamp) {
    if (count_ <= 0) {

        return;
    }
    state_out = states_imu[wrapIndex(head_ - 1)];

    if (count_ < 2) {
        return;
    }

    const double latest_t = time_buffer[wrapIndex(head_ - 1)];
    const double oldest_t = time_buffer[wrapIndex(head_ - count_)];
    if (std::abs(time_stamp - latest_t) < 1E-6) {
        state_out = states_imu[wrapIndex(head_ - 1)];
        return;
    }
    if (std::abs(time_stamp - oldest_t) < 1E-6) {
        state_out = states_imu[wrapIndex(head_ - count_)];
        return;
    }
    if (time_stamp >= latest_t + 1E-6 ||
        time_stamp <= oldest_t - 1E-6) {

        return;
    }
    int lower_index = -1;
    for (int i = 0; i < count_; ++i) {
        int time_index = wrapIndex(head_ - 1 - i);
        if (time_buffer[time_index] < time_stamp - 1E-6 ) {
            lower_index = time_index;
            break;
        }
    }

    if (lower_index < 0) {
        state_out = states_imu[wrapIndex(head_ - count_)];
        return;
    }
    int next_index = wrapIndex(lower_index + 1);
    bool replace = std::abs(time_buffer[next_index] - time_stamp) < 1E-6;

     if (replace) {
        state_out = states_imu[next_index];
    }else {
        state_out = states_imu[next_index]; // 补丁，不特别关注的量直接复制
        int left_index = lower_index;
        int right_index = next_index;
        double t_left = time_buffer[left_index];
        double t_right = time_buffer[right_index];
        const State& left_state = states_imu[left_index];
        const State& right_state = states_imu[right_index];
        if (std::abs(t_right - t_left) < 1E-9) {
            state_out = right_state;
            return;
        }
        double ratio = (time_stamp - t_left) / (t_right - t_left);
        // Propagate mean and covariance to the same query time and tangent frame.
        // Mode changes and their covariance resets still occur at the IMU sample.
        const Eigen::Vector3d acc_at_t = (1.0 - ratio) * imu_acc_buffer[left_index] +
            ratio * imu_acc_buffer[right_index];
        const Eigen::Vector3d gyr_at_t = (1.0 - ratio) * imu_gyr_buffer[left_index] +
            ratio * imu_gyr_buffer[right_index];
        state_out = predictOnce(left_state,
            0.5 * (imu_acc_buffer[left_index] + acc_at_t),
            0.5 * (imu_gyr_buffer[left_index] + gyr_at_t),
            time_stamp - t_left, time_stamp);
    }
}

inline Pose IMUProcess::get_pose(int index , double base_time) {
    Pose IMU_Pose;
    State state_ = states_imu[index];
    IMU_Pose.q = state_.q;
    IMU_Pose.p = state_.p;
    IMU_Pose.v = state_.v;
    Eigen::Vector3d acc_world = state_.q * (imu_acc_buffer[index] - state_.ba) + state_.g;
    Eigen::Vector3d gyr = imu_gyr_buffer[index];
    IMU_Pose.acc = acc_world ;
    IMU_Pose.gyr = gyr - state_.bw;
    IMU_Pose.offset_time = time_buffer[index] - base_time;
    return IMU_Pose;
}

void IMUProcess::get_imu_pose_from_t1_to_t2(std::vector<Pose> &IMU_Pose_vec, double time_stamp_1, double time_stamp_2) {
    IMU_Pose_vec.clear();
    if (time_stamp_1 > time_stamp_2) {

        double time_temp = time_stamp_1;
        time_stamp_1 = time_stamp_2;
        time_stamp_2 = time_temp;
    }
    double min_time = time_buffer[wrapIndex(head_ - count_)];
    double max_time = time_buffer[wrapIndex(head_ - 1)];
    if (time_stamp_1 < min_time - 1E-6 || time_stamp_2 > max_time + 1E-6) {

        return;
    }
    int left_index = -1,  right_index = -1 , i ;
    
    for (i = 0; i < count_; ++i) { // 第一次查找小于 time_stamp_2 的索引
        int time_index = wrapIndex(head_ - 1 - i);
        if (time_buffer[time_index] < time_stamp_2 - 1E-6 ) {
            right_index = time_index;
            break;
        }
    }
    
    for (; i < count_; ++i) { // 第二次查找小于 time_stamp_1 的索引
        int time_index = wrapIndex(head_ - 1 - i);
        if (time_buffer[time_index] < time_stamp_1 - 1E-6 ) {
            left_index = time_index;
            break;
        }
    }
    if (right_index == -1) {

        return;
    }
    State state_begin;
    // A scan may start exactly at the oldest retained sample. There is no
    // preceding sample in that case, but the start itself is fully covered.
    int next_index = left_index == -1 ? wrapIndex(head_ - count_) : wrapIndex(left_index + 1);
    bool replace = std::abs(time_buffer[next_index] - time_stamp_1) < 1E-6;
    if (left_index == -1 && !replace) return;

    if (replace) {
        for (int j = next_index; j != wrapIndex(right_index + 1); j = wrapIndex(j+1)) {
            IMU_Pose_vec.push_back(get_pose(j,time_stamp_1));
        }
    }else {
        double ratio = (time_stamp_1 - time_buffer[left_index]) /
                       (time_buffer[next_index] - time_buffer[left_index]);
        get_state_at_t(state_begin, time_stamp_1);

        Eigen::Vector3d imu_acc_begin = (1.0 - ratio) * imu_acc_buffer[left_index] + ratio * imu_acc_buffer[next_index];
        Eigen::Vector3d imu_gyr_begin = (1.0 - ratio) * imu_gyr_buffer[left_index] + ratio * imu_gyr_buffer[next_index];

        Pose IMU_Pose;
        IMU_Pose.q = state_begin.q;
        IMU_Pose.p = state_begin.p;
        IMU_Pose.v = state_begin.v;
        Eigen::Vector3d acc_world = state_begin.q * (imu_acc_begin - state_begin.ba) + state_begin.g;
        Eigen::Vector3d gyr = imu_gyr_begin;
        IMU_Pose.acc = acc_world ;
        IMU_Pose.gyr = gyr - state_begin.bw;
        IMU_Pose.offset_time = 0;

        IMU_Pose_vec.push_back(IMU_Pose);
        for (int j = wrapIndex(left_index + 1); j != wrapIndex(right_index + 1); j = wrapIndex(j+1)) {
            IMU_Pose_vec.push_back(get_pose(j,time_stamp_1));
        }
    }
    State state_end;
    next_index = wrapIndex(right_index + 1);
    replace = std::abs(time_buffer[next_index] - time_stamp_2) < 1E-6;

    if (replace) {
        IMU_Pose_vec.push_back(get_pose(next_index,time_stamp_1));
    }else {
        double ratio = (time_stamp_2 - time_buffer[right_index]) /
                      (time_buffer[next_index] - time_buffer[right_index]);
        get_state_at_t(state_end, time_stamp_2);

        Eigen::Vector3d imu_acc_end = (1.0 - ratio) * imu_acc_buffer[right_index] + ratio * imu_acc_buffer[next_index];
        Eigen::Vector3d imu_gyr_end = (1.0 - ratio) * imu_gyr_buffer[right_index] + ratio * imu_gyr_buffer[next_index];

        Pose IMU_Pose;
        IMU_Pose.q = state_end.q;
        IMU_Pose.p = state_end.p;
        IMU_Pose.v = state_end.v;
        Eigen::Vector3d acc_world = state_end.q * (imu_acc_end - state_end.ba) + state_end.g;
        Eigen::Vector3d gyr = imu_gyr_end;
        IMU_Pose.acc = acc_world ;
        IMU_Pose.gyr = gyr - state_end.bw;
        IMU_Pose.offset_time = time_stamp_2 - time_stamp_1;

        IMU_Pose_vec.push_back(IMU_Pose);
    }
}

void IMUProcess::resetInitialization() {
    init_acc_buf_.clear();
    init_gyr_buf_.clear();
}

bool IMUProcess::init(const ::lingtu::ltlio::ImuSample &msg, int init_imu_num,
                      Eigen::Vector3d &mean_acc_, Eigen::Vector3d &mean_gyr_,
                      State &state) {
    Eigen::Vector3d cur_acc = msg.acceleration;
    Eigen::Vector3d cur_gyr = msg.angular_velocity_rps;

    init_acc_buf_.push_back(cur_acc);
    init_gyr_buf_.push_back(cur_gyr);

    if (init_acc_buf_.size() < init_imu_num) {
        return false;
    }

    Eigen::Vector3d mean_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_gyr = Eigen::Vector3d::Zero();
    for (const auto &acc : init_acc_buf_) mean_acc += acc;
    for (const auto &gyr : init_gyr_buf_) mean_gyr += gyr;
    mean_acc /= init_acc_buf_.size();
    mean_gyr /= init_gyr_buf_.size();
    if (ctx_.imu_static_acc_thresh > 0.0 || ctx_.imu_static_gyro_thresh > 0.0) {
        double acc_variance = 0.0;
        double gyro_variance = 0.0;
        for (const auto& acc : init_acc_buf_)
            acc_variance += ((acc - mean_acc) * ctx_.acc_scale).squaredNorm();
        for (const auto& gyro : init_gyr_buf_)
            gyro_variance += (gyro - mean_gyr).squaredNorm();
        acc_variance /= static_cast<double>(init_acc_buf_.size());
        gyro_variance /= static_cast<double>(init_gyr_buf_.size());
        if ((ctx_.imu_static_acc_thresh > 0.0 &&
             acc_variance >= ctx_.imu_static_acc_thresh * ctx_.imu_static_acc_thresh) ||
            (ctx_.imu_static_gyro_thresh > 0.0 &&
             gyro_variance >= ctx_.imu_static_gyro_thresh * ctx_.imu_static_gyro_thresh)) {
            init_acc_buf_.clear();
            init_gyr_buf_.clear();
            return false;
        }
    }
    mean_acc *= ctx_.acc_scale;
    mean_acc_ = mean_acc;
    mean_gyr_ = mean_gyr;

    state.bw = mean_gyr_;
    double gravity_norm = mean_acc.norm();
    state.g = Eigen::Vector3d(0, 0, -gravity_norm);
    state.q = Eigen::Quaterniond::FromTwoVectors(-mean_acc, state.g);
    state.ba = Eigen::Vector3d::Zero();
    state.p = Eigen::Vector3d::Zero();
    state.v = Eigen::Vector3d::Zero();
    state.z = 0.0;
    state.vz = 0.0;
    state.az = 0.0;
    state.in_elevator = false;

    state.P.setZero(StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL);
    state.P.block<3, 3>(StateIndex::R, StateIndex::R) = Eigen::Matrix3d::Identity() * 1e-3;
    state.P.block<3, 3>(StateIndex::P, StateIndex::P) = Eigen::Matrix3d::Identity() * 1e-2;
    state.P.block<3, 3>(StateIndex::V, StateIndex::V) = Eigen::Matrix3d::Identity() * 1e-2;
    state.P.block<3, 3>(StateIndex::BW, StateIndex::BW) = Eigen::Matrix3d::Identity() * 1e-4;
    state.P.block<3, 3>(StateIndex::BA, StateIndex::BA) = Eigen::Matrix3d::Identity() * 1e-2;
    state.P.block<3, 3>(StateIndex::G, StateIndex::G) = Eigen::Matrix3d::Identity() * 1e-3;
    state.P(StateIndex::Z, StateIndex::Z) = 1e-3;
    state.P(StateIndex::VZ, StateIndex::VZ) = 1e-2;
    state.P(StateIndex::AZ, StateIndex::AZ) = 1e-1;

    init_acc_buf_.clear();
    init_gyr_buf_.clear();

    return true;
}

} // namespace lingtu::ltlio::detail
