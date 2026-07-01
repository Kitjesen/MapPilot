use nalgebra::{Matrix3, Quaternion, SVector, UnitQuaternion, Vector3};

pub type Vector6 = SVector<f64, 6>;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose3 {
    pub t_xyz: [f64; 3],
    pub q_wxyz: [f64; 4],
}

impl Pose3 {
    pub fn identity() -> Self {
        Self {
            t_xyz: [0.0, 0.0, 0.0],
            q_wxyz: [1.0, 0.0, 0.0, 0.0],
        }
    }

    pub fn try_new(t_xyz: [f64; 3], q_wxyz: [f64; 4]) -> Option<Self> {
        if !t_xyz.iter().all(|value| value.is_finite())
            || !q_wxyz.iter().all(|value| value.is_finite())
        {
            return None;
        }
        let norm = q_wxyz.iter().map(|value| value * value).sum::<f64>().sqrt();
        if norm <= 1e-12 {
            return None;
        }
        let mut q = [
            q_wxyz[0] / norm,
            q_wxyz[1] / norm,
            q_wxyz[2] / norm,
            q_wxyz[3] / norm,
        ];
        if q[0] < 0.0 {
            for value in &mut q {
                *value = -*value;
            }
        }
        Some(Self { t_xyz, q_wxyz: q })
    }

    pub fn from_parts(translation: Vector3<f64>, rotation: UnitQuaternion<f64>) -> Self {
        let q = rotation.into_inner();
        Self::try_new(
            [translation.x, translation.y, translation.z],
            [q.w, q.i, q.j, q.k],
        )
        .expect("unit quaternion must produce a valid pose")
    }

    pub fn is_finite(self) -> bool {
        self.t_xyz.iter().all(|value| value.is_finite())
            && self.q_wxyz.iter().all(|value| value.is_finite())
    }

    pub fn translation(self) -> Vector3<f64> {
        Vector3::new(self.t_xyz[0], self.t_xyz[1], self.t_xyz[2])
    }

    pub fn rotation(self) -> UnitQuaternion<f64> {
        UnitQuaternion::new_normalize(Quaternion::new(
            self.q_wxyz[0],
            self.q_wxyz[1],
            self.q_wxyz[2],
            self.q_wxyz[3],
        ))
    }

    pub fn compose(self, rhs: Self) -> Self {
        let rotation = self.rotation();
        let composed_rotation = rotation * rhs.rotation();
        let composed_translation =
            self.translation() + rotation.transform_vector(&rhs.translation());
        Self::from_parts(composed_translation, composed_rotation)
    }

    pub fn inverse(self) -> Self {
        let rotation_inv = self.rotation().inverse();
        let translation_inv = rotation_inv.transform_vector(&(-self.translation()));
        Self::from_parts(translation_inv, rotation_inv)
    }

    pub fn between(self, rhs: Self) -> Self {
        self.inverse().compose(rhs)
    }

    pub fn exp(delta: &Vector6) -> Self {
        let omega = Vector3::new(delta[0], delta[1], delta[2]);
        let upsilon = Vector3::new(delta[3], delta[4], delta[5]);
        let rotation = UnitQuaternion::from_scaled_axis(omega);
        let translation = left_jacobian_so3(&omega) * upsilon;
        Self::from_parts(translation, rotation)
    }

    pub fn log(self) -> Vector6 {
        let omega = self.rotation().scaled_axis();
        let upsilon = left_jacobian_so3_inverse(&omega) * self.translation();
        Vector6::from_row_slice(&[omega.x, omega.y, omega.z, upsilon.x, upsilon.y, upsilon.z])
    }

    pub fn retract(self, delta: &Vector6) -> Self {
        self.compose(Self::exp(delta))
    }
}

pub fn residual_between(measurement: Pose3, from: Pose3, to: Pose3) -> Vector6 {
    measurement.inverse().compose(from.between(to)).log()
}

pub fn residual_prior(measurement: Pose3, pose: Pose3) -> Vector6 {
    measurement.inverse().compose(pose).log()
}

fn skew(vector: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -vector.z, vector.y, vector.z, 0.0, -vector.x, -vector.y, vector.x, 0.0,
    )
}

fn left_jacobian_so3(omega: &Vector3<f64>) -> Matrix3<f64> {
    let theta = omega.norm();
    let omega_hat = skew(omega);
    let omega_hat2 = omega_hat * omega_hat;
    if theta < 1e-8 {
        Matrix3::identity() + 0.5 * omega_hat + (1.0 / 6.0) * omega_hat2
    } else {
        let theta2 = theta * theta;
        Matrix3::identity()
            + ((1.0 - theta.cos()) / theta2) * omega_hat
            + ((theta - theta.sin()) / (theta2 * theta)) * omega_hat2
    }
}

fn left_jacobian_so3_inverse(omega: &Vector3<f64>) -> Matrix3<f64> {
    let theta = omega.norm();
    let omega_hat = skew(omega);
    let omega_hat2 = omega_hat * omega_hat;
    if theta < 1e-8 {
        Matrix3::identity() - 0.5 * omega_hat + (1.0 / 12.0) * omega_hat2
    } else {
        let theta2 = theta * theta;
        let half_theta = 0.5 * theta;
        let coefficient = (1.0 / theta2) * (1.0 - half_theta * theta.sin() / (1.0 - theta.cos()));
        Matrix3::identity() - 0.5 * omega_hat + coefficient * omega_hat2
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_vec6_near(a: Vector6, b: Vector6, tolerance: f64) {
        for idx in 0..6 {
            assert!(
                (a[idx] - b[idx]).abs() <= tolerance,
                "idx {idx}: {} vs {}",
                a[idx],
                b[idx]
            );
        }
    }

    #[test]
    fn se3_exp_log_roundtrip() {
        let delta = Vector6::from_row_slice(&[0.1, -0.2, 0.05, 1.0, -0.4, 0.2]);
        let pose = Pose3::exp(&delta);
        assert_vec6_near(pose.log(), delta, 1e-9);
    }

    #[test]
    fn se3_small_angle_roundtrip() {
        let delta = Vector6::from_row_slice(&[1e-10, -2e-10, 3e-10, 0.2, -0.1, 0.05]);
        let pose = Pose3::exp(&delta);
        assert_vec6_near(pose.log(), delta, 1e-8);
    }

    #[test]
    fn compose_inverse_returns_identity() {
        let delta = Vector6::from_row_slice(&[0.3, 0.1, -0.2, 2.0, 1.0, -0.5]);
        let pose = Pose3::exp(&delta);
        let identity = pose.compose(pose.inverse());
        assert_vec6_near(identity.log(), Vector6::zeros(), 1e-9);
    }

    #[test]
    fn between_matches_composed_delta() {
        let base = Pose3::exp(&Vector6::from_row_slice(&[0.1, 0.0, 0.2, 1.0, 2.0, 0.5]));
        let delta = Pose3::exp(&Vector6::from_row_slice(&[0.0, 0.0, 0.3, 2.0, 0.0, 0.0]));
        let target = base.compose(delta);
        assert_vec6_near(base.between(target).log(), delta.log(), 1e-9);
    }
}
