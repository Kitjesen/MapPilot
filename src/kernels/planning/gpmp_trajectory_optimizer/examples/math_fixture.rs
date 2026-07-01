use lingtu_gpmp_trajectory_optimizer::{wnoa, wnoj, Mat4, Mat6, Row6, Vec4, Vec6};

fn f64_json(value: f64) -> String {
    if value.is_finite() {
        format!("{value:.17}")
    } else {
        "null".to_string()
    }
}

fn vec_json(values: &[f64]) -> String {
    let items = values
        .iter()
        .map(|value| f64_json(*value))
        .collect::<Vec<_>>();
    format!("[{}]", items.join(","))
}

fn mat4_json(matrix: &Mat4) -> String {
    let rows = (0..4)
        .map(|row| vec_json(&(0..4).map(|col| matrix[(row, col)]).collect::<Vec<f64>>()))
        .collect::<Vec<_>>();
    format!("[{}]", rows.join(","))
}

fn mat6_json(matrix: &Mat6) -> String {
    let rows = (0..6)
        .map(|row| vec_json(&(0..6).map(|col| matrix[(row, col)]).collect::<Vec<f64>>()))
        .collect::<Vec<_>>();
    format!("[{}]", rows.join(","))
}

fn row6_json(row: &Row6) -> String {
    vec_json(&(0..6).map(|col| row[(0, col)]).collect::<Vec<f64>>())
}

fn wnoj_case(case_name: &str, x1: Vec6, x2: Vec6, max_heading_rate: f64) -> String {
    let qc = 0.1;
    let delta = 1.0;
    let tau = 0.35;
    let sub_sample_num = 2;
    let (lambda, psi) = wnoj::lambda_psi(qc, delta, tau);
    let interpolated_state = wnoj::interpolate(&x1, &x2, qc, delta, tau);
    let interpolated_heading_rate = wnoj::heading_rate(&interpolated_state);
    let (interpolated_heading_rate_residual, jac_x1, jac_x2) =
        wnoj::interpolate_heading_rate_residual_jacobians(
            &x1,
            &x2,
            qc,
            delta,
            tau,
            max_heading_rate,
        );
    let path_point_state = wnoj::state_from_path_point(2.0, 3.0, std::f64::consts::FRAC_PI_4, 0.2);
    let trajectory_tau_1 = delta / (sub_sample_num as f64 + 1.0);
    let trajectory_tau_2 = 2.0 * trajectory_tau_1;
    let prior_jacobian_x2 = -Mat6::identity();

    format!(
        concat!(
            "{{",
            "\"case\":\"{case}\",",
            "\"mode\":\"wnoj\",",
            "\"state_order\":[\"x\",\"vx\",\"ax\",\"y\",\"vy\",\"ay\"],",
            "\"input\":{{\"qc\":{qc},\"delta\":{delta},\"tau\":{tau},",
            "\"max_heading_rate\":{max_heading_rate},",
            "\"sub_sample_num\":{sub_sample_num},",
            "\"x1\":{x1_json},\"x2\":{x2_json},",
            "\"path_point\":{{\"x\":2.0,\"y\":3.0,\"heading\":0.78539816339744828,\"reference_velocity\":0.2}}}},",
            "\"output\":{{",
            "\"q\":{q},\"q_inverse\":{q_inverse},\"phi\":{phi},",
            "\"q_delta\":{q},\"q_inverse_delta\":{q_inverse},\"phi_delta\":{phi},",
            "\"q_tau\":{q_tau},\"phi_tau\":{phi_tau},",
            "\"phi_delta_minus_tau\":{phi_delta_minus_tau},",
            "\"lambda\":{lambda},\"psi\":{psi},",
            "\"prior_residual\":{prior_residual},",
            "\"prior_jacobian_x1\":{prior_jacobian_x1},",
            "\"prior_jacobian_x2\":{prior_jacobian_x2},",
            "\"interpolated_state\":{interpolated_state},",
            "\"interpolate_jacobian_x1\":{lambda},",
            "\"interpolate_jacobian_x2\":{psi},",
            "\"heading_rate\":{heading_rate},",
            "\"heading_rate_residual\":{heading_rate_residual},",
            "\"heading_rate_jacobian\":{heading_rate_jacobian},",
            "\"interpolated_heading_rate\":{interpolated_heading_rate},",
            "\"interpolated_heading_rate_residual\":{interpolated_heading_rate_residual},",
            "\"interpolated_heading_jacobian_x1\":{jac_x1},",
            "\"interpolated_heading_jacobian_x2\":{jac_x2},",
            "\"path_point_state\":{path_point_state},",
            "\"trajectory\":{trajectory}",
            "}}",
            "}}"
        ),
        case = case_name,
        qc = f64_json(qc),
        delta = f64_json(delta),
        tau = f64_json(tau),
        max_heading_rate = f64_json(max_heading_rate),
        sub_sample_num = sub_sample_num,
        x1_json = vec_json(x1.as_slice()),
        x2_json = vec_json(x2.as_slice()),
        q = mat6_json(&wnoj::q(qc, delta)),
        q_inverse = mat6_json(&wnoj::q_inverse(qc, delta)),
        phi = mat6_json(&wnoj::phi(delta)),
        q_tau = mat6_json(&wnoj::q(qc, tau)),
        phi_tau = mat6_json(&wnoj::phi(tau)),
        phi_delta_minus_tau = mat6_json(&wnoj::phi(delta - tau)),
        lambda = mat6_json(&lambda),
        psi = mat6_json(&psi),
        prior_residual = vec_json(wnoj::prior_residual(&x1, &x2, delta).as_slice()),
        prior_jacobian_x1 = mat6_json(&wnoj::phi(delta)),
        prior_jacobian_x2 = mat6_json(&prior_jacobian_x2),
        interpolated_state = vec_json(interpolated_state.as_slice()),
        heading_rate = f64_json(wnoj::heading_rate(&x1)),
        heading_rate_residual = f64_json(wnoj::heading_rate_residual(&x1, max_heading_rate)),
        heading_rate_jacobian = row6_json(&wnoj::heading_rate_jacobian(&x1, max_heading_rate)),
        interpolated_heading_rate = f64_json(interpolated_heading_rate),
        interpolated_heading_rate_residual = f64_json(interpolated_heading_rate_residual),
        jac_x1 = row6_json(&jac_x1),
        jac_x2 = row6_json(&jac_x2),
        path_point_state = vec_json(path_point_state.as_slice()),
        trajectory = format!(
            "[{},{},{},{}]",
            vec_json(x1.as_slice()),
            vec_json(wnoj::interpolate(&x1, &x2, qc, delta, trajectory_tau_1).as_slice()),
            vec_json(wnoj::interpolate(&x1, &x2, qc, delta, trajectory_tau_2).as_slice()),
            vec_json(x2.as_slice()),
        ),
    )
}

fn wnoa_case() -> String {
    let qc = 0.1;
    let delta = 1.0;
    let tau = 0.35;
    let sub_sample_num = 2;
    let x1 = Vec4::from_row_slice(&[0.2, 1.1, -0.3, 0.7]);
    let x2 = Vec4::from_row_slice(&[0.9, 1.0, 0.4, 0.8]);
    let (lambda, psi) = wnoa::lambda_psi(qc, delta, tau);
    let path_point_state = wnoa::state_from_path_point(2.0, 3.0, std::f64::consts::FRAC_PI_4, 2.0);
    let trajectory_tau_1 = delta / (sub_sample_num as f64 + 1.0);
    let trajectory_tau_2 = 2.0 * trajectory_tau_1;
    let prior_jacobian_x2 = -Mat4::identity();

    format!(
        concat!(
            "{{",
            "\"case\":\"wnoa_nominal\",",
            "\"mode\":\"wnoa\",",
            "\"state_order\":[\"x\",\"vx\",\"y\",\"vy\"],",
            "\"input\":{{\"qc\":{qc},\"delta\":{delta},\"tau\":{tau},",
            "\"sub_sample_num\":{sub_sample_num},",
            "\"x1\":{x1_json},\"x2\":{x2_json},",
            "\"path_point\":{{\"x\":2.0,\"y\":3.0,\"heading\":0.78539816339744828,\"reference_velocity\":2.0}}}},",
            "\"output\":{{",
            "\"q\":{q},\"q_inverse\":{q_inverse},\"phi\":{phi},",
            "\"q_delta\":{q},\"q_inverse_delta\":{q_inverse},\"phi_delta\":{phi},",
            "\"q_tau\":{q_tau},\"phi_tau\":{phi_tau},",
            "\"phi_delta_minus_tau\":{phi_delta_minus_tau},",
            "\"lambda\":{lambda},\"psi\":{psi},",
            "\"prior_residual\":{prior_residual},",
            "\"prior_jacobian_x1\":{prior_jacobian_x1},",
            "\"prior_jacobian_x2\":{prior_jacobian_x2},",
            "\"interpolated_state\":{interpolated_state},",
            "\"interpolate_jacobian_x1\":{lambda},",
            "\"interpolate_jacobian_x2\":{psi},",
            "\"path_point_state\":{path_point_state},",
            "\"trajectory\":{trajectory}",
            "}}",
            "}}"
        ),
        qc = f64_json(qc),
        delta = f64_json(delta),
        tau = f64_json(tau),
        sub_sample_num = sub_sample_num,
        x1_json = vec_json(x1.as_slice()),
        x2_json = vec_json(x2.as_slice()),
        q = mat4_json(&wnoa::q(qc, delta)),
        q_inverse = mat4_json(&wnoa::q_inverse(qc, delta)),
        phi = mat4_json(&wnoa::phi(delta)),
        q_tau = mat4_json(&wnoa::q(qc, tau)),
        phi_tau = mat4_json(&wnoa::phi(tau)),
        phi_delta_minus_tau = mat4_json(&wnoa::phi(delta - tau)),
        lambda = mat4_json(&lambda),
        psi = mat4_json(&psi),
        prior_residual = vec_json(wnoa::prior_residual(&x1, &x2, delta).as_slice()),
        prior_jacobian_x1 = mat4_json(&wnoa::phi(delta)),
        prior_jacobian_x2 = mat4_json(&prior_jacobian_x2),
        interpolated_state = vec_json(wnoa::interpolate(&x1, &x2, qc, delta, tau).as_slice()),
        path_point_state = vec_json(path_point_state.as_slice()),
        trajectory = format!(
            "[{},{},{},{}]",
            vec_json(x1.as_slice()),
            vec_json(wnoa::interpolate(&x1, &x2, qc, delta, trajectory_tau_1).as_slice()),
            vec_json(wnoa::interpolate(&x1, &x2, qc, delta, trajectory_tau_2).as_slice()),
            vec_json(x2.as_slice()),
        ),
    )
}

fn main() {
    println!(
        "{{\"schema\":\"lingtu.pct_gpmp_math.result.v1\",\
         \"producer\":\"rust_gpmp_trajectory_optimizer\",\
         \"cases\":[{},{},{},{}]}}",
        wnoj_case(
            "wnoj_positive_heading_rate",
            Vec6::from_row_slice(&[0.2, 1.1, -0.4, -0.3, 0.7, 1.8]),
            Vec6::from_row_slice(&[0.9, 1.0, -0.2, 0.4, 0.8, 1.3]),
            0.2,
        ),
        wnoj_case(
            "wnoj_negative_heading_rate",
            Vec6::from_row_slice(&[0.2, 1.1, 0.4, -0.3, 0.7, -1.8]),
            Vec6::from_row_slice(&[0.9, 1.0, 0.2, 0.4, 0.8, -1.3]),
            0.2,
        ),
        wnoj_case(
            "wnoj_heading_rate_inside_limit",
            Vec6::from_row_slice(&[0.2, 1.1, 0.0, -0.3, 0.7, 0.0]),
            Vec6::from_row_slice(&[0.9, 1.0, 0.0, 0.4, 0.8, 0.0]),
            0.2,
        ),
        wnoa_case()
    );
}
