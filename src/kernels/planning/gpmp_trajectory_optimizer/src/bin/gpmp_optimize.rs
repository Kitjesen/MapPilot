use std::io::{self, Read};

use lingtu_gpmp_trajectory_optimizer::{optimize_request_json, WnojOptimizeResponse};

fn main() {
    let mut input = String::new();
    if let Err(error) = io::stdin().read_to_string(&mut input) {
        let response = WnojOptimizeResponse::error(format!("failed to read stdin: {error}"));
        println!("{}", serde_json::to_string(&response).unwrap());
        std::process::exit(2);
    }

    let outcome = optimize_request_json(&input);
    println!("{}", outcome.response_json);
    if outcome.status_code != 0 {
        std::process::exit(outcome.status_code);
    }
}
