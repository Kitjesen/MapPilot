use lingtu_sdk::Client;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let endpoint = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "http://127.0.0.1:5050".into());
    let api_key = std::env::var("LINGTU_API_KEY").ok();
    let client = Client::new(&endpoint, api_key.as_deref(), Duration::from_secs(10))?;
    println!("{}", client.system().health()?);
    Ok(())
}
