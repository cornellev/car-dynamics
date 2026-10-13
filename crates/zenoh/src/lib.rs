use std::path::Path;

use cornellev_dynamics_wire::respond;
use zenoh::Config;

pub const DEFAULT_KEY: &str = "vendor/cornellev/dynamics/bicycle";

pub fn load_config(path: Option<&Path>) -> zenoh::Result<Config> {
    match path {
        Some(path) => Config::from_file(path),
        None => Ok(Config::default()),
    }
}

pub async fn serve_queries(session: zenoh::Session, key: &str) -> zenoh::Result<()> {
    let queryable = session.declare_queryable(key).await?;
    while let Ok(query) = queryable.recv_async().await {
        let request = match query.payload() {
            Some(payload) => payload.to_bytes().as_ref().to_vec(),
            None => Vec::new(),
        };
        let response = respond(&request);
        if let Err(error) = query.reply(query.key_expr().clone(), response).await {
            eprintln!("dynamics-zenoh reply failed: {error}");
        }
    }
    Ok(())
}
