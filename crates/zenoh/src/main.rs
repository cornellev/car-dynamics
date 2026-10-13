use std::env;
use std::path::PathBuf;
use std::process::ExitCode;

use cornellev_dynamics_zenoh::{load_config, serve_queries, DEFAULT_KEY};

#[tokio::main]
async fn main() -> ExitCode {
    let (key, config_path) = match parse_args(env::args().skip(1)) {
        Ok(parsed) => parsed,
        Err(message) => {
            eprintln!("{message}");
            return ExitCode::from(1);
        }
    };
    let config = match load_config(config_path.as_deref()) {
        Ok(config) => config,
        Err(error) => {
            eprintln!("dynamics-zenoh config: {error}");
            return ExitCode::from(1);
        }
    };
    let session = match zenoh::open(config).await {
        Ok(session) => session,
        Err(error) => {
            eprintln!("dynamics-zenoh session: {error}");
            return ExitCode::from(1);
        }
    };
    if let Err(error) = serve_queries(session, &key).await {
        eprintln!("dynamics-zenoh: {error}");
        return ExitCode::from(1);
    }
    ExitCode::SUCCESS
}

fn parse_args<I>(args: I) -> Result<(String, Option<PathBuf>), String>
where
    I: IntoIterator<Item = String>,
{
    let mut key = DEFAULT_KEY.to_string();
    let mut config = None;
    let mut args = args.into_iter();
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--key" => {
                key = args.next().ok_or_else(|| usage("--key requires a value"))?;
            }
            "--config" => {
                config = Some(PathBuf::from(
                    args.next().ok_or_else(|| usage("--config requires a value"))?,
                ));
            }
            "--help" | "-h" => return Err(usage("")),
            other => return Err(usage(&format!("unknown argument {other}"))),
        }
    }
    Ok((key, config))
}

fn usage(message: &str) -> String {
    let help = "\
usage: dynamics-zenoh [--key KEY] [--config FILE]

Serves bicycle predictions on a Zenoh queryable.
The default key is vendor/cornellev/dynamics/bicycle.
Each query payload is one 136-byte request. The reply is the predicted state.";
    if message.is_empty() {
        help.to_string()
    } else {
        format!("{message}\n{help}")
    }
}
