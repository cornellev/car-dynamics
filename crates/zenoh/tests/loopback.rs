use std::time::Duration;

use cornellev_dynamics_predict::{predict_horizon, PlantState};
use cornellev_dynamics_wire::{decode_response, encode_request, Request};
use cornellev_dynamics_zenoh::{serve_queries, DEFAULT_KEY};
use zenoh::Config;

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn loopback_query_matches_predict() {
    let server = zenoh::open(peer_config(None)).await.unwrap();
    let locator = server
        .info()
        .locators()
        .await
        .into_iter()
        .find(|locator| locator.to_string().starts_with("tcp/127.0.0.1:"))
        .expect("server locator");
    let key = DEFAULT_KEY.to_string();
    let session = server.clone();
    let serve_key = key.clone();
    tokio::spawn(async move {
        serve_queries(session, &serve_key).await.unwrap();
    });

    let client = zenoh::open(peer_config(Some(locator.to_string()))).await.unwrap();
    let request = Request {
        steps: 1,
        dt: 0.02,
        wheelbase: 1.2446,
        max_steering_angle: 1.5393804002589986,
        state: PlantState {
            position: [1.0, 0.0, 2.0],
            rotation: [0.0, 0.2, 0.0],
            velocity: [3.0, 0.0, 0.0],
            acceleration: [0.4, 0.0, 0.0],
            steering: 0.05,
        },
    };
    let replies = client
        .get(&key)
        .payload(encode_request(&request))
        .timeout(Duration::from_secs(5))
        .await
        .unwrap();
    let reply = replies.recv_async().await.unwrap();
    let sample = reply.result().unwrap();
    let response = decode_response(sample.payload().to_bytes().as_ref()).unwrap();
    let expected = predict_horizon(
        request.state,
        &[request.wheelbase, request.max_steering_angle],
        request.dt,
        1,
    )
    .unwrap();
    assert_eq!(response.code, 0);
    assert_eq!(response.states.len(), expected.len());
    let next = &response.states[0];
    let wanted = &expected[0];
    assert_eq!(next.position, wanted.position);
    assert_eq!(next.rotation, wanted.rotation);
    assert_eq!(next.velocity, wanted.velocity);
    assert_eq!(next.acceleration, wanted.acceleration);
    assert_eq!(next.steering, wanted.steering);
}

fn peer_config(connect: Option<String>) -> Config {
    let mut config = Config::default();
    insert(&mut config, "mode", r#""peer""#);
    insert(&mut config, "scouting/multicast/enabled", "false");
    insert(&mut config, "scouting/gossip/enabled", "false");
    match connect {
        Some(locator) => {
            insert(&mut config, "listen/endpoints", "[]");
            insert(&mut config, "connect/endpoints", &format!("[\"{locator}\"]"));
        }
        None => insert(&mut config, "listen/endpoints", r#"["tcp/127.0.0.1:0"]"#),
    }
    config
}

fn insert(config: &mut Config, key: &str, value: &str) {
    config.insert_json5(key, value).unwrap_or_else(|error| {
        panic!("config {key}: {error}");
    });
}
