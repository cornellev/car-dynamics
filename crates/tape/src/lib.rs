use std::fs;
use std::path::{Path, PathBuf};

use cornellev_dynamics::{kinematic_step as plant_step, PlantState};
use serde::Deserialize;

const TOLERANCE: f64 = 1e-9;

#[derive(Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Tape {
    pub kind: String,
    pub version: u32,
    pub cases: Vec<Case>,
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Case {
    pub id: String,
    pub wheelbase: f64,
    pub max_steering_angle: f64,
    pub initial: InitialState,
    pub steps: Vec<Step>,
}

#[derive(Debug, Deserialize)]
pub struct InitialState {
    pub position: [f64; 3],
    pub rotation: [f64; 3],
    pub velocity: [f64; 3],
    pub acceleration: [f64; 3],
    pub steering: f64,
}

#[derive(Debug, Deserialize)]
pub struct Step {
    pub dt: f64,
    pub acceleration: f64,
    pub steering: f64,
    pub expected: Expected,
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Expected {
    pub x: f64,
    pub z: f64,
    pub yaw: f64,
    pub velocity_x: f64,
}

pub fn fixture_path() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../fixtures/bicycle-tape.v1.json")
}

pub fn load(path: &Path) -> Result<Tape, String> {
    let text = fs::read_to_string(path).map_err(|error| format!("read {}: {error}", path.display()))?;
    let tape: Tape = serde_json::from_str(&text).map_err(|error| format!("parse {}: {error}", path.display()))?;
    if tape.kind != "cev-sim.bicycle-tape" || tape.version != 1 {
        return Err(format!("unsupported tape {}.{}", tape.kind, tape.version));
    }
    Ok(tape)
}

pub fn assert_tape(tape: &Tape) -> Result<(), String> {
    if tape.cases.is_empty() {
        return Err("tape has no cases".to_string());
    }
    for case in &tape.cases {
        replay_case(case)?;
    }
    Ok(())
}

fn replay_case(case: &Case) -> Result<(), String> {
    let mut state = PlantState {
        position: case.initial.position,
        rotation: case.initial.rotation,
        velocity: case.initial.velocity,
        acceleration: case.initial.acceleration,
        steering: case.initial.steering,
    };
    let params = [case.wheelbase, case.max_steering_angle];
    for (index, sample) in case.steps.iter().enumerate() {
        state.acceleration[0] = sample.acceleration;
        state.steering = sample.steering;
        let code = plant_step(&mut state, &params, sample.dt);
        if code != 0 {
            return Err(format!("{} step {index} returned {code}", case.id));
        }
        let label = format!("{} step {index}", case.id);
        near(state.position[0], sample.expected.x, &format!("{label} x"))?;
        near(state.position[2], sample.expected.z, &format!("{label} z"))?;
        near(state.rotation[1], sample.expected.yaw, &format!("{label} yaw"))?;
        near(state.velocity[0], sample.expected.velocity_x, &format!("{label} velocityX"))?;
        if state.position[1] != case.initial.position[1]
            || state.rotation[0] != case.initial.rotation[0]
            || state.rotation[2] != case.initial.rotation[2]
        {
            return Err(format!("{label} changed a host-draped axis"));
        }
    }
    Ok(())
}

fn near(actual: f64, expected: f64, label: &str) -> Result<(), String> {
    let delta = (actual - expected).abs();
    if delta <= TOLERANCE {
        Ok(())
    } else {
        Err(format!("{label}: {actual} != {expected} (delta {delta})"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn committed_tape_matches_the_plant() {
        let tape = load(&fixture_path()).unwrap();
        assert_tape(&tape).unwrap();
    }
}
