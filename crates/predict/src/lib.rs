use cornellev_dynamics::kinematic_step;

pub use cornellev_dynamics::PlantState;

/// Integrate `state` for `steps` ticks of `dt`, holding its acceleration and steering.
/// `steps == 1` is the next state. A nonzero plant code discards any states already produced.
pub fn predict_horizon(
    state: PlantState,
    params: &[f64],
    dt: f64,
    steps: usize,
) -> Result<Vec<PlantState>, i32> {
    let mut current = state;
    let mut predicted = Vec::with_capacity(steps);
    for _ in 0..steps {
        let code = kinematic_step(&mut current, params, dt);
        if code != 0 {
            return Err(code);
        }
        predicted.push(current);
    }
    Ok(predicted)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::Deserialize;
    use std::fs;
    use std::path::Path;

    #[test]
    fn constant_acceleration_horizon_follows_explicit_euler() {
        let initial = PlantState {
            position: [0.0, 4.0, 0.0],
            rotation: [0.2, 0.0, -0.3],
            velocity: [0.0, 0.0, 0.0],
            acceleration: [2.0, 0.0, 0.0],
            steering: 0.0,
        };
        let dt = 0.1;
        let states = predict_horizon(initial, &[1.5, 0.6], dt, 3).unwrap();
        assert_eq!(states.len(), 3);

        let mut expected = initial;
        for state in &states {
            expected.velocity[0] += expected.acceleration[0] * dt;
            expected.position[0] += expected.velocity[0] * dt;
            assert!((state.velocity[0] - expected.velocity[0]).abs() < 1e-12);
            assert!((state.position[0] - expected.position[0]).abs() < 1e-12);
            assert_eq!(state.position[1], initial.position[1]);
            assert_eq!(state.position[2], initial.position[2]);
            assert_eq!(state.rotation, initial.rotation);
            assert_eq!(state.acceleration, initial.acceleration);
            assert_eq!(state.steering, initial.steering);
        }
    }

    #[test]
    fn first_tape_sample_matches_one_step() {
        let fixture = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../fixtures/bicycle-tape.v1.json");
        let tape: Tape = serde_json::from_str(&fs::read_to_string(fixture).unwrap()).unwrap();
        let case = &tape.cases[0];
        let sample = &case.steps[0];
        let mut state = PlantState {
            position: case.initial.position,
            rotation: case.initial.rotation,
            velocity: case.initial.velocity,
            acceleration: case.initial.acceleration,
            steering: case.initial.steering,
        };
        state.acceleration[0] = sample.acceleration;
        state.steering = sample.steering;
        let predicted = predict_horizon(
            state,
            &[case.wheelbase, case.max_steering_angle],
            sample.dt,
            1,
        )
        .unwrap();
        let next = predicted[0];
        assert!((next.position[0] - sample.expected.x).abs() <= 1e-9);
        assert!((next.position[2] - sample.expected.z).abs() <= 1e-9);
        assert!((next.rotation[1] - sample.expected.yaw).abs() <= 1e-9);
        assert!((next.velocity[0] - sample.expected.velocity_x).abs() <= 1e-9);
    }

    #[derive(Deserialize)]
    struct Tape {
        cases: Vec<Case>,
    }

    #[derive(Deserialize)]
    #[serde(rename_all = "camelCase")]
    struct Case {
        wheelbase: f64,
        max_steering_angle: f64,
        initial: Initial,
        steps: Vec<Step>,
    }

    #[derive(Deserialize)]
    struct Initial {
        position: [f64; 3],
        rotation: [f64; 3],
        velocity: [f64; 3],
        acceleration: [f64; 3],
        steering: f64,
    }

    #[derive(Deserialize)]
    struct Step {
        dt: f64,
        acceleration: f64,
        steering: f64,
        expected: Expected,
    }

    #[derive(Deserialize)]
    #[serde(rename_all = "camelCase")]
    struct Expected {
        x: f64,
        z: f64,
        yaw: f64,
        velocity_x: f64,
    }
}
