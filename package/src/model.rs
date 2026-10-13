#[derive(Clone, Copy, Debug)]
pub struct PlantState {
    pub position: [f64; 3],
    pub rotation: [f64; 3],
    pub velocity: [f64; 3],
    pub acceleration: [f64; 3],
    pub steering: f64,
}

fn parameter(params: &[f64], index: usize, fallback: f64) -> f64 {
    params.get(index).copied().unwrap_or(fallback)
}

/// basic ahh bicycle model, with no slip, no roll, no pitch, and no lateral velocity.
/// need to implement those soon.
pub fn step(state: &mut PlantState, params: &[f64], dt: f64) -> i32 {
    let wheelbase = parameter(params, 0, 1.5).max(1.0e-9);
    let maximum = parameter(params, 1, 0.6).min(std::f64::consts::PI * 0.49);
    state.velocity[0] += state.acceleration[0] * dt;
    state.velocity[1] += state.acceleration[1] * dt;
    state.velocity[2] += state.acceleration[2] * dt;
    let speed = state.velocity[0];
    let steering = state.steering.clamp(-maximum, maximum);
    let yaw = state.rotation[1];
    state.position[0] += yaw.cos() * speed * dt;
    state.position[2] += -yaw.sin() * speed * dt;
    state.rotation[1] += (speed / wheelbase) * steering.tan() * dt;
    0
}

/// Arguably if the model is changed then i'd prob just remove the following tests lowk
/// but for now they are here to make sure the ABI is stable and that the model is still correct.
/// just run the test cases pls after changing the model and either update them (or ask claude to update them)
/// or remove them for ease.
#[cfg(test)]
mod tests {
    use super::*;

    fn resting() -> PlantState {
        PlantState {
            position: [0.0, 4.0, 0.0],
            rotation: [0.2, 0.0, -0.3],
            velocity: [0.0, 1.0, -2.0],
            acceleration: [10.0, 0.5, 0.25],
            steering: 1.2,
        }
    }

    #[test]
    fn integrates_acceleration_before_translation() {
        let mut state = resting();
        state.steering = 0.0;
        assert_eq!(step(&mut state, &[1.5, 0.6], 0.1), 0);
        assert!((state.velocity[0] - 1.0).abs() < 1e-12);
        assert!((state.position[0] - 0.1).abs() < 1e-12);
        assert!((state.velocity[1] - 1.05).abs() < 1e-12);
        assert!((state.velocity[2] - -1.975).abs() < 1e-12);
        assert_eq!(state.position[1], 4.0);
        assert_eq!(state.rotation[0], 0.2);
        assert_eq!(state.rotation[2], -0.3);
        assert_eq!(state.steering, 0.0);
    }

    #[test]
    fn clamps_steering_without_writing_it_back() {
        let mut state = resting();
        state.velocity[0] = 2.0;
        state.acceleration = [0.0, 0.0, 0.0];
        assert_eq!(step(&mut state, &[1.5, 0.6], 0.1), 0);
        let expected_yaw = (2.0 / 1.5) * 0.6_f64.tan() * 0.1;
        assert!((state.rotation[1] - expected_yaw).abs() < 1e-12);
        assert_eq!(state.steering, 1.2);
        assert_eq!(state.position[1], 4.0);
    }

    #[test]
    fn empty_parameters_use_javascript_fallbacks() {
        let mut state = resting();
        state.velocity = [1.0, 0.0, 0.0];
        state.acceleration = [0.0, 0.0, 0.0];
        state.steering = 0.0;
        state.rotation = [0.0, 0.0, 0.0];
        assert_eq!(step(&mut state, &[], 1.0), 0);
        assert!((state.position[0] - 1.0).abs() < 1e-12);
        assert!(state.position[2].abs() < 1e-12);
    }
}
