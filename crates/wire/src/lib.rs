use cornellev_dynamics_predict::{predict_horizon, PlantState};

pub const VERSION: u32 = 1;
pub const MAX_STEPS: u32 = 64;
pub const STATE_VALUES: usize = 13;
pub const REQUEST_LEN: usize = 4 + 4 + 8 + 8 + 8 + STATE_VALUES * 8;

pub const ERR_LENGTH: i32 = -1;
pub const ERR_VERSION: i32 = -2;
pub const ERR_STEPS: i32 = -3;
pub const ERR_NON_FINITE: i32 = -4;
pub const ERR_NEGATIVE_DT: i32 = -5;

#[derive(Clone, Copy, Debug)]
pub struct Request {
    pub steps: u32,
    pub dt: f64,
    pub wheelbase: f64,
    pub max_steering_angle: f64,
    pub state: PlantState,
}

#[derive(Clone, Debug)]
pub struct Response {
    pub code: i32,
    pub states: Vec<PlantState>,
}

pub fn encode_request(request: &Request) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(REQUEST_LEN);
    write_u32(&mut bytes, VERSION);
    write_u32(&mut bytes, request.steps);
    write_f64(&mut bytes, request.dt);
    write_f64(&mut bytes, request.wheelbase);
    write_f64(&mut bytes, request.max_steering_angle);
    write_state(&mut bytes, &request.state);
    bytes
}

pub fn decode_response(bytes: &[u8]) -> Result<Response, ()> {
    if bytes.len() < 12 {
        return Err(());
    }
    let mut cursor = Cursor { bytes, index: 0 };
    let version = cursor.u32().ok_or(())?;
    if version != VERSION {
        return Err(());
    }
    let code = cursor.i32().ok_or(())?;
    let count = cursor.u32().ok_or(())? as usize;
    if bytes.len() != 12 + count * STATE_VALUES * 8 {
        return Err(());
    }
    let mut states = Vec::with_capacity(count);
    for _ in 0..count {
        states.push(cursor.state().ok_or(())?);
    }
    if cursor.index != bytes.len() {
        return Err(());
    }
    Ok(Response { code, states })
}

pub fn respond(bytes: &[u8]) -> Vec<u8> {
    let request = match decode_request(bytes) {
        Ok(request) => request,
        Err(code) => return encode_response(code, &[]),
    };
    match predict_horizon(
        request.state,
        &[request.wheelbase, request.max_steering_angle],
        request.dt,
        request.steps as usize,
    ) {
        Ok(states) => encode_response(0, &states),
        Err(code) => encode_response(code, &[]),
    }
}

fn decode_request(bytes: &[u8]) -> Result<Request, i32> {
    if bytes.len() != REQUEST_LEN {
        return Err(ERR_LENGTH);
    }
    let mut cursor = Cursor { bytes, index: 0 };
    let version = cursor.u32().ok_or(ERR_LENGTH)?;
    if version != VERSION {
        return Err(ERR_VERSION);
    }
    let steps = cursor.u32().ok_or(ERR_LENGTH)?;
    if !(1..=MAX_STEPS).contains(&steps) {
        return Err(ERR_STEPS);
    }
    let dt = cursor.f64().ok_or(ERR_LENGTH)?;
    let wheelbase = cursor.f64().ok_or(ERR_LENGTH)?;
    let max_steering_angle = cursor.f64().ok_or(ERR_LENGTH)?;
    let state = cursor.state().ok_or(ERR_LENGTH)?;
    if dt < 0.0 {
        return Err(ERR_NEGATIVE_DT);
    }
    if !dt.is_finite()
        || !wheelbase.is_finite()
        || !max_steering_angle.is_finite()
        || !state_is_finite(&state)
    {
        return Err(ERR_NON_FINITE);
    }
    Ok(Request {
        steps,
        dt,
        wheelbase,
        max_steering_angle,
        state,
    })
}

fn encode_response(code: i32, states: &[PlantState]) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(12 + states.len() * STATE_VALUES * 8);
    write_u32(&mut bytes, VERSION);
    write_i32(&mut bytes, code);
    write_u32(&mut bytes, states.len() as u32);
    for state in states {
        write_state(&mut bytes, state);
    }
    bytes
}

fn state_is_finite(state: &PlantState) -> bool {
    state.position.iter().all(|value| value.is_finite())
        && state.rotation.iter().all(|value| value.is_finite())
        && state.velocity.iter().all(|value| value.is_finite())
        && state.acceleration.iter().all(|value| value.is_finite())
        && state.steering.is_finite()
}

fn write_state(bytes: &mut Vec<u8>, state: &PlantState) {
    for value in state.position {
        write_f64(bytes, value);
    }
    for value in state.rotation {
        write_f64(bytes, value);
    }
    for value in state.velocity {
        write_f64(bytes, value);
    }
    for value in state.acceleration {
        write_f64(bytes, value);
    }
    write_f64(bytes, state.steering);
}

fn write_u32(bytes: &mut Vec<u8>, value: u32) {
    bytes.extend_from_slice(&value.to_le_bytes());
}

fn write_i32(bytes: &mut Vec<u8>, value: i32) {
    bytes.extend_from_slice(&value.to_le_bytes());
}

fn write_f64(bytes: &mut Vec<u8>, value: f64) {
    bytes.extend_from_slice(&value.to_le_bytes());
}

struct Cursor<'a> {
    bytes: &'a [u8],
    index: usize,
}

impl<'a> Cursor<'a> {
    fn take(&mut self, len: usize) -> Option<&'a [u8]> {
        let end = self.index.checked_add(len)?;
        let slice = self.bytes.get(self.index..end)?;
        self.index = end;
        Some(slice)
    }

    fn u32(&mut self) -> Option<u32> {
        Some(u32::from_le_bytes(self.take(4)?.try_into().ok()?))
    }

    fn i32(&mut self) -> Option<i32> {
        Some(i32::from_le_bytes(self.take(4)?.try_into().ok()?))
    }

    fn f64(&mut self) -> Option<f64> {
        Some(f64::from_le_bytes(self.take(8)?.try_into().ok()?))
    }

    fn state(&mut self) -> Option<PlantState> {
        Some(PlantState {
            position: [self.f64()?, self.f64()?, self.f64()?],
            rotation: [self.f64()?, self.f64()?, self.f64()?],
            velocity: [self.f64()?, self.f64()?, self.f64()?],
            acceleration: [self.f64()?, self.f64()?, self.f64()?],
            steering: self.f64()?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_request(steps: u32) -> Request {
        Request {
            steps,
            dt: 0.1,
            wheelbase: 1.5,
            max_steering_angle: 0.6,
            state: PlantState {
                position: [0.0, 2.0, 0.0],
                rotation: [0.15, -0.4, 0.05],
                velocity: [2.0, 0.0, 0.0],
                acceleration: [0.5, 0.0, 0.0],
                steering: 0.1,
            },
        }
    }

    #[test]
    fn one_step_round_trip() {
        let request = sample_request(1);
        let response = decode_response(&respond(&encode_request(&request))).unwrap();
        let expected = predict_horizon(
            request.state,
            &[request.wheelbase, request.max_steering_angle],
            request.dt,
            1,
        )
        .unwrap();
        assert_eq!(response.code, 0);
        assert_states(&response.states, &expected);
    }

    #[test]
    fn three_step_horizon_holds_the_command() {
        let request = sample_request(3);
        let response = decode_response(&respond(&encode_request(&request))).unwrap();
        let expected = predict_horizon(
            request.state,
            &[request.wheelbase, request.max_steering_angle],
            request.dt,
            3,
        )
        .unwrap();
        assert_eq!(response.code, 0);
        assert_states(&response.states, &expected);
        assert_eq!(response.states[2].acceleration, request.state.acceleration);
        assert_eq!(response.states[2].steering, request.state.steering);
    }

    #[test]
    fn rejections_use_count_zero() {
        let request = sample_request(1);
        let mut encoded = encode_request(&request);

        let short = decode_response(&respond(&encoded[..encoded.len() - 1])).unwrap();
        assert_eq!(short.code, ERR_LENGTH);
        assert!(short.states.is_empty());

        encoded[0] = 2;
        let version = decode_response(&respond(&encoded)).unwrap();
        assert_eq!(version.code, ERR_VERSION);

        let mut zero_steps = sample_request(0);
        let zero = decode_response(&respond(&encode_request(&zero_steps))).unwrap();
        assert_eq!(zero.code, ERR_STEPS);
        zero_steps.steps = MAX_STEPS + 1;
        let too_many = decode_response(&respond(&encode_request(&zero_steps))).unwrap();
        assert_eq!(too_many.code, ERR_STEPS);

        let mut non_finite = sample_request(1);
        non_finite.wheelbase = f64::NAN;
        let finite = decode_response(&respond(&encode_request(&non_finite))).unwrap();
        assert_eq!(finite.code, ERR_NON_FINITE);

        let mut negative = sample_request(1);
        negative.dt = -0.01;
        let dt = decode_response(&respond(&encode_request(&negative))).unwrap();
        assert_eq!(dt.code, ERR_NEGATIVE_DT);
        assert!(dt.states.is_empty());
    }

    fn assert_states(actual: &[PlantState], expected: &[PlantState]) {
        assert_eq!(actual.len(), expected.len());
        for (left, right) in actual.iter().zip(expected) {
            assert_eq!(left.position, right.position);
            assert_eq!(left.rotation, right.rotation);
            assert_eq!(left.velocity, right.velocity);
            assert_eq!(left.acceleration, right.acceleration);
            assert_eq!(left.steering, right.steering);
        }
    }
}
