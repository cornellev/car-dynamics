#![allow(static_mut_refs)]

mod abi;
mod model;

pub use model::{kinematic_step, step, PlantState};
