//! Circuit autopilot simulator core.
//!
//! A light single takes off, flies a traffic pattern and lands under autopilot, with go-arounds,
//! crosswind de-crab and engine-failure handling. Two interchangeable flight models: the original
//! point-mass model and a six-degree-of-freedom rigid-body model with its own control laws.

pub mod aircraft;
pub mod airfield;
pub mod atmosphere;
pub mod autopilot;
pub mod control;
pub mod emergency;
pub mod scenarios;
pub mod simulation;
pub mod sixdof;
pub mod units;
pub mod wasm;

pub use aircraft::ModelKind;
pub use simulation::{SimOptions, Simulation};
