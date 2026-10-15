//! Navigation for an eqoxide agent harness: zone geometry loading and the A* planner that runs on
//! top of it. Chase-and-face and zone-cross are still to be ported (see docs/scope.md).

pub mod collision;
pub mod diagnostics;
pub mod planner;
pub mod steering;
pub mod status;
pub mod traversability;
pub mod walker;
pub mod zone;
