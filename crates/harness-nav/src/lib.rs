//! Navigation for an eqoxide agent harness: zone geometry loading and the A* planner that runs on
//! top of it. Zone-cross walks to the nearest zone line; eqoxide fires the crossing itself.

pub mod collision;
pub mod diagnostics;
pub mod planner;
pub mod steering;
pub mod status;
pub mod traversability;
pub mod walker;
pub mod zone;
