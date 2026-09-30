//! Zone geometry loading (eqoxide spec §10) — builds an `eqoxide_zone_geometry::collision::Collision`
//! from a zone's asset files, using the same shared geometry-grid crate eqoxide itself uses. A* path
//! planning on top of this geometry is a follow-up plan (see docs/scope.md), not this crate today.

pub mod zone;
