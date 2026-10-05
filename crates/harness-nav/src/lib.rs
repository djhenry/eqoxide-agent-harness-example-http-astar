//! Zone geometry loading (eqoxide spec §10). This crate builds an
//! `eqoxide_zone_geometry::collision::Collision` from a zone's asset files, using the same
//! shared geometry-grid crate used by eqoxide itself. A* path planning on top of this geometry
//! is a follow-up plan (see docs/scope.md), not part of this crate yet.

pub mod zone;
