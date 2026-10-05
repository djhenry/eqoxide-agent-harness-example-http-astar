//! Zone geometry loading (eqoxide spec §10, §11). This module builds a `Collision` from a
//! zone's asset GLB, using the exact same construction path used by eqoxide's own client
//! (`eqoxide_assets::ZoneAssets` and `eqoxide_zone_geometry::collision::Collision::build`).
//!
//! §11 gives the harness the on-disk path to eqoxide's asset cache through the handshake's
//! `asset_cache_dir` field, but the protocol does not implement that field yet (eqoxide spec
//! §9/§11). Until the protocol implements it, callers supply the GLB path directly.

use eqoxide_assets::ZoneAssets;
use eqoxide_zone_geometry::collision::Collision;
use std::path::Path;

/// Loads a zone's collision geometry from its asset GLB at `glb_path`, gridded at `cell_size`
/// (world units per collision-grid cell).
///
/// eqoxide's own zone loader uses the same construction path but chooses its own cell size.
/// Callers of this function choose their own `cell_size`.
pub fn load_zone(glb_path: &Path, cell_size: f32) -> anyhow::Result<Collision> {
    let assets = ZoneAssets::from_glb(glb_path)?;
    Ok(Collision::build(&assets, cell_size))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn load_zone_on_a_nonexistent_path_is_an_error() {
        let result = load_zone(Path::new("/nonexistent/definitely-not-a-zone.glb"), 32.0);
        assert!(
            result.is_err(),
            "a missing GLB file must surface as an error, not panic or silently succeed"
        );
    }

    /// Proves the pinned eqoxide-zone-geometry dependency builds a real, queryable `Collision`
    /// through the same `ZoneAssets` to `Collision::build` path that `load_zone` uses.
    ///
    /// This test uses eqoxide's own test-only flat-floor fixture (the `test-fixtures` feature)
    /// instead of a real GLB file. This workspace has no GLB fixture yet. This test does not
    /// exercise `load_zone`'s own GLB-parsing step (`ZoneAssets::from_glb`). That step needs a
    /// real `.glb` file, and checking one into this repo is follow-up-plan work (see
    /// docs/scope.md).
    #[test]
    fn the_pinned_zone_geometry_crate_builds_a_real_queryable_collision() {
        let state = eqoxide_zone_geometry::zone_assets::ZoneAssetState::test_ready();
        let collision = state
            .collision()
            .expect("test_ready() always returns a Ready state");
        assert!(
            collision.has_geometry(),
            "the flat-floor fixture must produce real grid geometry"
        );
        assert!(
            collision.has_triangles(),
            "the flat-floor fixture must produce real triangles to query"
        );
    }
}
