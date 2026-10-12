//! Zone geometry loading (eqoxide spec §10, §11). This module builds a `Collision` from a
//! zone's asset GLB, using the exact same construction path used by eqoxide's own client
//! (`eqoxide_assets::ZoneAssets` and `eqoxide_zone_geometry::collision::Collision::build`).
//!
//! The handshake's `asset_cache_dir` (see `harness_socket::AgentClient::asset_cache_dir`) is where
//! eqoxide keeps `<zone>.glb`; [`load_zone_from_cache`] finds a zone there by name.

use eqoxide_assets::ZoneAssets;
use eqoxide_core::region_map::RegionMap;
use std::sync::Arc;
use eqoxide_zone_geometry::collision::Collision;
use std::path::{Path, PathBuf};

/// Loads a zone's collision geometry from its asset GLB at `glb_path`, gridded at `cell_size`
/// (world units per collision-grid cell).
///
/// eqoxide's own zone loader uses the same construction path but chooses its own cell size.
/// Callers of this function choose their own `cell_size`.
pub fn load_zone(glb_path: &Path, cell_size: f32) -> anyhow::Result<Collision> {
    let assets = ZoneAssets::from_glb(glb_path)?;
    Ok(Collision::build(&assets, cell_size))
}

/// Collision-grid cell size eqoxide itself uses when loading a zone.
pub const EQOXIDE_CELL_SIZE: f32 = 32.0;

/// Path of `zone_name`'s GLB inside eqoxide's asset cache.
pub fn zone_glb_path(asset_cache_dir: &Path, zone_name: &str) -> PathBuf {
    asset_cache_dir.join(format!("{zone_name}.glb"))
}

/// Loads `zone_name` from the asset cache disclosed in the handshake, at eqoxide's own cell size,
/// with the zone's region data (`maps/water/<zone>.wtr`: water, lava and zone lines) attached. A
/// missing or unreadable region file is not an error: the grid records why it has none, and
/// zone-line queries then report that rather than an empty answer.
pub fn load_zone_from_cache(asset_cache_dir: &Path, zone_name: &str) -> anyhow::Result<Collision> {
    let mut collision = load_zone(&zone_glb_path(asset_cache_dir, zone_name), EQOXIDE_CELL_SIZE)?;
    let water = RegionMap::try_load(&asset_cache_dir.join("maps").join("water"), zone_name).map(Arc::new);
    if let Err(e) = &water {
        tracing::warn!("zone '{zone_name}' has no usable region data: {e}");
    }
    collision.set_region_data(water);
    Ok(collision)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_zone_is_looked_up_by_name_in_the_cache_dir() {
        assert_eq!(zone_glb_path(Path::new("/cache/models"), "qeynos"), Path::new("/cache/models/qeynos.glb"));
        assert!(load_zone_from_cache(Path::new("/nonexistent"), "qeynos").is_err());
    }

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
