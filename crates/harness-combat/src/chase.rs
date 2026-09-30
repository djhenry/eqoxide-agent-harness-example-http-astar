//! Chase-and-face-while-engaged (eqoxide spec §7.2) — reads own position/heading and the target's
//! live position straight from `Observation`, computes range/bearing itself (exactly what a human
//! player derives by looking at their screen), and drives movement + wish_heading toward the
//! target. No special access to eqoxide internals.

use eqoxide_agent_protocol::movement::AgentMovement;
use eqoxide_agent_protocol::observation::{OwnState, VisibleEntity};

/// EQ heading in degrees (0..360) for a movement delta in server axes: heading 0 faces +Y
/// (north) and increases counter-clockwise (90 = west, 180 = south, 270 = east) — reimplemented
/// here from eqoxide's own `eqoxide_core::coord::eq_heading` (crates/eqoxide-core/src/coord.rs in
/// the eqoxide repo), which this workspace cannot depend on (Global Constraints: only
/// eqoxide-agent-protocol/eqoxide-assets/eqoxide-zone-geometry are allowed dependencies).
fn eq_heading(d_east: f32, d_north: f32) -> f32 {
    (-d_east).atan2(d_north).to_degrees().rem_euclid(360.0)
}

/// Compute the movement to close on and face `target`, given the agent's own current state.
/// Always faces the target (`wish_heading` is always `Some`); stops closing distance once within
/// `engage_range` but keeps facing it — the "while engaged" half of the behavior.
pub fn chase_and_face(own: &OwnState, target: &VisibleEntity, engage_range: f32) -> AgentMovement {
    let d_east = target.pos[0] - own.pos[0];
    let d_north = target.pos[1] - own.pos[1];
    let range = (d_east * d_east + d_north * d_north).sqrt();
    let heading = eq_heading(d_east, d_north);

    let dir = if range > engage_range && range > 1e-4 {
        [d_east / range, d_north / range]
    } else {
        [0.0, 0.0]
    };

    AgentMovement {
        dir,
        up: 0.0,
        jump: false,
        wish_heading: Some(heading),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use eqoxide_agent_protocol::observation::OwnState;

    fn own_at_origin() -> OwnState {
        OwnState {
            pos: [0.0, 0.0, 0.0],
            heading: 0.0,
            hp: 100,
            hp_max: 100,
            hp_verified: true,
            mana: 0,
            mana_max: 0,
            endurance: 0,
            endurance_max: 0,
            endurance_confirmed: true,
            casting: None,
            buffs: vec![],
            zone_name: "qeynos".into(),
            target_id: None,
            target_name: None,
            auto_attack: false,
            sitting: false,
            held: false,
            player_class: "Warrior".into(),
            player_level: 1,
        }
    }

    fn target_at(pos: [f32; 3]) -> VisibleEntity {
        VisibleEntity {
            spawn_id: 7,
            name: "a_rat00".into(),
            is_npc: true,
            level: 3,
            race: "Rat".into(),
            pos,
            heading: 0.0,
            hp_pct: 100.0,
            dead: false,
        }
    }

    #[test]
    fn a_far_target_due_east_is_chased_and_faced_at_heading_270() {
        let m = chase_and_face(&own_at_origin(), &target_at([100.0, 0.0, 0.0]), 20.0);
        assert!(
            (m.dir[0] - 1.0).abs() < 1e-4 && m.dir[1].abs() < 1e-4,
            "must move straight east: {:?}",
            m.dir
        );
        assert!(
            (m.wish_heading.unwrap() - 270.0).abs() < 1e-3,
            "east must be heading 270: {:?}",
            m.wish_heading
        );
    }

    #[test]
    fn a_far_target_due_north_is_chased_and_faced_at_heading_0() {
        let m = chase_and_face(&own_at_origin(), &target_at([0.0, 100.0, 0.0]), 20.0);
        assert!(
            m.dir[0].abs() < 1e-4 && (m.dir[1] - 1.0).abs() < 1e-4,
            "must move straight north: {:?}",
            m.dir
        );
        assert!(
            m.wish_heading.unwrap().abs() < 1e-3,
            "north must be heading 0: {:?}",
            m.wish_heading
        );
    }

    #[test]
    fn a_target_within_engage_range_stops_closing_but_keeps_facing() {
        let m = chase_and_face(&own_at_origin(), &target_at([5.0, 0.0, 0.0]), 20.0);
        assert_eq!(
            m.dir,
            [0.0, 0.0],
            "within engage_range: must stand still, not keep closing"
        );
        assert!(
            (m.wish_heading.unwrap() - 270.0).abs() < 1e-3,
            "must still face the target while engaged: {:?}",
            m.wish_heading
        );
    }
}
