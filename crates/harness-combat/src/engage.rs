//! Auto-engage melee: while auto-attack is on and the target is a live, visible spawn, close to
//! melee range and keep facing it. Port of eqoxide's `drive_auto_engage_melee`, reading everything
//! from the `Observation`.

use crate::chase::chase_and_face;
use eqoxide_agent_protocol::movement::AgentMovement;
use eqoxide_agent_protocol::observation::Observation;

/// Distance at which eqoxide's engage driver stops closing (`MELEE_ENGAGE_RANGE` in eqoxide).
/// The pet standoff range is not ported: `Observation` does not say whether the character has a pet.
pub const MELEE_ENGAGE_RANGE: f32 = 5.0;

/// A target further above the character than this is not chased directly: the driver steers in the
/// XY plane only, so it would stall at the foot of the ledge (`MELEE_ENGAGE_MAX_Z_GAP` in eqoxide).
pub const MELEE_ENGAGE_MAX_Z_GAP: f32 = 20.0;

/// The movement that engages the current target, or `None` when there is nothing to engage:
/// auto-attack off, the character dead, no target, the target out of view or dead, or unreachable
/// by a straight chase.
pub fn engage(obs: &Observation) -> Option<AgentMovement> {
    if !obs.own.auto_attack || obs.dead {
        return None;
    }
    let id = obs.own.target_id?;
    let target = obs.visible.iter().find(|e| e.spawn_id == id && !e.dead)?;
    if target.pos[2] - obs.own.pos[2] > MELEE_ENGAGE_MAX_Z_GAP {
        return None;
    }
    Some(chase_and_face(&obs.own, target, MELEE_ENGAGE_RANGE))
}

#[cfg(test)]
mod tests {
    use super::*;
    use eqoxide_agent_protocol::observation::{LegalActionMask, OwnState, VisibleEntity};

    fn obs(target: Option<VisibleEntity>) -> Observation {
        Observation {
            own: OwnState {
                pos: [0.0, 0.0, 0.0], heading: 0.0, hp: 100, hp_max: 100, hp_verified: true,
                mana: 0, mana_max: 0, endurance: 0, endurance_max: 0, endurance_confirmed: true,
                casting: None, buffs: vec![], zone_name: "qeynos".into(),
                target_id: target.as_ref().map(|t| t.spawn_id), target_name: None,
                auto_attack: true, sitting: false, held: false,
                player_class: "Warrior".into(), player_level: 1,
            },
            visible: target.into_iter().collect(),
            legal_actions: LegalActionMask { gems: [false; 9], abilities: vec![] },
            dead: false, terminated: false, truncated: false, visibility_available: true, tick: 0,
        }
    }

    fn rat(pos: [f32; 3]) -> VisibleEntity {
        VisibleEntity {
            spawn_id: 7, name: "a_rat00".into(), is_npc: true, level: 1, race: "Rat".into(),
            pos, heading: 0.0, hp_pct: 100.0, dead: false,
        }
    }

    #[test]
    fn a_far_target_is_chased() {
        let m = engage(&obs(Some(rat([50.0, 0.0, 0.0])))).unwrap();
        assert!(m.dir[0] > 0.99);
    }

    #[test]
    fn inside_melee_range_it_holds_and_faces() {
        let m = engage(&obs(Some(rat([3.0, 0.0, 0.0])))).unwrap();
        assert_eq!(m.dir, [0.0, 0.0]);
        assert!(m.wish_heading.is_some());
    }

    #[test]
    fn nothing_to_engage_without_auto_attack_or_a_live_visible_target() {
        let mut o = obs(Some(rat([50.0, 0.0, 0.0])));
        o.own.auto_attack = false;
        assert!(engage(&o).is_none());
        let mut dead = rat([50.0, 0.0, 0.0]);
        dead.dead = true;
        assert!(engage(&obs(Some(dead))).is_none());
        let mut o = obs(Some(rat([50.0, 0.0, 0.0])));
        o.visible.clear();
        assert!(engage(&o).is_none());
        assert!(engage(&obs(None)).is_none());
    }

    #[test]
    fn a_target_on_a_ledge_above_the_gap_is_not_chased() {
        assert!(engage(&obs(Some(rat([10.0, 0.0, 25.0])))).is_none());
        assert!(engage(&obs(Some(rat([10.0, 0.0, 15.0])))).is_some());
    }
}
