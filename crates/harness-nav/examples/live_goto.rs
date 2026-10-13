//! Live check for the nav stack: over the Agent Plugin API socket only, walk to a named spawn and
//! then (optionally) to the nearest zone line until the zone changes.
//!
//! ```text
//! # in the eqoxide checkout
//! cargo run --release -- --agent-socket /tmp/eqoxide-agent.sock --config <config>.yaml
//! # in this repo
//! cargo run --release -p harness-nav --example live_goto -- /tmp/eqoxide-agent.sock <spawn-name> [--cross]
//! ```

use harness_nav::status::nav_state_is_terminal;
use harness_nav::diagnostics::NavDebugView;
use harness_nav::walker::Walker;
use harness_nav::{status::NavSlots, zone::load_zone_from_cache};
use harness_socket::AgentClient;
use eqoxide_agent_protocol::movement::AgentMovement;
use eqoxide_agent_protocol::observation::Observation;
use eqoxide_agent_protocol::step::Step;
use std::sync::Arc;

const MAX_TICKS: u32 = 4000;
const ARRIVE_DIST: f32 = 15.0;

fn dist(a: [f32; 3], b: [f32; 3]) -> f32 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

/// Movement latches on the eqoxide side, so leave the character standing, not mid-stride.
fn finish(client: &mut AgentClient, code: i32) -> ! {
    let stop = AgentMovement { dir: [0.0, 0.0], up: 0.0, jump: false, wish_heading: None };
    let _ = client.send_step(&Step { movement: Some(stop), verb: None });
    std::process::exit(code);
}

fn step(client: &mut AgentClient, walker: &mut Walker, obs: &Observation) {
    let movement = walker.tick(obs);
    for n in walker.take_notices() {
        println!("tick {}: notice: {n}", obs.tick);
    }
    client.send_step(&Step { movement, verb: None }).expect("send step");
}

fn main() {
    let mut args = std::env::args().skip(1);
    let (Some(socket), Some(name)) = (args.next(), args.next()) else {
        eprintln!("usage: live_goto <agent-socket> <spawn-name-substring> [--cross]");
        std::process::exit(2);
    };
    let cross = args.next().as_deref() == Some("--cross");

    let mut client = AgentClient::connect(&socket).expect("connect + handshake");
    let cache = client.asset_cache_dir().to_path_buf();
    println!("connected; asset_cache_dir = {}", cache.display());

    let obs = client.recv_observation().expect("first observation");
    let zone = obs.own.zone_name.clone();
    let collision = load_zone_from_cache(&cache, &zone).expect("load zone from asset cache");
    let mut walker = Walker::new(NavSlots::default(), NavDebugView::default());
    walker.set_zone(&zone, Arc::new(collision));
    println!("zone {zone} loaded; at {:?}", obs.own.pos);

    let needle = name.to_lowercase();
    let target = obs
        .visible
        .iter()
        .filter(|e| !e.dead && e.name.to_lowercase().contains(&needle))
        .min_by(|a, b| dist(a.pos, obs.own.pos).total_cmp(&dist(b.pos, obs.own.pos)))
        .unwrap_or_else(|| {
            let seen: Vec<_> = obs.visible.iter().map(|e| format!("{} {:?}", e.name, e.pos)).collect();
            eprintln!("FAIL: no visible live spawn matching '{name}'; visible: {seen:?}");
            std::process::exit(1);
        });
    let goal = target.pos;
    println!("goto {} at {:?} ({:.0} away)", target.name, goal, dist(goal, obs.own.pos));
    walker.slots().goto((goal[0], goal[1], goal[2]));
    step(&mut client, &mut walker, &obs);

    let mut last = obs;
    for _ in 0..MAX_TICKS {
        last = client.recv_observation().expect("observation");
        step(&mut client, &mut walker, &last);
        let st = walker.slots().status();
        if last.tick % 20 == 0 {
            println!("tick {}: {} at {:?}, {:.0} to go", last.tick, st.state, last.own.pos, dist(goal, last.own.pos));
        }
        if nav_state_is_terminal(&st.state) {
            break;
        }
    }
    let state = walker.slots().status().state;
    let left = dist(goal, last.own.pos);
    let arrived = state == "arrived" || left <= ARRIVE_DIST;
    println!("goto {}: state={state}, {left:.0} from the spawn", if arrived { "OK" } else { "FAIL" });
    if !arrived {
        finish(&mut client, 1);
    }

    if !cross {
        finish(&mut client, 0);
    }
    let goal_id = walker.zone_cross(last.own.pos).unwrap_or_else(|e| {
        eprintln!("FAIL: zone_cross: {e:?}");
        finish(&mut client, 1);
    });
    println!("zone_cross goal {goal_id}");
    for _ in 0..MAX_TICKS {
        last = match client.recv_observation() {
            Ok(o) => o,
            Err(e) => {
                println!("observation stream ended after the cross attempt: {e:?}");
                break;
            }
        };
        if last.own.zone_name != zone {
            println!("zone-cross OK: {zone} -> {}", last.own.zone_name);
            finish(&mut client, 0);
        }
        step(&mut client, &mut walker, &last);
        if last.tick % 20 == 0 {
            println!("tick {}: {} at {:?}", last.tick, walker.slots().status().state, last.own.pos);
        }
        if nav_state_is_terminal(&walker.slots().status().state) {
            break;
        }
    }
    eprintln!("FAIL: still in {zone}; at {:?}; status {:#?}", last.own.pos, walker.slots().status());
    finish(&mut client, 1);
}
