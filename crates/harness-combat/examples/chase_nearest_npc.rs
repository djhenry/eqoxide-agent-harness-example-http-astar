//! Reference example (eqoxide spec §10's anticipated `examples/` directory): connects to a live
//! eqoxide instance's Agent Plugin API socket and chases the nearest visible, living NPC for 20
//! ticks, printing what it sees. Run against a real eqoxide instance:
//!
//! ```text
//! # in the eqoxide checkout
//! cargo run -- --testzone --agent-socket /tmp/eqoxide-agent.sock
//! # in this repo
//! cargo run -p harness-combat --example chase_nearest_npc -- /tmp/eqoxide-agent.sock
//! ```

use eqoxide_agent_protocol::movement::AgentMovement;
use eqoxide_agent_protocol::step::Step;
use harness_combat::chase::chase_and_face;
use harness_socket::AgentClient;

fn main() {
    let socket_path = std::env::args().nth(1).unwrap_or_else(|| {
        eprintln!("usage: chase_nearest_npc <agent-socket-path>");
        std::process::exit(1);
    });

    let mut client = AgentClient::connect(&socket_path).expect("connect + handshake");
    println!("connected to {socket_path}, chasing the nearest visible NPC for 20 ticks");

    for _ in 0..20 {
        let obs = client.recv_observation().expect("recv observation");
        let nearest = obs
            .visible
            .iter()
            .filter(|e| e.is_npc && !e.dead)
            .min_by(|a, b| {
                let da = (a.pos[0] - obs.own.pos[0]).hypot(a.pos[1] - obs.own.pos[1]);
                let db = (b.pos[0] - obs.own.pos[0]).hypot(b.pos[1] - obs.own.pos[1]);
                da.total_cmp(&db)
            });

        let movement = match nearest {
            Some(target) => {
                println!(
                    "tick {}: chasing {} (hp {:.0}%)",
                    obs.tick, target.name, target.hp_pct
                );
                chase_and_face(&obs.own, target, 15.0)
            }
            None => {
                println!("tick {}: no NPC visible, standing still", obs.tick);
                AgentMovement {
                    dir: [0.0, 0.0],
                    up: 0.0,
                    jump: false,
                    wish_heading: None,
                }
            }
        };

        client
            .send_step(&Step {
                movement: Some(movement),
                verb: None,
            })
            .expect("send step");
    }
}
