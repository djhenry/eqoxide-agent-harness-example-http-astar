# Scope

This repo bootstraps the reference agent harness described in eqoxide's
[`docs/specs/2026-09-21-agent-harness-separation-design.md`](https://github.com/djhenry/eqoxide/blob/main/docs/specs/2026-09-21-agent-harness-separation-design.md)
§10. This repo intentionally does not implement everything described by that design yet.

## Implemented today

- **`harness-socket`**: a real Agent Plugin API client that connects, handshakes, sends `Step`
  values, and receives `Observation` values. Tests exercise this client against an in-process
  fake server, not yet against a live eqoxide instance in CI.
- **`harness-nav`**: `load_zone(glb_path, cell_size)` and `load_zone_from_cache(asset_cache_dir, zone)`
  build a real `Collision` via the pinned `eqoxide-zone-geometry` and `eqoxide-assets` crates, plus
  the ported planner, steering and walker.
- **`harness-combat`**: `chase_and_face(own, target, engage_range)`, a pure function computing
  movement and facing toward a visible target from raw `Observation` data.
- **`harness-http`**: the agent control routes, driven through `harness-socket`. One driver thread
  owns the socket, answers every `Observation` with one `Step`, and runs `harness-nav`'s walker and
  `harness-combat`'s engage driver. Route handlers only publish intent to it. Served, at the same
  `/v1/<group>/<action>` paths eqoxide's own HTTP API used:
  - `move`: `goto`, `follow`, `stop`, `manual`, `jump`
  - `combat`: `target`, `target/name`, `attack` (`POST` and `DELETE`), `consider`, `cast`
  - `interact`: `sit`, `stand`. `lifecycle`: `respawn`
  - `observe`: `state` (the whole `Observation`), `entities`, `nav` (goal status and walker notices),
    `zone_lines` (each zone line's index and a standable point). There is no cross route: walk to a
    line with `goto` and eqoxide fires the crossing itself.
  - `GET /health`
- **`crates/harness-combat/examples/chase_nearest_npc.rs`**: a worked, buildable example
  combining `harness-socket` and `harness-combat` the way a real caller would: connect, watch for
  the nearest visible living NPC, chase and face it. Needs a live eqoxide instance to actually run
  (`cargo run -p harness-combat --example chase_nearest_npc -- <socket-path>`).

## Deliberately deferred (follow-up plans, not started)

- **Full A* planner/walker/steering port** into `harness-nav`, from eqoxide's
  `crates/eqoxide-nav/src/{planner,walker,steering,traversability}.rs` (~19,000 lines combined in
  the eqoxide repo today). A separate, dedicated plan covers this port.
- **The rest of eqoxide's HTTP surface** (`merchant`, `inventory`, `quests`, `chat`, `group`,
  `guild`, `pet`, `trainer`, `social`, `camera`, the `events` feed, and the rendered-frame and
  packet observe routes). The Agent Plugin API carries no verb or observation for them yet:
  `AgentVerb`'s `Merchant`, `Inventory`, `Quests` and `Chat` groups are empty enums, and
  `Observation` has no door, zone-point or message data. Each route group returns to this repo
  as the protocol grows its verb. Two routes differ from eqoxide's: `zone_cross` is gone, because crossing is passive, and `manual`
  cancels any nav goal instead of yielding to keyboard input.
- **A checked-in test `.glb` fixture** for `harness-nav`'s `load_zone` itself. Today `load_zone`'s
  own test only checks the not-found-path error case. Eqoxide's `test_ready()` fixture covers the
  "does it actually build a real `Collision`" case indirectly, bypassing `from_glb` entirely.

## Bumping the eqoxide pin

Every git dependency in this workspace on `https://github.com/djhenry/eqoxide.git` **must** share
one `rev`. To bump it (e.g. when eqoxide's `main` gains commits needed by this harness):

1. `git -C ~/git/eqoxide log --oneline -1` (or check the PR) to get the new commit sha.
2. Replace every occurrence of the old sha with the new one across every `Cargo.toml` in
   `crates/*/Cargo.toml` in this repo. `grep -rl 'rev = "' crates/*/Cargo.toml` lists every file
   that needs the edit.
3. `cargo build --workspace` to confirm the new commit still provides everything this workspace
   depends on.
4. Commit all the manifest changes together in one commit (a partial bump leaves some crates
   pinned to the old rev and others to the new one. This mismatch is the exact "two revs, one
   URL" error warned about by the bootstrap plan's Global Constraints.)
