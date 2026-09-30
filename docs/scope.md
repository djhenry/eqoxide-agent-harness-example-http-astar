# Scope

This repo bootstraps the reference agent harness described in eqoxide's
[`docs/specs/2026-09-21-agent-harness-separation-design.md`](https://github.com/djhenry/eqoxide/blob/main/docs/specs/2026-09-21-agent-harness-separation-design.md)
§10. It intentionally does not implement everything that design describes yet.

## Implemented today

- **`harness-socket`** — a real Agent Plugin API client: connect, handshake, send `Step`, receive
  `Observation`. Tested against an in-process fake server, not yet exercised against a live
  eqoxide instance in CI.
- **`harness-nav`** — `load_zone(glb_path, cell_size)` builds a real `Collision` via the pinned
  `eqoxide-zone-geometry`/`eqoxide-assets` crates. No A* planner, walker, or steering yet.
- **`harness-combat`** — `chase_and_face(own, target, engage_range)`, a pure function computing
  movement + facing toward a visible target from raw `Observation` data.
- **`harness-http`** — a `GET /health` route proving the axum scaffolding's place in the
  workspace. No routes that actually drive `harness-socket` yet.
- **`crates/harness-combat/examples/chase_nearest_npc.rs`** — a worked, buildable example
  combining `harness-socket` + `harness-combat` the way a real caller would: connect, watch for
  the nearest visible living NPC, chase and face it. Needs a live eqoxide instance to actually run
  (`cargo run -p harness-combat --example chase_nearest_npc -- <socket-path>`).

## Deliberately deferred (follow-up plans, not started)

- **Full A* planner/walker/steering port** into `harness-nav`, from eqoxide's
  `crates/eqoxide-nav/src/{planner,walker,steering,traversability}.rs` (~19,000 lines combined in
  the eqoxide repo today). This is its own dedicated plan.
- **Full HTTP route port** into `harness-http`, from eqoxide's `crates/eqoxide-http/src/` (~20,000
  lines of route handlers). Also its own dedicated plan — and optional per spec §10 even then.
- **§11's `asset_cache_dir` handshake field.** eqoxide spec §11 proposes eqoxide disclosing its
  on-disk asset cache path during the handshake, so `harness-nav` can auto-discover zone files
  instead of a caller supplying a GLB path by hand. That protocol addition (eqoxide spec §9/§11)
  hasn't landed in `eqoxide-agent-protocol` yet — check `HandshakeReply::Accepted` in eqoxide's
  `crates/eqoxide-agent-protocol/src/handshake.rs`; as long as it's a unit variant with no fields,
  this hasn't happened, and `load_zone` needs an explicit path.
- **A checked-in test `.glb` fixture** for `harness-nav`'s `load_zone` itself. Today its own test
  only checks the not-found-path error case; the "does it actually build a real Collision" case is
  covered indirectly via eqoxide's `test_ready()` fixture, which bypasses `from_glb` entirely.

## Bumping the eqoxide pin

Every git dependency in this workspace on `https://github.com/djhenry/eqoxide.git` **must** share
one `rev`. To bump it (e.g. once eqoxide's `worktree-rl-api-design` branch gains new commits this
harness needs, or once that branch merges to `main`):

1. `git -C ~/git/eqoxide log --oneline -1` (or check the PR) to get the new commit sha.
2. Replace every occurrence of the old sha with the new one across every `Cargo.toml` in
   `crates/*/Cargo.toml` in this repo — `grep -rl 'rev = "' crates/*/Cargo.toml` lists every file
   that needs the edit.
3. `cargo build --workspace` to confirm the new commit still provides everything this workspace
   depends on.
4. Commit all the manifest changes together in one commit (a partial bump leaves some crates
   pinned to the old rev and others to the new one, which is the exact "two revs, one URL" error
   Global Constraints in the bootstrap plan warns about).
