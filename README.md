# eqoxide-agent-harness-example-http-astar

[![CI](https://github.com/djhenry/eqoxide-agent-harness-example-http-astar/actions/workflows/ci.yml/badge.svg)](https://github.com/djhenry/eqoxide-agent-harness-example-http-astar/actions/workflows/ci.yml)

A **reference** agent harness for [eqoxide](https://github.com/djhenry/eqoxide): A* navigation,
chase-and-face combat, and an optional HTTP convenience wrapper, all driving a running eqoxide
instance from a genuinely separate OS process over the [Agent Plugin
API](https://github.com/djhenry/eqoxide/blob/main/docs/agent-api.md)'s Unix domain socket only.
This harness uses no shared memory and no in-process coupling. See
[`docs/specs/2026-09-21-agent-harness-separation-design.md`](https://github.com/djhenry/eqoxide/blob/main/docs/specs/2026-09-21-agent-harness-separation-design.md)
§10 in the eqoxide repo for the design this implements.

This is a reference implementation, not a privileged first-party client. Any other agent-harness
project would integrate the same way: connect to `--agent-socket <path>`, handshake, and drive the
character with `Step` and `Observation`.

## Status

This repo is a **bootstrap**: each crate below has one small, real, tested vertical slice. Full
A* planner porting (`harness-nav`) and full HTTP route porting (`harness-http`) are tracked as
follow-up work — see [`docs/scope.md`](docs/scope.md) for exactly what's implemented today versus
deferred.

## Crates

| Crate | Purpose |
|---|---|
| `harness-socket` | Agent Plugin API client: handshake, send `Step`, receive `Observation` |
| `harness-nav` | Zone geometry loading, built on eqoxide's shared `eqoxide-zone-geometry` crate |
| `harness-combat` | Chase-and-face-while-engaged, computed from raw `Observation` data |
| `harness-http` | Optional HTTP convenience wrapper around `harness-socket` |

## Running against a live eqoxide instance

```bash
# in the eqoxide checkout
cargo run -- --testzone --agent-socket /tmp/eqoxide-agent.sock

# in this repo
cargo run -p harness-http
```

## Dependency pinning

This workspace depends on `eqoxide-agent-protocol`, `eqoxide-assets`, and `eqoxide-zone-geometry`
as **git** dependencies pinned to one commit on eqoxide's `worktree-rl-api-design` branch (that
work is not on `main` yet — see eqoxide spec §13). See
[`docs/scope.md`](docs/scope.md#bumping-the-eqoxide-pin) for the bump procedure.
