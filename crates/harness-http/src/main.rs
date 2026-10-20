//! Convenience binary that connects to a running eqoxide's `--agent-socket`, drives it with
//! `harness-nav` and `harness-combat`, and serves the control routes over HTTP (eqoxide spec §10).
//! This binary is optional, and runs out-of-process from eqoxide either way.
//!
//! ```text
//! harness-http <agent-socket> [bind-addr]     # bind-addr defaults to 127.0.0.1:8090
//! ```

use harness_http::driver::{self, Control};
use harness_nav::status::NavSlots;
use harness_socket::AgentClient;

#[tokio::main]
async fn main() {
    let mut args = std::env::args().skip(1);
    let Some(socket) = args.next() else {
        eprintln!("usage: harness-http <agent-socket> [bind-addr]");
        std::process::exit(2);
    };
    let addr = args.next().unwrap_or_else(|| "127.0.0.1:8090".to_string());

    let client = AgentClient::connect(&socket).expect("connect + handshake");
    let loader = driver::cache_loader(client.asset_cache_dir().to_path_buf());
    let control = Control::new(NavSlots::default());
    driver::spawn(client, control.clone(), loader);

    let listener = tokio::net::TcpListener::bind(&addr).await.unwrap();
    println!("harness-http listening on http://{addr}, driving {socket}");
    axum::serve(listener, harness_http::router(control)).await.unwrap();
}
