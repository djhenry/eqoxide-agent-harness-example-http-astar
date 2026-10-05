//! Convenience binary that binds `harness_http::router()` to a local port (eqoxide spec §10).
//! This binary is optional, and runs out-of-process from eqoxide either way.

#[tokio::main]
async fn main() {
    let app = harness_http::router();
    let listener = tokio::net::TcpListener::bind("127.0.0.1:8090")
        .await
        .unwrap();
    println!("harness-http listening on http://127.0.0.1:8090 (routes: GET /health)");
    axum::serve(listener, app).await.unwrap();
}
