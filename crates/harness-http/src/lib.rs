//! Optional HTTP convenience wrapper around harness-socket (eqoxide spec §10) — talks to
//! harness-socket, never to eqoxide directly.
//!
//! This task only wires up a `/health` endpoint to prove the axum scaffolding and its place in
//! the workspace. Real socket-backed routes (movement, combat verbs, observation polling) are
//! out of scope for this plan — see docs/scope.md.

use axum::routing::get;
use axum::{Json, Router};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
struct Health {
    status: String,
}

async fn health() -> Json<Health> {
    Json(Health { status: "ok".to_string() })
}

pub fn router() -> Router {
    Router::new().route("/health", get(health))
}

#[cfg(test)]
mod tests {
    use super::*;
    use tower::ServiceExt;

    #[tokio::test]
    async fn health_returns_200_and_ok_status() {
        let app = router();
        let response = app
            .oneshot(axum::http::Request::builder().uri("/health").body(axum::body::Body::empty()).unwrap())
            .await
            .unwrap();
        assert_eq!(response.status(), axum::http::StatusCode::OK);
        let body = axum::body::to_bytes(response.into_body(), usize::MAX).await.unwrap();
        let health: Health = serde_json::from_slice(&body).unwrap();
        assert_eq!(health.status, "ok");
    }
}
