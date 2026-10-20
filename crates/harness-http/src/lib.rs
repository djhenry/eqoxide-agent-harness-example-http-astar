//! Optional HTTP convenience wrapper around harness-socket (eqoxide spec §10). This wrapper
//! talks to harness-socket, never to eqoxide directly.
//!
//! It serves the agent control routes eqoxide's own HTTP API used to carry: movement, navigation,
//! combat and interact verbs, and the observations that go with them. Routes queue intent for the
//! [`driver`], the only code that touches the socket. See docs/scope.md for what is not served.

pub mod driver;
mod routes;

use axum::routing::get;
use axum::{Json, Router};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
struct Health {
    status: String,
}

async fn health() -> Json<Health> {
    Json(Health {
        status: "ok".to_string(),
    })
}

/// The control routes and `GET /health`, answering from `control`.
pub fn router(control: std::sync::Arc<driver::Control>) -> Router {
    routes::router().route("/health", get(health)).with_state(control)
}

#[cfg(test)]
mod tests {
    use super::*;
    use tower::ServiceExt;

    #[tokio::test]
    async fn health_returns_200_and_ok_status() {
        let app = router(driver::Control::new(harness_nav::status::NavSlots::default()));
        let response = app
            .oneshot(
                axum::http::Request::builder()
                    .uri("/health")
                    .body(axum::body::Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(response.status(), axum::http::StatusCode::OK);
        let body = axum::body::to_bytes(response.into_body(), usize::MAX)
            .await
            .unwrap();
        let health: Health = serde_json::from_slice(&body).unwrap();
        assert_eq!(health.status, "ok");
    }
}
