//! Every `/v1/...` request the harness does not serve itself is forwarded to eqoxide as a `Call`
//! over the agent socket, and eqoxide's reply is turned back into an HTTP response. The harness
//! never answers one of these itself, so what the caller sees is what eqoxide said.
//!
//! The call's `op` is `"<METHOD> <path>"`, its `args` carry `query` (the query parameters) and
//! `body` (the JSON request body), as eqoxide's `ApiDispatcher` expects.

use crate::driver::Control;
use axum::body::Bytes;
use axum::extract::State;
use axum::http::{Method, StatusCode, Uri};
use axum::response::{IntoResponse, Response};
use axum::Json;
use eqoxide_agent_protocol::call::CallStatus;
use serde_json::{json, Map, Value};
use std::sync::Arc;

/// Largest request body forwarded.
const MAX_BODY_BYTES: usize = 1024 * 1024;

pub fn http_status(status: CallStatus) -> StatusCode {
    match status {
        CallStatus::Ok => StatusCode::OK,
        CallStatus::Accepted => StatusCode::ACCEPTED,
        CallStatus::Invalid => StatusCode::BAD_REQUEST,
        CallStatus::NotFound => StatusCode::NOT_FOUND,
        CallStatus::Conflict => StatusCode::CONFLICT,
        CallStatus::Unavailable => StatusCode::SERVICE_UNAVAILABLE,
        CallStatus::Failed => StatusCode::INTERNAL_SERVER_ERROR,
    }
}

fn refuse(status: StatusCode, why: String) -> Response {
    (status, Json(json!({ "error": why }))).into_response()
}

/// The `args` for a forwarded request, or why the request cannot be forwarded.
fn build_args(uri: &Uri, body: &Bytes) -> Result<Value, String> {
    let mut args = Map::new();
    if let Some(q) = uri.query() {
        let query: Map<String, Value> = form_urlencoded::parse(q.as_bytes())
            .map(|(k, v)| (k.into_owned(), Value::String(v.into_owned())))
            .collect();
        if !query.is_empty() {
            args.insert("query".into(), Value::Object(query));
        }
    }
    if !body.is_empty() {
        let parsed: Value =
            serde_json::from_slice(body).map_err(|e| format!("the body is not JSON: {e}"))?;
        args.insert("body".into(), parsed);
    }
    Ok(Value::Object(args))
}

pub async fn forward(
    State(control): State<Arc<Control>>,
    method: Method,
    uri: Uri,
    body: Bytes,
) -> Response {
    let path = uri.path();
    if !path.starts_with("/v1/") {
        return refuse(StatusCode::NOT_FOUND, format!("no route {path}"));
    }
    if body.len() > MAX_BODY_BYTES {
        return refuse(
            StatusCode::PAYLOAD_TOO_LARGE,
            "the body is too large".into(),
        );
    }
    let args = match build_args(&uri, &body) {
        Ok(a) => a,
        Err(why) => return refuse(StatusCode::BAD_REQUEST, why),
    };
    let reply = control.call(format!("{method} {path}"), args).await;
    (http_status(reply.status), Json(reply.body)).into_response()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_refusal_status_maps_to_an_error_status() {
        for s in [
            CallStatus::Invalid,
            CallStatus::NotFound,
            CallStatus::Conflict,
            CallStatus::Unavailable,
            CallStatus::Failed,
        ] {
            assert!(!http_status(s).is_success(), "{s:?}");
        }
        assert_eq!(http_status(CallStatus::Accepted), StatusCode::ACCEPTED);
    }

    #[test]
    fn args_carry_query_and_body_and_omit_what_is_absent() {
        let uri: Uri = "/v1/observe/who?name=a%20b".parse().unwrap();
        let args = build_args(&uri, &Bytes::from_static(b"{\"x\":1}")).unwrap();
        assert_eq!(args, json!({"query": {"name": "a b"}, "body": {"x": 1}}));
        let none = build_args(&"/v1/x".parse().unwrap(), &Bytes::new()).unwrap();
        assert_eq!(none, json!({}));
        assert!(build_args(&"/v1/x".parse().unwrap(), &Bytes::from_static(b"nope")).is_err());
    }
}
