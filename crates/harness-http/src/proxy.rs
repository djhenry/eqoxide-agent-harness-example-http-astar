//! Every `/v1/...` request the harness does not serve itself is forwarded to eqoxide as a `Call`
//! over the agent socket, and eqoxide's reply is turned back into an HTTP response. The harness
//! never answers one of these itself, so what the caller sees is what eqoxide said.
//!
//! The route `/v1/<group>/<action>` names the op `<group>.<action>`. The query parameters and the
//! JSON body merge into the op's one `args` object, and [`Op::from_wire`] reads them as the typed
//! op. Two routes are the exception. `/v1/social/friends` is `social.friends` for a `GET` and
//! `social.edit_friends` for a `POST`, and `/v1/events/<category>` is `events.read` with that
//! `category`.

use crate::driver::Control;
use axum::body::Bytes;
use axum::extract::State;
use axum::http::{Method, StatusCode, Uri};
use axum::response::{IntoResponse, Response};
use axum::Json;
use eqoxide_agent_protocol::call::CallStatus;
use eqoxide_agent_protocol::op::{Op, ReadError};
use serde_json::{json, Map, Value};
use std::sync::Arc;

/// Largest request body forwarded.
const MAX_BODY_BYTES: usize = 1024 * 1024;

/// The HTTP status for the `CallStatus` of a reply.
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

/// Whether the op only reads state, so only a `GET` may run it.
fn is_read(name: &str) -> bool {
    name.starts_with("observe.")
        || name.starts_with("events.")
        || matches!(
            name,
            "group.roster"
                | "guild.roster"
                | "merchant.list"
                | "quests.log"
                | "quests.completed"
                | "quests.offers"
                | "social.friends"
                | "trainer.list"
        )
}

/// The op name for a route and the arguments the route itself supplies, or `None` for a path that
/// is not `/v1/<group>/<action>`.
fn route_op(method: &Method, path: &str) -> Option<(String, Map<String, Value>)> {
    let mut parts = path.strip_prefix("/v1/")?.split('/');
    let (group, action) = (parts.next()?, parts.next()?);
    if parts.next().is_some() || group.is_empty() || action.is_empty() {
        return None;
    }
    let mut args = Map::new();
    let name = match (group, action) {
        ("events", "all") => "events.read".to_owned(),
        ("events", category) => {
            args.insert("category".into(), Value::String(category.to_owned()));
            "events.read".to_owned()
        }
        ("social", "friends") if method == Method::POST => "social.edit_friends".to_owned(),
        _ => format!("{group}.{action}"),
    };
    Some((name, args))
}

/// The arguments of a request: the route's own, the query parameters, then the JSON body. A key
/// given twice is refused, and a body that is not an object is refused.
fn build_args(
    route: Map<String, Value>,
    uri: &Uri,
    body: &Bytes,
    numbers: bool,
) -> Result<Map<String, Value>, String> {
    let mut args = route;
    let mut put = |key: String, value: Value| match args.insert(key.clone(), value) {
        None => Ok(()),
        Some(_) => Err(format!("`{key}` is given more than once")),
    };
    if let Some(q) = uri.query() {
        for (k, v) in form_urlencoded::parse(q.as_bytes()) {
            put(k.into_owned(), query_value(&v, numbers))?;
        }
    }
    if !body.is_empty() {
        match serde_json::from_slice(body).map_err(|e| format!("the body is not JSON: {e}"))? {
            Value::Object(fields) => {
                for (k, v) in fields {
                    put(k, v)?;
                }
            }
            Value::Null => {}
            _ => return Err("the body must be a JSON object".into()),
        }
    }
    Ok(args)
}

/// A query parameter as a JSON value. A query string has no types, so with `numbers` set a value
/// that reads as an integer becomes a number and any other value stays a string.
fn query_value(raw: &str, numbers: bool) -> Value {
    if numbers {
        if let Ok(n) = raw.parse::<u64>() {
            return json!(n);
        }
        if let Ok(n) = raw.parse::<i64>() {
            return json!(n);
        }
    }
    Value::String(raw.to_owned())
}

/// Reads `name` with `args` as a typed op, or the reason the arguments do not fit it.
fn read_op(name: &str, args: Map<String, Value>) -> Result<Op, ReadError> {
    if args.is_empty() {
        // an op with no arguments reads without `args`, and one that needs some names the first
        // missing field when it reads from an empty object
        return Op::from_wire(name, None).or_else(|_| Op::from_wire(name, Some(json!({}))));
    }
    Op::from_wire(name, Some(Value::Object(args)))
}

/// The op a request names, or the status and reason it cannot become one. A request whose method
/// does not suit a known op is refused as a 405 ahead of any argument error, and an unknown route
/// stays a 404.
fn request_op(method: &Method, uri: &Uri, body: &Bytes) -> Result<Op, (StatusCode, String)> {
    let path = uri.path();
    let (name, route_args) = route_op(method, path)
        .ok_or_else(|| (StatusCode::NOT_FOUND, format!("no route {path}")))?;
    let bad = |why: String| (StatusCode::BAD_REQUEST, why);
    // a query string has no types, so numbers are tried first and a string read is the fallback
    // for an argument such as a name that is all digits
    let typed = build_args(route_args.clone(), uri, body, true).map_err(bad)?;
    let read = match read_op(&name, typed) {
        Ok(op) => Ok(op),
        Err(ReadError::UnknownOp(_)) => {
            return Err((StatusCode::NOT_FOUND, format!("no route {path}")));
        }
        Err(first) => {
            let text = build_args(route_args, uri, body, false).map_err(bad)?;
            read_op(&name, text).map_err(|_| first)
        }
    };
    if is_read(&name) != (method == Method::GET) {
        return Err((
            StatusCode::METHOD_NOT_ALLOWED,
            format!("{path} does not answer {method}"),
        ));
    }
    read.map_err(|why| (StatusCode::BAD_REQUEST, why.to_string()))
}

pub async fn forward(
    State(control): State<Arc<Control>>,
    method: Method,
    uri: Uri,
    body: Bytes,
) -> Response {
    if !uri.path().starts_with("/v1/") {
        return refuse(StatusCode::NOT_FOUND, format!("no route {}", uri.path()));
    }
    if body.len() > MAX_BODY_BYTES {
        return refuse(
            StatusCode::PAYLOAD_TOO_LARGE,
            "the body is too large".into(),
        );
    }
    let op = match request_op(&method, &uri, &body) {
        Ok(op) => op,
        Err((status, why)) => return refuse(status, why),
    };
    let reply = control.call(op).await;
    let status = http_status(reply.status);
    (status, Json(error_body(status, reply.body))).into_response()
}

/// A reply body in the `{"error": ...}` shape when `status` is an error status. A host refusal is a
/// bare JSON string, and a body that already has an `error` field stays as it is.
fn error_body(status: StatusCode, body: Value) -> Value {
    if status.is_success() {
        return body;
    }
    match body {
        Value::String(why) => json!({ "error": why }),
        Value::Object(ref fields) if fields.contains_key("error") => body,
        other => json!({ "error": other }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use eqoxide_agent_protocol::op::{
        EventCategory, EventsRead, FriendsEdit, MerchantBuy, ObserveMessages,
    };

    fn op(method: Method, uri: &str, body: &'static str) -> Result<Op, (StatusCode, String)> {
        request_op(
            &method,
            &uri.parse().unwrap(),
            &Bytes::from_static(body.as_bytes()),
        )
    }

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
        assert_eq!(http_status(CallStatus::Ok), StatusCode::OK);
        assert_eq!(http_status(CallStatus::Accepted), StatusCode::ACCEPTED);
    }

    #[test]
    fn the_frame_route_is_the_frame_op_and_only_a_get_reads_it() {
        assert_eq!(
            op(Method::GET, "/v1/observe/frame", "").unwrap(),
            Op::ObserveFrame
        );
        assert_eq!(
            op(Method::POST, "/v1/observe/frame", "").unwrap_err().0,
            StatusCode::METHOD_NOT_ALLOWED
        );
    }

    #[test]
    fn a_route_names_its_op_and_a_body_fills_its_arguments() {
        assert_eq!(
            op(
                Method::POST,
                "/v1/merchant/buy",
                r#"{"merchant":"Vendor","slot":3}"#
            )
            .unwrap(),
            Op::MerchantBuy(MerchantBuy {
                merchant: "Vendor".into(),
                slot: 3
            })
        );
        assert_eq!(
            op(Method::GET, "/v1/merchant/list", "").unwrap(),
            Op::MerchantList
        );
    }

    #[test]
    fn query_parameters_are_typed_by_the_op() {
        assert_eq!(
            op(Method::GET, "/v1/observe/messages?kind=npc", "").unwrap(),
            Op::ObserveMessages(ObserveMessages {
                kind: Some("npc".into())
            })
        );
        assert_eq!(
            op(
                Method::GET,
                "/v1/events/chat?since=12&wait=5&directed=1",
                ""
            )
            .unwrap(),
            Op::EventsRead(EventsRead {
                category: Some(EventCategory::Chat),
                since: Some(12),
                wait: Some(5),
                directed: Some(1),
            })
        );
        // a name that is all digits stays a string
        assert_eq!(
            op(Method::GET, "/v1/observe/messages?kind=123", "").unwrap(),
            Op::ObserveMessages(ObserveMessages {
                kind: Some("123".into())
            })
        );
    }

    #[test]
    fn events_all_reads_every_category() {
        assert_eq!(
            op(Method::GET, "/v1/events/all", "").unwrap(),
            Op::EventsRead(EventsRead::default())
        );
    }

    #[test]
    fn a_category_outside_the_set_is_a_bad_argument_and_not_an_unknown_route() {
        for path in ["/v1/events/system", "/v1/events/read"] {
            let (code, why) = op(Method::GET, path, "").unwrap_err();
            assert_eq!(code, StatusCode::BAD_REQUEST, "{path}");
            assert!(why.contains("category"), "{why}");
        }
    }

    #[test]
    fn a_wrong_method_is_refused_before_the_arguments_are_checked() {
        let status = |r: Result<Op, (StatusCode, String)>| r.unwrap_err().0;
        assert_eq!(
            status(op(Method::GET, "/v1/merchant/buy", "")),
            StatusCode::METHOD_NOT_ALLOWED
        );
        assert_eq!(
            status(op(Method::POST, "/v1/observe/messages?kind[]=1", "")),
            StatusCode::METHOD_NOT_ALLOWED
        );
    }

    #[test]
    fn an_error_body_is_always_an_error_object() {
        let bad = StatusCode::BAD_REQUEST;
        assert_eq!(error_body(bad, json!("why")), json!({ "error": "why" }));
        assert_eq!(
            error_body(bad, json!({ "error": "why" })),
            json!({ "error": "why" })
        );
        assert_eq!(error_body(StatusCode::OK, json!("fine")), json!("fine"));
    }

    #[test]
    fn the_friends_route_is_two_ops_by_method() {
        assert_eq!(
            op(Method::GET, "/v1/social/friends", "").unwrap(),
            Op::SocialFriends
        );
        assert_eq!(
            op(Method::POST, "/v1/social/friends", r#"{"add":"Alpha"}"#).unwrap(),
            Op::SocialEditFriends(FriendsEdit {
                add: Some("Alpha".into()),
                remove: None
            })
        );
    }

    #[test]
    fn a_wrong_method_unknown_route_or_bad_argument_is_refused_with_its_own_status() {
        let status = |r: Result<Op, (StatusCode, String)>| r.unwrap_err().0;
        assert_eq!(
            status(op(Method::POST, "/v1/observe/skills", "")),
            StatusCode::METHOD_NOT_ALLOWED
        );
        assert_eq!(
            status(op(Method::GET, "/v1/merchant/buy?merchant=V&slot=3", "")),
            StatusCode::METHOD_NOT_ALLOWED
        );
        assert_eq!(
            status(op(Method::GET, "/v1/no/such", "")),
            StatusCode::NOT_FOUND
        );
        assert_eq!(
            status(op(Method::GET, "/v1/observe/skills/extra", "")),
            StatusCode::NOT_FOUND
        );
        assert_eq!(
            status(op(Method::POST, "/v1/merchant/buy", "")),
            StatusCode::BAD_REQUEST
        );
        let (code, why) = op(
            Method::POST,
            "/v1/merchant/buy",
            r#"{"merchant":"V","slot":"x"}"#,
        )
        .unwrap_err();
        assert_eq!(code, StatusCode::BAD_REQUEST);
        assert!(why.contains("slot"), "{why}");
        assert_eq!(
            status(op(Method::POST, "/v1/merchant/buy", "nope")),
            StatusCode::BAD_REQUEST
        );
        assert_eq!(
            status(op(Method::POST, "/v1/merchant/buy", "[1]")),
            StatusCode::BAD_REQUEST
        );
    }

    #[test]
    fn an_argument_given_in_the_query_and_the_body_is_refused() {
        let (code, why) = op(
            Method::POST,
            "/v1/merchant/buy?slot=1",
            r#"{"merchant":"V","slot":2}"#,
        )
        .unwrap_err();
        assert_eq!(code, StatusCode::BAD_REQUEST);
        assert!(why.contains("slot"), "{why}");
    }
}
