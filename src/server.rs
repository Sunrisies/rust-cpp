use axum::{extract::Multipart, http::StatusCode, response::Json, routing::post, Router};
use cover_map_obs_plan_rs::{
    initialize, plan, terminate, MapData, ObstacleData, PlanningParams, PolygonData,
};
use serde::{Deserialize, Serialize};
use std::net::SocketAddr;
use std::sync::Mutex;
use std::time::Duration;
use tower_http::cors::{AllowOrigin, CorsLayer};

// ─── 请求体 (JSON) ───

#[derive(Debug, Deserialize)]
struct PlanRequest {
    width: f64,
    yaw: f64,
    polygon: Vec<LatLonReq>,
    speed: f64,
    dir: f64,
    #[serde(default)]
    pyg_polygon: Vec<LonLatReq>,
    #[serde(default)]
    pyg_pnt_str: Vec<i32>,
    #[serde(default)]
    pyg_state_str: Vec<f64>,
    #[serde(default)]
    obs_r_str: Vec<f64>,
    #[serde(default)]
    obs_center_str: Vec<LonLatReq>,
    #[serde(default = "default_safe_dist")]
    safe_dist_obs: f64,
    #[serde(default = "default_safe_dist")]
    safe_dist_map: f64,
    #[serde(default)]
    long_edge_yaw_flag: i32,
}

#[derive(Debug, Deserialize)]
struct LatLonReq {
    lat: f64,
    lon: f64,
}

#[derive(Debug, Deserialize, Clone)]
struct LonLatReq {
    lon: f64,
    lat: f64,
}

fn default_safe_dist() -> f64 {
    0.0
}

// ─── 响应体 ───

#[derive(Debug, Serialize)]
struct PlanResponse {
    code: i32,
    msg: Option<String>,
    data: Vec<PlanResult>,
    count: Option<i32>,
    obj: Option<String>,
}

#[derive(Debug, Serialize)]
struct PlanResult {
    #[serde(rename = "wpTime")]
    wp_time: f64,
    #[serde(rename = "wpYaw")]
    wp_yaw: f64,
    list: Vec<Waypoint>,
    #[serde(rename = "wpArea")]
    wp_area: f64,
    #[serde(rename = "wpLen")]
    wp_len: f64,
}

#[derive(Debug, Serialize)]
struct Waypoint {
    lon: f64,
    lat: f64,
}

// ─── 全局锁 ───
static FFI_LOCK: Mutex<()> = Mutex::new(());

// ─── 核心规划逻辑 ───

fn run_planning(req: PlanRequest) -> (StatusCode, Json<PlanResponse>) {
    let _guard = match FFI_LOCK.lock() {
        Ok(g) => g,
        Err(_) => {
            return (
                StatusCode::INTERNAL_SERVER_ERROR,
                Json(PlanResponse {
                    code: -1,
                    msg: Some("Lock poisoned".into()),
                    data: vec![],
                    count: None,
                    obj: None,
                }),
            );
        }
    };

    initialize();

    let mut map = MapData::default();
    let cnt = req.polygon.len().min(200);
    map.Cnt = cnt as f64;
    for (i, p) in req.polygon.iter().enumerate().take(cnt) {
        map.Lat[i] = p.lat;
        map.Lon[i] = p.lon;
    }

    let mut pyg = PolygonData::default();
    if !req.pyg_polygon.is_empty() && !req.pyg_pnt_str.is_empty() {
        let total_pnt: usize = req.pyg_pnt_str.iter().map(|&c| c as usize).sum();
        let n = req.pyg_polygon.len().min(total_pnt.min(200));
        pyg.Cnt = req.pyg_pnt_str.len() as f64;
        for i in 0..pyg.Cnt as usize {
            pyg.Pnt[i] = req.pyg_pnt_str[i];
        }
        for i in 0..n {
            pyg.Lon[i] = req.pyg_polygon[i].lon;
            pyg.Lat[i] = req.pyg_polygon[i].lat;
            pyg.State[i] = req.pyg_state_str.get(i).copied().unwrap_or(1.0);
        }
    }

    let mut obs = ObstacleData::default();
    if !req.obs_r_str.is_empty() && !req.obs_center_str.is_empty() {
        let n = req.obs_r_str.len().min(req.obs_center_str.len()).min(50);
        obs.Cnt = n as f64;
        for i in 0..n {
            obs.R[i] = req.obs_r_str[i];
            obs.Lat[i] = req.obs_center_str[i].lat;
            obs.Lon[i] = req.obs_center_str[i].lon;
        }
    }

    let params = PlanningParams {
        width: req.width.max(0.1),
        yaw: req.yaw,
        dir: req.dir,
        speed: req.speed.max(0.1),
        safe_dist_obs: req.safe_dist_obs,
        safe_dist_map: req.safe_dist_map,
        long_edge_yaw_flag: req.long_edge_yaw_flag,
    };

    let result = match plan(&map, &pyg, &mut obs, &params) {
        Ok(r) => r,
        Err(e) => {
            terminate();
            return (
                StatusCode::OK,
                Json(PlanResponse {
                    code: -1,
                    msg: Some(format!("Planning failed: {e}")),
                    data: vec![],
                    count: None,
                    obj: None,
                }),
            );
        }
    };

    let list: Vec<Waypoint> = result
        .waypoint_points()
        .iter()
        .map(|wp| Waypoint {
            lon: wp.lon,
            lat: wp.lat,
        })
        .collect();

    let plan_result = PlanResult {
        wp_time: result.estimated_time,
        wp_yaw: result.yaw,
        list,
        wp_area: result.coverage_area,
        wp_len: result.path_length,
    };

    terminate();

    (
        StatusCode::OK,
        Json(PlanResponse {
            code: 1,
            msg: None,
            data: vec![plan_result],
            count: None,
            obj: None,
        }),
    )
}

// ─── JSON 处理 ───

async fn handle_json(Json(req): Json<PlanRequest>) -> (StatusCode, Json<PlanResponse>) {
    run_planning(req)
}

// ─── Multipart 处理 ───

fn parse_val_as_f64(v: &serde_json::Value, key: &str) -> Option<f64> {
    match v.get(key)? {
        serde_json::Value::Number(n) => n.as_f64(),
        serde_json::Value::String(s) => s.parse().ok(),
        _ => None,
    }
}

async fn handle_multipart(mut multipart: Multipart) -> (StatusCode, Json<PlanResponse>) {
    let mut req = PlanRequest {
        width: 0.0,
        yaw: 0.0,
        polygon: vec![],
        speed: 1.0,
        dir: 1.0,
        pyg_polygon: vec![],
        pyg_pnt_str: vec![],
        pyg_state_str: vec![],
        obs_r_str: vec![],
        obs_center_str: vec![],
        safe_dist_obs: 0.0,
        safe_dist_map: 0.0,
        long_edge_yaw_flag: 0,
    };

    while let Some(field) = match multipart.next_field().await {
        Ok(f) => f,
        Err(e) => {
            return (
                StatusCode::BAD_REQUEST,
                Json(PlanResponse {
                    code: -1,
                    msg: Some(format!("Multipart parse error: {e}")),
                    data: vec![],
                    count: None,
                    obj: None,
                }),
            );
        }
    } {
        let name = match field.name() {
            Some(n) => n.to_string(),
            None => continue,
        };
        let text = match field.text().await {
            Ok(t) => t,
            Err(_) => continue,
        };

        match name.as_str() {
            "width" => req.width = text.parse().unwrap_or(0.0),
            "yaw" => req.yaw = text.parse().unwrap_or(0.0),
            "speed" => req.speed = text.parse().unwrap_or(1.0),
            "dir" => req.dir = text.parse().unwrap_or(1.0),
            "safeDistObs" | "safe_dist_obs" => req.safe_dist_obs = text.parse().unwrap_or(0.0),
            "safeDistMap" | "safe_dist_map" => req.safe_dist_map = text.parse().unwrap_or(0.0),
            "longEdgeYawFlag" | "long_edge_yaw_flag" => {
                req.long_edge_yaw_flag = text.parse().unwrap_or(0)
            }
            "polygon" => {
                if let Ok(v) = serde_json::from_str::<Vec<serde_json::Value>>(&text) {
                    req.polygon = v
                        .iter()
                        .filter_map(|p| {
                            Some(LatLonReq {
                                lat: parse_val_as_f64(p, "lat")?,
                                lon: parse_val_as_f64(p, "lon")?,
                            })
                        })
                        .collect();
                }
            }
            "pygPolygon" | "pyg_polygon" => {
                if !text.is_empty() && text != "[]" {
                    if let Ok(v) = serde_json::from_str::<Vec<serde_json::Value>>(&text) {
                        req.pyg_polygon = v
                            .iter()
                            .filter_map(|p| {
                                Some(LonLatReq {
                                    lon: parse_val_as_f64(p, "lon")?,
                                    lat: parse_val_as_f64(p, "lat")?,
                                })
                            })
                            .collect();
                    }
                }
            }
            "pygPntStr" | "pyg_pnt_str" => {
                if !text.is_empty() && text != "[]" {
                    if let Ok(v) = serde_json::from_str::<Vec<i32>>(&text) {
                        req.pyg_pnt_str = v;
                    }
                }
            }
            "pygStateStr" | "pyg_state_str" => {
                if !text.is_empty() && text != "[]" {
                    if let Ok(v) = serde_json::from_str::<Vec<f64>>(&text) {
                        req.pyg_state_str = v;
                    }
                }
            }
            "obsRStr" | "obs_r_str" => {
                if !text.is_empty() && text != "[]" {
                    if let Ok(v) = serde_json::from_str::<Vec<f64>>(&text) {
                        req.obs_r_str = v;
                    }
                }
            }
            "obsCenterStr" | "obs_center_str" => {
                if !text.is_empty() && text != "[]" {
                    if let Ok(v) = serde_json::from_str::<Vec<serde_json::Value>>(&text) {
                        req.obs_center_str = v
                            .iter()
                            .filter_map(|p| {
                                Some(LonLatReq {
                                    lon: parse_val_as_f64(p, "lon")?,
                                    lat: parse_val_as_f64(p, "lat")?,
                                })
                            })
                            .collect();
                    }
                }
            }
            _ => {}
        }
    }

    run_planning(req)
}

// ─── 启动 ───

#[tokio::main]
async fn main() {
    let cors = CorsLayer::new()
        .allow_origin(AllowOrigin::mirror_request())
        .allow_methods([axum::http::Method::POST, axum::http::Method::OPTIONS])
        .allow_headers([
            axum::http::header::CONTENT_TYPE,
            axum::http::header::AUTHORIZATION,
        ])
        .allow_credentials(true)
        .max_age(Duration::from_secs(86400));

    let app = Router::new()
        .route(
            "/mapobsplan/mapObsPlan/polygonPoint",
            post(handle_json).options(|| async { StatusCode::OK }),
        )
        .route(
            "/mapobsplan/mapObsPlan/polygonPointForm",
            post(handle_multipart).options(|| async { StatusCode::OK }),
        )
        .layer(cors);

    let addr = SocketAddr::from(([0, 0, 0, 0], 3000));
    println!("Server starting on http://{}", addr);
    println!("POST /mapobsplan/mapObsPlan/polygonPoint        (JSON)");
    println!("POST /mapobsplan/mapObsPlan/polygonPointForm    (multipart/form-data)");

    let listener = tokio::net::TcpListener::bind(addr).await.unwrap();
    axum::serve(listener, app).await.unwrap();
}
