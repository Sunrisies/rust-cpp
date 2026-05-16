// FFI bindings to C++ functions
#![allow(dead_code, non_camel_case_types, non_snake_case)]

use crate::types::*;
use std::os::raw::{c_double, c_int};

// C ABI shim declarations. The shim is implemented in cpp_shim.cpp and calls
// the original MATLAB Coder generated C++ functions.
extern "C" {
    // Initialize the C++ library
    pub fn cover_map_obs_plan_initialize_shim();

    // Terminate the C++ library
    pub fn cover_map_obs_plan_terminate_shim();

    // Main planning function from C++
    pub fn cover_map_obs_plan_run_shim(
        map: *const MapData,
        pyg: *const PolygonData,
        obs: *mut ObstacleData,
        params: *const PlanningParams,
        wpCnt: *mut c_double,
        wp: *mut WaypointData,
        wpLen: *mut c_double,
        wpArea: *mut c_double,
        wpTime: *mut c_double,
        wpYaw: *mut c_double,
        Err: *mut c_double,
    );

    pub fn cover_map_obs_plan_main_shr_out_shim(
        lat_in: *const c_double,
        lon_in: *const c_double,
        len: c_double,
        distance: c_double,
        shr_out_vertex: *mut c_double,
        out_cnt: *mut c_double,
    );
    pub fn cover_map_obs_plan_main_shr_out_expand_shim(
        lat_in: *const c_double,
        lon_in: *const c_double,
        len: c_int,
        distance: c_double,
        shr_out_vertex: *mut c_double,
        out_cnt: *mut c_int,
    );

    pub fn cover_map_obs_plan_cover_map_by_yaw_shim(
        map: *const c_double,
        map_cnt: c_double,
        width: c_double,
        yaw: c_double,
        f2c: c_double,
        dir: c_double,
        wp_cnt: *mut c_double,
        wp: *mut c_double,
    );

    pub fn cover_map_obs_plan_edge_collision_shim(
        map_new: *const c_double,
        map_cnt: c_double,
        c_points: *const c_double,
        c3_points_out: *mut c_double,
        c3_points_cnt: *mut c_double,
    );

    pub fn cover_map_obs_plan_circle_avoid_rtl_shim(
        p_cn_new: *const c_double,
        p_nc: c_double,
        obs_circle: *const c_double,
        obs_cnt: c_double,
        my_eps: c_double,
        p_n_new: *mut c_double,
        p_n_new_cnt: *mut c_double,
    );

    pub fn cover_map_obs_plan_pyg_obs_collision_shim(
        map_new: *const c_double,
        map_rows: std::os::raw::c_int,
        map_cnt: std::os::raw::c_int,
        pyg_state: c_double,
        c_points: *const c_double,
        c3_points_out: *mut c_double,
        c3_points_cnt: *mut c_double,
    );

    pub fn cover_map_obs_plan_pyg_obs_avoid_shim(
        p_cn_new: *const c_double,
        p_nc: c_double,
        pyg_new: *const c_double,
        pyg_pnt: *const std::os::raw::c_int,
        pyg_cnt: c_double,
        pyg_state: *const c_double,
        p_n_new: *mut c_double,
        p_n_new_cnt: *mut c_double,
    );

    // Additional C++ functions can be added here as needed
    // For example:
    // pub fn coverMapbyYaw(...);
    // pub fn circleAvoidRTL(...);
    // pub fn pygObsAvoid(...);
}

// Safe Rust wrapper for C++ functions
pub fn initialize() {
    unsafe {
        cover_map_obs_plan_initialize_shim();
    }
}

#[cfg(test)]
pub fn run_cpp_pyg_obs_avoid(
    path: &[(f64, f64, f64)],
    polygons: &[Vec<(f64, f64, f64)>],
    states: &[f64],
) -> (Vec<(f64, f64)>, f64) {
    let p_len = path.len().min(5000);
    let mut p_cn_new = [0.0; 15000];
    for (i, (x, y, state)) in path.iter().copied().take(p_len).enumerate() {
        p_cn_new[i * 3] = x;
        p_cn_new[i * 3 + 1] = y;
        p_cn_new[i * 3 + 2] = state;
    }

    let pyg_cnt = polygons.len().min(200);
    let mut pyg_new = [0.0; 600];
    let mut pyg_pnt = [0; 200];
    let mut pyg_state = [0.0; 200];
    let mut offset = 0;
    for (poly_idx, poly) in polygons.iter().take(pyg_cnt).enumerate() {
        pyg_pnt[poly_idx] = poly.len() as std::os::raw::c_int;
        for (point_idx, (x, y, state)) in poly.iter().copied().enumerate() {
            let i = offset + point_idx;
            pyg_new[i] = x;
            pyg_new[200 + i] = y;
            pyg_new[400 + i] = state;
            pyg_state[i] = states.get(poly_idx).copied().unwrap_or(state);
        }
        offset += poly.len();
    }

    let mut p_n_new = [0.0; 10000];
    let mut p_n_new_cnt = 0.0;
    unsafe {
        cover_map_obs_plan_pyg_obs_avoid_shim(
            p_cn_new.as_ptr(),
            p_len as c_double,
            pyg_new.as_ptr(),
            pyg_pnt.as_ptr(),
            pyg_cnt as c_double,
            pyg_state.as_ptr(),
            p_n_new.as_mut_ptr(),
            &mut p_n_new_cnt,
        );
    }

    let points = (0..p_n_new_cnt as usize)
        .map(|i| (p_n_new[i * 2], p_n_new[i * 2 + 1]))
        .collect();
    (points, p_n_new_cnt)
}

#[cfg(test)]
pub fn run_cpp_pyg_obs_collision(
    polygon: &[(f64, f64, f64)],
    pyg_state: f64,
    c_points: (f64, f64, f64, f64),
) -> (Vec<(f64, f64)>, f64) {
    let rows = polygon.len().min(200);
    let mut map_new = [0.0; 600];
    for (i, (x, y, state)) in polygon.iter().copied().take(rows).enumerate() {
        map_new[i] = x;
        map_new[rows + i] = y;
        map_new[rows * 2 + i] = state;
    }

    let c_points = [c_points.0, c_points.1, 0.0, c_points.2, c_points.3, 0.0];
    let mut out = [0.0; 1000];
    let mut out_cnt = 0.0;
    unsafe {
        cover_map_obs_plan_pyg_obs_collision_shim(
            map_new.as_ptr(),
            rows as std::os::raw::c_int,
            rows as std::os::raw::c_int,
            pyg_state,
            c_points.as_ptr(),
            out.as_mut_ptr(),
            &mut out_cnt,
        );
    }

    let points = (0..out_cnt as usize)
        .map(|i| (out[i * 2], out[i * 2 + 1]))
        .collect();
    (points, out_cnt)
}

#[cfg(test)]
pub fn run_cpp_circle_avoid_rtl(
    path: &[(f64, f64)],
    obstacles: &[(f64, f64, f64)],
    my_eps: f64,
) -> (Vec<(f64, f64)>, f64) {
    let p_len = path.len().min(5000);
    let mut p_cn_new = [0.0; 10000];
    for (i, (x, y)) in path.iter().copied().take(p_len).enumerate() {
        p_cn_new[i * 2] = x;
        p_cn_new[i * 2 + 1] = y;
    }

    let obs_len = obstacles.len().min(50);
    let mut obs_circle = [0.0; 150];
    for (i, (x, y, r)) in obstacles.iter().copied().take(obs_len).enumerate() {
        obs_circle[i * 3] = x;
        obs_circle[i * 3 + 1] = y;
        obs_circle[i * 3 + 2] = r;
    }

    let mut p_n_new = [0.0; 10000];
    let mut p_n_new_cnt = 0.0;
    unsafe {
        cover_map_obs_plan_circle_avoid_rtl_shim(
            p_cn_new.as_ptr(),
            p_len as c_double,
            obs_circle.as_ptr(),
            obs_len as c_double,
            my_eps,
            p_n_new.as_mut_ptr(),
            &mut p_n_new_cnt,
        );
    }

    let points = (0..p_n_new_cnt as usize)
        .map(|i| (p_n_new[i * 2], p_n_new[i * 2 + 1]))
        .collect();
    (points, p_n_new_cnt)
}

#[cfg(test)]
pub fn run_cpp_edge_collision(
    map_points: &[(f64, f64)],
    c_points: (f64, f64, f64, f64),
) -> (Vec<(f64, f64)>, f64) {
    let len = map_points.len().min(200);
    let mut map = [0.0; 400];
    for (i, (x, y)) in map_points.iter().copied().take(len).enumerate() {
        map[i] = x;
        map[200 + i] = y;
    }

    let c_points = [c_points.0, c_points.1, c_points.2, c_points.3];
    let mut out = [0.0; 400];
    let mut out_cnt = 0.0;
    unsafe {
        cover_map_obs_plan_edge_collision_shim(
            map.as_ptr(),
            len as c_double,
            c_points.as_ptr(),
            out.as_mut_ptr(),
            &mut out_cnt,
        );
    }

    let points = (0..out_cnt as usize)
        .map(|i| (out[i * 2], out[i * 2 + 1]))
        .collect();
    (points, out_cnt)
}

#[cfg(test)]
pub fn run_cpp_cover_map_by_yaw(
    map_points: &[(f64, f64)],
    width: f64,
    yaw: f64,
    f2c: f64,
    dir: f64,
) -> (Vec<[f64; 3]>, f64) {
    let len = map_points.len().min(200);
    let mut map = [0.0; 400];
    for (i, (x, y)) in map_points.iter().copied().take(len).enumerate() {
        map[i] = x;
        map[200 + i] = y;
    }

    let mut wp = [0.0; 15000];
    let mut wp_cnt = 0.0;
    unsafe {
        cover_map_obs_plan_cover_map_by_yaw_shim(
            map.as_ptr(),
            len as c_double,
            width,
            yaw,
            f2c,
            dir,
            &mut wp_cnt,
            wp.as_mut_ptr(),
        );
    }

    let waypoints = (0..wp_cnt as usize)
        .map(|i| [wp[3 * i], wp[3 * i + 1], wp[3 * i + 2]])
        .collect();
    (waypoints, wp_cnt)
}

#[cfg(test)]
pub fn run_cpp_main_shr_out(lat: &[f64], lon: &[f64], distance: f64) -> (Vec<[f64; 3]>, f64) {
    let mut lat_in = [0.0; 200];
    let mut lon_in = [0.0; 200];
    let len = lat.len().min(lon.len()).min(200);
    lat_in[..len].copy_from_slice(&lat[..len]);
    lon_in[..len].copy_from_slice(&lon[..len]);

    let mut out = [0.0; 600];
    let mut out_cnt = 0.0;
    unsafe {
        cover_map_obs_plan_main_shr_out_shim(
            lat_in.as_ptr(),
            lon_in.as_ptr(),
            len as c_double,
            distance,
            out.as_mut_ptr(),
            &mut out_cnt,
        );
    }

    let vertices = (0..out_cnt as usize)
        .map(|i| [out[3 * i], out[3 * i + 1], out[3 * i + 2]])
        .collect();
    (vertices, out_cnt)
}

#[cfg(test)]
pub fn run_cpp_main_shr_out_expand(
    lat: &[f64],
    lon: &[f64],
    distance: f64,
) -> (Vec<[f64; 3]>, i32) {
    let mut lat_in = [0.0; 200];
    let mut lon_in = [0.0; 200];
    let len = lat.len().min(lon.len()).min(200);
    lat_in[..len].copy_from_slice(&lat[..len]);
    lon_in[..len].copy_from_slice(&lon[..len]);

    let mut out = [0.0; 600];
    let mut out_cnt = 0;
    unsafe {
        cover_map_obs_plan_main_shr_out_expand_shim(
            lat_in.as_ptr(),
            lon_in.as_ptr(),
            len as c_int,
            distance,
            out.as_mut_ptr(),
            &mut out_cnt,
        );
    }

    let vertices = (0..out_cnt as usize)
        .map(|i| [out[3 * i], out[3 * i + 1], out[3 * i + 2]])
        .collect();
    (vertices, out_cnt)
}

pub fn terminate() {
    unsafe {
        cover_map_obs_plan_terminate_shim();
    }
}

pub fn run_planning(
    map: &MapData,
    pyg: &PolygonData,
    obs: &mut ObstacleData,
    params: &PlanningParams,
) -> Result<PlanningResult, String> {
    let mut wp_cnt = 0.0;
    let mut wp = WaypointData {
        Lat: [0.0; 5000],
        Lon: [0.0; 5000],
        State: [0.0; 5000],
    };
    let mut wp_len = 0.0;
    let mut wp_area = 0.0;
    let mut wp_time = 0.0;
    let mut wp_yaw = 0.0;
    let mut err = 0.0;

    unsafe {
        cover_map_obs_plan_run_shim(
            map,
            pyg,
            obs,
            params,
            &mut wp_cnt,
            &mut wp,
            &mut wp_len,
            &mut wp_area,
            &mut wp_time,
            &mut wp_yaw,
            &mut err,
        );
    }

    if err != 0.0 {
        return Err(format!("planning failed with error code {err}"));
    }

    // Convert waypoints to a flat vector
    let waypoint_count = wp_cnt as usize;
    let mut waypoints = Vec::with_capacity(waypoint_count * 3);
    for i in 0..waypoint_count {
        waypoints.push(wp.Lat[i]);
        waypoints.push(wp.Lon[i]);
        waypoints.push(wp.State[i]);
    }

    Ok(crate::types::PlanningResult {
        waypoints,
        waypoint_count,
        path_length: wp_len,
        coverage_area: wp_area,
        estimated_time: wp_time,
        yaw: wp_yaw,
        error_code: err,
    })
}
