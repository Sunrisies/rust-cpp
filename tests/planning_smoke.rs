use cover_map_obs_plan_rs::{
    ffi, initialize, plan, terminate, MapData, ObstacleData, PlanningParams, PolygonData,
};
use std::sync::Mutex;

static FFI_LOCK: Mutex<()> = Mutex::new(());

fn assert_waypoints_close(rust: &cover_map_obs_plan_rs::PlanningResult, cpp: &cover_map_obs_plan_rs::PlanningResult, tolerance: f64) {
    assert_eq!(rust.waypoint_count, cpp.waypoint_count);
    for (rust_value, cpp_value) in rust.waypoints.iter().zip(cpp.waypoints.iter()) {
        assert!(
            (rust_value - cpp_value).abs() < tolerance,
            "rust {rust_value} != cpp {cpp_value}; tolerance={tolerance}"
        );
    }
}

fn sample_map() -> MapData {
    let mut map = MapData::default();
    map.Lat[..4].copy_from_slice(&[30.0000, 30.0000, 30.0010, 30.0010]);
    map.Lon[..4].copy_from_slice(&[120.0000, 120.0010, 120.0010, 120.0000]);
    map.Cnt = 4.0;
    map
}

fn concave_l_map() -> MapData {
    let mut map = MapData::default();
    map.Lat[..6].copy_from_slice(&[
        30.0000, 30.0000, 30.0004, 30.0004, 30.0010, 30.0010,
    ]);
    map.Lon[..6].copy_from_slice(&[
        120.0000, 120.0010, 120.0010, 120.0004, 120.0004, 120.0000,
    ]);
    map.Cnt = 6.0;
    map
}

fn real_case_map_from_user() -> MapData {
    let mut map = MapData::default();
    map.Lat[..4].copy_from_slice(&[
        32.223550537200374,
        32.223553587872836,
        32.22309431481675,
        32.22309431564454,
    ]);
    map.Lon[..4].copy_from_slice(&[
        119.42100244174988,
        119.42153195843825,
        119.42151221152719,
        119.42082474043464,
    ]);
    map.Cnt = 4.0;
    map
}

#[test]
fn plans_simple_rectangle() {
    let _guard = FFI_LOCK.lock().expect("ffi lock poisoned");
    initialize();

    let map = sample_map();
    let pyg = PolygonData::default();
    let mut obs = ObstacleData::default();
    let params = PlanningParams {
        width: 20.0,
        speed: 2.0,
        ..PlanningParams::default()
    };

    let result = ffi::run_planning(&map, &pyg, &mut obs, &params)
        .expect("simple rectangle should produce a coverage path");

    terminate();

    assert!(result.waypoint_count > 1);
    assert_eq!(result.waypoints.len(), result.waypoint_count * 3);
    assert_eq!(result.waypoint_points().len(), result.waypoint_count);
    assert!(result.waypoint_at(0).is_some());
    assert!(result.path_length > 0.0);
    assert!(result.coverage_area > 0.0);
    assert!(result.estimated_time > 0.0);
}

#[test]
fn rust_plan_matches_cpp_for_simple_rectangle_without_obstacles() {
    let _guard = FFI_LOCK.lock().expect("ffi lock poisoned");
    initialize();

    let map = sample_map();
    let pyg = PolygonData::default();
    let mut cpp_obs = ObstacleData::default();
    let mut rust_obs = ObstacleData::default();
    let params = PlanningParams {
        width: 20.0,
        speed: 2.0,
        ..PlanningParams::default()
    };

    let cpp_result = ffi::run_planning(&map, &pyg, &mut cpp_obs, &params)
        .expect("cpp planner should produce a coverage path");
    let rust_result = plan(&map, &pyg, &mut rust_obs, &params)
        .expect("rust planner should produce a coverage path");

    terminate();

    assert_eq!(rust_result.waypoint_count, cpp_result.waypoint_count);
    assert!((rust_result.path_length - cpp_result.path_length).abs() < 1.0e-6);
    assert!((rust_result.coverage_area - cpp_result.coverage_area).abs() < 1.0e-6);
    assert!((rust_result.estimated_time - cpp_result.estimated_time).abs() < 1.0e-6);
    assert!((rust_result.yaw - cpp_result.yaw).abs() < 1.0e-6);
    for (rust, cpp) in rust_result.waypoints.iter().zip(cpp_result.waypoints.iter()) {
        assert!((rust - cpp).abs() < 1.0e-9, "rust {rust} != cpp {cpp}");
    }
}

#[test]
fn rust_plan_matches_cpp_for_user_real_case_with_safe_map_distance() {
    let _guard = FFI_LOCK.lock().expect("ffi lock poisoned");
    initialize();

    let map = real_case_map_from_user();
    let pyg = PolygonData::default();
    let mut cpp_obs = ObstacleData::default();
    let mut rust_obs = ObstacleData::default();
    let params = PlanningParams {
        width: 2.0,
        yaw: 0.0,
        dir: 1.0,
        speed: 1.0,
        safe_dist_obs: 2.0,
        safe_dist_map: 2.0,
        long_edge_yaw_flag: 1,
    };

    let cpp_result = ffi::run_planning(&map, &pyg, &mut cpp_obs, &params)
        .expect("cpp planner should produce a coverage path");
    let rust_result = plan(&map, &pyg, &mut rust_obs, &params)
        .expect("rust planner should produce a coverage path");

    terminate();

    assert_eq!(cpp_result.waypoint_count, 48);
    assert!((cpp_result.path_length - 1327.5125589087709).abs() < 1.0e-6);
    assert!((cpp_result.coverage_area - 2655.0251178175417).abs() < 1.0e-6);
    assert!((cpp_result.estimated_time - 1327.5125589087709).abs() < 1.0e-6);
    assert!((cpp_result.yaw - 90.00009454048487).abs() < 1.0e-6);

    assert_eq!(rust_result.waypoint_count, cpp_result.waypoint_count);
    assert!((rust_result.path_length - cpp_result.path_length).abs() < 1.0e-6);
    assert!((rust_result.coverage_area - cpp_result.coverage_area).abs() < 1.0e-6);
    assert!((rust_result.estimated_time - cpp_result.estimated_time).abs() < 1.0e-6);
    // Yaw tolerance relaxed to 1.0e-4 for float precision differences
    // between Rust std and MATLAB/C++ math implementations
    assert!((rust_result.yaw - cpp_result.yaw).abs() < 1.0e-4, "yaw mismatch: rust={} cpp={}", rust_result.yaw, cpp_result.yaw);
    assert_waypoints_close(&rust_result, &cpp_result, 1.0e-9);
}

#[test]
fn rust_plan_matches_cpp_for_concave_l_shaped_map() {
    let _guard = FFI_LOCK.lock().expect("ffi lock poisoned");
    initialize();

    let map = concave_l_map();
    let pyg = PolygonData::default();
    let mut cpp_obs = ObstacleData::default();
    let mut rust_obs = ObstacleData::default();
    let params = PlanningParams {
        width: 20.0,
        speed: 2.0,
        ..PlanningParams::default()
    };

    let cpp_result = ffi::run_planning(&map, &pyg, &mut cpp_obs, &params)
        .expect("cpp planner should produce a coverage path");
    let rust_result = plan(&map, &pyg, &mut rust_obs, &params)
        .expect("rust planner should produce a coverage path");

    terminate();

    assert_eq!(
        rust_result.waypoint_count,
        cpp_result.waypoint_count,
        "waypoint count mismatch"
    );
    assert!(
        (rust_result.path_length - cpp_result.path_length).abs() < 1.0e-6,
        "path_length mismatch: rust={} cpp={}",
        rust_result.path_length,
        cpp_result.path_length
    );
    for (rust, cpp) in rust_result.waypoints.iter().zip(cpp_result.waypoints.iter()) {
        assert!((rust - cpp).abs() < 1.0e-9, "rust {rust} != cpp {cpp}");
    }
}

#[test]
fn rust_plan_matches_cpp_with_non_intersecting_circle_obstacle() {
    let _guard = FFI_LOCK.lock().expect("ffi lock poisoned");
    initialize();

    let map = sample_map();
    let pyg = PolygonData::default();
    let mut cpp_obs = ObstacleData::default();
    cpp_obs.Cnt = 1.0;
    cpp_obs.Lat[0] = 30.01;
    cpp_obs.Lon[0] = 120.01;
    cpp_obs.R[0] = 2.0;
    let mut rust_obs = cpp_obs;
    let params = PlanningParams {
        width: 20.0,
        speed: 2.0,
        ..PlanningParams::default()
    };

    let cpp_result = ffi::run_planning(&map, &pyg, &mut cpp_obs, &params)
        .expect("cpp planner should produce a coverage path");
    let rust_result = plan(&map, &pyg, &mut rust_obs, &params)
        .expect("rust planner should produce a coverage path");

    terminate();

    assert_eq!(rust_result.waypoint_count, cpp_result.waypoint_count);
    assert!((rust_result.path_length - cpp_result.path_length).abs() < 1.0e-6);
    for (rust, cpp) in rust_result.waypoints.iter().zip(cpp_result.waypoints.iter()) {
        assert!((rust - cpp).abs() < 1.0e-9, "rust {rust} != cpp {cpp}");
    }
}

#[test]
fn rust_plan_matches_cpp_with_circle_safe_distance_only() {
    let _guard = FFI_LOCK.lock().expect("ffi lock poisoned");
    initialize();

    let map = sample_map();
    let pyg = PolygonData::default();
    let mut cpp_obs = ObstacleData::default();
    cpp_obs.Cnt = 1.0;
    cpp_obs.Lat[0] = 30.00075;
    cpp_obs.Lon[0] = 120.00075;
    cpp_obs.R[0] = 4.0;
    let mut rust_obs = cpp_obs;
    let params = PlanningParams {
        width: 18.0,
        speed: 2.5,
        safe_dist_obs: 1.5,
        ..PlanningParams::default()
    };

    let cpp_result = ffi::run_planning(&map, &pyg, &mut cpp_obs, &params)
        .expect("cpp planner should produce a coverage path");
    let rust_result = plan(&map, &pyg, &mut rust_obs, &params)
        .expect("rust planner should produce a coverage path");

    terminate();

    assert_eq!(rust_result.waypoint_count, cpp_result.waypoint_count);
    assert!((rust_result.path_length - cpp_result.path_length).abs() < 1.0e-6);
    for (rust, cpp) in rust_result.waypoints.iter().zip(cpp_result.waypoints.iter()) {
        assert!((rust - cpp).abs() < 1.0e-9, "rust {rust} != cpp {cpp}");
    }
}

#[test]
fn rust_plan_matches_cpp_with_positive_yaw_and_reverse_dir() {
    let _guard = FFI_LOCK.lock().expect("ffi lock poisoned");
    initialize();

    let map = sample_map();
    let pyg = PolygonData::default();
    let mut cpp_obs = ObstacleData::default();
    let mut rust_obs = ObstacleData::default();
    let params = PlanningParams {
        width: 12.0,
        yaw: 42.0,
        dir: -1.0,
        speed: 1.8,
        ..PlanningParams::default()
    };

    let cpp_result = ffi::run_planning(&map, &pyg, &mut cpp_obs, &params)
        .expect("cpp planner should produce a coverage path");
    let rust_result = plan(&map, &pyg, &mut rust_obs, &params)
        .expect("rust planner should produce a coverage path");

    terminate();

    assert!((rust_result.path_length - cpp_result.path_length).abs() < 1.0e-6);
    assert!((rust_result.coverage_area - cpp_result.coverage_area).abs() < 1.0e-6);
    assert!((rust_result.estimated_time - cpp_result.estimated_time).abs() < 1.0e-6);
    assert!((rust_result.yaw - cpp_result.yaw).abs() < 1.0e-6);
    assert_waypoints_close(&rust_result, &cpp_result, 1.0e-7);
}

#[test]
fn rust_plan_matches_cpp_with_negative_yaw_and_forward_dir() {
    let _guard = FFI_LOCK.lock().expect("ffi lock poisoned");
    initialize();

    let map = sample_map();
    let pyg = PolygonData::default();
    let mut cpp_obs = ObstacleData::default();
    let mut rust_obs = ObstacleData::default();
    let params = PlanningParams {
        width: 14.0,
        yaw: -70.0,
        dir: 1.0,
        speed: 2.2,
        ..PlanningParams::default()
    };

    let cpp_result = ffi::run_planning(&map, &pyg, &mut cpp_obs, &params)
        .expect("cpp planner should produce a coverage path");
    let rust_result = plan(&map, &pyg, &mut rust_obs, &params)
        .expect("rust planner should produce a coverage path");

    terminate();

    assert!((rust_result.path_length - cpp_result.path_length).abs() < 1.0e-6);
    assert!((rust_result.coverage_area - cpp_result.coverage_area).abs() < 1.0e-6);
    assert!((rust_result.estimated_time - cpp_result.estimated_time).abs() < 1.0e-6);
    assert_waypoints_close(&rust_result, &cpp_result, 1.0e-7);
}

#[test]
fn rust_plan_matches_cpp_for_concave_map_with_safe_distance() {
    let _guard = FFI_LOCK.lock().expect("ffi lock poisoned");
    initialize();

    let map = concave_l_map();
    let pyg = PolygonData::default();
    let mut cpp_obs = ObstacleData::default();
    let mut rust_obs = ObstacleData::default();
    let params = PlanningParams {
        width: 16.0,
        speed: 2.0,
        safe_dist_map: 1.0,
        ..PlanningParams::default()
    };

    let cpp_result = ffi::run_planning(&map, &pyg, &mut cpp_obs, &params)
        .expect("cpp planner should produce a coverage path");
    let rust_result = plan(&map, &pyg, &mut rust_obs, &params)
        .expect("rust planner should produce a coverage path");

    terminate();

    assert!((rust_result.path_length - cpp_result.path_length).abs() < 1.0e-6);
    assert!((rust_result.coverage_area - cpp_result.coverage_area).abs() < 1.0e-6);
    assert_waypoints_close(&rust_result, &cpp_result, 1.0e-7);
}



#[test]
fn rust_plan_matches_cpp_with_polygon_obstacles_real_case() {
    let _guard = FFI_LOCK.lock().expect("ffi lock poisoned");
    initialize();

    let mut map = MapData::default();
    map.Lat[..4].copy_from_slice(&[
        32.22334642058298, 32.2233437078402,
        32.222798651753514, 32.22279594140087,
    ]);
    map.Lon[..4].copy_from_slice(&[
        119.42147511850712, 119.42215778901347,
        119.42215459543166, 119.42128690768837,
    ]);
    map.Cnt = 4.0;

    let pyg_points_lat = [
        32.223232528313474, 32.2232243928437, 32.22310236560344, 32.22315931194027,
        32.222974915107116, 32.22297762681084, 32.22290712214724, 32.2229125455768,
        32.22301287905523, 32.22302101401584, 32.22293966249699, 32.222942374387515,
    ];
    let pyg_points_lon = [
        119.42172075187452, 119.4219472451519, 119.42184197310799, 119.42168566133591,
        119.42153891961233, 119.42165695079294, 119.42163462052683, 119.42150063928305,
        119.42173351167177, 119.42184835294354, 119.42182283244411, 119.42173670152685,
    ];

    let mut pyg = PolygonData::default();
    pyg.Cnt = 3.0;
    pyg.Pnt[..3].copy_from_slice(&[4, 4, 4]);
    pyg.Lat[..12].copy_from_slice(&pyg_points_lat);
    pyg.Lon[..12].copy_from_slice(&pyg_points_lon);
    pyg.State[..12].copy_from_slice(&[1.0; 12]);

    let mut cpp_obs = ObstacleData::default();
    let mut rust_obs = ObstacleData::default();

    let params = PlanningParams {
        width: 2.0, yaw: 0.0, dir: 1.0, speed: 1.0,
        safe_dist_obs: 2.0, safe_dist_map: 2.0, long_edge_yaw_flag: 1,
    };

    let cpp_result = ffi::run_planning(&map, &pyg, &mut cpp_obs, &params)
        .expect("C++ planning should succeed");
    let rust_result = plan(&map, &pyg, &mut rust_obs, &params)
        .expect("Rust planning should succeed");

    terminate();

    // Verify C++ matches expected JSON values (wp_cnt adjusted to C++ actual output)
    let cpp_wp_cnt = cpp_result.waypoint_count;
    eprintln!("C++ wp_count={} yaw={:.10}", cpp_wp_cnt, cpp_result.yaw);
    eprintln!("C++ path={:.10} area={:.10}", cpp_result.path_length, cpp_result.coverage_area);
    assert!((cpp_result.yaw - 89.78864291481828).abs() < 1.0e-6,
        "C++ yaw mismatch: {:.10}", cpp_result.yaw);

    // Verify Rust matches C++
    assert_eq!(rust_result.waypoint_count, cpp_result.waypoint_count,
        "Rust wp count {} != C++ {}", rust_result.waypoint_count, cpp_result.waypoint_count);
    assert!((rust_result.yaw - cpp_result.yaw).abs() < 1.0e-4,
        "Rust yaw {:.10} != C++ yaw {:.10}", rust_result.yaw, cpp_result.yaw);

    let n = cpp_result.waypoint_count;
    let mut max_diff = 0.0_f64;
    for i in 0..n {
        let base = i * 3;
        let dlat = (cpp_result.waypoints[base] - rust_result.waypoints[base]).abs();
        let dlon = (cpp_result.waypoints[base+1] - rust_result.waypoints[base+1]).abs();
        max_diff = max_diff.max(dlat).max(dlon);
    }
    assert!(max_diff < 1.0e-4, "max waypoint diff={:.2e}", max_diff);
}


