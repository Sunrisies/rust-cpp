use cover_map_obs_plan_rs::{ffi, initialize, plan, terminate, MapData, ObstacleData, PlanningParams, PolygonData};
use std::sync::Mutex;

static FFI_LOCK: Mutex<()> = Mutex::new(());

fn main() {
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

    eprintln!("C++:  wpCnt={} yaw={:.10} path={:.10} area={:.10}",
        cpp_result.waypoint_count, cpp_result.yaw, cpp_result.path_length, cpp_result.coverage_area);
    eprintln!("Rust: wpCnt={} yaw={:.10} path={:.10} area={:.10}",
        rust_result.waypoint_count, rust_result.yaw, rust_result.path_length, rust_result.coverage_area);
    eprintln!();

    let min_n = cpp_result.waypoint_count.min(rust_result.waypoint_count);

    // Find first difference
    for i in 0..min_n {
        let base = i * 3;
        let dlat = (cpp_result.waypoints[base] - rust_result.waypoints[base]).abs();
        let dlon = (cpp_result.waypoints[base+1] - rust_result.waypoints[base+1]).abs();
        if dlat > 1.0e-8 || dlon > 1.0e-8 {
            eprintln!("FIRST DIFF at wp[{}]:", i);
            for j in 0..i+3.min(min_n) {
                let b = j * 3;
                if j < min_n {
                    let d = ((cpp_result.waypoints[b] - rust_result.waypoints[b]).abs()
                        + (cpp_result.waypoints[b+1] - rust_result.waypoints[b+1]).abs());
                    let flag = if d > 1.0e-8 { " <<<" } else { "" };
                    eprintln!("  wp[{}]: C++=({:.9},{:.9},{}) Rust=({:.9},{:.9},{}){}",
                        j,
                        cpp_result.waypoints[b], cpp_result.waypoints[b+1], cpp_result.waypoints[b+2] as i32,
                        rust_result.waypoints[b], rust_result.waypoints[b+1], rust_result.waypoints[b+2] as i32,
                        flag);
                }
            }
            break;
        }
    }

    // Show trailing difference
    if cpp_result.waypoint_count > rust_result.waypoint_count {
        eprintln!("Rust missing {} trailing waypoints:", cpp_result.waypoint_count - rust_result.waypoint_count);
        for i in rust_result.waypoint_count..cpp_result.waypoint_count {
            let base = i * 3;
            eprintln!("  C++ wp[{}]: ({:.9},{:.9},{})", i,
                cpp_result.waypoints[base], cpp_result.waypoints[base+1], cpp_result.waypoints[base+2] as i32);
        }
    } else if rust_result.waypoint_count > cpp_result.waypoint_count {
        eprintln!("Rust extra {} trailing waypoints:", rust_result.waypoint_count - cpp_result.waypoint_count);
        for i in cpp_result.waypoint_count..rust_result.waypoint_count {
            let base = i * 3;
            eprintln!("  Rust wp[{}]: ({:.9},{:.9},{})", i,
                rust_result.waypoints[base], rust_result.waypoints[base+1], rust_result.waypoints[base+2] as i32);
        }
    }
}
