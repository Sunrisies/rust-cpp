use cover_map_obs_plan_rs::{
    initialize, plan, terminate, MapData, ObstacleData, PlanningParams, PolygonData,
};

fn main() {
    initialize();

    println!("CoverMapObsPlan initialized");

    let mut map = MapData::default();
    map.Lat[..4].copy_from_slice(&[
        32.22334642058298,
        32.2233437078402,
        32.222798651753514,
        32.22279594140087,
    ]);
    map.Lon[..4].copy_from_slice(&[
        119.42147511850712,
        119.42215778901347,
        119.42215459543166,
        119.42128690768837,
    ]);
    map.Cnt = 4.0;

    let pyg = PolygonData::default();
    let mut obs = ObstacleData::default();
    let params = PlanningParams {
        width: 2.0,
        yaw: 0.0,
        dir: 1.0,
        speed: 1.0,
        safe_dist_obs: 2.0,
        safe_dist_map: 2.0,
        long_edge_yaw_flag: 1,
    };

    match plan(&map, &pyg, &mut obs, &params) {
        Ok(result) => {
            println!("Planning succeeded{:?}", result);
            println!("waypoint_count={}", result.waypoint_count);
            println!("path_length={:.3}", result.path_length);
            println!("coverage_area={:.3}", result.coverage_area);
            println!("estimated_time={:.3}", result.estimated_time);
            println!("yaw={:.3}", result.yaw);
        }
        Err(err) => {
            println!("Planning failed: {err}");
        }
    }

    terminate();

    println!("CoverMapObsPlan terminated");
}
