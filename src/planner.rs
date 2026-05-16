// Rust-first path planning module.
// C++ FFI is kept as a parity oracle in tests, not as the default planner path.

use crate::geometry::*;
use crate::types::*;

const EARTH_RADIUS_M: f64 = 6.371_393e6;

type PolygonPath = Vec<(f64, f64, f64)>;
type RotatedPolygons = (Vec<PolygonPath>, Vec<f64>);

/// Main planning function
pub fn plan(
    map: &MapData,
    pyg: &PolygonData,
    obs: &mut ObstacleData,
    params: &PlanningParams,
) -> Result<PlanningResult, PlanningError> {
    // Validate inputs
    if map.Cnt < 3.0 || params.width < 0.1 || params.speed <= 0.0 {
        return Err(PlanningError::InvalidInput);
    }

    plan_rust(map, pyg, obs, params)
}

fn plan_rust(
    map: &MapData,
    pyg: &PolygonData,
    obs: &ObstacleData,
    params: &PlanningParams,
) -> Result<PlanningResult, PlanningError> {
    let map_cnt = map.Cnt as usize;
    let input_lat = &map.Lat[..map_cnt];
    let input_lon = &map.Lon[..map_cnt];

    if input_lat
        .iter()
        .zip(input_lon.iter())
        .any(|(lat, lon)| *lat > 180.0 || *lat < -180.0 || *lon > 180.0 || *lon < -180.0)
    {
        return Err(PlanningError::InvalidInput);
    }

    // 1. 地块边界安全距离偏移。safe_dist_map 为 0 时通常保持原始地块。
    let shrunk = main_shr_out(input_lat, input_lon, params.safe_dist_map);
    let map_ll: Vec<(f64, f64)> = shrunk
        .vertices
        .iter()
        .take(shrunk.out_cnt)
        .map(|point| (point[0], point[1]))
        .collect();
    if map_ll.len() < 3 {
        return Err(PlanningError::PlanningFailed);
    }

    // 2. 经纬度转局部平面坐标。这里沿用 C++ 中的等距近似投影。
    let ref_lat = map_ll[0].0;
    let ref_lon = map_ll[0].1;
    let lat_r = EARTH_RADIUS_M * ref_lat.to_radians().cos();
    let map_xy: Vec<(f64, f64)> = map_ll
        .iter()
        .map(|(lat, lon)| {
            (
                EARTH_RADIUS_M * (lat - ref_lat).to_radians(),
                lat_r * (lon - ref_lon).to_radians(),
            )
        })
        .collect();

    // 3. 根据最长边得到基础航向，再叠加用户 yaw 修正值。
    let mut cal_yaw_value = cal_yaw_from_map(&map_xy, &map_ll) + 270.0;
    if cal_yaw_value > 360.0 {
        cal_yaw_value -= 360.0;
    }
    let mut yaw = cal_yaw_value + params.yaw;
    if yaw > 360.0 {
        yaw -= 360.0;
    }

    // 4. 计算扫描线从下到上或从上到下的方向标志，保持 C++ 的 long_edge_yaw_flag 语义。
    let f2c = if (0.0..=90.0).contains(&yaw) || (270.0..=360.0).contains(&yaw) {
        if eq_eps_i32(params.long_edge_yaw_flag) != 0.0 {
            if params.yaw >= 0.0 {
                0.0
            } else {
                1.0
            }
        } else if params.yaw >= 0.0 {
            1.0
        } else {
            0.0
        }
    } else if eq_eps_i32(params.long_edge_yaw_flag) != 0.0 {
        if params.yaw >= 0.0 {
            1.0
        } else {
            0.0
        }
    } else if params.yaw >= 0.0 {
        0.0
    } else {
        1.0
    };

    // 5. 把地块旋转到扫描坐标系，让 cover_map_by_yaw 只需要处理竖向扫描线。
    let yaw_dir = -yaw.to_radians();
    let cos_yaw = yaw_dir.cos();
    let sin_yaw = yaw_dir.sin();
    let map_offset = map_xy[0];
    let rotated_map: Vec<(f64, f64)> = map_xy
        .iter()
        .map(|(x, y)| {
            let dx = x - map_offset.0;
            let dy = y - map_offset.1;
            (
                cos_yaw * dx - sin_yaw * dy + map_offset.0,
                sin_yaw * dx + cos_yaw * dy + map_offset.1,
            )
        })
        .collect();

    let cover = cover_map_by_yaw(&rotated_map, params.width, yaw, f2c, params.dir);
    if cover.waypoints.is_empty() || cover.waypoints.len() > 5000 {
        return Err(PlanningError::PlanningFailed);
    }

    let mut planned_xy: Vec<(f64, f64)> = cover
        .waypoints
        .iter()
        .map(|point| (point[0], point[1]))
        .collect();

    // 6. 多边形障碍处理。
    // C++ 主流程会先按 safe_dist_obs 对多边形障碍外扩，然后把障碍点投影并旋转到
    // 与航线相同的坐标系。这里保持相同顺序，避免后续绕行点和扫描坐标不一致。
    let (polygons, states) = build_rotated_polygons(
        pyg,
        params.safe_dist_obs,
        ref_lat,
        ref_lon,
        lat_r,
        map_offset,
        cos_yaw,
        sin_yaw,
    );
    if !polygons.is_empty() {
        planned_xy = pyg_obs_avoid(&planned_xy, &polygons, &states);
    }

    // 只有圆形障碍时，先把障碍物转换到同一个旋转平面坐标系，再插入绕行点。
    // 圆形障碍半径在进入算法前加上 safe_dist_obs，等价于 C++ 中 obs.R += safe_dist_obs。
    let obs_cnt = obs.Cnt as usize;
    if obs_cnt > 0 {
        let circular_obstacles: Vec<CircleObstacle> = (0..obs_cnt.min(50))
            .map(|i| {
                let x = EARTH_RADIUS_M * (obs.Lat[i] - ref_lat).to_radians();
                let y = lat_r * (obs.Lon[i] - ref_lon).to_radians();
                let dx = x - map_offset.0;
                let dy = y - map_offset.1;
                CircleObstacle {
                    x: cos_yaw * dx - sin_yaw * dy + map_offset.0,
                    y: sin_yaw * dx + cos_yaw * dy + map_offset.1,
                    r: obs.R[i] + params.safe_dist_obs,
                }
            })
            .collect();
        planned_xy = circle_avoid_rtl(&planned_xy, &circular_obstacles, 0.1);
    }

    // 6. C++ 在障碍物处理前后使用平面坐标统计长度；无障碍时直接统计扫描结果。
    let path_length: f64 = planned_xy
        .windows(2)
        .map(|pair| norm2([pair[0].0 - pair[1].0, pair[0].1 - pair[1].1]))
        .sum();
    let coverage_area = path_length * params.width;
    let estimated_time = path_length / params.speed;

    // 7. 反旋转，再从局部平面坐标转回经纬度。
    let mut flat_waypoints = Vec::with_capacity(planned_xy.len() * 3);
    for point in &planned_xy {
        let dx = point.0 - map_offset.0;
        let dy = point.1 - map_offset.1;
        let x = cos_yaw * dx + sin_yaw * dy + map_offset.0;
        let y = -sin_yaw * dx + cos_yaw * dy + map_offset.1;
        let lat = x.to_degrees() / EARTH_RADIUS_M + ref_lat;
        let lon = y.to_degrees() / lat_r + ref_lon;
        flat_waypoints.push(lat);
        flat_waypoints.push(lon);
        flat_waypoints.push(0.0);
    }

    let wp_yaw = if flat_waypoints.len() >= 6 {
        cal_yaw(
            flat_waypoints[0],
            flat_waypoints[1],
            flat_waypoints[3],
            flat_waypoints[4],
        )
    } else {
        0.0
    };

    Ok(PlanningResult {
        waypoint_count: planned_xy.len(),
        waypoints: flat_waypoints,
        path_length,
        coverage_area,
        estimated_time,
        yaw: wp_yaw,
        error_code: 0.0,
    })
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn build_rotated_polygons(
    pyg: &PolygonData,
    safe_dist_obs: f64,
    ref_lat: f64,
    ref_lon: f64,
    lat_r: f64,
    map_offset: (f64, f64),
    cos_yaw: f64,
    sin_yaw: f64,
) -> RotatedPolygons {
    let pyg_cnt = pyg.Cnt as usize;
    if pyg_cnt == 0 {
        return (Vec::new(), Vec::new());
    }

    let mut polygons = Vec::new();
    let mut states = Vec::new();
    let mut start = 0usize;
    for poly_idx in 0..pyg_cnt.min(200) {
        let count = pyg.Pnt[poly_idx].max(0) as usize;
        if count < 3 || start + count > 200 {
            start += count;
            continue;
        }

        // 1. 对单个多边形障碍按安全距离外扩；外扩失败时 main_shr_out 会回退原始边界。
        // C++ 对地块使用内缩入口，对障碍物使用外扩入口；二者方向相反，不能复用
        // 地块内缩语义，否则 safe_dist_obs 会把障碍物缩小。
        let lat = &pyg.Lat[start..start + count];
        let lon = &pyg.Lon[start..start + count];
        let expanded = main_shr_out_expand(lat, lon, safe_dist_obs);

        // 2. 经纬度转局部平面坐标，再旋转到覆盖扫描坐标系。
        let mut polygon = Vec::with_capacity(expanded.vertices.len());
        for point in expanded.vertices.iter().take(expanded.out_cnt) {
            let x = EARTH_RADIUS_M * (point[0] - ref_lat).to_radians();
            let y = lat_r * (point[1] - ref_lon).to_radians();
            let dx = x - map_offset.0;
            let dy = y - map_offset.1;
            polygon.push((
                cos_yaw * dx - sin_yaw * dy + map_offset.0,
                sin_yaw * dx + cos_yaw * dy + map_offset.1,
                // C++ 完整 planner 旋转障碍物时只把 X/Y 两列写入 pygXY2，第三列保持 0；
                // 单独的 pygState 参数才表示该多边形是否需要避让。这里也把二者分开，
                // 避免误触发 pygObsCollision 中“边界类障碍”的第三列分支。
                0.0,
            ));
        }

        if polygon.len() >= 3 {
            states.push(pyg.State[start]);
            polygons.push(polygon);
        }
        start += count;
    }

    (polygons, states)
}

/// Generate coverage waypoints
pub fn generate_coverage_waypoints(_map: &MapData, _params: &PlanningParams) -> Vec<(f64, f64)> {
    // TODO: Implement coverage waypoint generation
    Vec::new()
}

/// Avoid obstacles
pub fn avoid_obstacles(
    waypoints: &[(f64, f64)],
    _obs: &ObstacleData,
    _params: &PlanningParams,
) -> Vec<(f64, f64)> {
    // TODO: Implement obstacle avoidance
    waypoints.to_vec()
}

// More planning functions will be added here as we migrate from C++

#[cfg(test)]
mod tests {
    use super::*;

    fn same_point(a: (f64, f64), b: (f64, f64)) -> bool {
        (a.0 - b.0).abs() < 1.0e-5 && (a.1 - b.1).abs() < 1.0e-5
    }

    #[test]
    fn rotated_safe_distance_polygon_matches_cpp_pyg_avoid() {
        let mut map = MapData::default();
        map.Lat[..4].copy_from_slice(&[30.0000, 30.0000, 30.0010, 30.0010]);
        map.Lon[..4].copy_from_slice(&[120.0000, 120.0010, 120.0010, 120.0000]);
        map.Cnt = 4.0;

        let mut pyg = PolygonData::default();
        pyg.Cnt = 1.0;
        pyg.Pnt[0] = 4;
        pyg.Lat[..4].copy_from_slice(&[30.0004, 30.0004, 30.0006, 30.0006]);
        pyg.Lon[..4].copy_from_slice(&[120.0004, 120.0006, 120.0006, 120.0004]);
        pyg.State[..4].copy_from_slice(&[1.0; 4]);

        let params = PlanningParams {
            width: 18.0,
            speed: 2.5,
            safe_dist_obs: 1.5,
            ..PlanningParams::default()
        };

        let shrunk = main_shr_out(&map.Lat[..4], &map.Lon[..4], params.safe_dist_map);
        let map_ll: Vec<(f64, f64)> = shrunk
            .vertices
            .iter()
            .take(shrunk.out_cnt)
            .map(|p| (p[0], p[1]))
            .collect();
        let ref_lat = map_ll[0].0;
        let ref_lon = map_ll[0].1;
        let lat_r = EARTH_RADIUS_M * ref_lat.to_radians().cos();
        let map_xy: Vec<(f64, f64)> = map_ll
            .iter()
            .map(|(lat, lon)| {
                (
                    EARTH_RADIUS_M * (lat - ref_lat).to_radians(),
                    lat_r * (lon - ref_lon).to_radians(),
                )
            })
            .collect();
        let mut yaw = cal_yaw_from_map(&map_xy, &map_ll) + 270.0 + params.yaw;
        if yaw > 360.0 {
            yaw -= 360.0;
        }
        let yaw_dir = -yaw.to_radians();
        let cos_yaw = yaw_dir.cos();
        let sin_yaw = yaw_dir.sin();
        let map_offset = map_xy[0];
        let rotated_map: Vec<(f64, f64)> = map_xy
            .iter()
            .map(|(x, y)| {
                let dx = x - map_offset.0;
                let dy = y - map_offset.1;
                (
                    cos_yaw * dx - sin_yaw * dy + map_offset.0,
                    sin_yaw * dx + cos_yaw * dy + map_offset.1,
                )
            })
            .collect();
        let f2c = if (0.0..=90.0).contains(&yaw) || (270.0..=360.0).contains(&yaw) {
            if eq_eps_i32(params.long_edge_yaw_flag) != 0.0 {
                if params.yaw >= 0.0 {
                    0.0
                } else {
                    1.0
                }
            } else if params.yaw >= 0.0 {
                1.0
            } else {
                0.0
            }
        } else if eq_eps_i32(params.long_edge_yaw_flag) != 0.0 {
            if params.yaw >= 0.0 {
                1.0
            } else {
                0.0
            }
        } else if params.yaw >= 0.0 {
            0.0
        } else {
            1.0
        };
        let cover = cover_map_by_yaw(&rotated_map, params.width, yaw, f2c, params.dir);
        let path2: Vec<(f64, f64)> = cover.waypoints.iter().map(|p| (p[0], p[1])).collect();
        let path3: Vec<(f64, f64, f64)> =
            cover.waypoints.iter().map(|p| (p[0], p[1], p[2])).collect();
        let (polygons, states) = build_rotated_polygons(
            &pyg,
            params.safe_dist_obs,
            ref_lat,
            ref_lon,
            lat_r,
            map_offset,
            cos_yaw,
            sin_yaw,
        );

        let rust_path = pyg_obs_avoid(&path2, &polygons, &states);
        let (cpp_path, cpp_count) = crate::ffi::run_cpp_pyg_obs_avoid(&path3, &polygons, &states);

        assert_eq!(rust_path.len(), cpp_count as usize);
        for (rust, cpp) in rust_path.iter().zip(cpp_path.iter()) {
            assert!(
                same_point(*rust, *cpp),
                "rust {rust:?} != cpp {cpp:?}; rust_path={rust_path:?}; cpp_path={cpp_path:?}"
            );
        }
    }
}
