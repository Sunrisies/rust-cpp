//! Rust 侧几何、扫描、边界偏移和障碍物绕行实现。
//!
//! 这些函数按 MATLAB Coder 生成的 C++ 行为迁移，并通过 `ffi` 中的 C++ shim
//! 做单元级对比测试。复杂分支尽量保留 C++ 的输出点序和边界行为。
pub fn distance(a: (f64, f64), b: (f64, f64)) -> f64 {
    let dx = a.0 - b.0;
    let dy = a.1 - b.1;
    (dx * dx + dy * dy).sqrt()
}

pub fn cal_two_point_dis(a: [f64; 2], b: [f64; 2]) -> f64 {
    let a_lat = a[0].to_radians();
    let b_lat = b[0].to_radians();
    let d_lon = a[1].to_radians() - b[1].to_radians();
    6.378_137e6 * (a_lat.sin() * b_lat.sin() + a_lat.cos() * b_lat.cos() * d_lon.cos()).acos()
}

pub fn cal_yaw(lat_a: f64, lon_a: f64, lat_b: f64, lon_b: f64) -> f64 {
    let lat_now_rad = lat_a.to_radians();
    let lat_next_rad = lat_b.to_radians();
    let d_lon = lon_b.to_radians() - lon_a.to_radians();

    (d_lon.sin() * lat_next_rad.cos()).atan2(
        lat_now_rad.cos() * lat_next_rad.sin()
            - lat_now_rad.sin() * lat_next_rad.cos() * d_lon.cos(),
    ) / std::f64::consts::PI
        * 180.0
}

pub fn cal_yaw_from_map(map_xy: &[(f64, f64)], map_ll: &[(f64, f64)]) -> f64 {
    // 按 C++/MATLAB 逻辑：先找地块最长边，再用这条边的经纬度计算航向角。
    let len = map_xy.len().min(map_ll.len());
    if len < 2 {
        return 0.0;
    }

    let mut max_dist = 0.0;
    let mut max_index = 0usize;
    for i in 0..len {
        let next = (i + 1) % len;
        let curr_dist = norm2([map_xy[i].0 - map_xy[next].0, map_xy[i].1 - map_xy[next].1]);
        if curr_dist - max_dist > 1.0 {
            max_dist = curr_dist;
            max_index = i;
        }
    }

    let next = (max_index + 1) % len;
    cal_yaw(
        map_ll[max_index].0,
        map_ll[max_index].1,
        map_ll[next].0,
        map_ll[next].1,
    )
}

pub fn eq_eps_f64(a: f64, b: f64) -> f64 {
    if (a - b).abs() < 1.0e-5 {
        1.0
    } else {
        0.0
    }
}

pub fn eq_eps_i32(a: i32) -> f64 {
    let abs_a = if a == i32::MIN { i32::MAX } else { a.abs() };
    if (abs_a as f64) < 1.0e-5 {
        1.0
    } else {
        0.0
    }
}

pub fn matlab_mod(x: f64, y: f64) -> f64 {
    if y == 0.0 {
        x
    } else if y == y.floor() {
        x - (x / y).floor() * y
    } else {
        let r = x / y;
        if (r - r.round()).abs() <= f64::EPSILON * r.abs() {
            0.0
        } else {
            (r - r.floor()) * y
        }
    }
}

pub fn norm2(x: [f64; 2]) -> f64 {
    scaled_norm(&x)
}

pub fn norm3(x: [f64; 3]) -> f64 {
    scaled_norm(&x)
}

fn scaled_norm(x: &[f64]) -> f64 {
    let mut y = 0.0;
    let mut scale = f64::MIN_POSITIVE;
    for value in x {
        let absxk = value.abs();
        if absxk > scale {
            let t = scale / absxk;
            y = 1.0 + y * t * t;
            scale = absxk;
        } else {
            let t = absxk / scale;
            y += t * t;
        }
    }
    scale * y.sqrt()
}

pub fn det2(x: [f64; 4]) -> f64 {
    let mut b_x = x;
    let mut pivot_swapped = false;
    let ix = if x[1].abs() > x[0].abs() { 1 } else { 0 };

    if x[ix] != 0.0 {
        if ix != 0 {
            pivot_swapped = true;
            for k in 0..2 {
                b_x.swap(k * 2, 1 + k * 2);
            }
        }
        b_x[1] /= b_x[0];
    }

    if b_x[2] != 0.0 {
        b_x[3] += b_x[1] * -b_x[2];
    }

    let y = b_x[0] * b_x[3];
    if pivot_swapped {
        -y
    } else {
        y
    }
}

pub fn sum_bool_200(x: &[bool; 200]) -> f64 {
    x.iter().filter(|value| **value).count() as f64
}

pub fn sum_i32_200(x: &[i32; 200]) -> f64 {
    x.iter().map(|value| *value as f64).sum()
}

pub fn sum_f64_slice(x: &[f64]) -> f64 {
    x.iter().sum()
}

pub fn sum_i32_slice(x: &[i32]) -> f64 {
    x.iter().map(|value| *value as f64).sum()
}

pub fn vect_sin(a: [f64; 3], b: [f64; 3], c: [f64; 3], d: f64) -> [f64; 3] {
    let mut vect1 = [0.0; 3];
    let mut qi = [0.0; 3];
    for i in 0..3 {
        vect1[i] = b[i] - a[i];
        qi[i] = c[i] - b[i];
    }

    let norm_vect1 = norm3(vect1);
    let norm_qi = norm3(qi);
    for i in 0..3 {
        vect1[i] /= norm_vect1;
        qi[i] /= norm_qi;
    }

    let cross = [
        vect1[1] * qi[2] - vect1[2] * qi[1],
        vect1[2] * qi[0] - vect1[0] * qi[2],
        vect1[0] * qi[1] - vect1[1] * qi[0],
    ];
    let scale = d / norm3(cross);

    let mut out = [0.0; 3];
    for i in 0..3 {
        out[i] = b[i] + scale * (qi[i] - vect1[i]);
    }
    out
}

pub fn line_point2(
    line_x1: f64,
    line_y1: f64,
    line_x2: f64,
    line_y2: f64,
    point_x: f64,
    point_y: f64,
    my_eps: f64,
) -> f64 {
    if (point_x - line_x1) * (point_x - line_x2) <= my_eps
        && (point_y - line_y1) * (point_y - line_y2) <= my_eps
        && ((line_x1 - point_x) * (line_y2 - point_y) - (line_y1 - point_y) * (line_x2 - point_x))
            .abs()
            < my_eps
    {
        if ((point_x - line_x1).abs() < my_eps && (point_y - line_y1).abs() < my_eps)
            || ((point_x - line_x2).abs() < my_eps && (point_y - line_y2).abs() < my_eps)
        {
            0.0
        } else {
            1.0
        }
    } else {
        0.0
    }
}

pub fn line_point3(
    line_x1: f64,
    line_y1: f64,
    line_x2: f64,
    line_y2: f64,
    point_x: f64,
    point_y: f64,
) -> f64 {
    let a = line_y2 - line_y1;
    let b = line_x1 - line_x2;
    let den = (a * a + b * b).sqrt();
    let mut d = 10.0;
    if den > 0.0 {
        d = ((a * point_x + b * point_y) + (line_x2 * line_y1 - line_x1 * line_y2)).abs() / den;
    }

    if (point_x - line_x1) * (point_x - line_x2) <= 0.1
        && (point_y - line_y1) * (point_y - line_y2) <= 0.1
    {
        if ((line_x1 - point_x) * (line_y2 - point_y) - (line_y1 - point_y) * (line_x2 - point_x))
            .abs()
            < 0.1
            || d <= 0.1
        {
            return 1.0;
        }
        return 0.0;
    }

    if d <= 0.1 {
        let inside_x = point_x > line_x1.min(line_x2) && point_x < line_x1.max(line_x2);
        let inside_y = point_y > line_y1.min(line_y2) && point_y < line_y1.max(line_y2);
        if inside_x || inside_y {
            return 1.0;
        }
    }

    0.0
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SegmentIntersection {
    pub x: f64,
    pub y: f64,
    pub cross: f64,
}

pub fn segment_intersection(
    line1: (f64, f64, f64, f64),
    line2: (f64, f64, f64, f64),
) -> SegmentIntersection {
    segment_intersection_impl(line1, line2, 1.0e-5, false)
}

pub fn segment_intersection3(
    line1: (f64, f64, f64, f64),
    line2: (f64, f64, f64, f64),
) -> SegmentIntersection {
    segment_intersection_impl(line1, line2, 1.0e-5, true)
}

pub fn segment_intersection2(
    line1: (f64, f64, f64, f64),
    line2: (f64, f64, f64, f64),
) -> SegmentIntersection {
    // 与 MATLAB Coder 版本一致：先把轨迹线 line1 轻微延长，
    // 用来处理端点刚好贴在边界上导致漏判的情况。
    let (x1, y1, x2, y2) = line1;
    let mut extend_len = 1.0;
    let mut k = if (x2 - x1).abs() < 1.0e-5 {
        100.0
    } else if (x2 - x1).abs() < 0.5 {
        (y2 - y1) / (x2 - x1) * 0.001
    } else {
        (y2 - y1) / (x2 - x1)
    };

    if k > 10.0 && k < 50.0 {
        extend_len = 0.1;
    } else if (50.0..100.0).contains(&k) {
        extend_len = 0.05;
    } else if k >= 100.0 {
        extend_len = 0.01;
        k = 100.0;
    }

    let extended = if x1 > x2 {
        (
            x1 + extend_len,
            y1 + k * extend_len,
            x2 - extend_len,
            y2 - k * extend_len,
        )
    } else {
        (
            x1 - extend_len,
            y1 - k * extend_len,
            x2 + extend_len,
            y2 + k * extend_len,
        )
    };

    let mut cross = segment_intersection_impl(extended, line2, 0.1, false);
    if cross.cross == 1.0 {
        // 交点落在 line1 原始端点附近时，C++ 标记为 2，调用方仍按相交处理。
        if ((cross.x - x1).abs() < 0.1 && (cross.y - y1).abs() < 0.1)
            || ((cross.x - x2).abs() < 0.1 && (cross.y - y2).abs() < 0.1)
        {
            cross.cross = 2.0;
        }
    } else {
        // 如果延长线仍未相交，再检查轨迹端点是否贴在线段上。
        if line_point3(line2.0, line2.1, line2.2, line2.3, x1, y1) == 1.0 {
            cross = SegmentIntersection {
                x: x1,
                y: y1,
                cross: 1.0,
            };
        } else if line_point3(line2.0, line2.1, line2.2, line2.3, x2, y2) == 1.0 {
            cross = SegmentIntersection {
                x: x2,
                y: y2,
                cross: 1.0,
            };
        }
    }

    cross
}

fn segment_intersection_impl(
    line1: (f64, f64, f64, f64),
    line2: (f64, f64, f64, f64),
    eps: f64,
    exclude_endpoints: bool,
) -> SegmentIntersection {
    let (line1_x1, line1_y1, line1_x2, line1_y2) = line1;
    let (line2_x1, line2_y1, line2_x2, line2_y2) = line2;
    let a1 = line1_y1 - line1_y2;
    let b1 = line1_x2 - line1_x1;
    let c1 = line1_y2 * line1_x1 - line1_y1 * line1_x2;
    let a2 = line2_y1 - line2_y2;
    let b2 = line2_x2 - line2_x1;
    let c2 = line2_y2 * line2_x1 - line2_y1 * line2_x2;
    let d = det2([a1, a2, b1, b2]);

    if d <= eps && d >= -eps {
        return SegmentIntersection {
            x: 0.0,
            y: 0.0,
            cross: 0.0,
        };
    }

    let x = det2([-c1, -c2, b1, b2]) / d;
    let y = det2([a1, a2, -c1, -c2]) / d;
    let on_segments = (x - line1_x1) * (x - line1_x2) <= eps
        && (y - line1_y1) * (y - line1_y2) <= eps
        && (x - line2_x1) * (x - line2_x2) <= eps
        && (y - line2_y1) * (y - line2_y2) <= eps;

    let mut cross = if on_segments { 1.0 } else { 0.0 };
    if cross == 1.0 && exclude_endpoints {
        let at_endpoint = ((x - line1_x1).abs() < 1.0e-7 && (y - line1_y1).abs() < 1.0e-7)
            || ((x - line1_x2).abs() < 1.0e-7 && (y - line1_y2).abs() < 1.0e-7)
            || ((x - line2_x1).abs() < 1.0e-7 && (y - line2_y1).abs() < 1.0e-7)
            || ((x - line2_x2).abs() < 1.0e-7 && (y - line2_y2).abs() < 1.0e-7);
        if at_endpoint {
            cross = 0.0;
        }
    }

    SegmentIntersection { x, y, cross }
}

pub fn judge_clockwise(lat: &[f64], lon: &[f64]) -> f64 {
    let len = lat.len().min(lon.len());
    if len < 3 {
        return 0.0;
    }

    let mut d = 0.0;
    for i in 0..(len - 1) {
        d -= 0.5 * (lon[i + 1] + lon[i]) * (lat[i + 1] - lat[i]);
    }
    d -= 0.5 * (lon[0] + lon[len - 1]) * (lat[0] - lat[len - 1]);

    if d >= 0.0 {
        0.0
    } else {
        1.0
    }
}

pub fn judge_concave_vex(a: [f64; 3], b: [f64; 3], c: [f64; 3], cw_flag: f64) -> f64 {
    let vect1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let vect2 = [c[0] - b[0], c[1] - b[1], c[2] - b[2]];
    let judge_vex_z = vect1[0] * vect2[1] - vect1[1] * vect2[0];

    if cw_flag == 0.0 {
        if judge_vex_z > 0.0 {
            1.0
        } else {
            0.0
        }
    } else if judge_vex_z > 0.0 {
        0.0
    } else {
        1.0
    }
}

pub fn shr_out(
    a: [f64; 3],
    b: [f64; 3],
    c: [f64; 3],
    shr_out_flag: f64,
    distance: f64,
    cw_flag: f64,
) -> [f64; 3] {
    let convex = judge_concave_vex(a, b, c, cw_flag);
    let signed_distance = if shr_out_flag == 0.0 {
        if convex == 0.0 {
            -distance
        } else {
            distance
        }
    } else if convex == 0.0 {
        distance
    } else {
        -distance
    };

    vect_sin(a, b, c, signed_distance)
}

pub fn zoom_in_out(
    lat: &[f64],
    lon: &[f64],
    cwa_flag: f64,
    distance: f64,
    shrink: bool,
) -> Vec<[f64; 3]> {
    let len = lat.len().min(lon.len());
    if len == 0 {
        return Vec::new();
    }

    let distance = distance / 100000.0;
    let shr_out_flag = if shrink { 0.0 } else { 1.0 };
    let mut verttemp = vec![[0.0; 3]; len];

    for k in 0..len {
        let j = (k + 1) % len;
        let i = (k + 2) % len;
        let a = [lat[k], lon[k], 0.0];
        let b = [lat[j], lon[j], 0.0];
        let c = [lat[i], lon[i], 0.0];
        verttemp[k] = shr_out(a, b, c, shr_out_flag, distance, cwa_flag);
    }

    let mut out = vec![[0.0; 3]; len];
    out[0] = verttemp[len - 1];
    if len > 1 {
        out[1..].copy_from_slice(&verttemp[..len - 1]);
    }
    out
}

#[allow(clippy::too_many_arguments)]
pub fn circle_segment(
    center_x: f64,
    center_y: f64,
    r: f64,
    line_x1: f64,
    line_y1: f64,
    line_x2: f64,
    line_y2: f64,
    my_eps: f64,
) -> f64 {
    if (line_x1 - line_x2).abs() < 1.0e-5 && (line_y1 - line_y2).abs() < 1.0e-5 {
        return 0.0;
    }

    let a = line_y1 - line_y2;
    let b_a = line_x1 - line_x2;
    let line_delta = [line_x1 - line_x2, line_y1 - line_y2];
    let c_a = norm2(line_delta);
    let yk = (((center_x - line_x2) * (line_x1 - line_x2) * (line_y1 - line_y2)
        + center_y * (a * a))
        + line_y2 * (b_a * b_a))
        / (c_a * c_a);

    let xk = if (line_x1 - line_x2).abs() < 1.0e-5 {
        line_x1
    } else if (line_y1 - line_y2).abs() < 1.0e-5 {
        center_x
    } else {
        ((line_x1 - line_x2) * line_x2 * (line_y1 - line_y2)
            + (line_x1 - line_x2) * (line_x1 - line_x2) * (yk - line_y2))
            / ((line_x1 - line_x2) * (line_y1 - line_y2))
    };

    if norm2([xk - center_x, yk - center_y]) - r > -my_eps {
        return 0.0;
    }

    if norm2([center_x - line_x1, center_y - line_y1]) <= r
        || norm2([center_x - line_x2, center_y - line_y2]) <= r
    {
        return 1.0;
    }

    if (xk - line_x1) * (xk - line_x2) <= my_eps && (yk - line_y1) * (yk - line_y2) <= my_eps {
        1.0
    } else {
        0.0
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CircleObstacle {
    pub x: f64,
    pub y: f64,
    pub r: f64,
}

pub fn circle_avoid_rtl(
    path: &[(f64, f64)],
    obstacles: &[CircleObstacle],
    my_eps: f64,
) -> Vec<(f64, f64)> {
    if path.len() < 2 || obstacles.is_empty() {
        return path.to_vec();
    }

    let mut out = Vec::new();
    for segment in path.windows(2) {
        let start = segment[0];
        let end = segment[1];
        let mut hits: Vec<_> = obstacles
            .iter()
            .copied()
            .filter(|obs| {
                circle_segment(obs.x, obs.y, obs.r, start.0, start.1, end.0, end.1, my_eps) == 1.0
            })
            .collect();

        // 多个圆障碍同时命中同一段航线时，按起点到圆心的距离由近到远处理。
        hits.sort_by(|a, b| {
            norm2([start.0 - a.x, start.1 - a.y]).total_cmp(&norm2([start.0 - b.x, start.1 - b.y]))
        });

        out.push(start);
        for obs in hits {
            if let Some((entry, exit)) = line_circle_intersections(start, end, obs) {
                // C++ 的 circleAvoidRTL 会保留入圆/出圆交点，并在圆的一侧插入
                // 一条与原航线平行的切线段。这里用同样的点序表达绕行路径。
                let (entry_tangent, exit_tangent) =
                    circle_parallel_tangent_points(start, end, entry, exit, obs);
                for (point_idx, point) in [entry, entry_tangent, exit_tangent, exit]
                    .into_iter()
                    .enumerate()
                {
                    // C++ 在航段起点正好落在圆边界时，会输出“航段起点 + 入圆交点”
                    // 两个数值相同的点。普通相邻重复点仍然去掉，但这个首个入圆点必须保留，
                    // 否则完整路径会比 C++ 少一个航点，逐点对比不一致。
                    let keep_cpp_boundary_duplicate = point_idx == 0 && same_point(point, start);
                    if keep_cpp_boundary_duplicate
                        || out.last().is_none_or(|last| !same_point(*last, point))
                    {
                        out.push(point);
                    }
                }
            }
        }
    }

    if let Some(last) = path.last().copied() {
        if out.last().is_none_or(|p| !same_point(*p, last)) {
            out.push(last);
        }
    }
    out
}

fn line_circle_intersections(
    start: (f64, f64),
    end: (f64, f64),
    obs: CircleObstacle,
) -> Option<((f64, f64), (f64, f64))> {
    let dx = end.0 - start.0;
    let dy = end.1 - start.1;
    let fx = start.0 - obs.x;
    let fy = start.1 - obs.y;
    let a = dx * dx + dy * dy;
    if a == 0.0 {
        return None;
    }

    let b = 2.0 * (fx * dx + fy * dy);
    let c = fx * fx + fy * fy - obs.r * obs.r;
    let discriminant = b * b - 4.0 * a * c;
    if discriminant < 0.0 {
        return None;
    }

    let root = discriminant.sqrt();
    let mut t1 = (-b - root) / (2.0 * a);
    let mut t2 = (-b + root) / (2.0 * a);
    if t1 > t2 {
        std::mem::swap(&mut t1, &mut t2);
    }

    if t2 < 0.0 || t1 > 1.0 {
        return None;
    }

    let t1 = t1.clamp(0.0, 1.0);
    let t2 = t2.clamp(0.0, 1.0);
    Some((
        (start.0 + dx * t1, start.1 + dy * t1),
        (start.0 + dx * t2, start.1 + dy * t2),
    ))
}

fn circle_parallel_tangent_points(
    start: (f64, f64),
    end: (f64, f64),
    entry: (f64, f64),
    exit: (f64, f64),
    obs: CircleObstacle,
) -> ((f64, f64), (f64, f64)) {
    let dx = end.0 - start.0;
    let dy = end.1 - start.1;
    if dx.abs() < 1.0e-12 && dy.abs() < 1.0e-12 {
        return (entry, exit);
    }

    // C++ 的 circleAvoidRTL 构造方式：
    // 1. 判断圆心在航线方向的左/右侧，得到绕行侧 isLeft；
    // 2. 过入圆点、出圆点分别作圆的切线；
    // 3. 再作一条与原航线平行、距离圆心 r 的切线；
    // 4. 两组切线交点就是中间两个绕行点。
    // 这比“把交点按法线平移 r”多了沿航线方向的收缩量，圆心不在航线上时尤其明显。
    let s = (start.0 - obs.x) * (end.1 - obs.y) - (start.1 - obs.y) * (end.0 - obs.x);
    let dirc = if s.abs() < 1.0e-8 {
        0.0
    } else if s < 0.0 {
        -1.0
    } else {
        1.0
    };
    let is_left = if start.1 > end.1 {
        if dirc > 0.0 {
            1.0
        } else {
            -1.0
        }
    } else if dirc > 0.0 {
        -1.0
    } else {
        1.0
    };

    let line1 = tangent_line_through_circle_point(entry, obs, is_left);
    let line2 = tangent_line_through_circle_point(exit, obs, is_left);
    let line0 = parallel_tangent_line(start, end, obs, is_left);

    (
        line_intersection(line0, line1).unwrap_or(entry),
        line_intersection(line0, line2).unwrap_or(exit),
    )
}

fn tangent_line_through_circle_point(
    point: (f64, f64),
    obs: CircleObstacle,
    is_left: f64,
) -> ((f64, f64), (f64, f64)) {
    if (point.1 - obs.y).abs() < 1.0e-5 {
        (point, (point.0, point.1 - is_left * 2.0 * obs.r))
    } else {
        let k = -(point.0 - obs.x) / (point.1 - obs.y);
        let x2 = point.0 - is_left * 2.0 * obs.r;
        let y2 = (k * x2 + point.1) - k * point.0;
        (point, (x2, y2))
    }
}

fn parallel_tangent_line(
    start: (f64, f64),
    end: (f64, f64),
    obs: CircleObstacle,
    is_left: f64,
) -> ((f64, f64), (f64, f64)) {
    if (end.0 - start.0).abs() < 1.0e-5 {
        let x = obs.x - is_left * obs.r;
        ((x, start.1), (x, end.1))
    } else {
        let k = (end.1 - start.1) / (end.0 - start.0);
        let line_y_at_center_x = (k * obs.x - k * start.0) + start.1;
        let b = if line_y_at_center_x > obs.y {
            (obs.r * (k * k + 1.0).sqrt() - k * obs.x) + obs.y
        } else {
            (-obs.r * (k * k + 1.0).sqrt() - k * obs.x) + obs.y
        };
        ((start.0, k * start.0 + b), (end.0, k * end.0 + b))
    }
}

fn line_intersection(
    line_a: ((f64, f64), (f64, f64)),
    line_b: ((f64, f64), (f64, f64)),
) -> Option<(f64, f64)> {
    let (x1, y1) = line_a.0;
    let (x2, y2) = line_a.1;
    let (x3, y3) = line_b.0;
    let (x4, y4) = line_b.1;
    let denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if denom.abs() < 1.0e-12 {
        return None;
    }
    let det_a = x1 * y2 - y1 * x2;
    let det_b = x3 * y4 - y3 * x4;
    Some((
        (det_a * (x3 - x4) - (x1 - x2) * det_b) / denom,
        (det_a * (y3 - y4) - (y1 - y2) * det_b) / denom,
    ))
}

pub fn min_dis_vertice_to_vertice_and_line(vertice_x: &[f64], vertice_y: &[f64]) -> [f64; 2] {
    let len = vertice_x.len().min(vertice_y.len());
    if len < 3 {
        return [0.0, 0.0];
    }

    let mut edge_distances = vec![0.0; len];
    for k in 0..len {
        let index1 = (k + 1) % len;
        edge_distances[k] = cal_two_point_dis(
            [vertice_x[k], vertice_y[k]],
            [vertice_x[index1], vertice_y[index1]],
        );
    }
    let min_vertex_distance = edge_distances.iter().copied().fold(f64::INFINITY, f64::min);

    let mut point_line_distances = vec![0.0; len];
    let mut cal_height_temp = vec![0.0; len - 2];
    for k in 0..len {
        for j in 0..(len - 2) {
            let index1 = (k + j + 1) % len;
            let index2 = (k + j + 2) % len;
            let a = [vertice_x[k], vertice_y[k]];
            let b = [vertice_x[index1], vertice_y[index1]];
            let c = [vertice_x[index2], vertice_y[index2]];
            let dis_ab = cal_two_point_dis(a, b);
            let dis_bc = cal_two_point_dis(b, c);
            let dis_ac = cal_two_point_dis(a, c);
            let p = (dis_ab + dis_bc + dis_ac) / 2.0;
            let temp = p * (p - dis_ab) * (p - dis_bc) * (p - dis_ac);
            let area = if temp > 0.0 { temp.sqrt() } else { 0.0 };
            cal_height_temp[j] = 2.0 * area / dis_bc;

            if (vertice_x[index1] - vertice_x[k]) * (vertice_x[index1] - vertice_x[index2])
                + (vertice_y[index1] - vertice_y[k]) * (vertice_y[index1] - vertice_y[index2])
                < 0.0
            {
                cal_height_temp[j] *= 1000.0;
            }

            let vect1 = [
                vertice_x[index1] - vertice_x[k],
                vertice_y[index1] - vertice_y[k],
                0.0,
            ];
            let vect2 = [
                vertice_x[index2] - vertice_x[index1],
                vertice_y[index2] - vertice_y[index1],
                0.0,
            ];
            let denom = norm3(vect1) * norm3(vect2);
            if denom != 0.0 && ((vect1[0] * vect2[1] - vect1[1] * vect2[0]) / denom).abs() < 0.15 {
                if j == 0 {
                    cal_height_temp[0] = dis_ab.min(dis_ac);
                } else {
                    cal_height_temp[j] = cal_height_temp[j - 1];
                }
            }
        }

        point_line_distances[k] = cal_height_temp
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min);
    }

    [
        min_vertex_distance,
        point_line_distances
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min),
    ]
}

pub fn concave_vex(lat: &[f64], lon: &[f64], cw_flag: f64) -> Vec<f64> {
    let len = lat.len().min(lon.len());
    let mut out = vec![0.0; len + 2];
    let mut convex_count = 0.0;
    let mut concave_indices = Vec::new();

    for k in 0..len {
        let j = (k + 1) % len;
        let i = (k + 2) % len;
        let a = [lat[k], lon[k], 0.0];
        let b = [lat[j], lon[j], 0.0];
        let c = [lat[i], lon[i], 0.0];
        if judge_concave_vex(a, b, c, cw_flag) == 1.0 {
            convex_count += 1.0;
        } else {
            concave_indices.push((j + 1) as f64);
        }
    }

    out[0] = convex_count;
    out[1] = concave_indices.len() as f64;
    for (idx, value) in concave_indices.into_iter().enumerate() {
        out[idx + 2] = value;
    }
    out
}

#[derive(Debug, Clone)]
pub struct MainShrOutResult {
    pub vertices: Vec<[f64; 3]>,
    pub out_cnt: usize,
    pub shr_out_flag: f64,
    pub min_distance: f64,
}

#[derive(Debug, Clone)]
pub struct CoverMapResult {
    pub waypoints: Vec<[f64; 3]>,
}

pub fn cover_map_by_yaw(
    map: &[(f64, f64)],
    width: f64,
    yaw: f64,
    f2c: f64,
    dir: f64,
) -> CoverMapResult {
    if map.len() < 3 || width <= 0.0 {
        return CoverMapResult {
            waypoints: Vec::new(),
        };
    }

    let col_dir = if (0.0..180.0).contains(&yaw) {
        dir
    } else {
        -dir
    };
    let dw = if (map[0].1 - map[1].1).abs() > 1.0 {
        if col_dir > 0.0 {
            0.1
        } else {
            -0.1
        }
    } else {
        0.0
    };

    let min_x = map.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
    let max_x = map.iter().map(|p| p.0).fold(f64::NEG_INFINITY, f64::max);
    let max_y = map.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);
    let min_y = map.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);

    let (first_line_pos, last_line_pos) = if col_dir > 0.0 {
        (min_x, max_x)
    } else {
        (max_x, min_x)
    };
    let col_num_end = (((max_x - min_x) + dw) / width).ceil();
    if col_num_end + 1.0 > 2500.0 {
        return CoverMapResult {
            waypoints: Vec::new(),
        };
    }

    let mut waypoints: Vec<[f64; 3]> = Vec::new();
    let mut col_cnt = 0.0;
    while col_cnt < col_num_end + 1.0 {
        let mut x = if col_cnt == (col_num_end + 1.0) - 1.0 {
            let mut line_x = col_cnt * col_dir * width + first_line_pos;
            if col_dir > 0.0 {
                if line_x > last_line_pos && (line_x - last_line_pos).abs() < 1.0 {
                    line_x = last_line_pos - dw;
                }
            } else if line_x < last_line_pos && (line_x - last_line_pos).abs() < 1.0 {
                line_x = last_line_pos - dw;
            }
            line_x
        } else {
            col_cnt * col_dir * width + first_line_pos + dw
        };

        if x.abs() < 1.0e-12 {
            x = 0.0;
        }

        let floor_to_ceiling = if f2c != 0.0 {
            matlab_mod(col_cnt, 2.0) == 0.0
        } else {
            matlab_mod(col_cnt, 2.0) != 0.0
        };

        let mut line_points =
            scanline_intersections(map, x, max_y + 5.0, min_y - 5.0, floor_to_ceiling);
        if let Some(first) = line_points.first_mut() {
            first[2] = 1.0;
        }
        if let Some(last) = line_points.last_mut() {
            last[2] = 2.0;
        }

        // 当前扫描线和上一条扫描线距离过大时，直接连接可能穿出地块。
        // 此时沿地块边界生成一段绕行点，等价于 C++ 中的 edgeCollision 分支。
        if let (Some(prev), Some(curr)) = (waypoints.last().copied(), line_points.first().copied())
        {
            if norm2([prev[0] - curr[0], prev[1] - curr[1]]) > 2.0 * width {
                let bypass = edge_collision(map, (prev[0], prev[1], curr[0], curr[1]));
                if bypass.len() > 2 {
                    for point in bypass {
                        waypoints.push([point.0, point.1, 4.0]);
                    }
                }
            }
        }
        waypoints.extend(line_points);
        col_cnt += 1.0;
    }

    CoverMapResult { waypoints }
}

fn scanline_intersections(
    map: &[(f64, f64)],
    x: f64,
    y1: f64,
    y2: f64,
    floor_to_ceiling: bool,
) -> Vec<[f64; 3]> {
    let mut points: Vec<(f64, f64)> = Vec::new();
    for i in 0..map.len() {
        let next = (i + 1) % map.len();
        let cross = segment_intersection(
            (x, y1, x, y2),
            (map[i].0, map[i].1, map[next].0, map[next].1),
        );
        if cross.cross != 0.0
            && !points
                .iter()
                .any(|p| (p.0 - cross.x).abs() < 1.0e-5 && (p.1 - cross.y).abs() < 1.0e-5)
        {
            points.push((cross.x, cross.y));
        }
    }

    if floor_to_ceiling {
        points.sort_by(|a, b| a.1.total_cmp(&b.1));
    } else {
        points.sort_by(|a, b| b.1.total_cmp(&a.1));
    }

    if points.len() > 50 {
        return Vec::new();
    }

    points.into_iter().map(|(px, py)| [px, py, 4.0]).collect()
}

pub fn main_shr_out(lat_in: &[f64], lon_in: &[f64], distance: f64) -> MainShrOutResult {
    main_shr_out_with_mode(lat_in, lon_in, distance, true)
}

pub fn main_shr_out_expand(lat_in: &[f64], lon_in: &[f64], distance: f64) -> MainShrOutResult {
    main_shr_out_with_mode(lat_in, lon_in, distance, false)
}

fn main_shr_out_with_mode(
    lat_in: &[f64],
    lon_in: &[f64],
    distance: f64,
    shrink: bool,
) -> MainShrOutResult {
    let len = lat_in.len().min(lon_in.len());
    if len == 0 {
        return MainShrOutResult {
            vertices: Vec::new(),
            out_cnt: 0,
            shr_out_flag: 0.0,
            min_distance: 0.0,
        };
    }

    let lat_in = &lat_in[..len];
    let lon_in = &lon_in[..len];
    let cwa_flag = judge_clockwise(lat_in, lon_in);
    let mut concave_flags = vec![0.0; len];
    for k in 0..len {
        let j = (k + 1) % len;
        let i = (k + 2) % len;
        let value = judge_concave_vex(
            [lat_in[k], lon_in[k], 0.0],
            [lat_in[j], lon_in[j], 0.0],
            [lat_in[i], lon_in[i], 0.0],
            cwa_flag,
        );
        if k + 1 < len {
            concave_flags[k + 1] = value;
        } else {
            concave_flags[0] = value;
        }
    }

    let distance2 = min_dis_vertice_to_vertice_and_line(lat_in, lon_in);
    let mut min_distance = distance2[0].min(distance2[1]) / 2.0;

    for k in 0..len {
        let j1 = (k + 1) % len;
        let j2 = (k + 2) % len;
        let j3 = (k + 3) % len;
        if concave_flags[k] == 1.0
            && concave_flags[j1] == 0.0
            && concave_flags[j2] == 1.0
            && concave_flags[j3] == 0.0
        {
            let convex_points_dist =
                cal_two_point_dis([lat_in[k], lon_in[k]], [lat_in[j2], lon_in[j2]]);
            let dis_convex_points_dist =
                cal_two_point_dis([lat_in[j1], lon_in[j1]], [lat_in[j3], lon_in[j3]]);
            min_distance = min_distance.min(convex_points_dist.min(dis_convex_points_dist));
        }
    }

    let concave_vex_num = concave_vex(lat_in, lon_in, cwa_flag);
    if concave_vex_num[0] == len as f64 {
        min_distance = distance2[0].min(distance2[1]) / 2.0;
    }

    let zoomed = zoom_in_out(lat_in, lon_in, cwa_flag, distance, shrink);
    let mut lat_out: Vec<f64> = zoomed.iter().map(|point| point[0]).collect();
    let mut lon_out: Vec<f64> = zoomed.iter().map(|point| point[1]).collect();
    let mut shr_out_flag = 1.0;

    if has_edge_intersection(&lat_out, &lon_out) {
        shr_out_flag = 0.0;
        lat_out.copy_from_slice(lat_in);
        lon_out.copy_from_slice(lon_in);
    }

    let mut vertices = vec![[0.0; 3]; len];
    for i in 0..len {
        vertices[i] = [lat_out[i], lon_out[i], 0.0];
    }
    vertices[0][2] = shr_out_flag;
    if len > 1 {
        vertices[1][2] = min_distance;
    }

    MainShrOutResult {
        vertices,
        out_cnt: len,
        shr_out_flag,
        min_distance,
    }
}

fn has_edge_intersection(lat: &[f64], lon: &[f64]) -> bool {
    let len = lat.len().min(lon.len());
    if len < 4 {
        return false;
    }

    let ref_lat = lat[0];
    let ref_lon = lon[0];
    let lat_r = 6.371_393e6 * ref_lat.to_radians().cos();
    let mut map = vec![(0.0, 0.0); len];
    for i in 1..len {
        map[i].0 = 6.371_393e6 * (lat[i] - ref_lat).to_radians();
        map[i].1 = lat_r * (lon[i] - ref_lon).to_radians();
    }

    for i in 0..len {
        let line1 = edge_tuple(&map, i);
        for j in 0..len {
            if i == j {
                continue;
            }
            let line2 = edge_tuple(&map, j);
            if segment_intersection3(line1, line2).cross == 1.0 {
                return true;
            }
        }
    }
    false
}

fn edge_tuple(points: &[(f64, f64)], index: usize) -> (f64, f64, f64, f64) {
    let next = (index + 1) % points.len();
    (
        points[index].0,
        points[index].1,
        points[next].0,
        points[next].1,
    )
}

pub fn edge_collision(map: &[(f64, f64)], c_points: (f64, f64, f64, f64)) -> Vec<(f64, f64)> {
    if map.len() < 3 {
        return vec![(c_points.0, c_points.1), (c_points.2, c_points.3)];
    }

    let start = (c_points.0, c_points.1);
    let end = (c_points.2, c_points.3);
    let mut map_force = map.to_vec();
    let mut intersections: Vec<((f64, f64), f64)> = Vec::new();

    // 1. 找连接线与地块边界的交点；非顶点交点要插入到边界点序列中。
    for edge_index in 0..map.len() {
        let next = (edge_index + 1) % map.len();
        let edge = (
            map[edge_index].0,
            map[edge_index].1,
            map[next].0,
            map[next].1,
        );
        let cross = segment_intersection2(c_points, edge);
        if cross.cross == 1.0 || cross.cross == 2.0 {
            let point = (cross.x, cross.y);
            if !same_point(point, map[edge_index]) && !same_point(point, map[next]) {
                let insert_after = map_force
                    .iter()
                    .position(|p| same_point(*p, map[edge_index]))
                    .map(|idx| idx + 1)
                    .unwrap_or(map_force.len());
                if !map_force.iter().any(|p| same_point(*p, point)) {
                    map_force.insert(insert_after, point);
                }
            }

            let dist_from_start = norm2([point.0 - start.0, point.1 - start.1]);
            intersections.push((point, dist_from_start));
        }
    }

    // 没有足够交点时，无法判断绕边路径，保持直连。
    if intersections.len() < 2 {
        return vec![start, end];
    }

    intersections.sort_by(|a, b| a.1.total_cmp(&b.1));
    let first_cross = intersections[0].0;
    let last_cross = intersections[intersections.len() - 1].0;

    let Some(index1) = map_force.iter().position(|p| same_point(*p, first_cross)) else {
        return vec![start, end];
    };
    let Some(index2) = map_force.iter().position(|p| same_point(*p, last_cross)) else {
        return vec![start, end];
    };

    // 2. 从两个方向沿边界走，选择总距离更短的一条绕行路径。
    let forward = boundary_path(&map_force, index1, index2, 1);
    let backward = boundary_path(&map_force, index1, index2, -1);
    let forward_len = polyline_length(&forward);
    let backward_len = polyline_length(&backward);
    let boundary = if forward_len <= backward_len {
        forward
    } else {
        backward
    };

    // 3. 输出包含原始连接线起点和边界绕行点；不包含原始终点。
    // C++ 调用方会在随后追加下一条扫描线的航点，因此这里停在边界交点。
    let mut out = Vec::with_capacity(boundary.len() + 1);
    out.push(start);
    out.extend(boundary);
    dedup_adjacent_points(out)
}

pub fn pyg_obs_collision(
    polygon: &[(f64, f64, f64)],
    pyg_state: f64,
    c_points: (f64, f64, f64, f64),
) -> Vec<(f64, f64)> {
    if polygon.len() < 3 {
        return vec![(c_points.0, c_points.1)];
    }

    let start = (c_points.0, c_points.1);
    let mut map_force: Vec<(f64, f64)> = polygon.iter().map(|p| (p.0, p.1)).collect();
    let mut intersections: Vec<((f64, f64), f64)> = Vec::new();

    // 1. 找航线与多边形障碍边界的交点，并把非顶点交点插入边界序列。
    for edge_index in 0..polygon.len() {
        let next = (edge_index + 1) % polygon.len();
        let edge = (
            polygon[edge_index].0,
            polygon[edge_index].1,
            polygon[next].0,
            polygon[next].1,
        );
        let cross = segment_intersection2(c_points, edge);
        if cross.cross == 1.0 || cross.cross == 2.0 {
            let point = (cross.x, cross.y);
            if !same_point(point, (polygon[edge_index].0, polygon[edge_index].1))
                && !same_point(point, (polygon[next].0, polygon[next].1))
            {
                let insert_after = map_force
                    .iter()
                    .position(|p| same_point(*p, (polygon[edge_index].0, polygon[edge_index].1)))
                    .map(|idx| idx + 1)
                    .unwrap_or(map_force.len());
                if !map_force.iter().any(|p| same_point(*p, point)) {
                    map_force.insert(insert_after, point);
                }
            }
            intersections.push((point, norm2([point.0 - start.0, point.1 - start.1])));
        }
    }

    if intersections.len() < 2 {
        return vec![start];
    }

    intersections.sort_by(|a, b| a.1.total_cmp(&b.1));
    let first_cross = intersections[0].0;
    let last_cross = intersections[intersections.len() - 1].0;

    if pyg_state != 1.0 {
        // C++ 中 pygState=1 表示需要避让的普通障碍；其它状态不会沿障碍边界绕行。
        // 这里只把航线和多边形的入/出交点插入路径，相当于允许航线穿过该区域。
        // 保留这个分支可以避免把非避障区域误当成实体障碍导致路径变长。
        return dedup_adjacent_points(vec![start, first_cross, last_cross]);
    }

    let Some(index1) = map_force.iter().position(|p| same_point(*p, first_cross)) else {
        return vec![start];
    };
    let Some(index2) = map_force.iter().position(|p| same_point(*p, last_cross)) else {
        return vec![start];
    };

    // 2. pygState=1 表示需要避让；C++ 还会检查多边形第三列 State：
    // - 第三列总和为 0：普通内部障碍，优先选择较短的边界绕行；
    // - 第三列总和非 0：边界类障碍，按入/出交点在边界序列中的先后确定方向。
    // 这个分支会影响 safe_dist_obs 外扩后的交点落在哪条边上，必须保留。
    let forward = boundary_path(&map_force, index1, index2, 1);
    let backward = boundary_path(&map_force, index1, index2, -1);
    let forward_len = polyline_length(&forward);
    let backward_len = polyline_length(&backward);
    let state_sum: f64 = polygon.iter().map(|p| p.2).sum();
    let boundary = if state_sum != 0.0 {
        // C++ 对边界类障碍不用环绕最短路：先取两个交点索引的较小/较大值，
        // 再根据入点和出点的原始先后选择“较小到较大”或“较大到较小”的直连边界段。
        let start_idx = index1.min(index2);
        let end_idx = index1.max(index2);
        if index1 < index2 {
            boundary_path(&map_force, start_idx, end_idx, 1)
        } else {
            boundary_path(&map_force, end_idx, start_idx, -1)
        }
    } else if forward_len <= backward_len || (forward_len - backward_len).abs() < 1.0e-9 {
        forward
    } else {
        backward
    };

    // 3. 和 C++ 一致：输出起点和绕行边界点，不追加原始终点。
    let mut out = Vec::with_capacity(boundary.len() + 1);
    out.push(start);
    out.extend(boundary);
    dedup_adjacent_points(out)
}

pub fn pyg_obs_sort(
    polygons: &[Vec<(f64, f64, f64)>],
    c_points: (f64, f64, f64, f64),
) -> Vec<Vec<(f64, f64, f64)>> {
    let start = (c_points.0, c_points.1);
    let mut indexed: Vec<_> = polygons
        .iter()
        .cloned()
        .map(|poly| {
            let min_dist = poly
                .iter()
                .map(|p| norm2([p.0 - start.0, p.1 - start.1]))
                .fold(f64::INFINITY, f64::min);
            (min_dist, poly)
        })
        .collect();
    indexed.sort_by(|a, b| a.0.total_cmp(&b.0));
    indexed.into_iter().map(|(_, poly)| poly).collect()
}

pub fn pyg_obs_avoid(
    path: &[(f64, f64)],
    polygons: &[Vec<(f64, f64, f64)>],
    states: &[f64],
) -> Vec<(f64, f64)> {
    if path.len() < 2 || polygons.is_empty() {
        return path.to_vec();
    }

    let mut out = Vec::new();
    for segment in path.windows(2) {
        let start = segment[0];
        let end = segment[1];
        let sorted = pyg_obs_sort(polygons, (start.0, start.1, end.0, end.1));
        let mut segment_points = vec![start];

        // 对当前航线段依次处理可能相交的多边形障碍。
        // 每个 pyg_obs_collision 输出“起点 + 绕行边界点”，不包含原始终点；
        // 原始终点在本段末尾统一追加，保持 C++ 调用语义。
        for polygon in sorted {
            let state = polygons
                .iter()
                .position(|p| *p == polygon)
                .and_then(|idx| states.get(idx).copied())
                .unwrap_or(1.0);
            let bypass = pyg_obs_collision(&polygon, state, (start.0, start.1, end.0, end.1));
            if bypass.len() > 1 {
                // C++ 会在同一条航线段上连续追加所有相交障碍的绕行点。
                // 每段 bypass 都以航线段起点开头，因此后续障碍需要跳过该重复起点。
                for point in bypass.into_iter().skip(1) {
                    if segment_points
                        .last()
                        .is_none_or(|last| !same_point(*last, point))
                    {
                        segment_points.push(point);
                    }
                }
            }
        }

        out.extend(segment_points);
    }

    if let Some(last) = path.last().copied() {
        if out.last().is_none_or(|p| !same_point(*p, last)) {
            out.push(last);
        }
    }
    out
}

fn boundary_path(
    map_force: &[(f64, f64)],
    start: usize,
    end: usize,
    step: isize,
) -> Vec<(f64, f64)> {
    let mut out = Vec::new();
    let len = map_force.len() as isize;
    let mut idx = start as isize;
    loop {
        out.push(map_force[idx as usize]);
        if idx as usize == end {
            break;
        }
        idx = (idx + step).rem_euclid(len);
    }
    out
}

fn polyline_length(points: &[(f64, f64)]) -> f64 {
    points
        .windows(2)
        .map(|pair| distance(pair[0], pair[1]))
        .sum()
}

fn same_point(a: (f64, f64), b: (f64, f64)) -> bool {
    (a.0 - b.0).abs() < 1.0e-5 && (a.1 - b.1).abs() < 1.0e-5
}

fn dedup_adjacent_points(points: Vec<(f64, f64)>) -> Vec<(f64, f64)> {
    let mut out = Vec::with_capacity(points.len());
    for point in points {
        if out.last().is_none_or(|prev| !same_point(*prev, point)) {
            out.push(point);
        }
    }
    out
}

pub fn path_length(points: &[(f64, f64)]) -> f64 {
    points
        .windows(2)
        .map(|pair| distance(pair[0], pair[1]))
        .sum()
}

pub fn polygon_area(points: &[(f64, f64)]) -> f64 {
    if points.len() < 3 {
        return 0.0;
    }

    let mut area = 0.0;
    for i in 0..points.len() {
        let (x1, y1) = points[i];
        let (x2, y2) = points[(i + 1) % points.len()];
        area += x1 * y2 - x2 * y1;
    }
    area.abs() * 0.5
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64) {
        assert!((a - b).abs() < 1.0e-9, "{a} != {b}");
    }

    #[test]
    fn yaw_matches_cardinal_directions() {
        close(cal_yaw(30.0, 120.0, 30.001, 120.0), 0.0);
        close(cal_yaw(30.0, 120.0, 30.0, 120.001), 89.99975);
    }

    #[test]
    fn scalar_helpers_match_generated_semantics() {
        close(eq_eps_f64(1.0, 1.0 + 0.5e-5), 1.0);
        close(eq_eps_f64(1.0, 1.0 + 2.0e-5), 0.0);
        close(eq_eps_i32(0), 1.0);
        close(eq_eps_i32(1), 0.0);
        close(matlab_mod(5.5, 2.0), 1.5);
        close(matlab_mod(-1.0, 3.0), 2.0);
    }

    #[test]
    fn vector_helpers_work() {
        close(norm2([3.0, 4.0]), 5.0);
        close(norm3([2.0, 3.0, 6.0]), 7.0);
        close(det2([1.0, 3.0, 2.0, 4.0]), -2.0);
        close(sum_f64_slice(&[1.0, 2.0, 3.5]), 6.5);
        close(sum_i32_slice(&[1, 2, 3]), 6.0);

        let q = vect_sin([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], 2.0);
        close(q[0], -1.0);
        close(q[1], 2.0);
        close(q[2], 0.0);
    }

    #[test]
    fn line_and_segment_helpers_work() {
        close(line_point2(0.0, 0.0, 10.0, 0.0, 5.0, 0.0, 1.0e-5), 1.0);
        close(line_point2(0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 1.0e-5), 0.0);
        close(line_point3(0.0, 0.0, 10.0, 0.0, 5.0, 0.05), 1.0);

        let cross = segment_intersection((0.0, 0.0, 10.0, 0.0), (5.0, -1.0, 5.0, 1.0));
        close(cross.x, 5.0);
        close(cross.y, 0.0);
        close(cross.cross, 1.0);

        let endpoint = segment_intersection3((0.0, 0.0, 10.0, 0.0), (10.0, 0.0, 10.0, 1.0));
        close(endpoint.cross, 0.0);
    }

    #[test]
    fn polygon_orientation_and_vex_work() {
        close(
            judge_clockwise(&[0.0, 1.0, 1.0, 0.0], &[0.0, 0.0, 1.0, 1.0]),
            0.0,
        );
        close(
            judge_clockwise(&[0.0, 0.0, 1.0, 1.0], &[0.0, 1.0, 1.0, 0.0]),
            1.0,
        );
        close(
            judge_concave_vex([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], 0.0),
            1.0,
        );
    }

    #[test]
    fn offset_and_circle_segment_helpers_work() {
        let lat = [0.0, 1.0, 1.0, 0.0];
        let lon = [0.0, 0.0, 1.0, 1.0];
        let zoomed = zoom_in_out(&lat, &lon, 0.0, 10.0, true);
        assert_eq!(zoomed.len(), 4);
        assert!(zoomed.iter().all(|p| p.iter().all(|v| v.is_finite())));

        close(
            circle_segment(0.0, 0.0, 1.0, -2.0, 0.0, 2.0, 0.0, 1.0e-5),
            1.0,
        );
        close(
            circle_segment(0.0, 0.0, 1.0, -2.0, 2.0, 2.0, 2.0, 1.0e-5),
            0.0,
        );
    }

    #[test]
    fn min_distance_helper_works() {
        let x = [30.0, 30.0, 30.001, 30.001];
        let y = [120.0, 120.001, 120.001, 120.0];
        let distances = min_dis_vertice_to_vertice_and_line(&x, &y);
        assert!(distances[0] > 90.0);
        assert!(distances[1] > 90.0);
    }

    #[test]
    fn main_shr_out_offsets_simple_rectangle() {
        let lat = [30.0, 30.0, 30.001, 30.001];
        let lon = [120.0, 120.001, 120.001, 120.0];
        let result = main_shr_out(&lat, &lon, 0.0);
        assert_eq!(result.out_cnt, 4);
        close(result.shr_out_flag, 1.0);
        assert_eq!(result.vertices.len(), 4);
        assert!(result.min_distance > 40.0);
    }

    #[test]
    fn main_shr_out_matches_cpp_for_simple_rectangle() {
        let lat = [30.0, 30.0, 30.001, 30.001];
        let lon = [120.0, 120.001, 120.001, 120.0];
        let rust_result = main_shr_out(&lat, &lon, 0.0);
        let (cpp_vertices, cpp_count) = crate::ffi::run_cpp_main_shr_out(&lat, &lon, 0.0);

        close(rust_result.out_cnt as f64, cpp_count);
        assert_eq!(rust_result.vertices.len(), cpp_vertices.len());
        for (rust_vertex, cpp_vertex) in rust_result.vertices.iter().zip(cpp_vertices.iter()) {
            for i in 0..3 {
                assert!(
                    (rust_vertex[i] - cpp_vertex[i]).abs() < 1.0e-9,
                    "rust {:?} != cpp {:?}",
                    rust_vertex,
                    cpp_vertex
                );
            }
        }
    }

    #[test]
    fn main_shr_out_expand_matches_cpp_for_simple_rectangle() {
        let lat = [30.0004, 30.0004, 30.0006, 30.0006];
        let lon = [120.0004, 120.0006, 120.0006, 120.0004];
        let rust_result = main_shr_out_expand(&lat, &lon, 1.5);
        let (cpp_vertices, cpp_count) = crate::ffi::run_cpp_main_shr_out_expand(&lat, &lon, 1.5);

        close(rust_result.out_cnt as f64, cpp_count as f64);
        assert_eq!(rust_result.vertices.len(), cpp_vertices.len());
        for (rust_vertex, cpp_vertex) in rust_result.vertices.iter().zip(cpp_vertices.iter()) {
            for i in 0..3 {
                assert!(
                    (rust_vertex[i] - cpp_vertex[i]).abs() < 1.0e-9,
                    "rust {:?} != cpp {:?}; rust_vertices={:?}; cpp_vertices={:?}",
                    rust_vertex,
                    cpp_vertex,
                    rust_result.vertices,
                    cpp_vertices
                );
            }
        }
    }

    #[test]
    fn cover_map_by_yaw_scans_simple_rectangle() {
        let map = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0)];
        let result = cover_map_by_yaw(&map, 20.0, 0.0, 1.0, 1.0);
        assert_eq!(result.waypoints.len(), 12);
        close(result.waypoints[0][0], 0.0);
        close(result.waypoints[0][1], 0.0);
        close(result.waypoints[1][1], 100.0);
    }

    #[test]
    fn cover_map_by_yaw_matches_cpp_for_simple_rectangle() {
        let map = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0)];
        for (yaw, f2c, dir) in [
            (0.0, 1.0, 1.0),
            (0.0, 0.0, 1.0),
            (0.0, 1.0, -1.0),
            (180.0, 1.0, 1.0),
        ] {
            let rust_result = cover_map_by_yaw(&map, 20.0, yaw, f2c, dir);
            let (cpp_waypoints, cpp_count) =
                crate::ffi::run_cpp_cover_map_by_yaw(&map, 20.0, yaw, f2c, dir);

            close(rust_result.waypoints.len() as f64, cpp_count);
            for (rust_wp, cpp_wp) in rust_result.waypoints.iter().zip(cpp_waypoints.iter()) {
                for i in 0..3 {
                    assert!(
                        (rust_wp[i] - cpp_wp[i]).abs() < 1.0e-9,
                        "params yaw={yaw}, f2c={f2c}, dir={dir}; rust {:?} != cpp {:?}",
                        rust_wp,
                        cpp_wp
                    );
                }
            }
        }
    }

    #[test]
    fn segment_intersection2_handles_endpoint_touch() {
        let cross = segment_intersection2((0.0, 0.0, 10.0, 0.0), (10.0, -1.0, 10.0, 1.0));
        close(cross.x, 10.0);
        close(cross.y, 0.0);
        close(cross.cross, 2.0);
    }

    #[test]
    fn edge_collision_wraps_around_boundary() {
        let map = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0)];
        let path = edge_collision(&map, (-10.0, 50.0, 110.0, 50.0));
        assert!(path.len() >= 4);
        assert_eq!(path.first().copied(), Some((-10.0, 50.0)));
        assert_eq!(path.last().copied(), Some((100.0, 50.0)));
        assert!(path.iter().any(|p| same_point(*p, (0.0, 50.0))));
        assert!(path.iter().any(|p| same_point(*p, (100.0, 50.0))));
    }

    #[test]
    fn edge_collision_matches_cpp_for_rectangle_crossing() {
        let map = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0)];
        let c_points = (-10.0, 50.0, 110.0, 50.0);
        let rust_path = edge_collision(&map, c_points);
        let (cpp_path, cpp_count) = crate::ffi::run_cpp_edge_collision(&map, c_points);

        close(rust_path.len() as f64, cpp_count);
        for (rust_point, cpp_point) in rust_path.iter().zip(cpp_path.iter()) {
            assert!(
                same_point(*rust_point, *cpp_point),
                "rust {:?} != cpp {:?}; rust_path={:?}; cpp_path={:?}",
                rust_point,
                cpp_point,
                rust_path,
                cpp_path
            );
        }
    }

    #[test]
    fn circle_avoid_keeps_path_when_no_collision() {
        let path = [(0.0, 0.0), (10.0, 0.0)];
        let obstacles = [CircleObstacle {
            x: 5.0,
            y: 5.0,
            r: 1.0,
        }];
        assert_eq!(circle_avoid_rtl(&path, &obstacles, 1.0e-5), path);
    }

    #[test]
    fn circle_avoid_matches_cpp_when_no_collision() {
        let path = [(0.0, 0.0), (10.0, 0.0)];
        let obstacles = [(5.0, 5.0, 1.0)];
        let rust_obstacles = [CircleObstacle {
            x: obstacles[0].0,
            y: obstacles[0].1,
            r: obstacles[0].2,
        }];
        let rust_path = circle_avoid_rtl(&path, &rust_obstacles, 1.0e-5);
        let (cpp_path, cpp_count) = crate::ffi::run_cpp_circle_avoid_rtl(&path, &obstacles, 1.0e-5);

        close(rust_path.len() as f64, cpp_count);
        for (rust, cpp) in rust_path.iter().zip(cpp_path.iter()) {
            assert!(same_point(*rust, *cpp), "rust {rust:?} != cpp {cpp:?}");
        }
    }

    #[test]
    fn circle_avoid_inserts_detour_for_crossing() {
        let path = [(0.0, 0.0), (10.0, 0.0)];
        let obstacles = [CircleObstacle {
            x: 5.0,
            y: 0.0,
            r: 1.0,
        }];
        let avoided = circle_avoid_rtl(&path, &obstacles, 0.1);
        assert!(avoided.len() > path.len());
        assert!(avoided.iter().all(|p| norm2([p.0 - 5.0, p.1]) >= 1.0));
    }

    #[test]
    fn circle_avoid_matches_cpp_for_horizontal_crossing() {
        let path = [(0.0, 0.0), (10.0, 0.0)];
        let obstacles = [(5.0, 0.0, 1.0)];
        let rust_obstacles = [CircleObstacle {
            x: obstacles[0].0,
            y: obstacles[0].1,
            r: obstacles[0].2,
        }];
        let rust_path = circle_avoid_rtl(&path, &rust_obstacles, 0.1);
        let (cpp_path, cpp_count) = crate::ffi::run_cpp_circle_avoid_rtl(&path, &obstacles, 0.1);

        close(rust_path.len() as f64, cpp_count);
        for (rust, cpp) in rust_path.iter().zip(cpp_path.iter()) {
            assert!(
                same_point(*rust, *cpp),
                "rust {rust:?} != cpp {cpp:?}; rust_path={rust_path:?}; cpp_path={cpp_path:?}"
            );
        }
    }

    #[test]
    fn circle_avoid_matches_cpp_for_vertical_crossing() {
        let path = [(0.0, 0.0), (0.0, 10.0)];
        let obstacles = [(0.0, 5.0, 1.0)];
        let rust_obstacles = [CircleObstacle {
            x: obstacles[0].0,
            y: obstacles[0].1,
            r: obstacles[0].2,
        }];
        let rust_path = circle_avoid_rtl(&path, &rust_obstacles, 0.1);
        let (cpp_path, cpp_count) = crate::ffi::run_cpp_circle_avoid_rtl(&path, &obstacles, 0.1);

        close(rust_path.len() as f64, cpp_count);
        for (rust, cpp) in rust_path.iter().zip(cpp_path.iter()) {
            assert!(
                same_point(*rust, *cpp),
                "rust {rust:?} != cpp {cpp:?}; rust_path={rust_path:?}; cpp_path={cpp_path:?}"
            );
        }
    }

    #[test]
    fn circle_avoid_matches_cpp_for_diagonal_crossing() {
        let path = [(0.0, 0.0), (10.0, 10.0)];
        let obstacles = [(5.0, 5.0, 1.0)];
        let rust_obstacles = [CircleObstacle {
            x: obstacles[0].0,
            y: obstacles[0].1,
            r: obstacles[0].2,
        }];
        let rust_path = circle_avoid_rtl(&path, &rust_obstacles, 0.1);
        let (cpp_path, cpp_count) = crate::ffi::run_cpp_circle_avoid_rtl(&path, &obstacles, 0.1);

        close(rust_path.len() as f64, cpp_count);
        for (rust, cpp) in rust_path.iter().zip(cpp_path.iter()) {
            assert!(
                same_point(*rust, *cpp),
                "rust {rust:?} != cpp {cpp:?}; rust_path={rust_path:?}; cpp_path={cpp_path:?}"
            );
        }
    }

    #[test]
    fn circle_avoid_matches_cpp_for_two_circles_on_one_segment() {
        let path = [(0.0, 0.0), (20.0, 0.0)];
        let obstacles = [(5.0, 0.0, 1.0), (12.0, 0.0, 1.5)];
        let rust_obstacles = obstacles
            .iter()
            .map(|obs| CircleObstacle {
                x: obs.0,
                y: obs.1,
                r: obs.2,
            })
            .collect::<Vec<_>>();
        let rust_path = circle_avoid_rtl(&path, &rust_obstacles, 0.1);
        let (cpp_path, cpp_count) = crate::ffi::run_cpp_circle_avoid_rtl(&path, &obstacles, 0.1);

        close(rust_path.len() as f64, cpp_count);
        for (rust, cpp) in rust_path.iter().zip(cpp_path.iter()) {
            assert!(
                same_point(*rust, *cpp),
                "rust {rust:?} != cpp {cpp:?}; rust_path={rust_path:?}; cpp_path={cpp_path:?}"
            );
        }
    }

    #[test]
    fn circle_avoid_matches_cpp_when_segment_starts_on_circle_boundary() {
        let path = [(0.0, 0.0), (10.0, 0.0)];
        let obstacles = [(1.0, 0.0, 1.0)];
        let rust_obstacles = [CircleObstacle {
            x: obstacles[0].0,
            y: obstacles[0].1,
            r: obstacles[0].2,
        }];
        let rust_path = circle_avoid_rtl(&path, &rust_obstacles, 0.1);
        let (cpp_path, cpp_count) = crate::ffi::run_cpp_circle_avoid_rtl(&path, &obstacles, 0.1);

        close(rust_path.len() as f64, cpp_count);
        for (rust, cpp) in rust_path.iter().zip(cpp_path.iter()) {
            assert!(
                same_point(*rust, *cpp),
                "rust {rust:?} != cpp {cpp:?}; rust_path={rust_path:?}; cpp_path={cpp_path:?}"
            );
        }
    }

    #[test]
    fn pyg_obs_collision_matches_cpp_for_rectangle_crossing() {
        let polygon = [
            (4.0, -1.0, 0.0),
            (6.0, -1.0, 0.0),
            (6.0, 1.0, 0.0),
            (4.0, 1.0, 0.0),
        ];
        let rust_path = pyg_obs_collision(&polygon, 1.0, (0.0, 0.0, 10.0, 0.0));
        let (cpp_path, cpp_count) =
            crate::ffi::run_cpp_pyg_obs_collision(&polygon, 1.0, (0.0, 0.0, 10.0, 0.0));

        close(rust_path.len() as f64, cpp_count);
        for (rust, cpp) in rust_path.iter().zip(cpp_path.iter()) {
            assert!(
                same_point(*rust, *cpp),
                "rust {rust:?} != cpp {cpp:?}; rust_path={rust_path:?}; cpp_path={cpp_path:?}"
            );
        }
    }

    #[test]
    fn pyg_obs_collision_matches_cpp_for_state_zero_rectangle_crossing() {
        let polygon = [
            (4.0, -1.0, 0.0),
            (6.0, -1.0, 0.0),
            (6.0, 1.0, 0.0),
            (4.0, 1.0, 0.0),
        ];
        let rust_path = pyg_obs_collision(&polygon, 0.0, (0.0, 0.0, 10.0, 0.0));
        let (cpp_path, cpp_count) =
            crate::ffi::run_cpp_pyg_obs_collision(&polygon, 0.0, (0.0, 0.0, 10.0, 0.0));

        close(rust_path.len() as f64, cpp_count);
        for (rust, cpp) in rust_path.iter().zip(cpp_path.iter()) {
            assert!(
                same_point(*rust, *cpp),
                "rust {rust:?} != cpp {cpp:?}; rust_path={rust_path:?}; cpp_path={cpp_path:?}"
            );
        }
    }

    #[test]
    fn pyg_obs_collision_matches_cpp_for_concave_polygon_crossing() {
        let polygon = [
            (3.0, -2.0, 0.0),
            (7.0, -2.0, 0.0),
            (7.0, 2.0, 0.0),
            (5.0, 2.0, 0.0),
            (5.0, -0.5, 0.0),
            (3.0, -0.5, 0.0),
        ];
        let rust_path = pyg_obs_collision(&polygon, 1.0, (0.0, 0.0, 10.0, 0.0));
        let (cpp_path, cpp_count) =
            crate::ffi::run_cpp_pyg_obs_collision(&polygon, 1.0, (0.0, 0.0, 10.0, 0.0));

        close(rust_path.len() as f64, cpp_count);
        for (rust, cpp) in rust_path.iter().zip(cpp_path.iter()) {
            assert!(
                same_point(*rust, *cpp),
                "rust {rust:?} != cpp {cpp:?}; rust_path={rust_path:?}; cpp_path={cpp_path:?}"
            );
        }
    }

    #[test]
    fn pyg_obs_avoid_inserts_rectangle_bypass() {
        let path = [(0.0, 0.0), (10.0, 0.0)];
        let polygon = vec![
            (4.0, -1.0, 0.0),
            (6.0, -1.0, 0.0),
            (6.0, 1.0, 0.0),
            (4.0, 1.0, 0.0),
        ];
        let avoided = pyg_obs_avoid(&path, &[polygon], &[1.0]);
        assert_eq!(
            avoided,
            vec![
                (0.0, 0.0),
                (4.0, 0.0),
                (4.0, -1.0),
                (6.0, -1.0),
                (6.0, 0.0),
                (10.0, 0.0),
            ]
        );
    }

    #[test]
    fn pyg_obs_avoid_matches_cpp_for_rectangle_crossing() {
        let path2 = [(0.0, 0.0), (10.0, 0.0)];
        let path3 = [(0.0, 0.0, 1.0), (10.0, 0.0, 2.0)];
        let polygon = vec![
            (4.0, -1.0, 0.0),
            (6.0, -1.0, 0.0),
            (6.0, 1.0, 0.0),
            (4.0, 1.0, 0.0),
        ];
        let rust_path = pyg_obs_avoid(&path2, std::slice::from_ref(&polygon), &[1.0]);
        let (cpp_path, cpp_count) = crate::ffi::run_cpp_pyg_obs_avoid(&path3, &[polygon], &[1.0]);

        close(rust_path.len() as f64, cpp_count);
        for (rust, cpp) in rust_path.iter().zip(cpp_path.iter()) {
            assert!(
                same_point(*rust, *cpp),
                "rust {rust:?} != cpp {cpp:?}; rust_path={rust_path:?}; cpp_path={cpp_path:?}"
            );
        }
    }

    #[test]
    fn pyg_obs_avoid_matches_cpp_for_two_rectangles() {
        let path2 = [(0.0, 0.0), (14.0, 0.0)];
        let path3 = [(0.0, 0.0, 1.0), (14.0, 0.0, 2.0)];
        let polygons = vec![
            vec![
                (3.0, -1.0, 0.0),
                (5.0, -1.0, 0.0),
                (5.0, 1.0, 0.0),
                (3.0, 1.0, 0.0),
            ],
            vec![
                (8.0, -1.0, 0.0),
                (10.0, -1.0, 0.0),
                (10.0, 1.0, 0.0),
                (8.0, 1.0, 0.0),
            ],
        ];
        let rust_path = pyg_obs_avoid(&path2, &polygons, &[1.0, 1.0]);
        let (cpp_path, cpp_count) =
            crate::ffi::run_cpp_pyg_obs_avoid(&path3, &polygons, &[1.0, 1.0]);

        close(rust_path.len() as f64, cpp_count);
        for (rust, cpp) in rust_path.iter().zip(cpp_path.iter()) {
            assert!(
                same_point(*rust, *cpp),
                "rust {rust:?} != cpp {cpp:?}; rust_path={rust_path:?}; cpp_path={cpp_path:?}"
            );
        }
    }

    #[test]
    fn pyg_obs_avoid_matches_cpp_for_expanded_state_rectangle() {
        let lat = [30.0004, 30.0004, 30.0006, 30.0006];
        let lon = [120.0004, 120.0006, 120.0006, 120.0004];
        let expanded = main_shr_out_expand(&lat, &lon, 1.5);
        let polygon = expanded
            .vertices
            .iter()
            .take(expanded.out_cnt)
            .map(|p| (p[0], p[1], 1.0))
            .collect::<Vec<_>>();
        let path2 = [(30.0, 120.0005607268681), (30.001, 120.0005607268681)];
        let path3 = [(path2[0].0, path2[0].1, 1.0), (path2[1].0, path2[1].1, 2.0)];
        let rust_path = pyg_obs_avoid(&path2, std::slice::from_ref(&polygon), &[1.0]);
        let (cpp_path, cpp_count) = crate::ffi::run_cpp_pyg_obs_avoid(&path3, &[polygon], &[1.0]);

        close(rust_path.len() as f64, cpp_count);
        for (rust, cpp) in rust_path.iter().zip(cpp_path.iter()) {
            assert!(
                same_point(*rust, *cpp),
                "rust {rust:?} != cpp {cpp:?}; rust_path={rust_path:?}; cpp_path={cpp_path:?}"
            );
        }
    }
}
