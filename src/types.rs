#![allow(non_snake_case)]

// Data types for CoverMapObsPlan
// These types match the C++ structures defined in CoverMapObsPlan_types.h

use std::os::raw::{c_double, c_int};

/// Map boundary data (equivalent to struct0_T)
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct MapData {
    pub Lat: [c_double; 200],  // Latitude values
    pub Lon: [c_double; 200],  // Longitude values
    pub Cnt: c_double,         // Number of points
}

impl Default for MapData {
    fn default() -> Self {
        Self {
            Lat: [0.0; 200],
            Lon: [0.0; 200],
            Cnt: 0.0,
        }
    }
}

/// Polygon data (equivalent to struct1_T)
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct PolygonData {
    pub Lat: [c_double; 200],  // Latitude values
    pub Lon: [c_double; 200],  // Longitude values
    pub Pnt: [c_int; 200],    // Point indices
    pub State: [c_double; 200], // State values
    pub Cnt: c_double,         // Number of polygons
}

impl Default for PolygonData {
    fn default() -> Self {
        Self {
            Lat: [0.0; 200],
            Lon: [0.0; 200],
            Pnt: [0; 200],
            State: [0.0; 200],
            Cnt: 0.0,
        }
    }
}

/// Obstacle data (equivalent to struct2_T)
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct ObstacleData {
    pub Lat: [c_double; 50],   // Latitude values
    pub Lon: [c_double; 50],   // Longitude values
    pub R: [c_double; 50],     // Radius values
    pub Cnt: c_double,        // Number of obstacles
}

impl Default for ObstacleData {
    fn default() -> Self {
        Self {
            Lat: [0.0; 50],
            Lon: [0.0; 50],
            R: [0.0; 50],
            Cnt: 0.0,
        }
    }
}

/// Planning parameters (equivalent to struct3_T)
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct PlanningParams {
    pub width: c_double,              // Scan width
    pub yaw: c_double,                // Yaw angle
    pub dir: c_double,                // Direction
    pub speed: c_double,              // Speed
    pub safe_dist_obs: c_double,      // Safe distance from obstacles
    pub safe_dist_map: c_double,      // Safe distance from map boundary
    pub long_edge_yaw_flag: c_int,    // Long edge yaw flag
}

impl Default for PlanningParams {
    fn default() -> Self {
        Self {
            width: 10.0,
            yaw: 0.0,
            dir: 1.0,
            speed: 2.0,
            safe_dist_obs: 0.0,
            safe_dist_map: 0.0,
            long_edge_yaw_flag: 0,
        }
    }
}

/// Waypoint data (equivalent to struct4_T)
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct WaypointData {
    pub Lat: [c_double; 5000],   // Latitude values
    pub Lon: [c_double; 5000],   // Longitude values
    pub State: [c_double; 5000], // State values
}

impl Default for WaypointData {
    fn default() -> Self {
        Self {
            Lat: [0.0; 5000],
            Lon: [0.0; 5000],
            State: [0.0; 5000],
        }
    }
}

/// Planning result
#[derive(Debug, Clone)]
pub struct PlanningResult {
    pub waypoints: Vec<c_double>,
    pub waypoint_count: usize,
    pub path_length: c_double,
    pub coverage_area: c_double,
    pub estimated_time: c_double,
    pub yaw: c_double,
    pub error_code: c_double,
}

/// One planned waypoint.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Waypoint {
    pub lat: c_double,
    pub lon: c_double,
    pub state: c_double,
}

impl PlanningResult {
    pub fn waypoint_at(&self, index: usize) -> Option<Waypoint> {
        let base = index.checked_mul(3)?;
        Some(Waypoint {
            lat: *self.waypoints.get(base)?,
            lon: *self.waypoints.get(base + 1)?,
            state: *self.waypoints.get(base + 2)?,
        })
    }

    pub fn waypoint_points(&self) -> Vec<Waypoint> {
        (0..self.waypoint_count)
            .filter_map(|index| self.waypoint_at(index))
            .collect()
    }
}

/// Planning error
#[derive(Debug)]
pub enum PlanningError {
    InvalidInput,
    PlanningFailed,
    CollisionDetected,
}

impl std::fmt::Display for PlanningError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PlanningError::InvalidInput => write!(f, "Invalid input parameters"),
            PlanningError::PlanningFailed => write!(f, "Path planning failed"),
            PlanningError::CollisionDetected => write!(f, "Collision detected"),
        }
    }
}

impl std::error::Error for PlanningError {}
