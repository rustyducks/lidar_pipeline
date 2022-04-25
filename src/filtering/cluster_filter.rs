use crate::geometrical_tools::{PolarPoint, wrap_angle, CartesianPoint};
use std::vec::Vec;
use crate::clustering::clusterer::Cluster;
use crate::beacons::Beacons;
use crate::geometrical_tools::Pose;

pub trait ClusterFilter{
    fn filter(&self, clusters: &Option<Vec<Cluster>>, pose: &Pose) -> Option<Vec<Cluster>>;
}

pub struct BeaconFilter<'a>{
    pub beacons: &'a Beacons,
    pub cluster_min_size: usize,
    pub max_distance_from_robot: f64,
    pub min_intensity: u16,
    pub max_sq_distance_from_beacon: f64
}

impl<'a> ClusterFilter for BeaconFilter<'a>{
    fn filter(&self, clusters: &Option<Vec<Cluster>>, pose: &Pose) -> Option<Vec<Cluster>>{
        if clusters.is_none(){
            return None;
        }
        let clusters = match clusters{
            Some(c) => c,
            None => return None
        };
        let mut filtered = Vec::new();
        for cluster in clusters{
            if cluster.points.len() < self.cluster_min_size{
                continue;
            }
            if cluster.barycenter.distance > self.max_distance_from_robot{
                continue;
            }
            if cluster.max_intensity < self.min_intensity{
                continue;
            }
            for beacon in &self.beacons.positions{
                let x = pose.x + (cluster.closest_point.distance + self.beacons.radius) * (cluster.closest_point.angle - pose.theta).cos();
                let y = pose.y + (cluster.closest_point.distance + self.beacons.radius) * (cluster.closest_point.angle - pose.theta).sin();
                //let x = pose.x + cluster.barycenter.distance * (cluster.barycenter.angle + pose.theta).cos();
                //let y = pose.y + cluster.barycenter.distance * (cluster.barycenter.angle + pose.theta).sin();
                let d = (x - beacon.x).powi(2) + (y - beacon.y).powi(2);
                if d <= self.max_sq_distance_from_beacon{
                    //println!("Dist: {}", d);
                    filtered.push(cluster.clone());  // Copy
                }
            }
        }
        Some(filtered)
    }
}
