use crate::geometrical_tools::{PolarPoint, wrap_angle};
use std::vec::Vec;
use crate::clustering::clusterer::Cluster;

pub trait ClusterFilter{
    fn filter(self, clusters: &Option<Vec<Cluster>>) -> Option<Vec<Cluster>>;
}

pub struct BeaconFilter{
    pub cluster_min_size: usize,
    pub max_distance_from_robot: f64,
    pub min_intensity: u16,
    pub max_sq_distance_from_beacon: f64
}
/*
impl ClusterFilter for ClusterFilter{
    fn filter(clusters: Option<Vec<Cluster>>) -> Option<Vec<Cluster>>{
        if (clusters.is_none()){
            return None;
        }
        let clusters = clusters.unwrap();
        let mut filtered = Vec::new();
        for cluster in clusters.iter(){
            if (cluster.points.len() < cluster_min_size){
                continue;
            }
            if (cluster.barycenter.distance > max_distance_from_robot){
                continue;
            }
            if (cluster.max_intensity < min_intensity){
                continue;
            }
            
        }
    }
}
*/