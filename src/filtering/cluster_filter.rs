use std::vec::Vec;
use crate::clustering::clusterer::Cluster;
use crate::geometrical_tools::Pose;

pub trait ClusterFilter{
    fn filter(&self, clusters: &Option<Vec<Cluster>>, pose: &Pose) -> Option<Vec<Cluster>>;
}

pub struct QualityFilter{
    pub cluster_min_size: usize,
    pub max_distance_from_robot: f64,
    pub min_intensity: u16,
}

impl<'a> ClusterFilter for QualityFilter{
    fn filter(&self, clusters: &Option<Vec<Cluster>>, _: &Pose) -> Option<Vec<Cluster>>{
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
            filtered.push(cluster.clone());  // Copy
        }
        Some(filtered)
    }
}
