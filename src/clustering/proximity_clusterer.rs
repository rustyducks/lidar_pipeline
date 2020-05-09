use super::clusterer::{Clusterer, Cluster};
use std::mem;
use lidar_rd::Sample;

pub struct ProximityCluster{
    pub maximal_distance: f64
}

impl Clusterer for ProximityCluster{
    fn cluster(self, samples: Option<Vec<Option<Sample>>>) -> Option<Vec<Cluster>>{
        let mut last_point: Option<Sample> = Option::None;
        if samples.is_none() {
            return None;
        }
        let samples_ = samples.unwrap();
        let mut clusters = Vec::new();
        for sample in samples_.iter() {
            if sample.is_none() {
                continue;
            }
            let s = sample.unwrap();
            if last_point.is_none() || (last_point.unwrap().distance as f64 - s.distance as f64).abs() >= self.maximal_distance {
                clusters.push(Cluster::new(&s));
            }else{
                let n = clusters.len();
                clusters[n-1].push(&s);
            }
            last_point = Some(s); // Copy...
        }
        Some(clusters)
    }
}



#[cfg(test)]
mod tests{
    use super::*;

    #[test]
    fn test_proximity_clusterer(){
        let s1 = Some(Sample{angle: 2.34, distance: 345, quality: 13});
        let s2 = Some(Sample{angle: 2.35, distance: 340, quality: 12});
        let s3 = Some(Sample{angle: 2.36, distance: 338, quality: 13});
        let s4 = Some(Sample{angle: 2.37, distance: 218, quality: 6});
        let s5 = Some(Sample{angle: 2.38, distance: 225, quality: 9});
        let v = vec!(s1, s2, s3, s4, s5);
        let ov = Some(v);
        let pc = ProximityCluster{maximal_distance: 50.0};
        let clusters = pc.cluster(ov).unwrap();
        assert_eq!(clusters.len(), 2);
        assert_eq!((2.35 - clusters[0].barycenter.angle).abs() <= 0.000001, true);
        assert_eq!(clusters[0].barycenter.distance, 341.);
        assert_eq!(clusters[0].max_intensity, 13);
        assert_eq!(clusters[0].points.len(), 3);
        assert_eq!(clusters[0].closest_point.angle, 2.36);
        assert_eq!(clusters[0].closest_point.distance, 338.);

        assert_eq!(clusters[1].barycenter.angle, 2.375);
        assert_eq!(clusters[1].barycenter.distance, 221.5);
        assert_eq!(clusters[1].max_intensity, 9);
        assert_eq!(clusters[1].points.len(), 2);
        assert_eq!(clusters[1].closest_point.angle, 2.37);
        assert_eq!(clusters[1].closest_point.distance, 218.);
    }

    #[test]
    fn test_cluster_add(){
        let s1 = Sample{angle: 1.0, distance: 200, quality:54};
        let s2 = Sample{angle: 2.0, distance: 100, quality: 65};
        let s3 = Sample{angle: 3.0, distance: 300, quality: 243};
        let s4 = Sample{angle: 4.0, distance: 400, quality: 145};
        let mut c = Cluster::new(&s1);
        c.push(&s2);
        c.push(&s3);
        c.push(&s4);
        assert_eq!(c.barycenter.distance, 250.0);
        assert_eq!(c.barycenter.angle, 2.5);
        assert_eq!(c.points.len(), 4);
        assert_eq!(c.points[0].angle, 1.0);
        assert_eq!(c.points[0].distance, 200);
        assert_eq!(c.points[0].quality, 54);
        assert_eq!(c.max_intensity, 243);
        assert_eq!(c.closest_point.angle, 2.0);
        assert_eq!(c.closest_point.distance, 100.0);

        assert_eq!(c.size(), 4);
    }
}


