use super::clusterer::{Cluster, Clusterer};
use crate::geometrical_tools::wrap_angle;
use lidar_rd::Sample;

pub struct ProximityCluster {
    pub maximal_distance: f64,
    pub maximal_angle: f64, // If two following points, less than maximal_distance are more than this angle apart, still create a new group
}

impl Clusterer for ProximityCluster {
    fn cluster(&self, samples: &Vec<Option<Sample>>) -> Option<Vec<Cluster>> {
        let mut last_point: Option<Sample> = Option::None;
        let mut clusters = Vec::new();
        for sample in samples.iter() {
            if sample.is_none() {
                continue;
            }
            let s = sample.unwrap();
            if last_point.is_none() {
                // First cluster
                clusters.push(Cluster::new(&s));
            } else {
                let p = last_point.unwrap();
                if (p.distance as f64 - s.distance as f64).abs() >= self.maximal_distance
                    || wrap_angle(p.angle - s.angle).abs() >= self.maximal_angle
                {
                    clusters.push(Cluster::new(&s));
                } else {
                    let n = clusters.len();
                    clusters[n - 1].push(&s);
                }
            }
            last_point = Some(s); // Copy...
        }
        Some(clusters)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_proximity_clusterer() {
        let s1 = Some(Sample {
            angle: 2.34,
            distance: 345,
            quality: 13,
        });
        let s2 = Some(Sample {
            angle: 2.35,
            distance: 340,
            quality: 12,
        });
        let s3 = Some(Sample {
            angle: 2.36,
            distance: 338,
            quality: 13,
        });
        let s4 = Some(Sample {
            angle: 2.37,
            distance: 218,
            quality: 6,
        });
        let s5 = Some(Sample {
            angle: 2.38,
            distance: 225,
            quality: 9,
        });
        let s6 = Some(Sample {
            angle: 2.50,
            distance: 225,
            quality: 10,
        }); // Maximal angle test
        let v = vec![s1, s2, s3, s4, s5, s6];
        let pc = ProximityCluster {
            maximal_distance: 50.0,
            maximal_angle: 0.1,
        };
        let clusters = pc.cluster(&v).unwrap();
        assert_eq!(clusters.len(), 3);
        assert_eq!(
            (2.35 - clusters[0].barycenter.angle).abs() <= 0.000001,
            true
        );
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

        assert_eq!(clusters[2].barycenter.angle, 2.50);
        assert_eq!(clusters[2].barycenter.distance, 225.);
        assert_eq!(clusters[2].max_intensity, 10);
        assert_eq!(clusters[2].points.len(), 1);
        assert_eq!(clusters[2].closest_point.angle, 2.50);
        assert_eq!(clusters[2].closest_point.distance, 225.);
    }
}
