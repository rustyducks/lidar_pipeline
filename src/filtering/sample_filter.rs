use crate::clustering::clusterer::Cluster;
use crate::geometrical_tools::Pose;
use crate::geometrical_tools::{wrap_angle, PolarPoint};
use crate::obstacles::Mask;
use crate::obstacles::Obstacle;
use lidar_rd::Sample;
use std::vec::Vec;

pub trait SampleFilter {
    fn filter(&self, samples: &Vec<Option<Sample>>, pose: &Pose) -> Option<Vec<Option<Sample>>>;
}

pub struct MaskSampleFilter {
    mask: Mask,
    min_distance_to_robot: u16,
    min_quality: u16,
}

impl MaskSampleFilter {
    pub fn new(mask: Mask, min_distance_to_robot: u16, min_quality: u16) -> MaskSampleFilter {
        MaskSampleFilter {
            mask,
            min_distance_to_robot,
            min_quality,
        }
    }
}

impl SampleFilter for MaskSampleFilter {
    fn filter(&self, samples: &Vec<Option<Sample>>, pose: &Pose) -> Option<Vec<Option<Sample>>> {
        let mut filtered: Vec<Option<Sample>> = Vec::new();
        for s in samples {
            let s = match s {
                None => continue,
                Some(s) => s,
            };
            if s.distance < self.min_distance_to_robot {
                continue;
            }
            if s.quality < self.min_quality {
                continue;
            }
            let s_in_table = PolarPoint::new(s.distance as f64, s.angle)
                .to_cartesian()
                .from_pose(pose);
            let x_t = s_in_table.x;
            let y_t = s_in_table.y;
            //println!("{};{}", x_t, y_t);
            if !self.mask.table.contains(x_t, y_t) {
                continue;
            }
            let mut in_mask = false;
            for m in &self.mask.static_obstacles {
                if m.contains(x_t, y_t) {
                    in_mask = true;
                    break;
                }
            }
            if !in_mask {
                filtered.push(Some(*s));
            }
        }
        Some(filtered)
    }
}

#[cfg(test)]
mod tests {
    use super::super::super::geometrical_tools;
    use super::super::super::obstacles::BoundingBox;
    use super::super::super::obstacles::Circle;
    use super::*;

    #[test]
    fn test_mask_filter() {
        let s1 = Some(Sample {
            angle: 1.57,
            distance: 100,
            quality: 1,
        }); // 0.0; 200  -> Outside table
        let s2 = Some(Sample {
            angle: 3.141592,
            distance: 200,
            quality: 2,
        }); // 100; 0.0  -> Outside table
        let s3 = Some(Sample {
            angle: 4.7124,
            distance: 300,
            quality: 3,
        }); // 400; 200  -> Pass
        let s4 = Some(Sample {
            angle: 0.0,
            distance: 100,
            quality: 4,
        }); // 100; 300  -> In a bounding Box obs
        let s5 = Some(Sample {
            angle: 0.0,
            distance: 200,
            quality: 5,
        }); // 100; 400  -> In a circle obstacle
        let s6 = Some(Sample {
            angle: 4.7124,
            distance: 1000,
            quality: 6,
        }); // 1100; 200  -> Outside table
        let v = vec![s1, s2, s3, s4, s5, s6];
        let mask = Mask {
            table: BoundingBox::new(50., 50., 950., 950.),
            static_obstacles: vec![
                Box::new(BoundingBox::new(95., 280., 110., 305.)),
                Box::new(Circle::new(125., 425., 50.)),
            ],
        };
        let pose = Pose::new(100., 200., 1.57);
        let mut filter = MaskSampleFilter::new(mask);
        let filtered = filter.filter(&v, &pose).unwrap();
        assert_eq!(filtered.len(), 1);
        assert_eq!(filtered[0].unwrap().quality, 3);

        let mask = Mask {
            table: BoundingBox::new(50., 50., 950., 950.),
            static_obstacles: vec![Box::new(BoundingBox::new(95., 280., 110., 305.))],
        };
        let mut filter = MaskSampleFilter::new(mask);
        let filtered = filter.filter(&v, &pose).unwrap();
        assert_eq!(filtered.len(), 2);
        assert_eq!(filtered[0].unwrap().quality, 3);
        assert_eq!(filtered[1].unwrap().quality, 5);
    }
}
