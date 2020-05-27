use crate::geometrical_tools::{PolarPoint, wrap_angle};
use std::vec::Vec;
use crate::clustering::clusterer::Cluster;
use crate::redis_handler::RobotPoseGetter;
use lidar_rd::Sample;
use crate::obstacles::Mask;
use crate::obstacles::Obstacle;

pub trait SampleFilter{
    fn filter(&mut self, samples: &Option<Vec<Option<Sample>>>) -> Option<Vec<Option<Sample>>>;
}

pub struct MaskSampleFilter<T: RobotPoseGetter>{
    robot_pose_getter: T,
    mask: Mask
}

impl<T: RobotPoseGetter> MaskSampleFilter<T>{
    pub fn new(robot_pose_getter: T, mask: Mask) -> MaskSampleFilter<T>
    {
        MaskSampleFilter{
            robot_pose_getter,
            mask
        }
    }
}

impl<T: RobotPoseGetter> SampleFilter for MaskSampleFilter<T>{
    
    fn filter(&mut self, samples: &Option<Vec<Option<Sample>>>) -> Option<Vec<Option<Sample>>> {
        let samples = match samples{
            None => return None,
            Some(s) => s
        };
        let mut filtered: Vec<Option<Sample>> = Vec::new();
        let pose = self.robot_pose_getter.get_pose().unwrap();
        for s in samples{
            let s = match s{
                None => continue,
                Some(s) => s
            };
            let x_t = pose.x + s.distance as f64 * (s.angle + pose.theta).cos();
            let y_t = pose.y + s.distance as f64 * (s.angle + pose.theta).sin();
            //println!("{};{}", x_t, y_t);
            if !self.mask.table.contains(x_t, y_t){
                continue;
            }
            let mut in_mask = false;
            for m in &self.mask.static_obstacles{
                if m.contains(x_t, y_t){
                    in_mask = true;
                    break;
                }
            }
            if !in_mask{
                filtered.push(Some(*s));
            }
        }
        Some(filtered)
    }
}


#[cfg(test)]
mod tests{
    use super::*;
    use super::super::super::redis_handler;
    use super::super::super::geometrical_tools;
    use super::super::super::obstacles::BoundingBox;
    use super::super::super::obstacles::Circle;

    #[test]
    fn test_mask_filter(){
        let s1 = Some(Sample{angle: 1.57, distance: 100, quality: 1});   // 0.0; 200  -> Outside table
        let s2 = Some(Sample{angle: 3.141592, distance: 200, quality: 2});  // 100; 0.0  -> Outside table
        let s3 = Some(Sample{angle: 4.7124, distance: 300, quality: 3});  // 400; 200  -> Pass
        let s4 = Some(Sample{angle: 0.0, distance: 100, quality: 4});  // 100; 300  -> In a bounding Box obs
        let s5 = Some(Sample{angle: 0.0, distance: 200, quality: 5}); // 100; 400  -> In a circle obstacle
        let s6 = Some(Sample{angle: 4.7124, distance: 1000, quality: 6}); // 1100; 200  -> Outside table
        let v = vec!(s1, s2, s3, s4, s5, s6);
        let ov = Some(v);
        let red = redis_handler::FakeRedisHandler::new(geometrical_tools::Pose{x: 100.0, y: 200.0, theta: 1.57}).unwrap();
        let mask = Mask{
            table: BoundingBox::new(50., 50., 950., 950.),
            static_obstacles: vec![Box::new(BoundingBox::new(95., 280., 110., 305.)), Box::new(Circle::new(125., 425., 50.))]
        };
        let mut filter = MaskSampleFilter::new(red, mask);
        let filtered = filter.filter(&ov).unwrap();
        assert_eq!(filtered.len(), 1);
        assert_eq!(filtered[0].unwrap().quality, 3);

        let red = redis_handler::FakeRedisHandler::new(geometrical_tools::Pose{x: 100.0, y: 200.0, theta: 1.57}).unwrap();
        let mask = Mask{
            table: BoundingBox::new(50., 50., 950., 950.),
            static_obstacles: vec![Box::new(BoundingBox::new(95., 280., 110., 305.))]
        };
        let mut filter = MaskSampleFilter::new(red, mask);
        let filtered = filter.filter(&ov).unwrap();
        assert_eq!(filtered.len(), 2);
        assert_eq!(filtered[0].unwrap().quality, 3);
        assert_eq!(filtered[1].unwrap().quality, 5);


    }
}