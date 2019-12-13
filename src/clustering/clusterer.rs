use crate::geometrical_tools::{PolarPoint, wrap_angle};
use std::vec::Vec;
use lidar_rd::Sample;

pub struct Cluster<'a>{
    pub barycenter: PolarPoint,
    pub points: Vec<&'a Sample>,
    pub max_intensity: u16,
    pub closest_point: PolarPoint
}

impl <'a>Cluster<'a>{
    pub fn new(sample: &Sample) -> Cluster{
        Cluster{
            barycenter: PolarPoint::new(sample.distance as f64, sample.angle),
            points: vec![&sample],
            max_intensity: sample.quality,
            closest_point: PolarPoint::new(sample.distance as f64, sample.angle)
        }
    }

    pub fn push(&mut self, sample: &'a Sample){
        if (sample.distance as f64) < self.closest_point.distance{
            self.closest_point = PolarPoint{distance: sample.distance as f64, angle: sample.angle};
        }
        if sample.quality > self.max_intensity {
            self.max_intensity = sample.quality;
        }

        self.barycenter = PolarPoint::new((self.barycenter.distance * self.size() as f64 + sample.distance as f64) / ((self.size() + 1) as f64),
                                            wrap_angle((self.barycenter.angle * self.size() as f64 + sample.angle) / ((self.size() + 1)) as f64));

        self.points.push(sample);
    }

    pub fn size(&self) -> usize{
        self.points.len()
    }

}

