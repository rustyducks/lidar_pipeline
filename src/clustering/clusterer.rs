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

#[cfg(test)]
mod tests{
    use super::*;

    #[test]
    fn test_cluster_creation(){
        let s = Sample{angle: 2.54, distance: 135, quality: 34};
        let c = Cluster::new(&s);
        assert_eq!(c.barycenter.distance, 135.0);
        assert_eq!(c.barycenter.angle, 2.54);
        assert_eq!(c.points.len(), 1);
        assert_eq!(c.points[0].angle, 2.54);
        assert_eq!(c.points[0].distance, 135);
        assert_eq!(c.points[0].quality, 34);
        assert_eq!(c.max_intensity, 34);
        assert_eq!(c.closest_point.angle, 2.54);
        assert_eq!(c.closest_point.distance, 135.0);

        assert_eq!(c.size(), 1);
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

