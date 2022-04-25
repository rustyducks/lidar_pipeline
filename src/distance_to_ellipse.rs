use crate::geometrical_tools::{angle_between, wrap_angle};
use lidar_rd::Sample;

pub fn min_max_distance_to_ellipse(
    direction_angle: f64, cone_angle: f64, 
    semi_major: f64, semi_minor: f64, 
    samples: &Vec<Option<Sample>>) -> (f64, f64){

        let eccentricity = (1. - (semi_minor / semi_major).powi(2)).sqrt();
        let start_angle = wrap_angle(direction_angle - cone_angle/2.);
        let end_angle: f64 = wrap_angle(direction_angle + cone_angle/2.);
        let mut min_dist = f64::MAX;
        let mut max_dist = f64::MIN;
        for s in samples{
            let s = match s {
                Some(p) => p,
                None => continue
            };
            if angle_between(start_angle, end_angle, s.angle) {
                let r_ellipse = semi_minor / ((1. - (eccentricity * s.angle.cos()).powi(2)).sqrt());
                let d = s.distance as f64 - r_ellipse;
                min_dist = min_dist.min(d);
                max_dist = max_dist.max(d);
            }
        }
        (min_dist, max_dist)
}

#[cfg(test)]
mod tests{
    use super::*;

    #[test]
    fn test_distance_to_ellipse(){
        let epsilon = 0.000001;
        let s1 = Some(Sample{angle: 0.0, distance: 100, quality: 1});  // E1: -50
        let s2 = Some(Sample{angle: 0.0, distance: 250, quality: 2});  // E1: 100
        let s3 = Some(Sample{angle: 0.78, distance: 100, quality: 3});  // E1: 32.62638032873049
        let s4 = Some(Sample{angle: 1.0, distance: 1000, quality: 4}); // E1: 941.8961878667515
        let s5 = Some(Sample{angle: -0.5, distance: 30, quality: 5});  // E1: -59.02757959353967
        let s6 = Some(Sample{angle: 1.5707963267948966, distance: 40, quality: 6});  // E1: -10
        let s7 = Some(Sample{angle: 1.70, distance: 100, quality: 6});  // E1: 49.62695735057122
        let s8 = Some(Sample{angle: 1.31, distance: 150, quality: 4});  // E1: 98.45362920472493
        let s9 = Some(Sample{angle: -0.9, distance: 340, quality: 10});
        let (min, max ) = min_max_distance_to_ellipse(
            0.0, 1.57, 150., 50., vec![s1, s3, s4]);
        assert!((min - (-50.)).abs() < epsilon);
        assert!((max - 32.62638032873049).abs() < epsilon);
        let (min, max) = min_max_distance_to_ellipse(1.57, 0.5, 150., 50., vec![s1, s3, s4, s6, s7, s8]);
        assert!((min - (-10.)).abs() < epsilon);
        assert!((max - 49.62695735057122).abs() < epsilon);
        let (min, max) = min_max_distance_to_ellipse(0.0, 1.57, 150., 50., vec![s2, s5, s9]);
        assert!((min - (-59.02757959353967)).abs() < epsilon);
        assert!((max - 100.).abs() < epsilon);
    }
}