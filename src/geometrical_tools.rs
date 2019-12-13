use std::f64::consts::PI;

pub struct PolarPoint{
    pub angle: f64,
    pub distance: f64
}

impl PolarPoint{
    pub fn new(distance: f64, angle: f64) -> PolarPoint{
        PolarPoint{
            distance,
            angle: wrap_angle(angle)
        }
    }
}

pub struct CartesianPoint{
    pub x: f64,
    pub y: f64
}

impl CartesianPoint{
    pub fn new(x: f64, y: f64) -> CartesianPoint{
        CartesianPoint{
            x,
            y
        }
    }
}

pub fn wrap_angle(angle: f64) -> f64{
    let mut a = angle;
    while a < -PI {
        a += 2.0 * PI;
    }

    while a >= PI {
        a -= 2.0 * PI;
    }

    return a;
}

#[cfg(test)]
mod test{
    use super::*;

    #[test]
    fn test_new_polar_point(){
        let pp = PolarPoint::new(235.5, 2.45);
        let pp2 = PolarPoint::new(365.35, 6.0 * PI);
        assert_eq!(pp.distance, 235.5);
        assert_eq!(pp.angle, 2.45);

        assert_eq!(pp2.distance, 365.35);
        assert_eq!(pp2.angle, 0.0);
    }

    #[test]
    fn test_wrap_angle(){
        assert_eq!(wrap_angle(1.23), 1.23);
        assert_eq!(wrap_angle(6.0 * PI), 0.0);
        assert!(wrap_angle(-45.0 * PI) - (-PI) < 10.0f64.powi(-8));
    }

}