use std::f64::consts::PI;

#[derive(Debug, Copy, Clone)]
pub struct PolarPoint {
    pub angle: f64,
    pub distance: f64,
}

impl PolarPoint {
    pub fn new(distance: f64, angle: f64) -> PolarPoint {
        PolarPoint {
            distance,
            angle: wrap_angle(angle),
        }
    }

    pub fn to_cartesian(&self) -> CartesianPoint {
        CartesianPoint::new(
            self.distance * self.angle.cos(),
            self.distance * self.angle.sin(),
        )
    }
}

impl std::fmt::Display for PolarPoint {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "({:2}, {:1}°)", self.distance, self.angle * 180. / PI)
    }
}

pub struct CartesianPoint {
    pub x: f64,
    pub y: f64,
}

impl CartesianPoint {
    pub fn new(x: f64, y: f64) -> CartesianPoint {
        CartesianPoint { x, y }
    }

    pub fn from_pose(&self, pose: &Pose) -> CartesianPoint {
        let c = pose.theta.cos();
        let s = pose.theta.sin();
        CartesianPoint::new(
            pose.x + self.x * c - self.y * s,
            pose.y + self.x * s + self.y * c,
        )
    }
}

impl std::fmt::Display for CartesianPoint {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "({}, {})", self.x, self.y)
    }
}

pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

impl Pose {
    pub fn new(x: f64, y: f64, theta: f64) -> Pose {
        Pose {
            x: x,
            y: y,
            theta: wrap_angle(theta),
        }
    }
}

impl std::fmt::Display for Pose {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "({}, {}, {}°)", self.x, self.y, self.theta * 180. / PI)
    }
}

pub fn wrap_angle(angle: f64) -> f64 {
    let mut a = angle;
    while a < -PI {
        a += 2.0 * PI;
    }

    while a >= PI {
        a -= 2.0 * PI;
    }

    return a;
}

pub fn angle_between(begin: f64, end: f64, angle: f64) -> bool {
    let mut end = end - begin;
    while end < 0.0 {
        end += 2. * PI;
    }
    let mut angle = angle - begin;
    while angle < 0.0 {
        angle += 2. * PI;
    }
    angle < end
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_new_polar_point() {
        let pp = PolarPoint::new(235.5, 2.45);
        let pp2 = PolarPoint::new(365.35, 6.0 * PI);
        assert_eq!(pp.distance, 235.5);
        assert_eq!(pp.angle, 2.45);

        assert_eq!(pp2.distance, 365.35);
        assert_eq!(pp2.angle, 0.0);
    }

    #[test]
    fn test_new_cartesian_point() {
        let cp = CartesianPoint::new(34.63, 23.76);
        assert_eq!(cp.x, 34.63);
        assert_eq!(cp.y, 23.76);
    }

    #[test]
    fn test_wrap_angle() {
        assert_eq!(wrap_angle(1.23), 1.23);
        assert_eq!(wrap_angle(6.0 * PI), 0.0);
        assert!(wrap_angle(-45.0 * PI) - (-PI) < 10.0f64.powi(-8));
    }
}
