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
}