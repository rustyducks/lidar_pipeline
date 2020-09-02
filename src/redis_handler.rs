use redis;
use redis::Commands;
use std::fmt::Write;


use crate::geometrical_tools::{Pose, PolarPoint};


pub trait RobotPoseGetter{
    fn get_pose(&mut self) -> Option<Pose>;
}

pub trait DistancesToEllipseSender{
    fn send_distances(&mut self, min_distance: f64, max_distance: f64, ellipse_name: &str);
}

pub trait DistancesToBeaconsSender{
    fn send_distances_to_beacons(&mut self, d1: Option<PolarPoint>, d2: Option<PolarPoint>, d3: Option<PolarPoint>);
}

pub struct RedisHandler{
    client: redis::Client,
    con: redis::Connection
}

impl RedisHandler{
    pub fn new(address: &str) -> Option<RedisHandler>{
        let client = redis::Client::open(address).unwrap();
        let con = client.get_connection().unwrap();
        Some(RedisHandler{
            client: client,
            con: con
        })
    }
}

impl RobotPoseGetter for RedisHandler{
    
    fn get_pose(&mut self) -> std::option::Option<Pose> {
        let answer = redis::pipe()
            .atomic()
            .cmd("GET").arg("robot_pose/x")
            .cmd("GET").arg("robot_pose/y")
            .cmd("GET").arg("robot_pose/theta").query(&mut self.con);
        let (x, y, theta): (f64, f64, f64) = match answer{
            Ok(p) => p,
            Err(_) => return None
        };
        Some(Pose{
            x, y, theta
        })
    }
}

impl DistancesToEllipseSender for RedisHandler{
    fn send_distances(&mut self, min_distance: f64, max_distance: f64, ellipse_name: &str) {
        let _: () = redis::pipe().atomic()
        .set(format!("distance_to_ellipse/{}/min", ellipse_name), min_distance).ignore()
        .set(format!("distance_to_ellipse/{}/max", ellipse_name), max_distance).ignore()
        .query(&mut self.con).unwrap();
    }
}

impl DistancesToBeaconsSender for RedisHandler{
    fn send_distances_to_beacons(&mut self, d1: Option<PolarPoint>, d2: Option<PolarPoint>, d3: Option<PolarPoint>) {
        // if d1.is_none() && d2.is_none() && d3.is_none(){
        //     return;
        // }
        let d1_str = match d1{
            Some(d) => format!("{},{}", d.distance, d.angle),
            None => "None".to_string()
        };
        let d2_str = match d2{
            Some(d) => format!("{},{}", d.distance, d.angle),
            None => "None".to_string()
        };
        let d3_str = match d3{
            Some(d) => format!("{},{}", d.distance, d.angle),
            None => "None".to_string()
        };
        let serialized = format!("{};{};{}", d1_str, d2_str, d3_str);
        let pipe: () = self.con.publish("beacons/measurements", serialized).unwrap();
    }
}

pub struct FakeRedisHandler{
    fake_pose: Pose
}

impl FakeRedisHandler{
    pub fn new(fake_pose: Pose) -> Option<FakeRedisHandler>{
        Some(FakeRedisHandler{
            fake_pose
        })
    }
}

impl RobotPoseGetter for FakeRedisHandler{
    fn get_pose(&mut self) -> Option<Pose> {
        Some(Pose{
            x: self.fake_pose.x,
            y: self.fake_pose.y,
            theta: self.fake_pose.theta
        })
    }
}