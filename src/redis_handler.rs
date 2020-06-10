use redis;

use crate::geometrical_tools::Pose;


pub trait RobotPoseGetter{
    fn get_pose(&mut self) -> Option<Pose>;
}

pub trait DistancesToEllipseSender{
    fn send_distances(&mut self, min_distance: f64, max_distance: f64, ellipse_name: &str);
}

pub trait DistancesToBeaconsSender{
    fn send_distances_to_beacons(&mut self, d1: Option<f64>, d2: Option<f64>, d3: Option<f64>);
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
    fn send_distances_to_beacons(&mut self, d1: Option<f64>, d2: Option<f64>, d3: Option<f64>) {
        // if d1.is_none() && d2.is_none() && d3.is_none(){
        //     return;
        // }
        let mut pipe = redis::pipe();
        let mut pipeatom = pipe.atomic();
        pipeatom = match d1 {
            Some(d) => pipeatom.set("distance_to_beacon/1/distance", d).ignore().set("distance_to_beacon/1/new", true).ignore(),
            None => pipeatom.set("distance_to_beacon/1/new", false).ignore()
        };
        pipeatom = match d2 {
            Some(d) => pipeatom.set("distance_to_beacon/2/distance", d).ignore().set("distance_to_beacon/2/new", true).ignore(),
            None => pipeatom.set("distance_to_beacon/2/new", false).ignore()
        };
        pipeatom = match d3 {
            Some(d) => pipeatom.set("distance_to_beacon/3/distance", d).ignore().set("distance_to_beacon/3/new", true).ignore(),
            None => pipeatom.set("distance_to_beacon/3/new", false).ignore()
        };
        let _ : () = pipeatom.query(&mut self.con).unwrap();
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