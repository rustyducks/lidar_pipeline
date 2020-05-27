use redis;

use crate::geometrical_tools::Pose;


pub trait RobotPoseGetter{
    fn get_pose(&mut self) -> Option<Pose>;
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