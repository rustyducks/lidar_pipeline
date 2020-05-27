mod clustering;
mod geometrical_tools;
mod obstacles;
mod redis_handler;

pub use crate::clustering::clusterer;
pub use crate::clustering::clusterer::Clusterer;
pub use crate::clustering::proximity_clusterer;
pub use crate::obstacles::mask_from_file;
pub use crate::redis_handler::RobotPoseGetter;
pub use lidar_rd::Sample;

fn main() {
    let s1 = Some(Sample{angle: 2.34, distance: 345, quality: 13});
    let s2 = Some(Sample{angle: 2.35, distance: 340, quality: 12});
    let s3 = Some(Sample{angle: 2.36, distance: 338, quality: 13});
    let s4 = Some(Sample{angle: 2.37, distance: 218, quality: 6});
    let s5 = Some(Sample{angle: 2.38, distance: 225, quality: 9});
    let v = vec!(s1, s2, s3, s4, s5);
    let ov = Some(v);
    let pc = proximity_clusterer::ProximityCluster{maximal_distance: 50.0};
    let clusters = pc.cluster(&ov);
    for cl in &clusters.unwrap(){
        println!("{}", cl);
    }
    let clusters = pc.cluster(&ov);

    let mask = mask_from_file("obstacles_lidar_mask.yaml");

    println!("Mask:");
    println!("\tTable: {}", mask.table);
    println!("\tObstacles:");
    for o in &mask.static_obstacles{
        println!("\t\t{}", o);
    }

    //let mut red = redis_handler::RedisHandler::new("redis://127.0.0.1:6379").unwrap();
    let mut red = redis_handler::FakeRedisHandler::new(geometrical_tools::Pose{x: 200.0, y: 200.0, theta: 1.57}).unwrap();
    let p = red.get_pose();
    match p{
        Some(p) => println!("{}", p),
        None => println!("Aïe Caramba")
    }

}
