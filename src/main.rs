mod clustering;
mod geometrical_tools;
mod obstacles;
mod redis_handler;
mod filtering;
mod distance_to_ellipse;

pub use crate::clustering::clusterer;
pub use crate::clustering::clusterer::Clusterer;
pub use crate::clustering::proximity_clusterer;
pub use crate::obstacles::mask_from_file;
pub use crate::redis_handler::{RobotPoseGetter, DistancesToEllipseSender};
pub use crate::filtering::sample_filter;
pub use crate::filtering::sample_filter::SampleFilter;
pub use lidar_rd::{Sample, Lidar, XV11};

fn main() {
//    let s1 = Some(Sample{angle: 2.34, distance: 345, quality: 13});
//    let s2 = Some(Sample{angle: 2.35, distance: 340, quality: 12});
//    let s3 = Some(Sample{angle: 2.36, distance: 338, quality: 13});
    // let s4 = Some(Sample{angle: 2.37, distance: 218, quality: 6});
    // let s5 = Some(Sample{angle: 2.38, distance: 225, quality: 9});
    // let s6: Option<Sample> = Some(Sample{angle: 1.57, distance: 100, quality: 255});
    // let v = vec!(s1, s2, s3, s4, s5, s6);
    // let ov = Some(v);
    let mut l = XV11::new("/dev/ttyUSB0");
    l.start();
    for scan in l.iter() {
        let ov = Some(scan);
        let (min_far, max_far) = distance_to_ellipse::min_max_distance_to_ellipse(0.0, 1.3, 550., 200., &ov);        
        if min_far < 0.0{
            let (min_close, max_close) = distance_to_ellipse::min_max_distance_to_ellipse(0.0, 1.3, 250., 200., &ov);
            if min_close < 0.0{
                println!("STOP !");
            }else{
                println!("{:0}%", min_close / (550. - 250.) * 100.)
            }
        }else{
            println!("FULL SPEED !");
        }
    }
    // let pc = proximity_clusterer::ProximityCluster{maximal_distance: 50.0};
    // let clusters = pc.cluster(&ov);
    // for cl in &clusters.unwrap(){
    //     println!("{}", cl);
    // }
    // let clusters = pc.cluster(&ov);

    // let mask = mask_from_file("obstacles_lidar_mask.yaml");

    // println!("Mask:");
    // println!("\tTable: {}", mask.table);
    // println!("\tObstacles:");
    // for o in &mask.static_obstacles{
    //     println!("\t\t{}", o);
    // }

    // //let mut red = redis_handler::RedisHandler::new("redis://127.0.0.1:6379").unwrap();
    // let mut red = redis_handler::FakeRedisHandler::new(geometrical_tools::Pose{x: 200.0, y: 200.0, theta: 1.57}).unwrap();
    // let p = red.get_pose();
    // match p{
    //     Some(p) => println!("{}", p),
    //     None => println!("Aïe Caramba")
    // }

    // let mut rb = sample_filter::MaskSampleFilter::new(red, mask);

    // let filtered = rb.filter(&ov);

    // let (min, max) = distance_to_ellipse::min_max_distance_to_ellipse(1.57, 0.5, 150., 50., &filtered);
    // println!("{};{}", min, max);

    // let mut redd = redis_handler::RedisHandler::new("redis://127.0.0.1/").unwrap();
    // redd.send_distances(min, max, "far");

}
