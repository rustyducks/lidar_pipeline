mod clustering;
mod geometrical_tools;
mod obstacles;
mod redis_handler;
mod filtering;
mod distance_to_ellipse;

#[cfg(feature = "ivy")]
mod ivy_handler;


pub use crate::clustering::clusterer;
pub use crate::clustering::clusterer::Clusterer;
pub use crate::clustering::proximity_clusterer;
pub use crate::obstacles::mask_from_file;
pub use crate::redis_handler::{RobotPoseGetter, DistancesToEllipseSender};
pub use crate::filtering::sample_filter;
pub use crate::filtering::sample_filter::SampleFilter;
pub use crate::filtering::cluster_filter::ClusterFilter;
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
    let pc = proximity_clusterer::ProximityCluster{maximal_angle: 0.2, maximal_distance: 70.};
    let cf = filtering::cluster_filter::BeaconFilter{max_distance_from_robot: 3500., cluster_min_size: 1, min_intensity: 1000, max_sq_distance_from_beacon: 0.};
    //let ivy = ivy_handler::IvyHandler::new("127.0.0.1:2010".to_string());
    
    let mut l = XV11::new("/dev/ttyUSB0");
    l.start();
    let mut i = 0;
    for scan in l.iter() {
        let ov = Some(scan);
        let clusters = pc.cluster(&ov);
        let filtered = cf.filter(&clusters);
        i += 1;
        if i > 10{
        //    ivy.send_cluster(&filtered);
            i = 0
        }  
        
        println!("\n\n")
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
