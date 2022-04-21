mod clustering;
mod geometrical_tools;
mod obstacles;
//mod filtering;
mod distance_to_ellipse;
mod beacons;
//mod distance_to_beacons;
mod communication;


//pub use crate::clustering::clusterer;
//pub use crate::clustering::clusterer::Clusterer;
//pub use crate::clustering::proximity_clusterer;
pub use crate::obstacles::mask_from_file;
//pub use crate::filtering::sample_filter;
//pub use crate::filtering::sample_filter::SampleFilter;
//pub use crate::filtering::cluster_filter::ClusterFilter;
pub use crate::communication::udp_link;
pub use crate::communication::link::LinkMessage;
pub use lidar_rd::{Sample, Lidar, LD06};
use std::{thread, time::Duration, sync::mpsc};
use std::net::UdpSocket;
use crate::geometrical_tools::Pose;

fn main() {

    let (udp_incoming_producer_channel, udp_incoming_consumer_channel) = mpsc::channel::<LinkMessage>();
    let (udp_outgoing_producer_channl, udp_outgoing_consumer_channel) = mpsc::channel::<LinkMessage>();

    let socket = UdpSocket::bind("0.0.0.0:4321").expect("Could not bind UDP address");
    socket.set_read_timeout(Some(Duration::from_millis(1))).expect("UDP set timeout failed");

    let th_udp = thread::spawn(move || udp_link::run(socket, udp_outgoing_consumer_channel, udp_incoming_producer_channel));
    let mut robot_pose: Option<Pose>;
    loop {
        match udp_incoming_consumer_channel.try_recv() {
            Ok(msg) => {
                let message = msg.to_proto().unwrap();
                println!("{:?}", message);
            },
            _ => {}
        }
    }
    
    /*let mut l = LD06::new("/dev/ttyUSB0");
    l.start();
    let mut i = 0;
    for scan in l.iter() {
        let ov = Some(scan);
    }*/
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
