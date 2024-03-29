mod beacons;
mod clustering;
mod distance_to_ellipse;
mod filtering;
mod geometrical_tools;
mod obstacles;
//mod distance_to_beacons;
mod communication;

pub use crate::clustering::clusterer;
pub use crate::clustering::clusterer::Clusterer;
pub use crate::clustering::proximity_clusterer;
pub use crate::obstacles::mask_from_file;
//pub use crate::filtering::sample_filter;
//pub use crate::filtering::sample_filter::SampleFilter;
//pub use crate::filtering::cluster_filter::ClusterFilter;
pub use crate::communication::link::LinkMessage;
use crate::communication::protoduck_generated::messages;
pub use crate::communication::udp_link;
pub use crate::filtering::sample_filter;
use crate::filtering::sample_filter::SampleFilter;
use crate::geometrical_tools::Pose;
use filtering::cluster_filter::{self, ClusterFilter};
use geometrical_tools::wrap_angle;
pub use lidar_rd::{Lidar, Sample, LD06};
use protobuf::Message;
use std::env;
use std::net::UdpSocket;
use std::{sync::mpsc, thread, time::Duration};

fn main() {
    let args: Vec<String> = env::args().collect();
    let obstacle_filename = &args[1];
    let (udp_incoming_producer_channel, udp_incoming_consumer_channel) =
        mpsc::channel::<LinkMessage>();
    let (udp_outgoing_producer_channel, udp_outgoing_consumer_channel) =
        mpsc::channel::<LinkMessage>();

    let socket = UdpSocket::bind("0.0.0.0:4321").expect("Could not bind UDP address");
    socket
        .set_read_timeout(Some(Duration::from_millis(1)))
        .expect("UDP set timeout failed");

    let _th_udp = thread::spawn(move || {
        udp_link::run(
            socket,
            udp_outgoing_consumer_channel,
            udp_incoming_producer_channel,
        )
    });
    let mut l = LD06::new("/dev/lidar");
    println!("{}", obstacle_filename);
    let mask = mask_from_file(obstacle_filename);
    let mask_filter = sample_filter::MaskSampleFilter::new(mask, 150, 150);
    let clusterer = clustering::proximity_clusterer::ProximityCluster {
        maximal_distance: 25.0,
        maximal_angle: 0.1,
    };
    let quality_cluster_filter = cluster_filter::QualityFilter {
        cluster_min_size: 5,
        max_distance_from_robot: 2500.,
        min_intensity: 200,
    };
    let mut pose: Option<Pose> = None;
    let _ = l.start();
    loop {
        match udp_incoming_consumer_channel.try_recv() {
            Ok(msg) => {
                let message = msg.to_proto().unwrap();
                if message.get_msg_type() == messages::Message_MsgType::STATUS {
                    if message.has_pos() {
                        let msg_pose = message.get_pos();
                        pose = Some(Pose::new(
                            f64::from(msg_pose.x),
                            f64::from(msg_pose.y),
                            f64::from(msg_pose.theta),
                        ));
                    }
                }
            }
            _ => {}
        }
        if pose.is_some() {
            if let Some(scan) = l.get_scan() {
                let scan_rad = scan
                    .iter()
                    .map(|s| {
                        s.and_then(|s| {
                            Some(Sample {
                                angle: wrap_angle(-s.angle / 180. * 3.14159265),
                                distance: s.distance,
                                quality: s.quality,
                            })
                        })
                    })
                    .collect();
                if let Some(filtered) = mask_filter.filter(&scan_rad, pose.as_ref().unwrap()) {
                    let clusters = clusterer.cluster(&filtered);
                    let mut msg = messages::Message::new();
                    let mut player_poses = messages::PlayerPoses::new();
                    if let Some(filteredcluster) =
                        quality_cluster_filter.filter(&clusters, pose.as_ref().unwrap())
                    {
                        let mut i = 0;
                        for cluster in filteredcluster {
                            let mut player_pos = messages::PlayerPos::new();
                            let mut player_pos_pos = messages::Pos::new();
                            let pos = pose.as_ref().unwrap();
                            let pos_in_table = cluster.barycenter.to_cartesian().from_pose(pos);
                            player_pos_pos.x = pos_in_table.x as f32;
                            player_pos_pos.y = pos_in_table.y as f32;
                            player_pos.set_aruco_id(i);
                            player_pos.set_pos(player_pos_pos);
                            player_poses.player_poses.push(player_pos);
                            i = i + 1;
                        }
                    }
                    msg.set_player_poses(player_poses);
                    msg.set_msg_type(messages::Message_MsgType::STATUS);
                    let _ = udp_outgoing_producer_channel
                        .send(LinkMessage::from_bytes(&msg.write_to_bytes().unwrap()));
                };
            }
        }
    }
    // let pc = proximity_clusterer::ProximityCluster{maximal_distance: 50.0};
    // let clusters = pc.cluster(&ov);
    // for cl in &clusters.unwrap(){
    //     println!("{}", cl);
    // }
    // let clusters = pc.cluster(&ov);

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
