extern crate ivyrust;

use ivyrust::{ivy_init, ivy_start, ivy_send_msg};
use lidar_rd::Sample;
use crate::clusterer::Cluster;
use std::thread;

pub struct IvyHandler{

}

impl IvyHandler{
    pub fn new(address: String) -> IvyHandler{
        ivy_init("lidar_pipeline".to_string(), "Lidar Pipeline online".to_string());
        ivy_start(None);
        thread::spawn(|| {
            ivyrust::ivy_main_loop();
        });
        IvyHandler{}
    }

    pub fn send_samples(&self, samples: &Option<Vec<Option<Sample>>>){
        let samples = match samples {
            Some(s) => s,
            None => return

        };
        let mut i = 0;
        for sample in samples{
            match sample {
                Some(s) => ivy_send_msg(format!("Highlight point {};{};{}", i, s.distance as f64 * s.angle.cos() + 1500., s.distance as f64 * s.angle.sin() + 1000.)),
                None => continue
            }
            i += 1;
        }
    }

    pub fn send_cluster(&self, clusters: &Option<Vec<Cluster>>){
        let clusters = match clusters {
            Some(c) => c,
            None => return
        };
        let mut i = 0;
        for cluster in clusters{
            ivy_send_msg(format!("Highlight point {};{};{}", i, cluster.barycenter.distance as f64 * cluster.barycenter.angle.cos() + 1500., 
                            cluster.barycenter.distance as f64 * cluster.barycenter.angle.sin() + 1000.));
            i += 1;
        }
    }
}