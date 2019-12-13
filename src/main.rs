mod clustering;
mod geometrical_tools;

pub use crate::clustering::clusterer;
pub use lidar_rd::Sample;

fn main() {
    let s = Sample{angle: 2.34, distance: 345, quality: 13};
    let _c = clusterer::Cluster::new(&s);

}
