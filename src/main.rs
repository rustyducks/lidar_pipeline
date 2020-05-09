mod clustering;
mod geometrical_tools;

pub use crate::clustering::clusterer;
pub use crate::clustering::clusterer::Clusterer;
pub use crate::clustering::proximity_clusterer;
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
    let clusters = pc.cluster(ov);
    for cl in &clusters.unwrap(){
        println!("{}", cl);
    }

}
