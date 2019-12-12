mod clustering;
mod geometrical_tools;

pub use crate::clustering::clusterer;

fn main() {
    let _c = clusterer::Cluster::new();
}
