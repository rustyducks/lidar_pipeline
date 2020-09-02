use crate::beacons::Beacons;
use crate::clusterer::Cluster;
use crate::RobotPoseGetter;
use crate::geometrical_tools::{CartesianPoint, PolarPoint};

pub struct DistanceToBeacons<'a, T: RobotPoseGetter>{
    pub beacons: &'a Beacons,
    pub robot_pose_getter: T
}

impl<'a, T: RobotPoseGetter> DistanceToBeacons<'a, T>{
    const PERMUTATIONS: [[usize; 3]; 6] = [[0, 1, 2], [0, 2, 1], [1, 0, 2], [1, 2, 0], [2, 0, 1], [2, 1, 0]];

    pub fn distance_to_beacons(&mut self, clusters: &Option<Vec<Cluster>>) -> Option<Vec<Option<PolarPoint>>>{
    let clusters = match clusters {
        Some(c) => c,
        None => return None
    };
    if clusters.len() > self.beacons.positions.len(){
        return None;
    }
    let pose = self.robot_pose_getter.get_pose().unwrap();
    let mut beacons_centers = Vec::with_capacity(clusters.len());
    for c in clusters{
        let x = pose.x + (c.closest_point.distance + self.beacons.radius) * (c.closest_point.angle - pose.theta).cos();
        let y = pose.y + (c.closest_point.distance + self.beacons.radius) * (c.closest_point.angle - pose.theta).sin();
        beacons_centers.push(CartesianPoint::new(x, y));
    }
    let mut min_d_sq = f64::MAX;
    let mut right_permutation: Option<[usize; 3]> = None;
    for perm in &DistanceToBeacons::<T>::PERMUTATIONS{
        let mut d_sq = 0.;
        for (i, c) in beacons_centers.iter().enumerate(){
            let be = &self.beacons.positions[perm[i]];
            let d = (be.x - c.x).powi(2) + (be.y - c.y).powi(2);
            d_sq += d;
        }
        if d_sq < min_d_sq{
            min_d_sq = d_sq;
            right_permutation = Some(perm.clone());
        }
    }
    let mut ret = vec![None, None, None];

    for (i, c) in clusters.iter().enumerate(){
        ret[right_permutation.unwrap()[i]] = Some(PolarPoint::new(c.closest_point.distance + self.beacons.radius, c.barycenter.angle));
    }

    Some(ret)

}
}