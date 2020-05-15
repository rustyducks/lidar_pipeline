use crate::geometrical_tools::CartesianPoint;
extern crate yaml_rust;
use yaml_rust::YamlLoader;
use std::fs;

pub trait Obstacle: std::fmt::Display{
    fn contains(&self, x: f64, y: f64) -> bool;
}

pub struct BoundingBox{
    pub min_x: f64,
    pub min_y: f64,
    pub max_x: f64,
    pub max_y: f64
}

impl BoundingBox{
    pub fn new(x1: f64, y1: f64, x2: f64, y2: f64) -> BoundingBox{
        BoundingBox{
            min_x: x1.min(x2),
            min_y: y1.min(y2),
            max_x: x1.max(x2),
            max_y: y1.max(y2)
        }
    }
}

impl Obstacle for BoundingBox{
    fn contains(&self, x: f64, y: f64) -> bool{
        self.min_x <= x && x <= self.max_x && self.min_y <= y && y <= self.max_y
    }

}

impl std::fmt::Display for BoundingBox{
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result{
        write!(f, "BoundingBox ({}, {}, {}, {})", self.min_x, self.min_y, self.max_x, self.max_y)
    }
}

pub struct Circle{
    pub center: CartesianPoint,
    pub radius: f64
}

impl Circle{
    pub fn new(xc: f64, yc: f64, r: f64) -> Circle{
        Circle{
            center: CartesianPoint::new(xc, yc),
            radius: r
        }
    }
}

impl Obstacle for Circle{
    fn contains(&self, x: f64, y: f64) -> bool{
        (x - self.center.x).powi(2) + (y - self.center.y).powi(2) < self.radius.powi(2)
    }
}

impl std::fmt::Display for Circle{
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result{
        write!(f, "Circle {}, r: {}", self.center, self.radius)
    }
}


pub struct Mask{
    pub table: BoundingBox,
    pub static_obstacles: Vec<Box<dyn Obstacle>>
}

pub fn mask_from_file(filename: &str) -> Mask{
    let s = fs::read_to_string(filename).expect("Something went wrong reading the file");
    let docs = YamlLoader::load_from_str(&s).expect("The file is ill formatted");
    let doc = &docs[0];

    let table = BoundingBox::new(
        doc["mask"]["table"]["x_start"].as_f64().expect("x_start parameter of the table unreadable"), 
        doc["mask"]["table"]["y_start"].as_f64().expect("y_start parameter of the table unreadable"),
        doc["mask"]["table"]["x_stop"].as_f64().expect("x_stop parameter of the table unreadable"), 
        doc["mask"]["table"]["y_stop"].as_f64().expect("y_stop parameter of the table unreadable"));
    
    let mut static_obstacles: Vec<Box<dyn Obstacle>> = Vec::new();
    for obs in doc["mask"]["static_obstacles"].as_vec().unwrap(){
        if !obs["rect"].is_badvalue(){
            static_obstacles.push(Box::new(BoundingBox::new(
                obs["rect"]["x1"].as_f64().unwrap(), obs["rect"]["y1"].as_f64().unwrap(),
                obs["rect"]["x2"].as_f64().unwrap(), obs["rect"]["y2"].as_f64().unwrap()
            )))
        }else if !obs["circle"].is_badvalue(){
            static_obstacles.push(Box::new(Circle::new(
                obs["circle"]["center"]["x"].as_f64().unwrap(), obs["circle"]["center"]["y"].as_f64().unwrap(),
                obs["circle"]["radius"].as_f64().unwrap())))
        }else{
            panic!("Static obstacle is neither a `rect` nor `circle`");
        }
        
    }
    Mask{
        table,
        static_obstacles
    }


}
