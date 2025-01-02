use crate::decimal_vector_3d::DecimalVector3d;
use dashu_float::DBig;

#[derive(Debug, Clone)]
pub struct StaticBodyDynamics {
    pub position: DecimalVector3d,
}

#[derive(Debug, Clone)]
pub struct OrbitingBodyDynamics {
    pub orbit_radius: DBig,
    pub orbit_plane_normal: DecimalVector3d,
    pub orbit_period: DBig,
}

#[derive(Debug, Clone)]
pub enum BodyDynamics {
    Static(StaticBodyDynamics),
    Orbiting(OrbitingBodyDynamics),
}

#[derive(Debug, Clone)]
pub struct Body {
    pub name: String,
    pub rotation_axis: DecimalVector3d,
    pub rotation_period: DBig, // in seconds
    pub mass: DBig,            // in kg
    pub dynamics: BodyDynamics,
    pub satellites: Vec<Body>,
}
