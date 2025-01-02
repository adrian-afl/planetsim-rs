use crate::au::au_to_meters;
use crate::body::{Body, BodyDynamics, OrbitingBodyDynamics, StaticBodyDynamics};
use crate::decimal_matrix_3d::DecimalMatrix3d;
use crate::decimal_vector_3d::DecimalVector3d;
use crate::simulation::Simulation;
use crate::sin_cos::f64_to_dbig;
use dashu_float::DBig;
use std::str::FromStr;
use std::time::Instant;

mod au;
mod body;
mod decimal_matrix_3d;
mod decimal_vector_3d;
mod simulation;
mod sin_cos;

fn main() {
    let mut axis = DecimalVector3d::from_f64(1.0, 0.2, 0.3);
    axis.normalize();
    let angle = f64_to_dbig(2.1415);

    let start = Instant::now();
    let matrix = DecimalMatrix3d::axis_angle(&axis, angle);
    let duration = start.elapsed();
    axis = matrix.apply(&axis);

    println!(
        "Time elapsed in expensive_function() is: {:?}, {:?}",
        duration, axis
    );

    let moon = Body {
        name: String::from_str("moon").unwrap(),
        dynamics: BodyDynamics::Orbiting(OrbitingBodyDynamics {
            orbit_radius: DBig::from(384400000),
            orbit_period: DBig::from(27 * 24 * 3600),
            orbit_plane_normal: DecimalVector3d::from_f64(0.0, 1.0, 0.1).normalized(),
        }),
        satellites: vec![],
        rotation_axis: DecimalVector3d::from_f64(0.3, 1.0, 0.2).normalized(),
        rotation_period: DBig::from(27 * 24 * 3600),
    };

    let earth = Body {
        name: String::from_str("earth").unwrap(),
        dynamics: BodyDynamics::Orbiting(OrbitingBodyDynamics {
            orbit_radius: au_to_meters(f64_to_dbig(1.0)),
            orbit_period: DBig::from(365 * 24 * 3600),
            orbit_plane_normal: DecimalVector3d::from_f64(0.1, 1.0, 0.0).normalized(),
        }),
        satellites: vec![moon],
        rotation_axis: DecimalVector3d::from_f64(0.3, 1.0, 0.2).normalized(),
        rotation_period: DBig::from(27 * 24 * 3600),
    };

    let sun = Body {
        name: String::from_str("sun").unwrap(),
        dynamics: BodyDynamics::Static(StaticBodyDynamics {
            position: DecimalVector3d::from_str(
                "64959787070023434667",
                "23454569021239234304",
                "29349283489",
            ),
        }),
        satellites: vec![earth],
        rotation_axis: DecimalVector3d::from_f64(0.0, 1.0, 0.0).normalized(),
        rotation_period: DBig::from(7 * 24 * 3600),
    };

    let mut sim = Simulation::new();
    sim.add_hierarchy(sun, None);
    sim.update(f64_to_dbig(123123.0));

    println!("{:?}", sim);

    let a = DecimalVector3d::from_f64(1.0, 1.0, 1.0);
    let b = DecimalVector3d::from_f64(2.0, 2.0, 2.0);
    let c = DecimalVector3d::from_f64(3.0, 3.0, 3.0);
    let s = &a + &b + &c;
    println!("a {a} b {b} c {c} s {s}");
}
