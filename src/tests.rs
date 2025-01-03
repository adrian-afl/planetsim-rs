use crate::au::au_to_meters;
use crate::body::{Body, BodyDynamics, OrbitingBodyDynamics, StaticBodyDynamics};
use crate::decimal_vector_3d::DecimalVector3d;
use crate::simulation::Simulation;
use crate::sin_cos::f64_to_dbig;
use dashu_float::DBig;
use std::str::FromStr;
use std::time::Instant;

#[cfg(test)]
mod tests {
    use super::*;

    fn prepare_sim() -> Simulation {
        let ten_to_24 = DBig::from_str("1000000000000000000000000").unwrap();

        let moon = Body {
            name: String::from_str("moon").unwrap(),
            dynamics: BodyDynamics::Orbiting(OrbitingBodyDynamics {
                orbit_radius: DBig::from(384400000),
                orbit_period: DBig::from(27 * 24 * 3600),
                orbit_plane_normal: DecimalVector3d::from_f64(0.0, 1.0, 0.1).normalized(),
            }),
            mass: f64_to_dbig(0.073) * &ten_to_24,
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
            mass: f64_to_dbig(5.97219) * &ten_to_24,
            satellites: vec![moon.clone()],
            rotation_axis: DecimalVector3d::from_f64(0.0, 1.0, 0.0).normalized(),
            rotation_period: DBig::from(24 * 3600),
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
            mass: f64_to_dbig(1988470.0) * &ten_to_24,
            satellites: vec![earth.clone()],
            rotation_axis: DecimalVector3d::from_f64(0.0, 1.0, 0.0).normalized(),
            rotation_period: DBig::from(7 * 24 * 3600),
        };

        let mut sim = Simulation::new();
        sim.add_hierarchy(sun, None);
        sim
    }

    fn dbig_to_f64(v: &DBig) -> f64 {
        f64::from_str(v.to_string().as_str()).unwrap()
    }

    #[test]
    fn gravity_flux_works() {
        let mut sim = prepare_sim();
        sim.update(f64_to_dbig(123123.0));
        let earth_now = sim.get_body("earth");
        let start = Instant::now();
        let flux = sim.calculate_gravity_flux(
            &(&earth_now.position + DecimalVector3d::from_f64(6371000.0, 0.0, 0.0)),
        );
        // println!("flux is {}", flux.length());
        assert!((dbig_to_f64(&flux.length()) - 9.82).abs() < 0.01);
    }

    #[test]
    fn surface_velocity_works() {
        let mut sim = prepare_sim();
        let surf_vel =
            sim.get_surface_velocity("earth", &DecimalVector3d::from_f64(6371000.0, 0.0, 0.0));
        // println!("surf_vel is {}", surf_vel.length());
        assert!((dbig_to_f64(&surf_vel.length()) - 463.31).abs() < 0.01);
    }
}
