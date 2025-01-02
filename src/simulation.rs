use crate::body::{Body, BodyDynamics};
use crate::decimal_matrix_3d::DecimalMatrix3d;
use crate::decimal_vector_3d::DecimalVector3d;
use crate::sin_cos::PIMUL2;
use dashu_float::DBig;
use std::ops::Deref;

#[derive(Debug, Clone)]
pub struct SimulatedBody {
    id: i32,
    body: Body,
    position: DecimalVector3d,
    velocity: DecimalVector3d,
    orientation: DecimalMatrix3d,
    parent: Option<i32>, // -1 means no
    satellites: Vec<i32>,
}

#[derive(Debug)]
pub struct Simulation {
    pub bodies: Vec<SimulatedBody>,
    id_counter: i32,
}

impl Simulation {
    pub fn new() -> Self {
        Simulation {
            bodies: vec![],
            id_counter: 0,
        }
    }

    pub fn add_hierarchy(&mut self, body: Body, parent: Option<i32>) -> i32 {
        let new_id = self.id_counter;
        self.id_counter += 1;
        let mut simulated_body = SimulatedBody {
            id: new_id,
            parent,
            satellites: vec![],
            body: body.clone(),
            position: DecimalVector3d::zero(),
            velocity: DecimalVector3d::zero(),
            orientation: DecimalMatrix3d::identity(),
        };
        for i in 0..body.satellites.len() {
            simulated_body
                .satellites
                .push(self.add_hierarchy(body.satellites[i].clone(), Some(new_id)))
        }
        self.bodies.push(simulated_body);
        new_id
    }

    fn get_body_by_id(&self, id: i32) -> Option<&SimulatedBody> {
        for i in 0..self.bodies.len() {
            if self.bodies[i].id == id {
                return Some(&self.bodies[i]);
            }
        }

        None
    }

    fn get_mut_body_by_id(&mut self, id: i32) -> Option<&mut SimulatedBody> {
        for i in 0..self.bodies.len() {
            if self.bodies[i].id == id {
                return Some(&mut self.bodies[i]);
            }
        }

        None
    }

    fn resolve_hierarchy_up(&self, body: &SimulatedBody) -> Vec<&SimulatedBody> {
        /* how this will look like for example for the moon,
          moon gets into this function, we don't want to add it
          its parent is earth, it gets found, is added to the moon-result
          then earth gets into this function,
          its parent is the sun, it gets added to the earth-result
          then sun gets into this function, but doesn't have a parent, so sun-result is []
          then sun result [] gets appended to earth result [sun] results in [sun]
          then earth result [sun] gets appended to moon result [earth] results in [earth, sun]
          so it goes upward from the body
        */
        let mut result: Vec<&SimulatedBody> = vec![];
        match body.parent {
            None => (),
            Some(parent) => {
                match self.get_body_by_id(parent) {
                    None => (),
                    Some(parent) => {
                        result.push(parent);
                        let mut sub_result = self.resolve_hierarchy_up(parent);
                        result.append(&mut sub_result)
                    }
                };
            }
        }
        result
    }

    fn resolve_hierarchy_down(&self, body: &SimulatedBody) -> Vec<&SimulatedBody> {
        /* how this will look like for example for the sun,
        sun gets into this function, its satellites are iterated, lets simplify to Venus, Earth, and Mars
        to sun result first added is [Venus]
        then venus gets into this function, but has no satellites, so nothing gets added
        then to sun result [Earth] is added
        then earth gets into this function, results in [Moon], this is appended to
        then [Mars] is added
        so in final it will look like [Venus, Earth, Moon, Mars] it's not optimal,
        but it's good for this purpose here
        */
        let mut result: Vec<&SimulatedBody> = vec![];
        for i in 0..body.satellites.len() {
            match self.get_body_by_id(body.satellites[i]) {
                None => (),
                Some(sat) => {
                    result.push(sat);
                    let mut sub_result = self.resolve_hierarchy_down(sat);
                    result.append(&mut sub_result)
                }
            };
        }
        result
    }

    fn get_body_position(&self, time: &DBig, body: &SimulatedBody) -> DecimalVector3d {
        match body.clone().body.dynamics {
            BodyDynamics::Static(dynamics) => dynamics.position,
            BodyDynamics::Orbiting(dynamics) => {
                let parent = self.get_body_by_id(body.parent.unwrap()).unwrap(); // panic if not fulfilled
                let orbit_progression = (time / dynamics.orbit_period).fract();
                let angle = PIMUL2.deref() * orbit_progression;
                let rotation_matrix =
                    DecimalMatrix3d::axis_angle(&dynamics.orbit_plane_normal, angle);
                rotation_matrix.apply(&DecimalVector3d::new(
                    dynamics.orbit_radius,
                    DBig::ZERO,
                    DBig::ZERO,
                )) + &parent.position
            }
        }
    }

    fn get_body_orientation(&self, time: &DBig, body: &SimulatedBody) -> DecimalMatrix3d {
        let rotation_progression = (time / &body.body.rotation_period).fract();
        let angle = PIMUL2.deref() * rotation_progression;
        DecimalMatrix3d::axis_angle(&body.body.rotation_axis, angle)
    }

    pub fn update(&mut self, time: DBig) {
        let mut schedule: Vec<i32> = vec![];
        for i in 0..self.bodies.len() {
            let body = &self.bodies[i];
            match body.body.dynamics {
                BodyDynamics::Static(_) => {
                    let hierarchy = self.resolve_hierarchy_down(body);
                    for body in hierarchy {
                        schedule.push(body.id);
                    }
                }
                BodyDynamics::Orbiting(_) => (),
            }
        }
        for i in 0..schedule.len() {
            let body_immutable = self.get_body_by_id(schedule[i]).unwrap();

            let position = self.get_body_position(&time, &body_immutable);
            let pos_second_ago = self.get_body_position(&(&time - DBig::ONE), &body_immutable);
            let velocity = &position - pos_second_ago;
            let orientation = self.get_body_orientation(&time, &body_immutable);

            let body = self.get_mut_body_by_id(schedule[i]).unwrap();
            body.position = position;
            body.velocity = velocity;
            body.orientation = orientation;
        }
    }
}
