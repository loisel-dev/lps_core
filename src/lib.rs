use nalgebra::Vector3;


pub struct PosSystem {
    dist_anchor_1_2: f32,
    dist_anchor_1_3: f32,
    dist_anchor_2_3: f32,
    dist_probe_anchor_1: f32,
    dist_probe_anchor_2: f32,
    dist_probe_anchor_3: f32,
    system_offset: Vector3<f32>,
}

impl PosSystem {

    pub fn new(
        dist_anchor_1_2: f32,
        dist_anchor_1_3: f32,
        dist_anchor_2_3: f32,
    ) -> Self {
        let dist_probe_anchor_1 = 0.;
        let dist_probe_anchor_2 = 0.;
        let dist_probe_anchor_3 = 0.;

        let system_offset = Vector3::new(0., 0., 0.);

        PosSystem {
            dist_anchor_1_2,
            dist_anchor_1_3,
            dist_anchor_2_3, 
            dist_probe_anchor_1,
            dist_probe_anchor_2,
            dist_probe_anchor_3,
            system_offset,
        }
    }

    pub fn get_pos_anchor_1(&self) -> Vector3<f32> {
        self.system_offset.clone()
    }

    pub fn get_pos_anchor_2(&self) -> Vector3<f32> {
        let pos = Vector3::new(self.dist_anchor_1_2, 0., 0.);
        return pos + self.system_offset.clone();
    }

    pub fn get_pos_anchor_3(&self) -> Vector3<f32> {
        let y = 2. * self.get_area() / self.dist_anchor_1_2;
        let x = (self.dist_anchor_1_3.powf(2.) - y.powf(2.)).sqrt();

        let pos = Vector3::new(x, y, 0.);

        return pos + self.system_offset;
    }

    pub fn set_probe_distances(
        &mut self, 
        dist_anchor_1: f32,
        dist_anchor_2: f32,
        dist_anchor_3: f32,
    ) {
        self.dist_probe_anchor_1 = dist_anchor_1;
        self.dist_probe_anchor_2 = dist_anchor_2;
        self.dist_probe_anchor_3 = dist_anchor_3;
    }

    pub fn get_probe_position(&self) -> Option<Vector3<f32>> {
        let anchor_1 = self.get_pos_anchor_1();
        let anchor_2 = self.get_pos_anchor_2();
        let anchor_3 = self.get_pos_anchor_3();

        let anchor_positions = [
            (anchor_1[0], anchor_1[1], anchor_1[2]),
            (anchor_2[0], anchor_2[1], anchor_2[2]),
            (anchor_3[0], anchor_3[1], anchor_3[2]),
        ];
        let distances = [self.dist_probe_anchor_1, self.dist_probe_anchor_2, self.dist_probe_anchor_3];

        match trilaterate_3d(anchor_positions, distances) {
            Some(pos) => { return Some(pos + self.system_offset) }
            None => {return None}
        }
    }

    fn get_perimeter(&self) -> f32 {
        return self.dist_anchor_1_2 + self.dist_anchor_1_3 + self.dist_anchor_2_3
    }
    fn get_area(&self) -> f32 {
        let s = self.get_perimeter() / 2.;

        // sqrt(s * (s - a) * (s - b) * (s - c))
        return (s * (s - self.dist_anchor_1_2) 
        * (s - self.dist_anchor_1_3) 
        * (s - self.dist_anchor_2_3)).sqrt();
    }
}

pub fn trilaterate_3d(
    anchor_positions: [(f32, f32, f32); 3],
    distances: [f32; 3],
) -> Option<Vector3<f32>> {
    let (x1, y1, z1) = anchor_positions[0];
    let (x2, y2, z2) = anchor_positions[1];
    let (x3, y3, z3) = anchor_positions[2];

    let d1 = distances[0];
    let d2 = distances[1];
    let d3 = distances[2];

    // Calculate the coefficients of the linear system
    let p1 = Vector3::new(x1, y1, z1);
    let p2 = Vector3::new(x2, y2, z2);
    let p3 = Vector3::new(x3, y3, z3);

    let ex = (p2 - p1).normalize();
    let i = ex.dot(&(p3 - p1));
    let ey = ((p3 - p1) - ex * i).normalize();
    let ez = ex.cross(&ey);

    let d = (p2 - p1).norm();
    let j = ey.dot(&(p3 - p1));

    // Solve for the coordinates in the local frame
    let x = (d1.powi(2) - d2.powi(2) + d.powi(2)) / (2.0 * d);
    let y = ((d1.powi(2) - d3.powi(2) + i.powi(2) + j.powi(2)) / (2.0 * j)) - (i / j) * x;

    let z_squared = d1.powi(2) - x.powi(2) - y.powi(2);
    if z_squared < 0.0 {
        // No solution exists
        return None;
    }
    let z = z_squared.sqrt();

    // Convert back to the global coordinate system
    let result = p1 + ex * x + ey * y + ez * z;
    Some(result)
}