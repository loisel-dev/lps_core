use lps_core::*;

fn main() {
    let mut pos_system = PosSystem::new(1., 1., 1.);

    pos_system.set_probe_distances(1., 1., 1.);

    match pos_system.get_probe_position() {
        Some(position) => {
            println!("Probe position: {:?}", position);
        }
        None => {
            println!("Position not found!");
        }
    }
}
