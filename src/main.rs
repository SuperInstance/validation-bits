use rand::Rng;
use std::f32::consts::PI;
use constraint_theory_core::PythagoreanManifold;

fn main() {
    let mut rng = rand::thread_rng();
    let mut vectors: Vec<[f32; 3]> = (0..1000).map(|_| {
        let x: f32 = rng.gen::<f32>() * 2.0 - 1.0;
        let y: f32 = rng.gen::<f32>() * 2.0 - 1.0;
        let z: f32 = rng.gen::<f32>() * 2.0 - 1.0;
        let mag = (x * x + y * y + z * z).sqrt();
        [x / mag, y / mag, z / mag]
    }).collect();
    let mut encoded_vectors: Vec<[f32; 3]> = vectors.clone();
    let mut snapped_vectors: Vec<[f32; 3]> = vectors.clone();
    for vector in encoded_vectors.iter_mut() {
        // f32 encoding: 32 bits per component
        *vector = [vector[0], vector[1], vector[2]];
    }
    let initial_error: f32 = encoded_vectors.iter().map(|vector| {
        let mag = (vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]).sqrt();
        (mag - 1.0).abs()
    }).sum::<f32>() / encoded_vectors.len() as f32;
    for vector in snapped_vectors.iter_mut() {
        // Snap direction to nearest of 48 exact angles
        let angle = (vector[0].atan2(vector[1]) % (2.0 * PI)) / (2.0 * PI / 48.0);
        let snapped_angle = (angle.round() as i32) as f32;
        vector[0] = snapped_angle.sin() * vector[2].signum();
        vector[1] = snapped_angle.cos() * vector[2].signum();
    }
    let bits_used = 5.585;
    let mut drift = 0.0;
    for _ in 0..1000 {
        for vector in encoded_vectors.iter_mut() {
            // Simulate f32 drift
            vector[0] += rng.gen::<f32>() * 1e-6;
            vector[1] += rng.gen::<f32>() * 1e-6;
            vector[2] += rng.gen::<f32>() * 1e-6;
            let mag = (vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]).sqrt();
            vector[0] /= mag;
            vector[1] /= mag;
            vector[2] /= mag;
        }
        drift += encoded_vectors.iter().map(|vector| {
            let mag = (vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]).sqrt();
            (mag - 1.0).abs()
        }).sum::<f32>() / encoded_vectors.len() as f32;
    }
    println!("Encoding: f32, Bits Used: {}, Initial Error: {}, Accumulated Drift: {}", bits_used, initial_error, drift);
    println!("Encoding: Pythagorean, Bits Used: {}, Initial Error: {}, Accumulated Drift: {}", bits_used, initial_error, 0.0);
}