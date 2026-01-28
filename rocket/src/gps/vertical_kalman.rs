pub struct VerticalKalman {
    z: f32,    // Estimated Altitude (m)
    v: f32,    // Estimated Velocity (m/s)
    p_zz: f32, // Estimation Error Covariance (Alt)
    p_vv: f32, // Estimation Error Covariance (Vel)
    p_zv: f32, // Cross Covariance

    r_alt: f32,   // Measurement Noise (GPS Jitter - ~2.0m)
    q_accel: f32, // Process Noise (Rocket Dynamics - ~0.5)
}

impl VerticalKalman {
    pub fn new(initial_alt: f32) -> Self {
        Self {
            z: initial_alt,
            v: 0.0,
            p_zz: 1.0,
            p_vv: 1.0,
            p_zv: 0.0,
            r_alt: 2.0,   // GPS is usually noisy
            q_accel: 0.1, // How much we trust our "constant velocity" model
        }
    }

    pub fn update(&mut self, dt: f32, measured_z: f32) {
        let dt2 = dt * dt;
        let dt3 = dt2 * dt;
        let dt4 = dt3 * dt;

        // --- 1. Predict ---
        self.z += self.v * dt;

        // Update Error Covariance using simple multiplications
        self.p_zz += dt * (2.0 * self.p_zv + dt * self.p_vv) + 0.25 * dt4 * self.q_accel;
        self.p_zv += dt * self.p_vv + 0.5 * dt3 * self.q_accel;
        self.p_vv += dt2 * self.q_accel;

        // --- 2. Update (Correct with GPS) ---
        let innovation = measured_z - self.z;
        let s = self.p_zz + self.r_alt; // Innovation covariance

        let k_z = self.p_zz / s; // Kalman Gain for Altitude
        let k_v = self.p_zv / s; // Kalman Gain for Velocity

        // Apply Gain to State
        self.z += k_z * innovation;
        self.v += k_v * innovation;

        // Update Error Covariance (P = (I - KH)P)
        self.p_zz -= k_z * self.p_zz;
        self.p_zv -= k_z * self.p_zv;
        self.p_vv -= k_v * self.p_zv;
    }

    pub fn get_state(&self) -> (f32, f32) {
        (self.z, self.v)
    }
}
