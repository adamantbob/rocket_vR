use krpc_client::{
    Client,
    services::space_center::{Control, Flight, SpaceCenter, Vessel},
    stream::Stream,
};

pub struct KspAgent {
    pub space_center: SpaceCenter,
    vessel: Vessel,
    vessel_name: String,
    control: Control,
    flight: Flight,
    altitude_stream: Stream<f64>,
    v_speed_stream: Stream<f64>,
    g_force_stream: Stream<f32>,
}

impl KspAgent {
    pub fn new(name: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let client = Client::new(name, "127.0.0.1", 50000, 50001)?;
        let space_center = SpaceCenter::new(client);
        let vessel = space_center.get_active_vessel()?;
        let vessel_name = vessel.get_name()?;
        let control = vessel.get_control()?;

        let rf = vessel.get_orbit()?.get_body()?.get_reference_frame()?;
        let flight = vessel.flight(Some(&rf))?;

        let altitude_stream = flight.get_mean_altitude_stream()?;
        let v_speed_stream = flight.get_vertical_speed_stream()?;
        let g_force_stream = flight.get_g_force_stream()?;

        altitude_stream.set_rate(100.0)?;
        v_speed_stream.set_rate(100.0)?;
        g_force_stream.set_rate(100.0)?;

        Ok(Self {
            space_center,
            vessel,
            vessel_name,
            control,
            flight,
            altitude_stream,
            v_speed_stream,
            g_force_stream,
        })
    }

    pub fn check_active_vessel(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let new_vessel = self.space_center.get_active_vessel()?;
        let new_name = new_vessel.get_name()?;

        if new_name != self.vessel_name {
            println!(
                "\n[SITL] Active vessel changed: {} -> {}",
                self.vessel_name, new_name
            );
            self.vessel = new_vessel;
            self.vessel_name = new_name;
            self.control = self.vessel.get_control()?;

            let rf = self.vessel.get_orbit()?.get_body()?.get_reference_frame()?;
            self.flight = self.vessel.flight(Some(&rf))?;

            self.altitude_stream = self.flight.get_mean_altitude_stream()?;
            self.v_speed_stream = self.flight.get_vertical_speed_stream()?;
            self.g_force_stream = self.flight.get_g_force_stream()?;

            self.altitude_stream.set_rate(100.0)?;
            self.v_speed_stream.set_rate(100.0)?;
            self.g_force_stream.set_rate(100.0)?;
        }
        Ok(())
    }

    pub fn poll_telemetry(&mut self) -> (f64, f64, f64) {
        match (
            self.altitude_stream.get(),
            self.v_speed_stream.get(),
            self.g_force_stream.get(),
        ) {
            (Ok(a), Ok(v), Ok(g)) => (a, v, g as f64),
            _ => (0.0, 0.0, 0.0),
        }
    }

    pub fn activate_next_stage(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        self.control.activate_next_stage()?;
        Ok(())
    }

    pub fn deploy_parachutes(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        // Try staging first
        let _ = self.control.activate_next_stage();

        // Fallback: Deployment service
        if let Ok(parts) = self.vessel.get_parts() {
            if let Ok(parachutes) = parts.get_parachutes() {
                for p in parachutes {
                    let _ = p.deploy();
                }
            }
        }
        Ok(())
    }

    pub fn get_vessel_name(&self) -> &str {
        &self.vessel_name
    }
}
