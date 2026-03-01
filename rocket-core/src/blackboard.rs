use crate::datacells::DataCell;
use crate::{CPUHealth, GPSData, GPSHealth, IMUData, IMUHealth, SDCardHealth, WifiHealth};

/// The Global Blackboard for Sensor Data
pub struct SensorData {
    pub gps: DataCell<GPSData>,
    pub imu: DataCell<IMUData>,
}

/// The Global Blackboard for System Health and Diagnostics
pub struct SystemHealth {
    pub imu_health: DataCell<IMUHealth>,
    pub gps_health: DataCell<GPSHealth>,
    pub wifi_health: DataCell<WifiHealth>,
    pub sd_health: DataCell<SDCardHealth>,
    pub cpu_health: DataCell<CPUHealth>,
}

pub static SENSOR_DATA: SensorData = SensorData {
    imu: DataCell::new(IMUData::new()),
    gps: DataCell::new(GPSData::new()),
};

pub static SYSTEM_HEALTH: SystemHealth = SystemHealth {
    imu_health: DataCell::new(IMUHealth::new()),
    gps_health: DataCell::new(GPSHealth::new()),
    wifi_health: DataCell::new(WifiHealth::new()),
    sd_health: DataCell::new(SDCardHealth::new()),
    cpu_health: DataCell::new(CPUHealth::new()),
};
