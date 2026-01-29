// gps/parser/tests.rs
#[cfg(test)]
mod tests {
    use crate::gps::parser::{process_line, validate_checksum};
    use crate::gps::types::GPSData;

    #[test]
    fn test_validate_checksum() {
        let sentence = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
        assert!(validate_checksum(sentence));
    }

    #[test]
    fn test_process_gga() {
        let sentence = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
        let mut data = GPSData::new();
        let res = process_line(sentence, &mut data);
        assert!(res.is_ok());
        assert_eq!(data.utc_time_secs, 123519);
        assert_eq!(data.satellites, 8);
        assert_eq!(data.raw_alt_mm, 545400);
        assert!(data.fix_valid);
    }
}
