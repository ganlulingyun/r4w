//! IONEX Ionospheric TEC Map Parser
//!
//! Parses IONEX (IONosphere map EXchange) format files containing global
//! ionospheric Total Electron Content (TEC) maps. These provide more accurate
//! ionospheric delay estimates than the broadcast Klobuchar model.
//!
//! ## File Format
//!
//! IONEX files contain:
//! - Header with map grid parameters
//! - TEC maps at regular time intervals (typically 2 hours)
//! - Each map is a lat/lon grid of TEC values in TECU (10^16 el/m²)
//!
//! ## Interpolation
//!
//! For a given location and time:
//! 1. Spatial interpolation within each TEC map (bilinear)
//! 2. Temporal interpolation between adjacent maps (linear)
//!
//! ## Usage
//!
//! ```ignore
//! use r4w_core::waveform::gnss::ionex::IonexTec;
//!
//! let ionex = IonexTec::from_file("codg0150.25i")?;
//! let tec = ionex.get_tec(lat_deg, lon_deg, gps_time_s)?;
//! let delay_m = ionex.iono_delay_m(tec, elevation_deg, freq_hz);
//! ```

use std::path::Path;

/// IONEX file header
#[derive(Debug, Clone)]
pub struct IonexHeader {
    /// IONEX version
    pub version: f64,
    /// File type (I = Ionosphere)
    pub file_type: char,
    /// Satellite system (G=GPS, M=Mixed, etc.)
    pub sat_system: char,
    /// Mapping function (NONE, COSZ, QFAC, etc.)
    pub mapping_function: String,
    /// Elevation cutoff in degrees
    pub elevation_cutoff: f64,
    /// Base radius for mapping in km
    pub base_radius: f64,
    /// Map dimension (2 or 3)
    pub map_dimension: u8,
    /// Height of single layer model in km
    pub hgt1: f64,
    pub hgt2: f64,
    pub dhgt: f64,
    /// Latitude grid parameters
    pub lat1: f64,
    pub lat2: f64,
    pub dlat: f64,
    /// Longitude grid parameters
    pub lon1: f64,
    pub lon2: f64,
    pub dlon: f64,
    /// TEC exponent (-1 means values in 0.1 TECU)
    pub exponent: i32,
    /// Number of maps
    pub num_maps: u32,
    /// Interval between maps in seconds
    pub interval_s: f64,
    /// First epoch (GPS seconds)
    pub first_epoch_gps_s: f64,
    /// Last epoch (GPS seconds)
    pub last_epoch_gps_s: f64,
}

/// Single TEC map at one epoch
#[derive(Debug, Clone)]
pub struct TecMap {
    /// GPS time of this map
    pub gps_time_s: f64,
    /// TEC values in TECU (2D grid, indexed by [lat_idx][lon_idx])
    pub tec: Vec<Vec<f64>>,
    /// Map height in km (for this map, if multi-layer)
    pub height_km: f64,
}

/// Parsed IONEX TEC data
#[derive(Debug)]
pub struct IonexTec {
    /// File header
    pub header: IonexHeader,
    /// TEC maps indexed by time
    pub maps: Vec<TecMap>,
}

impl Default for IonexHeader {
    fn default() -> Self {
        Self {
            version: 1.0,
            file_type: 'I',
            sat_system: 'G',
            mapping_function: "NONE".to_string(),
            elevation_cutoff: 0.0,
            base_radius: 6371.0,
            map_dimension: 2,
            hgt1: 450.0,
            hgt2: 450.0,
            dhgt: 0.0,
            lat1: 87.5,
            lat2: -87.5,
            dlat: -2.5,
            lon1: -180.0,
            lon2: 180.0,
            dlon: 5.0,
            exponent: -1,
            num_maps: 0,
            interval_s: 7200.0,
            first_epoch_gps_s: 0.0,
            last_epoch_gps_s: 0.0,
        }
    }
}

impl IonexTec {
    /// Load IONEX data from a file
    pub fn from_file<P: AsRef<Path>>(path: P) -> Result<Self, String> {
        let path = path.as_ref();
        if !path.exists() {
            return Err(format!("IONEX file not found: {}", path.display()));
        }

        let content = std::fs::read_to_string(path)
            .map_err(|e| format!("Failed to read IONEX file: {}", e))?;

        Self::parse(&content)
    }

    /// Parse IONEX content from string
    pub fn parse(content: &str) -> Result<Self, String> {
        let mut header = IonexHeader::default();
        let mut maps = Vec::new();
        let mut in_header = true;
        let mut current_map: Option<TecMap> = None;
        let mut current_lat_data: Vec<f64> = Vec::new();
        let mut tec_grid: Vec<Vec<f64>> = Vec::new();

        for line in content.lines() {
            let label = if line.len() >= 60 {
                line[60..].trim()
            } else {
                ""
            };

            if in_header {
                // Parse header fields
                if label == "IONEX VERSION / TYPE" {
                    header.version = line[0..8].trim().parse().unwrap_or(1.0);
                    header.file_type = line.chars().nth(20).unwrap_or('I');
                    header.sat_system = line.chars().nth(40).unwrap_or('G');
                } else if label == "MAP DIMENSION" {
                    header.map_dimension = line[0..6].trim().parse().unwrap_or(2);
                } else if label == "HGT1 / HGT2 / DHGT" {
                    header.hgt1 = line[2..8].trim().parse().unwrap_or(450.0);
                    header.hgt2 = line[8..14].trim().parse().unwrap_or(450.0);
                    header.dhgt = line[14..20].trim().parse().unwrap_or(0.0);
                } else if label == "LAT1 / LAT2 / DLAT" {
                    header.lat1 = line[2..8].trim().parse().unwrap_or(87.5);
                    header.lat2 = line[8..14].trim().parse().unwrap_or(-87.5);
                    header.dlat = line[14..20].trim().parse().unwrap_or(-2.5);
                } else if label == "LON1 / LON2 / DLON" {
                    header.lon1 = line[2..8].trim().parse().unwrap_or(-180.0);
                    header.lon2 = line[8..14].trim().parse().unwrap_or(180.0);
                    header.dlon = line[14..20].trim().parse().unwrap_or(5.0);
                } else if label == "EXPONENT" {
                    header.exponent = line[0..6].trim().parse().unwrap_or(-1);
                } else if label == "# OF MAPS IN FILE" {
                    header.num_maps = line[0..6].trim().parse().unwrap_or(0);
                } else if label == "MAPPING FUNCTION" {
                    header.mapping_function = line[2..6].trim().to_string();
                } else if label == "ELEVATION CUTOFF" {
                    header.elevation_cutoff = line[0..8].trim().parse().unwrap_or(0.0);
                } else if label == "BASE RADIUS" {
                    header.base_radius = line[0..8].trim().parse().unwrap_or(6371.0);
                } else if label == "INTERVAL" {
                    header.interval_s = line[0..6].trim().parse().unwrap_or(7200.0);
                } else if label == "END OF HEADER" {
                    in_header = false;
                }
            } else {
                // Parse data records
                if label == "START OF TEC MAP" {
                    tec_grid = Vec::new();
                } else if label == "EPOCH OF CURRENT MAP" {
                    // Parse epoch: 2025  1 15  0  0  0
                    let year: i32 = line[0..6].trim().parse().unwrap_or(2025);
                    let month: u32 = line[6..12].trim().parse().unwrap_or(1);
                    let day: u32 = line[12..18].trim().parse().unwrap_or(1);
                    let hour: u32 = line[18..24].trim().parse().unwrap_or(0);
                    let minute: u32 = line[24..30].trim().parse().unwrap_or(0);
                    let second: f64 = line[30..36].trim().parse().unwrap_or(0.0);

                    let gps_time = gps_time_from_calendar(year, month, day, hour, minute, second);

                    current_map = Some(TecMap {
                        gps_time_s: gps_time,
                        tec: Vec::new(),
                        height_km: header.hgt1,
                    });

                    if header.first_epoch_gps_s == 0.0 {
                        header.first_epoch_gps_s = gps_time;
                    }
                    header.last_epoch_gps_s = gps_time;
                } else if label.starts_with("LAT/LON1/LON2/DLON/H") {
                    // New latitude row
                    if !current_lat_data.is_empty() {
                        tec_grid.push(current_lat_data.clone());
                    }
                    current_lat_data = Vec::new();
                } else if label == "END OF TEC MAP" {
                    // Save last latitude row and finalize map
                    if !current_lat_data.is_empty() {
                        tec_grid.push(current_lat_data.clone());
                    }
                    if let Some(mut map) = current_map.take() {
                        // Scale TEC values by exponent (typically -1 means 0.1 TECU)
                        let scale = (10.0_f64).powi(header.exponent);
                        for row in &mut tec_grid {
                            for val in row {
                                *val *= scale;
                            }
                        }
                        map.tec = tec_grid.clone();
                        maps.push(map);
                    }
                    tec_grid = Vec::new();
                    current_lat_data = Vec::new();
                } else if !label.contains("OF TEC MAP") && !label.contains("EPOCH") && !label.contains("LAT") {
                    // TEC data line - parse integer values
                    let values: Vec<f64> = line[..60.min(line.len())]
                        .split_whitespace()
                        .filter_map(|s| s.parse::<f64>().ok())
                        .collect();
                    current_lat_data.extend(values);
                }
            }
        }

        Ok(Self { header, maps })
    }

    /// Get interpolated TEC value at a location and time
    ///
    /// Returns TEC in TECU (10^16 electrons/m²)
    pub fn get_tec(&self, lat_deg: f64, lon_deg: f64, gps_time_s: f64) -> Option<f64> {
        if self.maps.is_empty() {
            return None;
        }

        // Find bracketing maps in time
        let (map1_idx, map2_idx, t_frac) = self.find_time_bracket(gps_time_s)?;

        // Get TEC from each map via spatial interpolation
        let tec1 = self.interpolate_spatial(&self.maps[map1_idx], lat_deg, lon_deg)?;
        let tec2 = self.interpolate_spatial(&self.maps[map2_idx], lat_deg, lon_deg)?;

        // Linear temporal interpolation
        Some(tec1 * (1.0 - t_frac) + tec2 * t_frac)
    }

    /// Find time bracket indices and interpolation fraction
    fn find_time_bracket(&self, gps_time_s: f64) -> Option<(usize, usize, f64)> {
        if self.maps.is_empty() {
            return None;
        }

        // Find first map at or after requested time
        let mut idx2 = 0;
        for (i, map) in self.maps.iter().enumerate() {
            if map.gps_time_s >= gps_time_s {
                idx2 = i;
                break;
            }
            idx2 = i;
        }

        let idx1 = if idx2 > 0 { idx2 - 1 } else { 0 };

        if idx1 == idx2 {
            return Some((idx1, idx1, 0.0));
        }

        let t1 = self.maps[idx1].gps_time_s;
        let t2 = self.maps[idx2].gps_time_s;
        let t_frac = (gps_time_s - t1) / (t2 - t1);
        let t_frac = t_frac.clamp(0.0, 1.0);

        Some((idx1, idx2, t_frac))
    }

    /// Bilinear spatial interpolation within a TEC map
    fn interpolate_spatial(&self, map: &TecMap, lat_deg: f64, lon_deg: f64) -> Option<f64> {
        if map.tec.is_empty() {
            return None;
        }

        // Normalize longitude to map range
        let mut lon = lon_deg;
        while lon < self.header.lon1 {
            lon += 360.0;
        }
        while lon > self.header.lon2 {
            lon -= 360.0;
        }

        // Calculate grid indices (note: dlat is typically negative)
        let lat_idx_f = (lat_deg - self.header.lat1) / self.header.dlat;
        let lon_idx_f = (lon - self.header.lon1) / self.header.dlon;

        let lat_idx0 = lat_idx_f.floor() as usize;
        let lon_idx0 = lon_idx_f.floor() as usize;

        let lat_frac = lat_idx_f - lat_idx_f.floor();
        let lon_frac = lon_idx_f - lon_idx_f.floor();

        // Get corner values with bounds checking
        let num_lats = map.tec.len();
        if num_lats == 0 {
            return None;
        }
        let num_lons = map.tec[0].len();

        let lat_idx1 = (lat_idx0 + 1).min(num_lats - 1);
        let lon_idx1 = (lon_idx0 + 1).min(num_lons - 1);
        let lat_idx0 = lat_idx0.min(num_lats - 1);
        let lon_idx0 = lon_idx0.min(num_lons - 1);

        let v00 = map.tec[lat_idx0][lon_idx0];
        let v01 = map.tec[lat_idx0][lon_idx1];
        let v10 = map.tec[lat_idx1][lon_idx0];
        let v11 = map.tec[lat_idx1][lon_idx1];

        // Bilinear interpolation
        let v0 = v00 * (1.0 - lon_frac) + v01 * lon_frac;
        let v1 = v10 * (1.0 - lon_frac) + v11 * lon_frac;
        let tec = v0 * (1.0 - lat_frac) + v1 * lat_frac;

        // Clamp negative values (can occur due to interpolation)
        Some(tec.max(0.0))
    }

    /// Convert TEC to ionospheric delay in meters
    ///
    /// Uses single-layer model with obliquity factor.
    ///
    /// # Arguments
    /// * `tec_tecu` - Vertical TEC in TECU
    /// * `elevation_deg` - Satellite elevation angle in degrees
    /// * `frequency_hz` - Signal frequency in Hz
    pub fn iono_delay_m(tec_tecu: f64, elevation_deg: f64, frequency_hz: f64) -> f64 {
        // Constants
        const EARTH_RADIUS_KM: f64 = 6371.0;
        const IONO_HEIGHT_KM: f64 = 450.0; // Single layer model height
        const K: f64 = 40.3; // TEC to delay constant (m³/s²)

        // Obliquity factor (mapping function)
        let elevation_rad = elevation_deg.to_radians();
        let sin_el = elevation_rad.sin();
        let cos_z_prime = (1.0 - (EARTH_RADIUS_KM / (EARTH_RADIUS_KM + IONO_HEIGHT_KM) * elevation_rad.cos()).powi(2)).sqrt();
        let obliquity = 1.0 / cos_z_prime.max(sin_el.max(0.1));

        // Slant TEC
        let stec = tec_tecu * obliquity;

        // Delay in meters
        // delay = K * TEC / f² where TEC is in electrons/m² and K = 40.3 m³/s²
        // TECU = 10^16 el/m², so TEC_el_per_m2 = tec_tecu * 1e16
        let tec_el_per_m2 = stec * 1e16;
        K * tec_el_per_m2 / (frequency_hz * frequency_hz)
    }

    /// Get time span of this IONEX file
    pub fn time_span(&self) -> Option<(f64, f64)> {
        if self.maps.is_empty() {
            return None;
        }
        Some((self.header.first_epoch_gps_s, self.header.last_epoch_gps_s))
    }

    /// Get number of TEC maps in file
    pub fn num_maps(&self) -> usize {
        self.maps.len()
    }
}

/// Convert calendar date to GPS time (seconds since Jan 6, 1980)
fn gps_time_from_calendar(year: i32, month: u32, day: u32, hour: u32, min: u32, sec: f64) -> f64 {
    let a = (14 - month as i64) / 12;
    let y = year as i64 + 4800 - a;
    let m = month as i64 + 12 * a - 3;
    let jd = day as i64 + (153 * m + 2) / 5 + 365 * y + y / 4 - y / 100 + y / 400 - 32045;

    let gps_epoch_jd = 2444244.5;
    let days_since_gps_epoch = jd as f64 - gps_epoch_jd;
    let utc_seconds = days_since_gps_epoch * 86400.0 + hour as f64 * 3600.0 + min as f64 * 60.0 + sec;

    // GPS time = UTC + leap seconds
    utc_seconds + 18.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_iono_delay_calculation() {
        // Typical GPS L1 values
        let tec = 20.0; // 20 TECU - moderate ionosphere
        let elevation = 45.0; // 45 degree elevation
        let freq_l1 = 1575.42e6; // GPS L1 frequency

        let delay = IonexTec::iono_delay_m(tec, elevation, freq_l1);

        // Expect roughly 3-10 meters delay for these conditions
        assert!(delay > 2.0, "Delay too small: {}", delay);
        assert!(delay < 20.0, "Delay too large: {}", delay);
    }

    #[test]
    fn test_iono_delay_frequency_dependence() {
        let tec = 20.0;
        let elevation = 90.0; // Zenith

        let delay_l1 = IonexTec::iono_delay_m(tec, elevation, 1575.42e6);
        let delay_l5 = IonexTec::iono_delay_m(tec, elevation, 1176.45e6);

        // L5 should have larger delay (lower frequency)
        assert!(delay_l5 > delay_l1 * 1.5, "L5 delay should be larger than L1");
    }
}
