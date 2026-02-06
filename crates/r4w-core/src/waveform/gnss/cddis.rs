//! CDDIS (Crustal Dynamics Data Information System) Ephemeris Download
//!
//! Downloads GNSS broadcast ephemerides from NASA CDDIS archive.
//!
//! ## Authentication
//!
//! CDDIS requires NASA Earthdata authentication. Set up credentials using one of:
//!
//! 1. **~/.netrc file** (recommended):
//!    ```text
//!    machine urs.earthdata.nasa.gov login <username> password <password>
//!    ```
//!
//! 2. **Environment variables**:
//!    ```bash
//!    export EARTHDATA_USERNAME=your_username
//!    export EARTHDATA_PASSWORD=your_password
//!    ```
//!
//! ## File Naming
//!
//! Daily merged multi-GNSS files (RINEX v3):
//! - `BRDC00IGS_R_YYYYDDD0000_01D_MN.rnx.gz`
//!
//! Where:
//! - YYYY = year
//! - DDD = day of year (001-366)
//!
//! ## Cache Location
//!
//! Downloaded files are cached in `~/.cache/r4w/ephemeris/`

use std::path::PathBuf;

/// CDDIS base URL for GNSS data (requires Earthdata auth)
pub const CDDIS_BASE_URL: &str = "https://cddis.nasa.gov/archive/gnss/data/daily";

/// IGS BKG mirror (no authentication required, recommended)
pub const BKG_BASE_URL: &str = "https://igs.bkg.bund.de/root_ftp/IGS/BRDC";

/// Cache directory name under user's cache directory
pub const CACHE_DIR: &str = "r4w/ephemeris";

/// Get the cache directory path
pub fn cache_dir() -> PathBuf {
    directories::BaseDirs::new()
        .map(|d| d.cache_dir().join(CACHE_DIR))
        .unwrap_or_else(|| PathBuf::from(".cache/r4w/ephemeris"))
}

/// Ensure the cache directory exists
pub fn ensure_cache_dir() -> std::io::Result<PathBuf> {
    let dir = cache_dir();
    std::fs::create_dir_all(&dir)?;
    Ok(dir)
}

/// Convert a date (year, month, day) to day of year
pub fn day_of_year(year: i32, month: u32, day: u32) -> u32 {
    let is_leap = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
    let days_in_month = [
        0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334,
    ];
    let mut doy = days_in_month[month as usize - 1] + day;
    if is_leap && month > 2 {
        doy += 1;
    }
    doy
}

/// Parse a date string "YYYY-MM-DD" into (year, month, day)
pub fn parse_date(date_str: &str) -> Result<(i32, u32, u32), String> {
    let parts: Vec<&str> = date_str.split('-').collect();
    if parts.len() != 3 {
        return Err(format!("Invalid date format '{}'. Expected YYYY-MM-DD", date_str));
    }
    let year: i32 = parts[0].parse().map_err(|_| "Invalid year")?;
    let month: u32 = parts[1].parse().map_err(|_| "Invalid month")?;
    let day: u32 = parts[2].parse().map_err(|_| "Invalid day")?;
    Ok((year, month, day))
}

/// Generate the CDDIS URL for a merged multi-GNSS broadcast ephemeris file
pub fn cddis_url(year: i32, day_of_year: u32) -> String {
    format!(
        "{}/{}/brdc/BRDC00IGS_R_{}{:03}0000_01D_MN.rnx.gz",
        CDDIS_BASE_URL, year, year, day_of_year
    )
}

/// Generate the IGS BKG URL for a merged multi-GNSS broadcast ephemeris file
/// BKG is a mirror that doesn't require authentication
pub fn bkg_url(year: i32, day_of_year: u32) -> String {
    format!(
        "{}/{}/{:03}/BRDC00IGS_R_{}{:03}0000_01D_MN.rnx.gz",
        BKG_BASE_URL, year, day_of_year, year, day_of_year
    )
}

/// Generate the local cache filename for a date
pub fn cache_filename(year: i32, day_of_year: u32) -> String {
    format!("BRDC00IGS_R_{}{:03}0000_01D_MN.rnx", year, day_of_year)
}

/// Get the full cache path for a date
pub fn cache_path(year: i32, month: u32, day: u32) -> PathBuf {
    let doy = day_of_year(year, month, day);
    cache_dir().join(cache_filename(year, doy))
}

/// Check if ephemeris for a date is already cached
pub fn is_cached(year: i32, month: u32, day: u32) -> bool {
    cache_path(year, month, day).exists()
}

/// Credentials for Earthdata authentication
#[derive(Debug, Clone)]
pub struct EarthdataCredentials {
    pub username: String,
    pub password: String,
}

impl EarthdataCredentials {
    /// Load credentials from environment variables
    pub fn from_env() -> Option<Self> {
        let username = std::env::var("EARTHDATA_USERNAME").ok()?;
        let password = std::env::var("EARTHDATA_PASSWORD").ok()?;
        Some(Self { username, password })
    }

    /// Load credentials from ~/.netrc file
    pub fn from_netrc() -> Option<Self> {
        let home = directories::BaseDirs::new()?.home_dir().to_path_buf();
        let netrc_path = home.join(".netrc");

        if !netrc_path.exists() {
            return None;
        }

        let contents = std::fs::read_to_string(&netrc_path).ok()?;

        // Simple netrc parser - look for urs.earthdata.nasa.gov
        let mut lines = contents.lines();
        while let Some(line) = lines.next() {
            if line.contains("urs.earthdata.nasa.gov") || line.contains("machine urs.earthdata.nasa.gov") {
                // Parse the rest of the line or following lines for login/password
                let tokens: Vec<&str> = line.split_whitespace().collect();
                let mut username = None;
                let mut password = None;

                let mut i = 0;
                while i < tokens.len() {
                    match tokens[i] {
                        "login" if i + 1 < tokens.len() => {
                            username = Some(tokens[i + 1].to_string());
                            i += 2;
                        }
                        "password" if i + 1 < tokens.len() => {
                            password = Some(tokens[i + 1].to_string());
                            i += 2;
                        }
                        _ => i += 1,
                    }
                }

                // If not found on same line, check next lines
                if username.is_none() || password.is_none() {
                    for next_line in lines.by_ref().take(5) {
                        let tokens: Vec<&str> = next_line.split_whitespace().collect();
                        let mut i = 0;
                        while i < tokens.len() {
                            match tokens[i] {
                                "login" if i + 1 < tokens.len() && username.is_none() => {
                                    username = Some(tokens[i + 1].to_string());
                                    i += 2;
                                }
                                "password" if i + 1 < tokens.len() && password.is_none() => {
                                    password = Some(tokens[i + 1].to_string());
                                    i += 2;
                                }
                                "machine" => break, // Next machine entry
                                _ => i += 1,
                            }
                        }
                        if username.is_some() && password.is_some() {
                            break;
                        }
                    }
                }

                if let (Some(u), Some(p)) = (username, password) {
                    return Some(Self {
                        username: u,
                        password: p,
                    });
                }
            }
        }

        None
    }

    /// Load credentials from environment or netrc
    pub fn load() -> Option<Self> {
        Self::from_env().or_else(Self::from_netrc)
    }
}

/// Download ephemeris from IGS servers
///
/// Returns the path to the downloaded (and decompressed) file.
///
/// Tries IGS BKG mirror first (no authentication required), then falls back
/// to CDDIS (requires NASA Earthdata credentials).
#[cfg(feature = "ephemeris")]
pub fn fetch_ephemeris(
    year: i32,
    month: u32,
    day: u32,
    force_refresh: bool,
) -> Result<PathBuf, String> {
    let doy = day_of_year(year, month, day);
    let cache_file = cache_path(year, month, day);

    // Return cached file if it exists and force_refresh is false
    if !force_refresh && cache_file.exists() {
        return Ok(cache_file);
    }

    // Ensure cache directory exists
    ensure_cache_dir().map_err(|e| format!("Failed to create cache directory: {}", e))?;

    // Build HTTP client
    let client = reqwest::blocking::Client::builder()
        .timeout(std::time::Duration::from_secs(60))
        .build()
        .map_err(|e| format!("Failed to create HTTP client: {}", e))?;

    // Try IGS BKG first (no authentication required)
    let bkg = bkg_url(year, doy);
    match try_download(&client, &bkg, None) {
        Ok(bytes) => {
            return save_rinex(&bytes, &cache_file);
        }
        Err(e) => {
            eprintln!("BKG download failed: {}. Trying CDDIS...", e);
        }
    }

    // Fall back to CDDIS (requires Earthdata credentials)
    let creds = EarthdataCredentials::load();
    if creds.is_none() {
        return Err(format!(
            "BKG download failed and no Earthdata credentials found.\n\
             Either the file doesn't exist for {}-{:02}-{:02}, or you need to configure credentials.\n\
             Set EARTHDATA_USERNAME/EARTHDATA_PASSWORD or add to ~/.netrc:\n\
             machine urs.earthdata.nasa.gov login <user> password <pass>",
            year, month, day
        ));
    }

    let cddis = cddis_url(year, doy);
    match try_download(&client, &cddis, creds.as_ref()) {
        Ok(bytes) => save_rinex(&bytes, &cache_file),
        Err(e) => Err(format!("CDDIS download also failed: {}", e)),
    }
}

/// Try to download from a URL, optionally with credentials
#[cfg(feature = "ephemeris")]
fn try_download(
    client: &reqwest::blocking::Client,
    url: &str,
    creds: Option<&EarthdataCredentials>,
) -> Result<Vec<u8>, String> {
    let mut request = client.get(url);

    if let Some(c) = creds {
        request = request.basic_auth(&c.username, Some(&c.password));
    }

    let response = request.send().map_err(|e| format!("Request failed: {}", e))?;

    if !response.status().is_success() {
        return Err(format!("HTTP {}", response.status()));
    }

    let bytes = response.bytes().map_err(|e| format!("Read failed: {}", e))?;

    // Check if we got HTML instead of gzip (error page)
    if bytes.len() > 5 {
        let start = &bytes[0..5];
        if start == b"<!DOC" || start == b"<html" || start == b"<HTML" {
            return Err("Received HTML error page instead of data".to_string());
        }
    }

    Ok(bytes.to_vec())
}

/// Decompress gzip and save RINEX file
#[cfg(feature = "ephemeris")]
fn save_rinex(bytes: &[u8], cache_file: &PathBuf) -> Result<PathBuf, String> {
    use std::io::Read;

    // Decompress gzip
    let mut decoder = flate2::read::GzDecoder::new(bytes);
    let mut decompressed = Vec::new();
    decoder
        .read_to_end(&mut decompressed)
        .map_err(|e| format!("Failed to decompress gzip: {}", e))?;

    // Verify it looks like RINEX data
    let header = String::from_utf8_lossy(&decompressed[..decompressed.len().min(100)]);
    if !header.contains("RINEX") && !header.contains("NAV") && !header.contains("BRDC") {
        return Err("Downloaded file does not appear to be RINEX format".to_string());
    }

    // Write to cache
    std::fs::write(cache_file, &decompressed)
        .map_err(|e| format!("Failed to write cache file: {}", e))?;

    Ok(cache_file.clone())
}

#[cfg(not(feature = "ephemeris"))]
pub fn fetch_ephemeris(
    _year: i32,
    _month: u32,
    _day: u32,
    _force_refresh: bool,
) -> Result<PathBuf, String> {
    Err("CDDIS download requires the 'ephemeris' feature".to_string())
}

/// List all cached ephemeris files
pub fn list_cached() -> Vec<(i32, u32, PathBuf)> {
    let dir = cache_dir();
    if !dir.exists() {
        return Vec::new();
    }

    let mut files = Vec::new();
    if let Ok(entries) = std::fs::read_dir(&dir) {
        for entry in entries.flatten() {
            let path = entry.path();
            if let Some(name) = path.file_name().and_then(|n| n.to_str()) {
                // Parse BRDC00IGS_R_YYYYDDD0000_01D_MN.rnx
                if name.starts_with("BRDC00IGS_R_") && name.ends_with(".rnx") {
                    if let Some(year_doy) = name.get(12..19) {
                        if let (Ok(year), Ok(doy)) = (
                            year_doy[0..4].parse::<i32>(),
                            year_doy[4..7].parse::<u32>(),
                        ) {
                            files.push((year, doy, path));
                        }
                    }
                }
            }
        }
    }

    files.sort_by_key(|(y, d, _)| (*y, *d));
    files
}

/// Convert GPS time to date (year, month, day)
pub fn gps_time_to_date(gps_time_s: f64) -> (i32, u32, u32) {
    // GPS epoch: Jan 6, 1980
    let gps_epoch_jd = 2_444_244.5;
    let days_since_epoch = gps_time_s / 86400.0;
    let jd = gps_epoch_jd + days_since_epoch;

    // Convert Julian Day to Gregorian calendar
    let z = (jd + 0.5).floor() as i64;
    let f = jd + 0.5 - z as f64;
    let a = if z < 2_299_161 {
        z
    } else {
        let alpha = ((z as f64 - 1867216.25) / 36524.25).floor() as i64;
        z + 1 + alpha - alpha / 4
    };
    let b = a + 1524;
    let c = ((b as f64 - 122.1) / 365.25).floor() as i64;
    let d = (365.25 * c as f64).floor() as i64;
    let e = ((b - d) as f64 / 30.6001).floor() as i64;

    let day = b - d - (30.6001 * e as f64).floor() as i64 + f.floor() as i64;
    let month = if e < 14 { e - 1 } else { e - 13 };
    let year = if month > 2 { c - 4716 } else { c - 4715 };

    (year as i32, month as u32, day as u32)
}

// ---------------------------------------------------------------------------
// SP3 Precise Ephemeris Downloads
// ---------------------------------------------------------------------------

/// CODE FTP server for SP3 files (no authentication required)
pub const CODE_SP3_URL: &str = "http://ftp.aiub.unibe.ch/CODE";

/// IGS SP3 products URL (via BKG mirror)
pub const BKG_SP3_URL: &str = "https://igs.bkg.bund.de/root_ftp/IGS/products";

/// Generate SP3 filename for CODE final products
/// Format: COD0OPSFIN_YYYYDDD0000_01D_05M_ORB.SP3.gz
pub fn code_sp3_filename(year: i32, doy: u32) -> String {
    format!("COD0OPSFIN_{}{:03}0000_01D_05M_ORB.SP3.gz", year, doy)
}

/// Generate SP3 URL from CODE
pub fn code_sp3_url(year: i32, doy: u32) -> String {
    format!("{}/{}/{}", CODE_SP3_URL, year, code_sp3_filename(year, doy))
}

/// Alternative SP3 URL from IGS BKG (uses GPS week format)
pub fn bkg_sp3_url(gps_week: u32, dow: u32) -> String {
    format!(
        "{}/{:04}/igs{:04}{}.sp3.Z",
        BKG_SP3_URL, gps_week, gps_week, dow
    )
}

/// Cache path for SP3 files (from day-of-year)
pub fn sp3_cache_path_doy(year: i32, doy: u32) -> PathBuf {
    cache_dir().join(format!("sp3_{}{:03}.sp3", year, doy))
}

/// Cache path for SP3 files (from date)
pub fn sp3_cache_path(year: i32, month: u32, day: u32) -> PathBuf {
    sp3_cache_path_doy(year, day_of_year(year, month, day))
}

/// Check if SP3 is cached (from day-of-year)
pub fn sp3_is_cached_doy(year: i32, doy: u32) -> bool {
    sp3_cache_path_doy(year, doy).exists()
}

/// Check if SP3 is cached (from date)
pub fn sp3_is_cached(year: i32, month: u32, day: u32) -> bool {
    sp3_is_cached_doy(year, day_of_year(year, month, day))
}

/// Download SP3 precise ephemeris from CODE
#[cfg(feature = "ephemeris")]
pub fn fetch_sp3(year: i32, month: u32, day: u32, force_refresh: bool) -> Result<PathBuf, String> {
    let doy = day_of_year(year, month, day);
    let cache_file = sp3_cache_path_doy(year, doy);

    if !force_refresh && cache_file.exists() {
        return Ok(cache_file);
    }

    ensure_cache_dir().map_err(|e| format!("Failed to create cache directory: {}", e))?;

    let client = reqwest::blocking::Client::builder()
        .timeout(std::time::Duration::from_secs(120))
        .build()
        .map_err(|e| format!("Failed to create HTTP client: {}", e))?;

    // Try CODE first
    let url = code_sp3_url(year, doy);
    match try_download(&client, &url, None) {
        Ok(bytes) => {
            return save_sp3(&bytes, &cache_file);
        }
        Err(e) => {
            eprintln!("CODE SP3 download failed: {}. Trying alternatives...", e);
        }
    }

    // Try alternative formats/sources
    // CODE final products have different naming
    let final_url = format!(
        "{}/{}/COD{:03}7.EPH_R.gz",
        CODE_SP3_URL, year, doy
    );
    match try_download(&client, &final_url, None) {
        Ok(bytes) => {
            return save_sp3(&bytes, &cache_file);
        }
        Err(e) => {
            eprintln!("CODE final SP3 download failed: {}", e);
        }
    }

    Err(format!(
        "Failed to download SP3 for {}-{:02}-{:02}. File may not be available yet.",
        year, month, day
    ))
}

/// Save and decompress SP3 file
#[cfg(feature = "ephemeris")]
fn save_sp3(bytes: &[u8], cache_file: &PathBuf) -> Result<PathBuf, String> {
    use std::io::Read;

    // Try gzip decompression first
    let content = if bytes.len() > 2 && bytes[0] == 0x1f && bytes[1] == 0x8b {
        let mut decoder = flate2::read::GzDecoder::new(bytes);
        let mut decompressed = Vec::new();
        decoder
            .read_to_end(&mut decompressed)
            .map_err(|e| format!("Failed to decompress SP3: {}", e))?;
        decompressed
    } else {
        // Already uncompressed or different compression
        bytes.to_vec()
    };

    // Verify it looks like SP3
    let header = String::from_utf8_lossy(&content[..content.len().min(100)]);
    if !header.starts_with('#') {
        return Err("Downloaded file does not appear to be SP3 format".to_string());
    }

    std::fs::write(cache_file, &content)
        .map_err(|e| format!("Failed to write SP3 cache file: {}", e))?;

    Ok(cache_file.clone())
}

// ---------------------------------------------------------------------------
// IONEX Ionosphere Map Downloads
// ---------------------------------------------------------------------------

/// CODE IONEX URL
pub const CODE_IONEX_URL: &str = "http://ftp.aiub.unibe.ch/CODE";

/// Generate IONEX filename for CODE products (final format)
/// Format: COD0OPSFIN_{YYYY}{DOY}0000_01D_01H_GIM.INX.gz
pub fn code_ionex_filename(year: i32, doy: u32) -> String {
    format!("COD0OPSFIN_{}{:03}0000_01D_01H_GIM.INX.gz", year, doy)
}

/// Generate IONEX URL from CODE
pub fn code_ionex_url(year: i32, doy: u32) -> String {
    format!("{}/{}/{}", CODE_IONEX_URL, year, code_ionex_filename(year, doy))
}

/// Alternative IONEX filename (legacy format)
/// Format: CODGddd0.YYI.Z
pub fn code_ionex_legacy_filename(year: i32, doy: u32) -> String {
    let yy = year % 100;
    format!("CODG{:03}0.{:02}I.Z", doy, yy)
}

/// Cache path for IONEX files (from day-of-year)
pub fn ionex_cache_path_doy(year: i32, doy: u32) -> PathBuf {
    cache_dir().join(format!("ionex_{}{:03}.ionex", year, doy))
}

/// Cache path for IONEX files (from date)
pub fn ionex_cache_path(year: i32, month: u32, day: u32) -> PathBuf {
    ionex_cache_path_doy(year, day_of_year(year, month, day))
}

/// Check if IONEX is cached (from day-of-year)
pub fn ionex_is_cached_doy(year: i32, doy: u32) -> bool {
    ionex_cache_path_doy(year, doy).exists()
}

/// Check if IONEX is cached (from date)
pub fn ionex_is_cached(year: i32, month: u32, day: u32) -> bool {
    ionex_is_cached_doy(year, day_of_year(year, month, day))
}

/// Download IONEX ionosphere maps from CODE
#[cfg(feature = "ephemeris")]
pub fn fetch_ionex(year: i32, month: u32, day: u32, force_refresh: bool) -> Result<PathBuf, String> {
    let doy = day_of_year(year, month, day);
    let cache_file = ionex_cache_path_doy(year, doy);

    if !force_refresh && cache_file.exists() {
        return Ok(cache_file);
    }

    ensure_cache_dir().map_err(|e| format!("Failed to create cache directory: {}", e))?;

    let client = reqwest::blocking::Client::builder()
        .timeout(std::time::Duration::from_secs(60))
        .build()
        .map_err(|e| format!("Failed to create HTTP client: {}", e))?;

    // Try new IONEX format first (COD0OPSFIN_...)
    let url = code_ionex_url(year, doy);
    match try_download(&client, &url, None) {
        Ok(bytes) => {
            return save_ionex(&bytes, &cache_file);
        }
        Err(e) => {
            eprintln!("CODE IONEX download failed (new format): {}", e);
        }
    }

    // Try legacy format (CODGddd0.YYI.Z)
    let legacy_url = format!(
        "{}/{}/{}",
        CODE_IONEX_URL, year, code_ionex_legacy_filename(year, doy)
    );
    match try_download(&client, &legacy_url, None) {
        Ok(bytes) => {
            return save_ionex(&bytes, &cache_file);
        }
        Err(e) => {
            eprintln!("CODE IONEX download failed (legacy format): {}", e);
        }
    }

    Err(format!(
        "Failed to download IONEX for {}-{:02}-{:02}. File may not be available yet.",
        year, month, day
    ))
}

/// Save and decompress IONEX file
#[cfg(feature = "ephemeris")]
fn save_ionex(bytes: &[u8], cache_file: &PathBuf) -> Result<PathBuf, String> {
    use std::io::Read;

    // Try Unix compress (.Z) format first, then gzip
    let content = if bytes.len() > 2 && bytes[0] == 0x1f && bytes[1] == 0x9d {
        // Unix compress format - not directly supported by flate2
        // For now, return error and suggest manual download
        return Err("Unix compress (.Z) format not supported. Please manually decompress.".to_string());
    } else if bytes.len() > 2 && bytes[0] == 0x1f && bytes[1] == 0x8b {
        // Gzip format
        let mut decoder = flate2::read::GzDecoder::new(bytes);
        let mut decompressed = Vec::new();
        decoder
            .read_to_end(&mut decompressed)
            .map_err(|e| format!("Failed to decompress IONEX: {}", e))?;
        decompressed
    } else {
        // Already uncompressed
        bytes.to_vec()
    };

    // Verify it looks like IONEX (check for version/type header or ION markers)
    let header = String::from_utf8_lossy(&content[..content.len().min(200)]);
    if !header.contains("IONEX") && !header.contains("ION") && !header.contains("IONOSPHERE") && !header.to_ascii_uppercase().contains("TEC") {
        return Err("Downloaded file does not appear to be IONEX format".to_string());
    }

    std::fs::write(cache_file, &content)
        .map_err(|e| format!("Failed to write IONEX cache file: {}", e))?;

    Ok(cache_file.clone())
}

/// List all cached SP3 files
pub fn list_cached_sp3() -> Vec<(i32, u32, PathBuf)> {
    let dir = cache_dir();
    let mut files = Vec::new();

    if let Ok(entries) = std::fs::read_dir(&dir) {
        for entry in entries.flatten() {
            let path = entry.path();
            if let Some(name) = path.file_name().and_then(|n| n.to_str()) {
                if name.starts_with("sp3_") && name.ends_with(".sp3") {
                    // Parse sp3_YYYYDDD.sp3
                    if let Some(year_doy) = name.get(4..11) {
                        if let (Ok(year), Ok(doy)) = (
                            year_doy[0..4].parse::<i32>(),
                            year_doy[4..7].parse::<u32>(),
                        ) {
                            files.push((year, doy, path));
                        }
                    }
                }
            }
        }
    }

    files.sort_by_key(|(y, d, _)| (*y, *d));
    files
}

/// List all cached IONEX files
pub fn list_cached_ionex() -> Vec<(i32, u32, PathBuf)> {
    let dir = cache_dir();
    let mut files = Vec::new();

    if let Ok(entries) = std::fs::read_dir(&dir) {
        for entry in entries.flatten() {
            let path = entry.path();
            if let Some(name) = path.file_name().and_then(|n| n.to_str()) {
                if name.starts_with("ionex_") && name.ends_with(".ionex") {
                    // Parse ionex_YYYYDDD.ionex
                    if let Some(year_doy) = name.get(6..13) {
                        if let (Ok(year), Ok(doy)) = (
                            year_doy[0..4].parse::<i32>(),
                            year_doy[4..7].parse::<u32>(),
                        ) {
                            files.push((year, doy, path));
                        }
                    }
                }
            }
        }
    }

    files.sort_by_key(|(y, d, _)| (*y, *d));
    files
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_day_of_year() {
        assert_eq!(day_of_year(2025, 1, 1), 1);
        assert_eq!(day_of_year(2025, 1, 15), 15);
        assert_eq!(day_of_year(2025, 9, 15), 258);
        assert_eq!(day_of_year(2025, 12, 31), 365);
        // Leap year
        assert_eq!(day_of_year(2024, 3, 1), 61);
        assert_eq!(day_of_year(2024, 12, 31), 366);
    }

    #[test]
    fn test_parse_date() {
        assert_eq!(parse_date("2025-09-15"), Ok((2025, 9, 15)));
        assert_eq!(parse_date("2024-01-01"), Ok((2024, 1, 1)));
        assert!(parse_date("2025-9-15").is_ok()); // Single digit month
        assert!(parse_date("invalid").is_err());
    }

    #[test]
    fn test_cddis_url() {
        let url = cddis_url(2025, 258);
        assert!(url.contains("2025"));
        assert!(url.contains("258"));
        assert!(url.ends_with(".rnx.gz"));
    }

    #[test]
    fn test_cache_path() {
        let path = cache_path(2025, 9, 15);
        assert!(path.to_string_lossy().contains("BRDC00IGS_R_2025258"));
    }

    #[test]
    fn test_gps_time_to_date() {
        // 2025-09-15 20:29:14 UTC -> GPS time ~ 1442003372
        // GPS time = UTC seconds since Jan 6, 1980 + 18 leap seconds
        let gps_time = 1442003372.0;
        let (year, month, day) = gps_time_to_date(gps_time);
        assert_eq!(year, 2025);
        assert_eq!(month, 9);
        assert_eq!(day, 15);
    }
}
