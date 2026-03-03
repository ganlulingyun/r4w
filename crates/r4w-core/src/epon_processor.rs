//! EPON (Ethernet Passive Optical Network) Processor
//!
//! Implements IEEE 802.3ah (1G-EPON) and IEEE 802.3av (10G-EPON) physical and
//! MAC control layer processing. Covers:
//!
//! - **MPCP** (Multi-Point Control Protocol): GATE/REPORT messages, discovery,
//!   grant scheduling, timestamp synchronization.
//! - **LLID** (Logical Link Identifier): 15-bit assignment, broadcast (0x7FFF),
//!   multicast.
//! - **FEC**: RS(255,239) for 1G-EPON, RS(255,223) for 10G-EPON.
//! - **DBA** (Dynamic Bandwidth Allocation): IPACT with limited/gated/fixed
//!   service modes.
//! - **10G-EPON**: 10.3125 Gbps downstream, 1.25/10.3125 Gbps upstream.
//! - **Wavelength plan**: 1490nm DS / 1310nm US (1G); 1577nm DS / 1270nm US (10G).
//! - **ONU discovery**: Discovery GATE, REGISTER_REQ/REGISTER/REGISTER_ACK,
//!   random backoff.
//! - **Burst-mode reception**: CDR level recovery, timing metadata.
//! - **Power budget**: PR10/PR20/PR30/PRX30 classes.
//! - **Traffic shaping**: Per-LLID queuing, priority scheduling, SLA enforcement.
//!
//! # Standards References
//! - IEEE 802.3ah-2004 (1G-EPON)
//! - IEEE 802.3av-2009 (10G-EPON)
//! - ITU-T G.984 (GPON, for context)
//!
//! # Constants
//! - Time quantum (TQ) = 16 ns (per IEEE 802.3ah)
//! - Max EPON frame = 1518 bytes (standard Ethernet + LLID overhead)
//! - Discovery window = 200 TQ (3.2 µs)
//! - Guard time = 32 TQ (512 ns) typical between bursts

// ── Type-quantum helpers ─────────────────────────────────────────────────────
/// MPCP time quantum in nanoseconds (16 ns per IEEE 802.3ah).
pub const TQ_NS: f64 = 16.0;
/// Broadcast LLID used for downstream multicast / control frames.
pub const BROADCAST_LLID: u16 = 0x7FFF;
/// MPCP Ethertype (0x8808 for MAC Control).
pub const ETHERTYPE_MAC_CTRL: u16 = 0x8808;
/// MPCP opcode for GATE message.
pub const MPCP_OPCODE_GATE: u16 = 0x0002;
/// MPCP opcode for REPORT message.
pub const MPCP_OPCODE_REPORT: u16 = 0x0003;
/// MPCP opcode for REGISTER_REQ.
pub const MPCP_OPCODE_REGISTER_REQ: u16 = 0x0004;
/// MPCP opcode for REGISTER.
pub const MPCP_OPCODE_REGISTER: u16 = 0x0005;
/// MPCP opcode for REGISTER_ACK.
pub const MPCP_OPCODE_REGISTER_ACK: u16 = 0x0006;
/// MPCP PDU length is always 64 bytes (Ethernet minimum frame size used here).
pub const MPCP_PDU_LEN: usize = 64;
/// Maximum ONU count per OLT PON port.
pub const MAX_ONU_COUNT: usize = 32;
/// Maximum grants per GATE message (IEEE 802.3ah up to 4).
pub const MAX_GRANTS_PER_GATE: usize = 4;
/// Maximum queue sets per REPORT message (up to 8 per IEEE 802.3ah).
pub const MAX_QUEUE_SETS: usize = 8;
/// Default discovery window in TQ units.
pub const DISCOVERY_WINDOW_TQ: u32 = 200;
/// Guard time in TQ between bursts.
pub const GUARD_TIME_TQ: u32 = 32;
/// RS(255,239) — 1G-EPON: n=255, k=239, t=8
pub const RS_1G_N: usize = 255;
pub const RS_1G_K: usize = 239;
pub const RS_1G_T: usize = 8;
/// RS(255,223) — 10G-EPON downstream: n=255, k=223, t=16
pub const RS_10G_N: usize = 255;
pub const RS_10G_K: usize = 223;
pub const RS_10G_T: usize = 16;
/// GF(2^8) primitive polynomial: x^8 + x^4 + x^3 + x^2 + 1 = 0x11D
pub const GF256_PRIM_POLY: u16 = 0x11D;

// ─────────────────────────────────────────────────────────────────────────────
// Enumerations
// ─────────────────────────────────────────────────────────────────────────────

/// EPON line rate variant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EponRate {
    /// IEEE 802.3ah: 1.25 Gbps DS + 1.25 Gbps US (1G symmetric).
    Rate1G,
    /// IEEE 802.3av: 10.3125 Gbps DS + 1.25 Gbps US (10G asymmetric).
    Rate10GAsym,
    /// IEEE 802.3av: 10.3125 Gbps DS + 10.3125 Gbps US (10G symmetric).
    Rate10GSym,
}

/// DBA (Dynamic Bandwidth Allocation) algorithm variant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DbaAlgorithm {
    /// IPACT: Interleaved Polling with Adaptive Cycle Time (RFC-like scheduling).
    Ipact,
    /// Limited service: grant up to reported queue occupancy.
    Limited,
    /// Gated service: grant exactly reported queue occupancy.
    Gated,
    /// Fixed service: fixed grant regardless of report.
    Fixed,
}

/// Power budget class per IEEE 802.3ah / 802.3av.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PowerBudget {
    /// PR10: 10 dB (short-reach)
    Pr10,
    /// PR20: 20 dB (standard reach, ≤20 km)
    Pr20,
    /// PR30: 30 dB (extended reach, ≤60 km)
    Pr30,
    /// PRX30: 29 dB for 10G-EPON extended reach
    Prx30,
}

/// ONU state in the discovery/registration FSM.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OnuState {
    /// ONU has not yet been discovered.
    Unregistered,
    /// ONU sent REGISTER_REQ, waiting for REGISTER from OLT.
    PendingRegister,
    /// ONU received REGISTER, sent REGISTER_ACK, fully operational.
    Registered,
    /// ONU deregistered or timed-out.
    Deregistered,
}

/// Service priority for traffic shaping.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ServicePriority {
    BestEffort = 0,
    Video = 1,
    Voice = 2,
    Control = 3,
}

// ─────────────────────────────────────────────────────────────────────────────
// Core data structures
// ─────────────────────────────────────────────────────────────────────────────

/// Configuration for an EPON processor instance.
#[derive(Debug, Clone)]
pub struct EponConfig {
    /// EPON line rate.
    pub rate: EponRate,
    /// DBA algorithm used by OLT scheduler.
    pub dba: DbaAlgorithm,
    /// Power budget class.
    pub power_budget: PowerBudget,
    /// Maximum cycle time in TQ (e.g. 125,000 TQ = 2 ms).
    pub max_cycle_tq: u32,
    /// Minimum grant size in TQ.
    pub min_grant_tq: u32,
    /// Whether FEC is enabled.
    pub fec_enabled: bool,
    /// Guard time between ONU bursts in TQ.
    pub guard_time_tq: u32,
    /// Discovery window size in TQ.
    pub discovery_window_tq: u32,
    /// Maximum number of ONUs supported.
    pub max_onus: usize,
}

impl Default for EponConfig {
    fn default() -> Self {
        Self {
            rate: EponRate::Rate1G,
            dba: DbaAlgorithm::Ipact,
            power_budget: PowerBudget::Pr20,
            max_cycle_tq: 125_000, // 2 ms
            min_grant_tq: 64,
            fec_enabled: true,
            guard_time_tq: GUARD_TIME_TQ,
            discovery_window_tq: DISCOVERY_WINDOW_TQ,
            max_onus: MAX_ONU_COUNT,
        }
    }
}

/// A single grant entry inside a GATE message.
#[derive(Debug, Clone, Copy)]
pub struct GrantEntry {
    /// Start time of the grant window in TQ relative to frame start.
    pub start_time_tq: u32,
    /// Length of the grant window in TQ.
    pub length_tq: u32,
}

/// An MPCP GATE message (OLT → ONU).
#[derive(Debug, Clone)]
pub struct MpcpGate {
    /// LLID this GATE targets (0x7FFF = discovery gate / broadcast).
    pub llid: u16,
    /// OLT timestamp at time of transmission (TQ units, wraps at 2^32).
    pub timestamp: u32,
    /// Number of valid grants (0 = poll only, 1..4 = bandwidth grants).
    pub num_grants: u8,
    /// Grants array (only `num_grants` elements are valid).
    pub grants: [GrantEntry; MAX_GRANTS_PER_GATE],
    /// If true this is a discovery GATE (llid == 0x7FFF, grants[0] = discovery window).
    pub is_discovery: bool,
}

/// Per-queue occupancy reported in an REPORT queue set.
#[derive(Debug, Clone, Copy)]
pub struct QueueReport {
    /// Priority queue index (0 = lowest).
    pub queue_id: u8,
    /// Number of bytes pending in this queue (0..=65535).
    pub report_bytes: u16,
}

/// An MPCP REPORT message (ONU → OLT).
#[derive(Debug, Clone)]
pub struct MpcpReport {
    /// LLID of the reporting ONU.
    pub llid: u16,
    /// ONU-local timestamp mirrored from OLT for round-trip measurement.
    pub timestamp: u32,
    /// Number of queue sets in this report.
    pub num_queue_sets: u8,
    /// Queue reports (only `num_queue_sets` entries are valid).
    pub queue_sets: [QueueReport; MAX_QUEUE_SETS],
}

/// An EPON Ethernet frame (preamble modified to carry LLID).
#[derive(Debug, Clone)]
pub struct EponFrame {
    /// Destination MAC address (6 bytes).
    pub dst_mac: [u8; 6],
    /// Source MAC address (6 bytes).
    pub src_mac: [u8; 6],
    /// LLID embedded in bytes 5-6 of the SLD preamble.
    pub llid: u16,
    /// Ethertype / length field.
    pub ethertype: u16,
    /// Payload (0..=1500 bytes for standard MTU).
    pub payload: Vec<u8>,
    /// Frame Check Sequence (CRC-32).
    pub fcs: u32,
}

/// ONU registration record maintained at the OLT.
#[derive(Debug, Clone)]
pub struct OnuRecord {
    /// Logical Link Identifier assigned to this ONU.
    pub llid: u16,
    /// ONU MAC address (used as unique identifier).
    pub mac: [u8; 6],
    /// Current FSM state.
    pub state: OnuState,
    /// Round-trip time in TQ (measured during discovery).
    pub rtt_tq: u32,
    /// Pending bytes reported in the most recent REPORT.
    pub pending_bytes: u32,
    /// Total bytes granted since registration.
    pub total_granted_bytes: u64,
    /// SLA maximum bandwidth in bytes per cycle.
    pub sla_max_bytes: u32,
}

/// A computed grant assignment result from the DBA scheduler.
#[derive(Debug, Clone)]
pub struct GrantAssignment {
    /// ONU LLID receiving the grant.
    pub llid: u16,
    /// Grant start time (TQ, absolute within cycle).
    pub start_tq: u32,
    /// Grant length (TQ).
    pub length_tq: u32,
    /// Bytes this grant can accommodate (accounting for FEC overhead).
    pub usable_bytes: u32,
}

/// Statistics for the EPON processor.
#[derive(Debug, Clone, Default)]
pub struct EponStats {
    /// Total GATE messages sent.
    pub gates_sent: u64,
    /// Total REPORT messages received.
    pub reports_received: u64,
    /// Total grants scheduled.
    pub grants_scheduled: u64,
    /// Total downstream bytes processed.
    pub ds_bytes: u64,
    /// Total upstream bytes processed.
    pub us_bytes: u64,
    /// FEC encode operations.
    pub fec_encodes: u64,
    /// FEC decode operations.
    pub fec_decodes: u64,
    /// FEC corrected errors.
    pub fec_corrected: u64,
    /// FEC uncorrectable frames.
    pub fec_uncorrectable: u64,
    /// Registered ONU count.
    pub registered_onus: u32,
    /// Discovery cycles run.
    pub discovery_cycles: u64,
}

// ─────────────────────────────────────────────────────────────────────────────
// GF(256) arithmetic (for RS codec)
// ─────────────────────────────────────────────────────────────────────────────

/// Pre-computed GF(256) log and anti-log tables using primitive polynomial 0x11D.
struct Gf256Tables {
    exp: [u8; 512], // anti-log: exp[i] = alpha^i
    log: [u8; 256], // log[x] = i such that alpha^i = x; log[0] = 0 (unused)
}

impl Gf256Tables {
    fn new() -> Self {
        let mut exp = [0u8; 512];
        let mut log = [0u8; 256];
        let mut x: u16 = 1;
        for i in 0..255usize {
            exp[i] = x as u8;
            log[x as usize] = i as u8;
            x <<= 1;
            if x & 0x100 != 0 {
                x ^= GF256_PRIM_POLY;
            }
        }
        // duplicate the table to avoid modulo
        for i in 0..255usize {
            exp[i + 255] = exp[i];
        }
        exp[510] = exp[0];
        Gf256Tables { exp, log }
    }

    #[inline]
    fn mul(&self, a: u8, b: u8) -> u8 {
        if a == 0 || b == 0 {
            return 0;
        }
        let la = self.log[a as usize] as usize;
        let lb = self.log[b as usize] as usize;
        self.exp[la + lb]
    }

    #[inline]
    fn div(&self, a: u8, b: u8) -> u8 {
        if a == 0 {
            return 0;
        }
        debug_assert!(b != 0, "division by zero in GF(256)");
        let la = self.log[a as usize] as usize;
        let lb = self.log[b as usize] as usize;
        // (la - lb + 255) mod 255
        self.exp[(la + 255 - lb) % 255]
    }

    #[inline]
    fn pow(&self, a: u8, n: usize) -> u8 {
        if n == 0 {
            return 1;
        }
        if a == 0 {
            return 0;
        }
        let la = self.log[a as usize] as usize;
        self.exp[(la * n) % 255]
    }

    /// Evaluate polynomial p at x using Horner's method.
    fn poly_eval(&self, p: &[u8], x: u8) -> u8 {
        let mut result = 0u8;
        for &coeff in p.iter() {
            result = self.mul(result, x) ^ coeff;
        }
        result
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Reed-Solomon codec (systematic, GF(2^8), generator poly from IEEE 802.3)
// ─────────────────────────────────────────────────────────────────────────────

/// Reed-Solomon encoder state for a given (n, k) configuration.
struct RsEncoder {
    n: usize,
    k: usize,
    t: usize,
    /// Generator polynomial coefficients g[0..=2t].
    /// g[0] = coefficient of x^0, g[2t] = 1 (monic, leading term implicit).
    /// Stored low-to-high: index 0 = degree 0, index 2t = degree 2t.
    gen_poly: Vec<u8>,
    gf: Gf256Tables,
}

impl RsEncoder {
    fn new(n: usize, k: usize) -> Self {
        let t = (n - k) / 2;
        let gf = Gf256Tables::new();
        // Build generator polynomial: g(x) = prod_{i=1}^{2t}(x - alpha^i)
        // In GF(2^m), subtraction = addition = XOR, so (x - alpha^i) = (x + alpha^i).
        // Roots are alpha^1, alpha^2, ..., alpha^(2t) — matching syndrome computation.
        // Stored low-to-high degree; index 0 = degree 0 coefficient.
        let mut gen = vec![1u8]; // g(x) = 1 initially
        for i in 1..=(2 * t) {
            let alpha_i = gf.exp[i % 255]; // alpha^i
            // multiply gen by (x + alpha^i) → shift and XOR
            let prev_len = gen.len();
            let mut new_gen = vec![0u8; prev_len + 1];
            for j in 0..prev_len {
                // coefficient of x^(j+1) from x term
                new_gen[j + 1] ^= gen[j];
                // coefficient of x^j from alpha^i constant term
                new_gen[j] ^= gf.mul(gen[j], alpha_i);
            }
            gen = new_gen;
        }
        // gen is now degree-2t stored low-to-high; gen[2t] should be 1
        RsEncoder { n, k, t, gen_poly: gen, gf }
    }

    /// Encode `data` (must be exactly `k` bytes). Returns `n` bytes (systematic).
    /// Codeword layout: [data_0 .. data_{k-1}, parity_{nsym-1} .. parity_0].
    fn encode(&self, data: &[u8]) -> Vec<u8> {
        assert_eq!(data.len(), self.k);
        let nsym = self.n - self.k;

        // Standard LFSR-based systematic encoding (same as reed_solomon.rs).
        // gen_poly stored ascending: gen_poly[j] = coefficient of x^j, gen_poly[nsym] = 1 (monic).
        // `feedback` is indexed [0..nsym-1], feedback[nsym-1] is the "top" cell (highest degree).
        let mut feedback = vec![0u8; nsym];
        for i in 0..self.k {
            let d = data[i] ^ feedback[nsym - 1];
            // Shift down and feed back
            for j in (1..nsym).rev() {
                feedback[j] = feedback[j - 1] ^ self.gf.mul(d, self.gen_poly[j]);
            }
            feedback[0] = self.gf.mul(d, self.gen_poly[0]);
        }

        // Systematic codeword: data bytes then parity bytes (feedback[nsym-1] first)
        let mut codeword = Vec::with_capacity(self.n);
        codeword.extend_from_slice(data);
        for j in (0..nsym).rev() {
            codeword.push(feedback[j]);
        }
        codeword
    }
}

/// Reed-Solomon decoder. Corrects up to `t` symbol errors.
struct RsDecoder {
    n: usize,
    k: usize,
    t: usize,
    gf: Gf256Tables,
}

impl RsDecoder {
    fn new(n: usize, k: usize) -> Self {
        let t = (n - k) / 2;
        RsDecoder { n, k, t, gf: Gf256Tables::new() }
    }

    /// Evaluate polynomial stored ascending (p[0]=x^0 coefficient) at x.
    fn poly_eval_asc(&self, p: &[u8], x: u8) -> u8 {
        // Horner from highest degree down to x^0
        let mut acc = 0u8;
        for &c in p.iter().rev() {
            acc = self.gf.mul(acc, x) ^ c;
        }
        acc
    }

    /// Formal derivative in GF(2^m): sigma'[i-1] = sigma[i] for odd i; even terms vanish.
    fn poly_deriv_asc(p: &[u8]) -> Vec<u8> {
        if p.len() <= 1 {
            return vec![0];
        }
        let mut d = Vec::with_capacity(p.len() - 1);
        for i in 1..p.len() {
            d.push(if i & 1 == 1 { p[i] } else { 0 });
        }
        while d.len() > 1 && *d.last().unwrap() == 0 {
            d.pop();
        }
        d
    }

    /// Compute `nsym = 2t` syndromes S_j = r(alpha^j) for j = 1..=nsym.
    /// Codeword `r` is stored high-degree-first: r[0]*x^(n-1) + ... + r[n-1]*x^0.
    fn syndromes(&self, r: &[u8]) -> Vec<u8> {
        let nsym = 2 * self.t;
        (1..=nsym)
            .map(|j| {
                let a = self.gf.exp[j % 255];
                let mut val = 0u8;
                for &ri in r.iter() {
                    val = self.gf.mul(val, a) ^ ri;
                }
                val
            })
            .collect()
    }

    /// Berlekamp-Massey algorithm.
    /// Returns sigma(x) in ascending order: sigma[0]=1.
    fn berlekamp_massey(&self, synd: &[u8]) -> Vec<u8> {
        let nsym = synd.len();
        let mut c_poly = vec![1u8]; // current error locator
        let mut b_poly = vec![1u8]; // previous best
        let mut l: usize = 0;
        let mut delta_b: u8 = 1;
        let mut m: usize = 1;

        for step in 0..nsym {
            // Discrepancy
            let mut delta: u8 = synd[step];
            for i in 1..c_poly.len() {
                if step >= i {
                    delta ^= self.gf.mul(c_poly[i], synd[step - i]);
                }
            }

            if delta == 0 {
                m += 1;
            } else if 2 * l <= step {
                let factor = self.gf.div(delta, delta_b);
                // new_c = c + factor * x^m * b
                let mut xm_b = vec![0u8; m];
                for &bi in b_poly.iter() {
                    xm_b.push(self.gf.mul(factor, bi));
                }
                let new_len = c_poly.len().max(xm_b.len());
                let mut t_poly = vec![0u8; new_len];
                for (i, &v) in c_poly.iter().enumerate() {
                    t_poly[i] ^= v;
                }
                for (i, &v) in xm_b.iter().enumerate() {
                    t_poly[i] ^= v;
                }
                b_poly = c_poly;
                c_poly = t_poly;
                l = step + 1 - l;
                delta_b = delta;
                m = 1;
            } else {
                let factor = self.gf.div(delta, delta_b);
                let mut xm_b = vec![0u8; m];
                for &bi in b_poly.iter() {
                    xm_b.push(self.gf.mul(factor, bi));
                }
                let new_len = c_poly.len().max(xm_b.len());
                let mut new_c = vec![0u8; new_len];
                for (i, &v) in c_poly.iter().enumerate() {
                    new_c[i] ^= v;
                }
                for (i, &v) in xm_b.iter().enumerate() {
                    new_c[i] ^= v;
                }
                c_poly = new_c;
                m += 1;
            }
        }
        // Trim trailing zeros
        while c_poly.len() > 1 && *c_poly.last().unwrap() == 0 {
            c_poly.pop();
        }
        c_poly
    }

    /// Chien search: find array positions with errors.
    ///
    /// For codeword stored as r[0]*x^(n-1) + ... + r[n-1]*x^0,
    /// array position `pos` corresponds to power index `n-1-pos`.
    /// X = alpha^(n-1-pos), so X^{-1} = alpha^(255 - (n-1-pos)) = alpha^(255 - n + 1 + pos).
    fn chien_search(&self, sigma: &[u8]) -> (Vec<usize>, Vec<u8>) {
        let mut positions = Vec::new();
        let mut x_invs = Vec::new();
        for pos in 0..self.n {
            let x_inv = self.gf.exp[(255 + pos + 255 - self.n + 1) % 255];
            if self.poly_eval_asc(sigma, x_inv) == 0 {
                positions.push(pos);
                x_invs.push(x_inv);
            }
        }
        (positions, x_invs)
    }

    /// Decode received codeword (n bytes). Returns corrected k-byte data or error.
    fn decode(&self, received: &[u8]) -> Result<(Vec<u8>, usize), &'static str> {
        if received.len() != self.n {
            return Err("invalid codeword length");
        }
        let synd = self.syndromes(received);
        // Check if all syndromes are zero (no errors)
        if synd.iter().all(|&x| x == 0) {
            return Ok((received[..self.k].to_vec(), 0));
        }
        let sigma = self.berlekamp_massey(&synd);
        let num_errors = sigma.len() - 1; // degree of sigma
        if num_errors > self.t {
            return Err("too many errors, uncorrectable");
        }
        let (positions, x_invs) = self.chien_search(&sigma);
        if positions.len() != num_errors {
            return Err("chien search found wrong number of roots");
        }

        // Forney algorithm: FCR = 1, so e_j = Omega(X_j^{-1}) / Sigma'(X_j^{-1}).
        // Omega(x) = S(x) * sigma(x) mod x^nsym, where S is ascending: S[i] = synd[i]
        let nsym = 2 * self.t;
        let mut omega_full = vec![0u8; nsym + sigma.len()];
        for (i, &si) in synd.iter().enumerate() {
            for (j, &sj) in sigma.iter().enumerate() {
                omega_full[i + j] ^= self.gf.mul(si, sj);
            }
        }
        let omega = &omega_full[..nsym];
        let sigma_prime = Self::poly_deriv_asc(&sigma);

        let mut corrected = received.to_vec();
        for (idx, &pos) in positions.iter().enumerate() {
            let x_inv = x_invs[idx];
            let omega_val = self.poly_eval_asc(omega, x_inv);
            let sp_val = self.poly_eval_asc(&sigma_prime, x_inv);
            if sp_val == 0 {
                return Err("Forney: zero sigma_prime, uncorrectable");
            }
            let magnitude = self.gf.div(omega_val, sp_val);
            corrected[pos] ^= magnitude;
        }

        // Verify correction
        let s2 = self.syndromes(&corrected);
        if !s2.iter().all(|&x| x == 0) {
            return Err("correction failed, uncorrectable");
        }
        Ok((corrected[..self.k].to_vec(), positions.len()))
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// CRC-32 for Ethernet FCS
// ─────────────────────────────────────────────────────────────────────────────

fn crc32_table() -> [u32; 256] {
    let mut table = [0u32; 256];
    for i in 0..256u32 {
        let mut crc = i;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
        table[i as usize] = crc;
    }
    table
}

fn compute_crc32(data: &[u8]) -> u32 {
    let table = crc32_table();
    let mut crc = 0xFFFF_FFFFu32;
    for &b in data {
        let idx = ((crc ^ b as u32) & 0xFF) as usize;
        crc = (crc >> 8) ^ table[idx];
    }
    crc ^ 0xFFFF_FFFF
}

// ─────────────────────────────────────────────────────────────────────────────
// Timestamp arithmetic
// ─────────────────────────────────────────────────────────────────────────────

/// Compute round-trip time in TQ from OLT transmit timestamp, ONU echo timestamp,
/// and OLT receive timestamp. Handles 32-bit wraparound.
pub fn compute_rtt_tq(tx_ts: u32, echo_ts: u32, rx_ts: u32) -> u32 {
    let one_way = rx_ts.wrapping_sub(tx_ts);
    let onu_proc = echo_ts.wrapping_sub(tx_ts);
    one_way.wrapping_sub(onu_proc)
}

/// Convert TQ count to nanoseconds.
pub fn tq_to_ns(tq: u32) -> f64 {
    tq as f64 * TQ_NS
}

/// Convert nanoseconds to TQ (round up).
pub fn ns_to_tq(ns: f64) -> u32 {
    (ns / TQ_NS).ceil() as u32
}

/// Convert TQ to bytes at the given line rate.
/// At 1G: 1 byte = 8 ns = 0.5 TQ → bytes = tq * 2 / 1 (at 1G = 1.25 Gbps nominal)
/// More precisely: 1G Ethernet byte period = 8 ns; 1 TQ = 16 ns → 2 bytes/TQ.
pub fn tq_to_bytes(tq: u32, rate: EponRate) -> u32 {
    match rate {
        EponRate::Rate1G => tq * 2,
        EponRate::Rate10GAsym | EponRate::Rate10GSym => tq * 16, // 10G: ~1.28 Gbps
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// MPCP frame builder / parser
// ─────────────────────────────────────────────────────────────────────────────

/// Build a raw MPCP GATE PDU (64 bytes on wire before FCS).
/// Layout per IEEE 802.3ah Table 64-1:
///  0-5  : Dst MAC (01:80:C2:00:00:01 for MPCP multicast)
///  6-11 : Src MAC (OLT MAC)
///  12-13: Ethertype 0x8808
///  14-15: Opcode 0x0002 (GATE)
///  16-19: Timestamp (BE)
///  20   : NumGrants
///  21-28: Grant 0 (startTime 4B + length 2B + ... actually 4B+2B per grant)
///  ...
///  padding to 60 bytes then 4-byte FCS appended separately.
pub fn build_gate_pdu(
    olt_mac: &[u8; 6],
    llid: u16,
    timestamp: u32,
    grants: &[GrantEntry],
) -> Vec<u8> {
    assert!(grants.len() <= MAX_GRANTS_PER_GATE);
    let mut pdu = vec![0u8; 60]; // 60 bytes + 4-byte FCS = 64
    // Destination: MPCP multicast 01:80:C2:00:00:01
    pdu[0..6].copy_from_slice(&[0x01, 0x80, 0xC2, 0x00, 0x00, 0x01]);
    // Source: OLT MAC
    pdu[6..12].copy_from_slice(olt_mac);
    // Ethertype
    pdu[12] = (ETHERTYPE_MAC_CTRL >> 8) as u8;
    pdu[13] = (ETHERTYPE_MAC_CTRL & 0xFF) as u8;
    // Opcode
    pdu[14] = (MPCP_OPCODE_GATE >> 8) as u8;
    pdu[15] = (MPCP_OPCODE_GATE & 0xFF) as u8;
    // Timestamp (4 bytes, BE)
    pdu[16] = (timestamp >> 24) as u8;
    pdu[17] = (timestamp >> 16) as u8;
    pdu[18] = (timestamp >> 8) as u8;
    pdu[19] = timestamp as u8;
    // NumGrants
    pdu[20] = grants.len() as u8;
    // Grants: each 6 bytes (startTime 4B + length 2B)
    let mut off = 21;
    for g in grants {
        if off + 6 > 60 {
            break;
        }
        pdu[off] = (g.start_time_tq >> 24) as u8;
        pdu[off + 1] = (g.start_time_tq >> 16) as u8;
        pdu[off + 2] = (g.start_time_tq >> 8) as u8;
        pdu[off + 3] = g.start_time_tq as u8;
        pdu[off + 4] = (g.length_tq >> 8) as u8;
        pdu[off + 5] = g.length_tq as u8;
        off += 6;
    }
    // Embed LLID in preamble conceptually; store in reserved bytes 40-41
    pdu[40] = (llid >> 8) as u8;
    pdu[41] = llid as u8;
    // Append FCS
    let fcs = compute_crc32(&pdu);
    pdu.push((fcs) as u8);
    pdu.push((fcs >> 8) as u8);
    pdu.push((fcs >> 16) as u8);
    pdu.push((fcs >> 24) as u8);
    pdu
}

/// Parse a raw MPCP GATE PDU (64 bytes).
pub fn parse_gate_pdu(data: &[u8]) -> Result<MpcpGate, &'static str> {
    if data.len() < 64 {
        return Err("gate pdu too short");
    }
    // Verify FCS
    let body = &data[..60];
    let fcs_rx = u32::from_le_bytes([data[60], data[61], data[62], data[63]]);
    let fcs_calc = compute_crc32(body);
    if fcs_rx != fcs_calc {
        return Err("gate pdu fcs mismatch");
    }
    // Verify ethertype
    let et = (data[12] as u16) << 8 | data[13] as u16;
    if et != ETHERTYPE_MAC_CTRL {
        return Err("not a MAC control frame");
    }
    let opcode = (data[14] as u16) << 8 | data[15] as u16;
    if opcode != MPCP_OPCODE_GATE {
        return Err("not a GATE opcode");
    }
    let timestamp = (data[16] as u32) << 24
        | (data[17] as u32) << 16
        | (data[18] as u32) << 8
        | data[19] as u32;
    let num_grants = data[20] as usize;
    if num_grants > MAX_GRANTS_PER_GATE {
        return Err("too many grants in gate");
    }
    let llid = (data[40] as u16) << 8 | data[41] as u16;
    let is_discovery = llid == BROADCAST_LLID || num_grants == 0;
    let mut grants = [GrantEntry { start_time_tq: 0, length_tq: 0 }; MAX_GRANTS_PER_GATE];
    let mut off = 21;
    for i in 0..num_grants {
        if off + 6 > 60 {
            break;
        }
        let st = (data[off] as u32) << 24
            | (data[off + 1] as u32) << 16
            | (data[off + 2] as u32) << 8
            | data[off + 3] as u32;
        let len = (data[off + 4] as u16) << 8 | data[off + 5] as u16;
        grants[i] = GrantEntry { start_time_tq: st, length_tq: len as u32 };
        off += 6;
    }
    Ok(MpcpGate {
        llid,
        timestamp,
        num_grants: num_grants as u8,
        grants,
        is_discovery,
    })
}

/// Build a raw MPCP REPORT PDU (64 bytes).
pub fn build_report_pdu(
    onu_mac: &[u8; 6],
    llid: u16,
    timestamp: u32,
    queue_sets: &[QueueReport],
) -> Vec<u8> {
    assert!(queue_sets.len() <= MAX_QUEUE_SETS);
    let mut pdu = vec![0u8; 60];
    // Dst: MPCP multicast
    pdu[0..6].copy_from_slice(&[0x01, 0x80, 0xC2, 0x00, 0x00, 0x01]);
    pdu[6..12].copy_from_slice(onu_mac);
    pdu[12] = (ETHERTYPE_MAC_CTRL >> 8) as u8;
    pdu[13] = (ETHERTYPE_MAC_CTRL & 0xFF) as u8;
    pdu[14] = (MPCP_OPCODE_REPORT >> 8) as u8;
    pdu[15] = (MPCP_OPCODE_REPORT & 0xFF) as u8;
    pdu[16] = (timestamp >> 24) as u8;
    pdu[17] = (timestamp >> 16) as u8;
    pdu[18] = (timestamp >> 8) as u8;
    pdu[19] = timestamp as u8;
    // NumQueueSets
    pdu[20] = queue_sets.len() as u8;
    let mut off = 21;
    for qs in queue_sets {
        if off + 3 > 60 {
            break;
        }
        pdu[off] = qs.queue_id;
        pdu[off + 1] = (qs.report_bytes >> 8) as u8;
        pdu[off + 2] = qs.report_bytes as u8;
        off += 3;
    }
    // LLID in reserved bytes
    pdu[40] = (llid >> 8) as u8;
    pdu[41] = llid as u8;
    let fcs = compute_crc32(&pdu);
    pdu.push(fcs as u8);
    pdu.push((fcs >> 8) as u8);
    pdu.push((fcs >> 16) as u8);
    pdu.push((fcs >> 24) as u8);
    pdu
}

/// Parse a raw MPCP REPORT PDU.
pub fn parse_report_pdu(data: &[u8]) -> Result<MpcpReport, &'static str> {
    if data.len() < 64 {
        return Err("report pdu too short");
    }
    let body = &data[..60];
    let fcs_rx = u32::from_le_bytes([data[60], data[61], data[62], data[63]]);
    if fcs_rx != compute_crc32(body) {
        return Err("report pdu fcs mismatch");
    }
    let opcode = (data[14] as u16) << 8 | data[15] as u16;
    if opcode != MPCP_OPCODE_REPORT {
        return Err("not a REPORT opcode");
    }
    let timestamp = (data[16] as u32) << 24
        | (data[17] as u32) << 16
        | (data[18] as u32) << 8
        | data[19] as u32;
    let num_qs = data[20] as usize;
    if num_qs > MAX_QUEUE_SETS {
        return Err("too many queue sets");
    }
    let llid = (data[40] as u16) << 8 | data[41] as u16;
    let mut queue_sets = [QueueReport { queue_id: 0, report_bytes: 0 }; MAX_QUEUE_SETS];
    let mut off = 21;
    for i in 0..num_qs {
        if off + 3 > 60 {
            break;
        }
        queue_sets[i] = QueueReport {
            queue_id: data[off],
            report_bytes: (data[off + 1] as u16) << 8 | data[off + 2] as u16,
        };
        off += 3;
    }
    Ok(MpcpReport {
        llid,
        timestamp,
        num_queue_sets: num_qs as u8,
        queue_sets,
    })
}

// ─────────────────────────────────────────────────────────────────────────────
// Ethernet frame encapsulation / decapsulation
// ─────────────────────────────────────────────────────────────────────────────

/// Build an EPON Ethernet frame byte stream including modified preamble with LLID.
/// Preamble bytes 5-6 carry LLID (per IEEE 802.3ah §65.1.3.2).
pub fn build_epon_frame(frame: &EponFrame) -> Vec<u8> {
    let mut buf = Vec::new();
    // Modified preamble (8 bytes): 0x55 0x55 0x55 0x55 0x55 <LLID_HI> <LLID_LO> 0xD5
    buf.push(0x55);
    buf.push(0x55);
    buf.push(0x55);
    buf.push(0x55);
    buf.push(0x55);
    buf.push((frame.llid >> 8) as u8);
    buf.push(frame.llid as u8);
    buf.push(0xD5); // SFD
    // Dst MAC
    buf.extend_from_slice(&frame.dst_mac);
    // Src MAC
    buf.extend_from_slice(&frame.src_mac);
    // Ethertype
    buf.push((frame.ethertype >> 8) as u8);
    buf.push(frame.ethertype as u8);
    // Payload
    buf.extend_from_slice(&frame.payload);
    // Pad to minimum 46 bytes payload if needed
    let payload_min = 46usize;
    if frame.payload.len() < payload_min {
        buf.extend(std::iter::repeat(0u8).take(payload_min - frame.payload.len()));
    }
    // FCS (CRC-32 over dst_mac..payload)
    let fcs_start = 8; // after preamble
    let fcs = compute_crc32(&buf[fcs_start..]);
    buf.push(fcs as u8);
    buf.push((fcs >> 8) as u8);
    buf.push((fcs >> 16) as u8);
    buf.push((fcs >> 24) as u8);
    buf
}

/// Parse an EPON frame from a byte slice. Returns the EponFrame or an error.
pub fn parse_epon_frame(data: &[u8]) -> Result<EponFrame, &'static str> {
    if data.len() < 8 + 14 + 4 {
        return Err("epon frame too short");
    }
    // Extract LLID from preamble bytes 5-6
    let llid = (data[5] as u16) << 8 | data[6] as u16;
    // Verify SFD
    if data[7] != 0xD5 {
        return Err("invalid SFD");
    }
    let off = 8; // after preamble
    let mut dst_mac = [0u8; 6];
    dst_mac.copy_from_slice(&data[off..off + 6]);
    let mut src_mac = [0u8; 6];
    src_mac.copy_from_slice(&data[off + 6..off + 12]);
    let ethertype = (data[off + 12] as u16) << 8 | data[off + 13] as u16;
    let payload_end = data.len() - 4;
    if payload_end <= off + 14 {
        return Err("no payload");
    }
    let payload = data[off + 14..payload_end].to_vec();
    let fcs_rx = u32::from_le_bytes([
        data[payload_end],
        data[payload_end + 1],
        data[payload_end + 2],
        data[payload_end + 3],
    ]);
    let fcs_calc = compute_crc32(&data[off..payload_end]);
    if fcs_rx != fcs_calc {
        return Err("epon frame fcs mismatch");
    }
    Ok(EponFrame {
        dst_mac,
        src_mac,
        llid,
        ethertype,
        payload,
        fcs: fcs_rx,
    })
}

// ─────────────────────────────────────────────────────────────────────────────
// Link budget
// ─────────────────────────────────────────────────────────────────────────────

/// Power budget parameters per class.
fn budget_params(class: PowerBudget) -> (f64, f64, f64) {
    // (OLT_tx_power_dBm, ONU_rx_sensitivity_dBm, max_split_loss_dB)
    match class {
        PowerBudget::Pr10 => (2.0, -8.0, 10.0),
        PowerBudget::Pr20 => (4.0, -24.0, 20.0),
        PowerBudget::Pr30 => (7.0, -27.0, 30.0),
        PowerBudget::Prx30 => (6.0, -28.0, 29.0),
    }
}

/// Calculate the available optical loss budget in dB for a given power budget class.
pub fn calculate_link_budget(budget_class: PowerBudget) -> f64 {
    let (tx, rx, _) = budget_params(budget_class);
    tx - rx
}

/// Estimate maximum fiber reach (km) for a given budget and fiber attenuation.
pub fn estimate_reach_km(budget_class: PowerBudget, attenuation_db_per_km: f64) -> f64 {
    let (tx, rx, _) = budget_params(budget_class);
    let budget = tx - rx;
    // Typical connector/splitter loss ~3 dB
    let margin = budget - 3.0;
    margin / attenuation_db_per_km
}

/// Wavelength plan constants (nm).
pub struct WavelengthPlan {
    pub ds_wavelength_nm: f64,
    pub us_wavelength_nm: f64,
}

/// Get wavelength plan for rate variant.
pub fn wavelength_plan(rate: EponRate) -> WavelengthPlan {
    match rate {
        EponRate::Rate1G => WavelengthPlan {
            ds_wavelength_nm: 1490.0,
            us_wavelength_nm: 1310.0,
        },
        EponRate::Rate10GAsym | EponRate::Rate10GSym => WavelengthPlan {
            ds_wavelength_nm: 1577.0,
            us_wavelength_nm: 1270.0,
        },
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// LLID management
// ─────────────────────────────────────────────────────────────────────────────

/// LLID pool manager: allocates 15-bit LLIDs to ONUs.
#[derive(Debug, Clone)]
pub struct LlidPool {
    next_llid: u16,
    max_llid: u16,
    in_use: Vec<bool>,
}

impl LlidPool {
    pub fn new(max_onus: usize) -> Self {
        let max_onus = max_onus.min(0x7FFE) as u16; // BROADCAST_LLID reserved
        LlidPool {
            next_llid: 1,
            max_llid: max_onus,
            in_use: vec![false; max_onus as usize + 1],
        }
    }

    /// Allocate a new LLID. Returns None if pool exhausted.
    pub fn allocate(&mut self) -> Option<u16> {
        for _ in 0..self.max_llid {
            let llid = self.next_llid;
            self.next_llid = (self.next_llid % self.max_llid) + 1;
            if !self.in_use[llid as usize] {
                self.in_use[llid as usize] = true;
                return Some(llid);
            }
        }
        None
    }

    /// Release an LLID back to the pool.
    pub fn release(&mut self, llid: u16) {
        if (llid as usize) < self.in_use.len() {
            self.in_use[llid as usize] = false;
        }
    }

    /// Check if an LLID is in use.
    pub fn is_in_use(&self, llid: u16) -> bool {
        (llid as usize) < self.in_use.len() && self.in_use[llid as usize]
    }

    /// Count allocated LLIDs.
    pub fn allocated_count(&self) -> usize {
        self.in_use.iter().filter(|&&x| x).count()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// IPACT DBA scheduler
// ─────────────────────────────────────────────────────────────────────────────

/// IPACT (Interleaved Polling with Adaptive Cycle Time) scheduler.
/// Implements limited service: each ONU is granted up to its reported queue depth
/// subject to max_grant_tq and SLA constraints.
#[derive(Debug, Clone)]
pub struct IpactScheduler {
    config: EponConfig,
    /// ONU records in round-robin order.
    onus: Vec<OnuRecord>,
    /// Current cycle timestamp in TQ.
    current_cycle_tq: u32,
}

impl IpactScheduler {
    pub fn new(config: EponConfig) -> Self {
        IpactScheduler {
            config,
            onus: Vec::new(),
            current_cycle_tq: 0,
        }
    }

    /// Register an ONU with the scheduler.
    pub fn register_onu(&mut self, record: OnuRecord) {
        self.onus.push(record);
    }

    /// Update pending bytes for an ONU from a received REPORT.
    pub fn update_report(&mut self, llid: u16, pending_bytes: u32) {
        for onu in self.onus.iter_mut() {
            if onu.llid == llid {
                onu.pending_bytes = pending_bytes;
                return;
            }
        }
    }

    /// Run one IPACT scheduling cycle. Returns the computed grant assignments.
    pub fn schedule_cycle(&mut self, reports: &[MpcpReport]) -> Vec<GrantAssignment> {
        // Update pending bytes from fresh reports
        for report in reports {
            let total_pending: u32 = (0..report.num_queue_sets as usize)
                .map(|i| report.queue_sets[i].report_bytes as u32)
                .sum();
            self.update_report(report.llid, total_pending);
        }

        let mut grants = Vec::new();
        let mut cursor_tq = self.current_cycle_tq;
        // Capture length before mutable borrow to avoid borrow conflict
        let num_onus = self.onus.iter().filter(|o| o.state == OnuState::Registered).count().max(1) as u32;
        let config = self.config.clone();

        for onu in self.onus.iter_mut() {
            if onu.state != OnuState::Registered {
                continue;
            }
            let rtt_overhead_tq = onu.rtt_tq + config.guard_time_tq;
            cursor_tq = cursor_tq.wrapping_add(rtt_overhead_tq);

            // Compute requested TQ from pending bytes
            let requested_tq = match config.rate {
                EponRate::Rate1G => onu.pending_bytes / 2 + 1, // 2 bytes/TQ at 1G
                _ => onu.pending_bytes / 16 + 1,
            };

            // Apply DBA algorithm
            let grant_tq = match config.dba {
                DbaAlgorithm::Fixed => config.max_cycle_tq / num_onus,
                DbaAlgorithm::Limited => {
                    requested_tq.min(onu.sla_max_bytes / 2).max(config.min_grant_tq)
                }
                DbaAlgorithm::Gated | DbaAlgorithm::Ipact => {
                    requested_tq.max(config.min_grant_tq)
                }
            };

            let grant_tq = grant_tq.min(config.max_cycle_tq / 2);

            // Calculate usable bytes (accounting for FEC overhead if enabled)
            let usable_bytes = if config.fec_enabled {
                let raw_bytes = tq_to_bytes(grant_tq, config.rate);
                // FEC overhead: RS(255,239) → ratio = 239/255
                (raw_bytes as f64 * (RS_1G_K as f64 / RS_1G_N as f64)) as u32
            } else {
                tq_to_bytes(grant_tq, config.rate)
            };

            grants.push(GrantAssignment {
                llid: onu.llid,
                start_tq: cursor_tq,
                length_tq: grant_tq,
                usable_bytes,
            });

            onu.total_granted_bytes += usable_bytes as u64;
            cursor_tq = cursor_tq.wrapping_add(grant_tq);
        }

        self.current_cycle_tq = cursor_tq;
        grants
    }

    /// Deregister an ONU.
    pub fn deregister_onu(&mut self, llid: u16) {
        self.onus.retain(|o| o.llid != llid);
    }

    /// Count registered ONUs.
    pub fn registered_count(&self) -> usize {
        self.onus.iter().filter(|o| o.state == OnuState::Registered).count()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Burst-mode reception metadata
// ─────────────────────────────────────────────────────────────────────────────

/// Burst-mode reception metadata recorded at the OLT receiver per burst.
#[derive(Debug, Clone)]
pub struct BurstModeMetadata {
    /// LLID of the transmitting ONU.
    pub llid: u16,
    /// Arrival time in TQ (OLT local clock).
    pub arrival_tq: u32,
    /// Estimated clock frequency offset (ppm).
    pub clock_offset_ppm: f64,
    /// AGC estimated input level (dBm).
    pub input_level_dbm: f64,
    /// CDR lock acquired: true if data recovery was successful.
    pub cdr_locked: bool,
    /// Received burst length in bytes.
    pub burst_length_bytes: u32,
}

impl BurstModeMetadata {
    /// Create a new burst metadata record.
    pub fn new(llid: u16, arrival_tq: u32) -> Self {
        BurstModeMetadata {
            llid,
            arrival_tq,
            clock_offset_ppm: 0.0,
            input_level_dbm: -20.0,
            cdr_locked: false,
            burst_length_bytes: 0,
        }
    }

    /// Level recovery: estimate input power from burst preamble amplitude.
    /// `preamble_amplitude` is normalized to [0.0, 1.0].
    pub fn estimate_level_dbm(&mut self, preamble_amplitude: f64) {
        if preamble_amplitude > 0.0 {
            self.input_level_dbm = 20.0 * preamble_amplitude.log10() - 3.0;
            self.cdr_locked = preamble_amplitude > 0.1;
        }
    }

    /// CDR lock check: returns true if clock recovery is stable.
    pub fn check_cdr_lock(&self) -> bool {
        self.cdr_locked && self.clock_offset_ppm.abs() < 100.0
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Per-LLID traffic shaper / priority queuing
// ─────────────────────────────────────────────────────────────────────────────

/// Per-LLID traffic shaper with 4 priority queues.
#[derive(Debug, Clone)]
pub struct LlidShaper {
    pub llid: u16,
    /// Priority queues: index 0 = BestEffort, 3 = Control.
    queues: [Vec<Vec<u8>>; 4],
    /// Maximum bytes per queue (for backpressure).
    max_queue_bytes: usize,
    /// Current byte count per queue.
    queue_bytes: [usize; 4],
    /// Token bucket: tokens available (bytes).
    tokens: f64,
    /// Token replenishment rate in bytes/TQ.
    rate_bytes_per_tq: f64,
    /// Peak burst size in bytes.
    bucket_size: f64,
}

impl LlidShaper {
    pub fn new(llid: u16, rate_bytes_per_tq: f64, bucket_size_bytes: usize) -> Self {
        LlidShaper {
            llid,
            queues: [Vec::new(), Vec::new(), Vec::new(), Vec::new()],
            max_queue_bytes: 65536,
            queue_bytes: [0; 4],
            tokens: bucket_size_bytes as f64,
            rate_bytes_per_tq,
            bucket_size: bucket_size_bytes as f64,
        }
    }

    /// Enqueue a frame to the appropriate priority queue.
    pub fn enqueue(&mut self, frame: Vec<u8>, priority: ServicePriority) -> bool {
        let qi = priority as usize;
        if self.queue_bytes[qi] + frame.len() > self.max_queue_bytes {
            return false; // drop
        }
        self.queue_bytes[qi] += frame.len();
        self.queues[qi].push(frame);
        true
    }

    /// Advance token bucket by `elapsed_tq` time quanta.
    pub fn tick(&mut self, elapsed_tq: u32) {
        self.tokens += self.rate_bytes_per_tq * elapsed_tq as f64;
        if self.tokens > self.bucket_size {
            self.tokens = self.bucket_size;
        }
    }

    /// Dequeue up to `budget_bytes` bytes in strict priority order.
    /// Returns dequeued frames.
    pub fn dequeue(&mut self, budget_bytes: usize) -> Vec<Vec<u8>> {
        let mut result = Vec::new();
        let mut remaining = budget_bytes;
        // Strict priority: dequeue from Control (3) down to BestEffort (0)
        for qi in (0..4).rev() {
            while !self.queues[qi].is_empty() && remaining > 0 {
                let frame_len = self.queues[qi][0].len();
                if frame_len > remaining || self.tokens < frame_len as f64 {
                    break;
                }
                let frame = self.queues[qi].remove(0);
                self.queue_bytes[qi] -= frame_len;
                self.tokens -= frame_len as f64;
                remaining -= frame_len;
                result.push(frame);
            }
        }
        result
    }

    /// Total bytes pending across all priority queues.
    pub fn total_pending_bytes(&self) -> u32 {
        self.queue_bytes.iter().sum::<usize>() as u32
    }

    /// Report structure for use in MPCP REPORT messages.
    pub fn build_queue_reports(&self) -> Vec<QueueReport> {
        (0..4)
            .filter(|&qi| self.queue_bytes[qi] > 0)
            .map(|qi| QueueReport {
                queue_id: qi as u8,
                report_bytes: self.queue_bytes[qi].min(65535) as u16,
            })
            .collect()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// FEC overhead calculations
// ─────────────────────────────────────────────────────────────────────────────

/// Calculate the FEC overhead ratio (parity bytes / total bytes) for a given rate.
pub fn fec_overhead_ratio(rate: EponRate) -> f64 {
    match rate {
        EponRate::Rate1G => {
            // RS(255,239): overhead = (255-239)/255
            (RS_1G_N - RS_1G_K) as f64 / RS_1G_N as f64
        }
        EponRate::Rate10GAsym | EponRate::Rate10GSym => {
            // RS(255,223): overhead = (255-223)/255
            (RS_10G_N - RS_10G_K) as f64 / RS_10G_N as f64
        }
    }
}

/// Calculate effective data throughput after FEC overhead removal.
pub fn effective_throughput_gbps(rate: EponRate) -> f64 {
    let raw_gbps = match rate {
        EponRate::Rate1G => 1.25,
        EponRate::Rate10GAsym | EponRate::Rate10GSym => 10.3125,
    };
    raw_gbps * (1.0 - fec_overhead_ratio(rate))
}

// ─────────────────────────────────────────────────────────────────────────────
// Main EponProcessor struct
// ─────────────────────────────────────────────────────────────────────────────

/// EPON processor: OLT-side implementation combining MPCP, FEC, DBA, and framing.
pub struct EponProcessor {
    config: EponConfig,
    llid_pool: LlidPool,
    scheduler: IpactScheduler,
    onu_records: Vec<OnuRecord>,
    rs_encoder_1g: RsEncoder,
    rs_decoder_1g: RsDecoder,
    rs_encoder_10g: RsEncoder,
    rs_decoder_10g: RsDecoder,
    shapers: Vec<LlidShaper>,
    stats: EponStats,
    /// OLT MAC address.
    olt_mac: [u8; 6],
    /// Current OLT MPCP timestamp counter (wraps at 2^32).
    mpcp_timestamp: u32,
}

impl EponProcessor {
    /// Create a new EPON processor with the given configuration.
    pub fn new(config: EponConfig) -> Self {
        let llid_pool = LlidPool::new(config.max_onus);
        let scheduler = IpactScheduler::new(config.clone());
        EponProcessor {
            rs_encoder_1g: RsEncoder::new(RS_1G_N, RS_1G_K),
            rs_decoder_1g: RsDecoder::new(RS_1G_N, RS_1G_K),
            rs_encoder_10g: RsEncoder::new(RS_10G_N, RS_10G_K),
            rs_decoder_10g: RsDecoder::new(RS_10G_N, RS_10G_K),
            llid_pool,
            scheduler,
            onu_records: Vec::new(),
            shapers: Vec::new(),
            stats: EponStats::default(),
            olt_mac: [0x00, 0x1A, 0x2B, 0x3C, 0x4D, 0x5E],
            mpcp_timestamp: 0,
            config,
        }
    }

    /// Set the OLT MAC address.
    pub fn set_olt_mac(&mut self, mac: [u8; 6]) {
        self.olt_mac = mac;
    }

    /// Advance the MPCP timestamp by `tq` quanta.
    pub fn advance_timestamp(&mut self, tq: u32) {
        self.mpcp_timestamp = self.mpcp_timestamp.wrapping_add(tq);
    }

    /// Get the current MPCP timestamp.
    pub fn current_timestamp(&self) -> u32 {
        self.mpcp_timestamp
    }

    // ── GATE / REPORT ─────────────────────────────────────────────────────

    /// Build a GATE PDU targeting a specific ONU.
    pub fn build_gate(&self, llid: u16, grants: &[GrantEntry]) -> Vec<u8> {
        build_gate_pdu(&self.olt_mac, llid, self.mpcp_timestamp, grants)
    }

    /// Build a discovery GATE (broadcast LLID, opens discovery window).
    pub fn build_discovery_gate(&self) -> Vec<u8> {
        let grant = GrantEntry {
            start_time_tq: self.mpcp_timestamp.wrapping_add(self.config.guard_time_tq),
            length_tq: self.config.discovery_window_tq,
        };
        build_gate_pdu(&self.olt_mac, BROADCAST_LLID, self.mpcp_timestamp, &[grant])
    }

    /// Parse an incoming GATE PDU.
    pub fn parse_gate(&self, data: &[u8]) -> Result<MpcpGate, &'static str> {
        parse_gate_pdu(data)
    }

    /// Build a REPORT PDU for an ONU.
    pub fn build_report(&self, llid: u16, queue_sets: &[QueueReport]) -> Vec<u8> {
        // Use a placeholder ONU MAC derived from LLID
        let onu_mac = [0x00, 0x0A, 0x0B, 0x0C, (llid >> 8) as u8, llid as u8];
        build_report_pdu(&onu_mac, llid, self.mpcp_timestamp, queue_sets)
    }

    /// Parse an incoming REPORT PDU.
    pub fn parse_report(&mut self, data: &[u8]) -> Result<MpcpReport, &'static str> {
        let report = parse_report_pdu(data)?;
        self.stats.reports_received += 1;
        Ok(report)
    }

    // ── FEC ────────────────────────────────────────────────────────────────

    /// Encode data using the RS codec appropriate for the configured rate.
    /// Input must be exactly k bytes; returns n bytes.
    pub fn rs_encode(&mut self, data: &[u8]) -> Vec<u8> {
        self.stats.fec_encodes += 1;
        match self.config.rate {
            EponRate::Rate1G => self.rs_encoder_1g.encode(data),
            _ => self.rs_encoder_10g.encode(data),
        }
    }

    /// Decode and correct a received RS codeword (n bytes).
    /// Returns corrected k-byte data or an error.
    pub fn rs_decode(&mut self, data: &[u8]) -> Result<Vec<u8>, &'static str> {
        self.stats.fec_decodes += 1;
        let result = match self.config.rate {
            EponRate::Rate1G => self.rs_decoder_1g.decode(data),
            _ => self.rs_decoder_10g.decode(data),
        };
        match result {
            Ok((corrected, n_errors)) => {
                self.stats.fec_corrected += n_errors as u64;
                Ok(corrected)
            }
            Err(e) => {
                self.stats.fec_uncorrectable += 1;
                Err(e)
            }
        }
    }

    /// Encode an arbitrary byte stream using RS FEC, producing RS-codeword-aligned output.
    /// Pads input to multiple of k bytes, returns multiple of n bytes.
    pub fn rs_encode_stream(&mut self, data: &[u8]) -> Vec<u8> {
        let k = match self.config.rate {
            EponRate::Rate1G => RS_1G_K,
            _ => RS_10G_K,
        };
        let mut padded = data.to_vec();
        while padded.len() % k != 0 {
            padded.push(0);
        }
        let mut out = Vec::with_capacity(padded.len() / k * RS_1G_N);
        for chunk in padded.chunks(k) {
            out.extend_from_slice(&self.rs_encode(chunk));
        }
        out
    }

    /// Decode an RS-encoded stream. Input must be a multiple of n bytes.
    pub fn rs_decode_stream(&mut self, data: &[u8]) -> Result<Vec<u8>, &'static str> {
        let n = match self.config.rate {
            EponRate::Rate1G => RS_1G_N,
            _ => RS_10G_N,
        };
        if data.len() % n != 0 {
            return Err("stream length not a multiple of n");
        }
        let mut out = Vec::new();
        for chunk in data.chunks(n) {
            let decoded = self.rs_decode(chunk)?;
            out.extend_from_slice(&decoded);
        }
        Ok(out)
    }

    // ── DBA / Scheduling ──────────────────────────────────────────────────

    /// Run the IPACT scheduler on received reports. Returns grant assignments.
    pub fn ipact_schedule(&mut self, reports: &[MpcpReport]) -> Vec<GrantAssignment> {
        let assignments = self.scheduler.schedule_cycle(reports);
        self.stats.grants_scheduled += assignments.len() as u64;
        assignments
    }

    // ── ONU management ────────────────────────────────────────────────────

    /// Begin ONU discovery: allocate an LLID and create a pending record.
    /// Returns the allocated LLID, or None if pool is exhausted.
    pub fn begin_discovery(&mut self, mac: [u8; 6]) -> Option<u16> {
        let llid = self.llid_pool.allocate()?;
        let record = OnuRecord {
            llid,
            mac,
            state: OnuState::PendingRegister,
            rtt_tq: 0,
            pending_bytes: 0,
            total_granted_bytes: 0,
            sla_max_bytes: 65536, // default SLA
        };
        self.onu_records.push(record.clone());
        self.scheduler.register_onu(record);
        self.stats.discovery_cycles += 1;
        Some(llid)
    }

    /// Complete ONU registration: mark the ONU as registered after REGISTER_ACK.
    pub fn complete_registration(&mut self, llid: u16, rtt_tq: u32) -> bool {
        let mut found = false;
        for record in self.onu_records.iter_mut() {
            if record.llid == llid {
                record.state = OnuState::Registered;
                record.rtt_tq = rtt_tq;
                self.stats.registered_onus += 1;
                found = true;
                break;
            }
        }
        // Always update scheduler's onus copy
        for onu in self.scheduler.onus.iter_mut() {
            if onu.llid == llid {
                onu.state = OnuState::Registered;
                onu.rtt_tq = rtt_tq;
            }
        }
        found
    }

    /// Deregister an ONU (e.g., timeout or explicit deregistration).
    pub fn deregister_onu(&mut self, llid: u16) {
        self.llid_pool.release(llid);
        self.scheduler.deregister_onu(llid);
        for record in self.onu_records.iter_mut() {
            if record.llid == llid {
                record.state = OnuState::Deregistered;
            }
        }
        if self.stats.registered_onus > 0 {
            self.stats.registered_onus -= 1;
        }
    }

    /// Get all ONU records.
    pub fn onu_records(&self) -> &[OnuRecord] {
        &self.onu_records
    }

    /// Find an ONU record by LLID.
    pub fn find_onu(&self, llid: u16) -> Option<&OnuRecord> {
        self.onu_records.iter().find(|r| r.llid == llid)
    }

    // ── Traffic shaping ───────────────────────────────────────────────────

    /// Add a traffic shaper for an LLID.
    pub fn add_shaper(&mut self, llid: u16, rate_bytes_per_tq: f64, bucket_size: usize) {
        self.shapers.push(LlidShaper::new(llid, rate_bytes_per_tq, bucket_size));
    }

    /// Enqueue a frame for a given LLID and priority.
    pub fn enqueue_frame(&mut self, llid: u16, frame: Vec<u8>, priority: ServicePriority) -> bool {
        if let Some(shaper) = self.shapers.iter_mut().find(|s| s.llid == llid) {
            return shaper.enqueue(frame, priority);
        }
        false
    }

    /// Dequeue frames for an LLID up to a budget (bytes).
    pub fn dequeue_frames(&mut self, llid: u16, budget_bytes: usize) -> Vec<Vec<u8>> {
        if let Some(shaper) = self.shapers.iter_mut().find(|s| s.llid == llid) {
            return shaper.dequeue(budget_bytes);
        }
        Vec::new()
    }

    // ── Link budget ───────────────────────────────────────────────────────

    /// Calculate link budget for the configured power budget class.
    pub fn calculate_link_budget(&self, budget_class: PowerBudget) -> f64 {
        calculate_link_budget(budget_class)
    }

    // ── Statistics ────────────────────────────────────────────────────────

    /// Get a snapshot of current statistics.
    pub fn stats(&self) -> &EponStats {
        &self.stats
    }

    /// Reset all statistics counters.
    pub fn reset_stats(&mut self) {
        self.stats = EponStats::default();
    }

    // ── Frame encapsulation ───────────────────────────────────────────────

    /// Encapsulate an Ethernet payload into an EPON frame with LLID.
    pub fn encapsulate_frame(
        &mut self,
        dst_mac: [u8; 6],
        llid: u16,
        ethertype: u16,
        payload: Vec<u8>,
    ) -> Vec<u8> {
        let fcs = compute_crc32(&payload);
        let frame = EponFrame {
            dst_mac,
            src_mac: self.olt_mac,
            llid,
            ethertype,
            payload,
            fcs,
        };
        self.stats.ds_bytes += frame.payload.len() as u64;
        build_epon_frame(&frame)
    }

    /// Decapsulate an EPON frame from raw bytes.
    pub fn decapsulate_frame(&mut self, data: &[u8]) -> Result<EponFrame, &'static str> {
        let frame = parse_epon_frame(data)?;
        self.stats.us_bytes += frame.payload.len() as u64;
        Ok(frame)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── GF(256) arithmetic ─────────────────────────────────────────────────

    #[test]
    fn test_gf256_mul_zero() {
        let gf = Gf256Tables::new();
        assert_eq!(gf.mul(0, 127), 0);
        assert_eq!(gf.mul(127, 0), 0);
    }

    #[test]
    fn test_gf256_mul_identity() {
        let gf = Gf256Tables::new();
        for x in 1u8..=255 {
            assert_eq!(gf.mul(x, 1), x);
            assert_eq!(gf.mul(1, x), x);
        }
    }

    #[test]
    fn test_gf256_mul_inverse() {
        let gf = Gf256Tables::new();
        for x in 1u8..=255 {
            let inv = gf.div(1, x);
            assert_eq!(gf.mul(x, inv), 1);
        }
    }

    #[test]
    fn test_gf256_mul_commutative() {
        let gf = Gf256Tables::new();
        for a in [3u8, 7, 11, 19, 37, 100, 200] {
            for b in [5u8, 13, 23, 41, 127, 255] {
                assert_eq!(gf.mul(a, b), gf.mul(b, a));
            }
        }
    }

    #[test]
    fn test_gf256_pow_zero_exponent() {
        let gf = Gf256Tables::new();
        for x in 1u8..=255 {
            assert_eq!(gf.pow(x, 0), 1);
        }
    }

    #[test]
    fn test_gf256_pow_one_exponent() {
        let gf = Gf256Tables::new();
        for x in 1u8..=255 {
            assert_eq!(gf.pow(x, 1), x);
        }
    }

    #[test]
    fn test_gf256_exp_log_roundtrip() {
        let gf = Gf256Tables::new();
        for i in 0..255usize {
            let x = gf.exp[i];
            assert_eq!(gf.log[x as usize] as usize, i);
        }
    }

    // ── RS(255,239) — 1G-EPON ─────────────────────────────────────────────

    #[test]
    fn test_rs_1g_encode_length() {
        let enc = RsEncoder::new(RS_1G_N, RS_1G_K);
        let data = vec![0u8; RS_1G_K];
        let cw = enc.encode(&data);
        assert_eq!(cw.len(), RS_1G_N);
    }

    #[test]
    fn test_rs_1g_encode_all_zeros_parity() {
        let enc = RsEncoder::new(RS_1G_N, RS_1G_K);
        let data = vec![0u8; RS_1G_K];
        let cw = enc.encode(&data);
        // Systematic: first k bytes are the data
        assert_eq!(&cw[..RS_1G_K], &data[..]);
    }

    #[test]
    fn test_rs_1g_decode_no_errors() {
        let enc = RsEncoder::new(RS_1G_N, RS_1G_K);
        let dec = RsDecoder::new(RS_1G_N, RS_1G_K);
        let data: Vec<u8> = (0..RS_1G_K as u8).collect();
        let cw = enc.encode(&data);
        let (decoded, errors) = dec.decode(&cw).unwrap();
        assert_eq!(decoded, data);
        assert_eq!(errors, 0);
    }

    #[test]
    fn test_rs_1g_decode_single_error() {
        let enc = RsEncoder::new(RS_1G_N, RS_1G_K);
        let dec = RsDecoder::new(RS_1G_N, RS_1G_K);
        let data: Vec<u8> = (0..RS_1G_K as u8).collect();
        let mut cw = enc.encode(&data);
        cw[10] ^= 0xFF; // inject single error
        let (decoded, errors) = dec.decode(&cw).unwrap();
        assert_eq!(decoded, data);
        assert_eq!(errors, 1);
    }

    #[test]
    fn test_rs_1g_decode_max_errors() {
        let enc = RsEncoder::new(RS_1G_N, RS_1G_K);
        let dec = RsDecoder::new(RS_1G_N, RS_1G_K);
        let data: Vec<u8> = (0..RS_1G_K as u8).collect();
        let mut cw = enc.encode(&data);
        // Inject exactly t=8 errors
        for i in 0..RS_1G_T {
            cw[i * 3 + 1] ^= (i as u8 + 1) * 17;
        }
        let result = dec.decode(&cw);
        assert!(result.is_ok());
        let (decoded, errors) = result.unwrap();
        assert_eq!(decoded, data);
        assert_eq!(errors, RS_1G_T);
    }

    #[test]
    fn test_rs_1g_decode_too_many_errors() {
        let enc = RsEncoder::new(RS_1G_N, RS_1G_K);
        let dec = RsDecoder::new(RS_1G_N, RS_1G_K);
        let data: Vec<u8> = (0..RS_1G_K as u8).collect();
        let mut cw = enc.encode(&data);
        // Inject t+1=9 errors → uncorrectable
        for i in 0..=RS_1G_T {
            cw[i * 3 + 2] ^= (i as u8 + 1) * 23;
        }
        assert!(dec.decode(&cw).is_err());
    }

    // ── RS stream encode/decode ────────────────────────────────────────────

    #[test]
    fn test_rs_stream_roundtrip_1g() {
        let mut proc = EponProcessor::new(EponConfig::default());
        let data: Vec<u8> = (0..RS_1G_K * 3).map(|x| x as u8).collect();
        let encoded = proc.rs_encode_stream(&data);
        assert_eq!(encoded.len(), RS_1G_N * 3);
        let decoded = proc.rs_decode_stream(&encoded).unwrap();
        assert_eq!(&decoded[..data.len()], &data[..]);
    }

    #[test]
    fn test_rs_stream_with_errors() {
        let mut proc = EponProcessor::new(EponConfig::default());
        let data: Vec<u8> = (0..RS_1G_K).map(|x| x as u8).collect();
        let mut encoded = proc.rs_encode_stream(&data);
        // One error per codeword
        encoded[5] ^= 0xAB;
        let decoded = proc.rs_decode_stream(&encoded).unwrap();
        assert_eq!(&decoded[..data.len()], &data[..]);
    }

    // ── CRC-32 ─────────────────────────────────────────────────────────────

    #[test]
    fn test_crc32_known_value() {
        // CRC-32 of "123456789" = 0xCBF43926
        let crc = compute_crc32(b"123456789");
        assert_eq!(crc, 0xCBF43926);
    }

    #[test]
    fn test_crc32_empty() {
        let crc = compute_crc32(b"");
        assert_eq!(crc, 0x00000000);
    }

    // ── MPCP GATE PDU ──────────────────────────────────────────────────────

    #[test]
    fn test_build_parse_gate_roundtrip() {
        let olt_mac = [0x00, 0x1A, 0x2B, 0x3C, 0x4D, 0x5E];
        let grants = [
            GrantEntry { start_time_tq: 1000, length_tq: 500 },
            GrantEntry { start_time_tq: 2000, length_tq: 300 },
        ];
        let pdu = build_gate_pdu(&olt_mac, 0x0042, 999, &grants);
        assert_eq!(pdu.len(), 64);
        let gate = parse_gate_pdu(&pdu).unwrap();
        assert_eq!(gate.llid, 0x0042);
        assert_eq!(gate.timestamp, 999);
        assert_eq!(gate.num_grants, 2);
        assert_eq!(gate.grants[0].start_time_tq, 1000);
        assert_eq!(gate.grants[0].length_tq, 500);
        assert_eq!(gate.grants[1].start_time_tq, 2000);
        assert_eq!(gate.grants[1].length_tq, 300);
    }

    #[test]
    fn test_gate_pdu_fcs_corruption_detected() {
        let olt_mac = [0x00, 0x1A, 0x2B, 0x3C, 0x4D, 0x5E];
        let grants = [];
        let mut pdu = build_gate_pdu(&olt_mac, BROADCAST_LLID, 0, &grants);
        // corrupt body
        pdu[5] ^= 0xFF;
        assert!(parse_gate_pdu(&pdu).is_err());
    }

    #[test]
    fn test_discovery_gate_broadcast_llid() {
        let proc = EponProcessor::new(EponConfig::default());
        let pdu = proc.build_discovery_gate();
        let gate = parse_gate_pdu(&pdu).unwrap();
        assert_eq!(gate.llid, BROADCAST_LLID);
        assert_eq!(gate.num_grants, 1);
        assert!(gate.is_discovery);
    }

    // ── MPCP REPORT PDU ───────────────────────────────────────────────────

    #[test]
    fn test_build_parse_report_roundtrip() {
        let onu_mac = [0x00, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE];
        let qs = [
            QueueReport { queue_id: 0, report_bytes: 1500 },
            QueueReport { queue_id: 2, report_bytes: 512 },
        ];
        let pdu = build_report_pdu(&onu_mac, 0x0007, 42000, &qs);
        assert_eq!(pdu.len(), 64);
        let report = parse_report_pdu(&pdu).unwrap();
        assert_eq!(report.llid, 0x0007);
        assert_eq!(report.timestamp, 42000);
        assert_eq!(report.num_queue_sets, 2);
        assert_eq!(report.queue_sets[0].queue_id, 0);
        assert_eq!(report.queue_sets[0].report_bytes, 1500);
        assert_eq!(report.queue_sets[1].queue_id, 2);
        assert_eq!(report.queue_sets[1].report_bytes, 512);
    }

    #[test]
    fn test_report_pdu_fcs_corruption_detected() {
        let onu_mac = [0x00, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE];
        let qs: &[QueueReport] = &[];
        let mut pdu = build_report_pdu(&onu_mac, 1, 0, qs);
        pdu[3] ^= 0xFF;
        assert!(parse_report_pdu(&pdu).is_err());
    }

    // ── EPON frame encapsulation ───────────────────────────────────────────

    #[test]
    fn test_epon_frame_build_parse_roundtrip() {
        let frame = EponFrame {
            dst_mac: [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF],
            src_mac: [0x00, 0x11, 0x22, 0x33, 0x44, 0x55],
            llid: 0x0003,
            ethertype: 0x0800,
            payload: b"Hello EPON!".to_vec(),
            fcs: 0,
        };
        let raw = build_epon_frame(&frame);
        // Check preamble LLID encoding
        assert_eq!(raw[5], 0x00);
        assert_eq!(raw[6], 0x03);
        assert_eq!(raw[7], 0xD5); // SFD
        let parsed = parse_epon_frame(&raw).unwrap();
        assert_eq!(parsed.llid, 0x0003);
        assert_eq!(parsed.dst_mac, frame.dst_mac);
        assert_eq!(parsed.src_mac, frame.src_mac);
        assert_eq!(parsed.ethertype, 0x0800);
    }

    #[test]
    fn test_epon_frame_fcs_detection() {
        let frame = EponFrame {
            dst_mac: [0x01, 0x02, 0x03, 0x04, 0x05, 0x06],
            src_mac: [0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F],
            llid: 1,
            ethertype: 0x0800,
            payload: vec![0u8; 46],
            fcs: 0,
        };
        let mut raw = build_epon_frame(&frame);
        // Corrupt FCS byte
        let last = raw.len() - 1;
        raw[last] ^= 0xFF;
        assert!(parse_epon_frame(&raw).is_err());
    }

    #[test]
    fn test_epon_frame_broadcast_llid() {
        let frame = EponFrame {
            dst_mac: [0xFF; 6],
            src_mac: [0x00; 6],
            llid: BROADCAST_LLID,
            ethertype: 0x8100,
            payload: vec![42u8; 64],
            fcs: 0,
        };
        let raw = build_epon_frame(&frame);
        let parsed = parse_epon_frame(&raw).unwrap();
        assert_eq!(parsed.llid, BROADCAST_LLID);
    }

    // ── LLID pool ─────────────────────────────────────────────────────────

    #[test]
    fn test_llid_pool_allocation() {
        let mut pool = LlidPool::new(8);
        let llid1 = pool.allocate().unwrap();
        let llid2 = pool.allocate().unwrap();
        assert_ne!(llid1, llid2);
        assert!(pool.is_in_use(llid1));
        pool.release(llid1);
        assert!(!pool.is_in_use(llid1));
    }

    #[test]
    fn test_llid_pool_exhaustion() {
        let mut pool = LlidPool::new(2);
        let a = pool.allocate().unwrap();
        let b = pool.allocate().unwrap();
        assert!(pool.allocate().is_none()); // pool exhausted
        pool.release(a);
        pool.release(b);
        // Should be allocatable again
        assert!(pool.allocate().is_some());
    }

    #[test]
    fn test_llid_broadcast_not_allocated() {
        let mut pool = LlidPool::new(32);
        for _ in 0..32 {
            let l = pool.allocate();
            if let Some(ll) = l {
                assert_ne!(ll, BROADCAST_LLID);
            }
        }
    }

    // ── Timestamp helpers ─────────────────────────────────────────────────

    #[test]
    fn test_tq_to_ns() {
        assert!((tq_to_ns(1) - 16.0).abs() < 1e-10);
        assert!((tq_to_ns(100) - 1600.0).abs() < 1e-10);
    }

    #[test]
    fn test_ns_to_tq_round_up() {
        assert_eq!(ns_to_tq(16.0), 1);
        assert_eq!(ns_to_tq(17.0), 2);
        assert_eq!(ns_to_tq(32.0), 2);
    }

    #[test]
    fn test_compute_rtt_tq_basic() {
        let rtt = compute_rtt_tq(100, 110, 220);
        // round-trip = 220 - 100 = 120
        // ONU processing = 110 - 100 = 10
        // RTT = 120 - 10 = 110
        assert_eq!(rtt, 110);
    }

    #[test]
    fn test_compute_rtt_tq_wraparound() {
        let rtt = compute_rtt_tq(u32::MAX - 5, 2, 50);
        // one_way = 50 - (u32::MAX - 5) wrapping = 56
        // proc = 2 - (u32::MAX - 5) wrapping = 8
        // rtt = 56 - 8 = 48
        assert_eq!(rtt, 48);
    }

    // ── Link budget ───────────────────────────────────────────────────────

    #[test]
    fn test_link_budget_pr20() {
        let budget = calculate_link_budget(PowerBudget::Pr20);
        // 4.0 - (-24.0) = 28 dB
        assert!((budget - 28.0).abs() < 0.01);
    }

    #[test]
    fn test_link_budget_pr30() {
        let budget = calculate_link_budget(PowerBudget::Pr30);
        // 7.0 - (-27.0) = 34 dB
        assert!((budget - 34.0).abs() < 0.01);
    }

    #[test]
    fn test_estimate_reach_km() {
        let reach = estimate_reach_km(PowerBudget::Pr20, 0.4); // 0.4 dB/km
        // (28 - 3) / 0.4 = 62.5 km
        assert!((reach - 62.5).abs() < 0.1);
    }

    // ── FEC overhead ──────────────────────────────────────────────────────

    #[test]
    fn test_fec_overhead_1g() {
        let ratio = fec_overhead_ratio(EponRate::Rate1G);
        // (255-239)/255 ≈ 0.0627
        assert!((ratio - 16.0 / 255.0).abs() < 1e-10);
    }

    #[test]
    fn test_fec_overhead_10g() {
        let ratio = fec_overhead_ratio(EponRate::Rate10GAsym);
        // (255-223)/255 ≈ 0.1255
        assert!((ratio - 32.0 / 255.0).abs() < 1e-10);
    }

    #[test]
    fn test_effective_throughput_1g() {
        let tput = effective_throughput_gbps(EponRate::Rate1G);
        // 1.25 * (239/255) ≈ 1.172 Gbps
        let expected = 1.25 * (239.0 / 255.0);
        assert!((tput - expected).abs() < 0.001);
    }

    // ── Wavelength plan ───────────────────────────────────────────────────

    #[test]
    fn test_wavelength_plan_1g() {
        let plan = wavelength_plan(EponRate::Rate1G);
        assert!((plan.ds_wavelength_nm - 1490.0).abs() < 0.1);
        assert!((plan.us_wavelength_nm - 1310.0).abs() < 0.1);
    }

    #[test]
    fn test_wavelength_plan_10g() {
        let plan = wavelength_plan(EponRate::Rate10GAsym);
        assert!((plan.ds_wavelength_nm - 1577.0).abs() < 0.1);
        assert!((plan.us_wavelength_nm - 1270.0).abs() < 0.1);
    }

    // ── IPACT scheduler ───────────────────────────────────────────────────

    #[test]
    fn test_ipact_schedule_empty_reports() {
        let config = EponConfig::default();
        let mut proc = EponProcessor::new(config);
        // Register two ONUs
        proc.begin_discovery([0x01, 0x02, 0x03, 0x04, 0x05, 0x06]);
        proc.begin_discovery([0x11, 0x12, 0x13, 0x14, 0x15, 0x16]);
        let grants = proc.ipact_schedule(&[]);
        // No registered ONUs yet → no grants
        assert_eq!(grants.len(), 0);
    }

    #[test]
    fn test_ipact_schedule_registered_onus() {
        let config = EponConfig {
            dba: DbaAlgorithm::Fixed,
            ..Default::default()
        };
        let mut proc = EponProcessor::new(config);
        let llid1 = proc.begin_discovery([0x01, 0x02, 0x03, 0x04, 0x05, 0x06]).unwrap();
        let llid2 = proc.begin_discovery([0x11, 0x12, 0x13, 0x14, 0x15, 0x16]).unwrap();
        proc.complete_registration(llid1, 100);
        proc.complete_registration(llid2, 120);
        let reports = [];
        let grants = proc.ipact_schedule(&reports);
        assert_eq!(grants.len(), 2);
    }

    #[test]
    fn test_ipact_schedule_with_reports() {
        let mut proc = EponProcessor::new(EponConfig::default());
        let llid = proc.begin_discovery([0x01, 0x02, 0x03, 0x04, 0x05, 0x06]).unwrap();
        proc.complete_registration(llid, 50);
        let report = MpcpReport {
            llid,
            timestamp: 1000,
            num_queue_sets: 1,
            queue_sets: {
                let mut qs = [QueueReport { queue_id: 0, report_bytes: 0 }; MAX_QUEUE_SETS];
                qs[0] = QueueReport { queue_id: 0, report_bytes: 8000 };
                qs
            },
        };
        let grants = proc.ipact_schedule(&[report]);
        assert_eq!(grants.len(), 1);
        assert!(grants[0].usable_bytes > 0);
    }

    // ── Traffic shaping ───────────────────────────────────────────────────

    #[test]
    fn test_shaper_enqueue_dequeue() {
        let mut shaper = LlidShaper::new(1, 10.0, 65536);
        shaper.tick(1000); // replenish tokens
        let frame = vec![0u8; 100];
        assert!(shaper.enqueue(frame.clone(), ServicePriority::BestEffort));
        assert_eq!(shaper.total_pending_bytes(), 100);
        let dequeued = shaper.dequeue(200);
        assert_eq!(dequeued.len(), 1);
        assert_eq!(dequeued[0].len(), 100);
        assert_eq!(shaper.total_pending_bytes(), 0);
    }

    #[test]
    fn test_shaper_priority_ordering() {
        let mut shaper = LlidShaper::new(1, 1000.0, 65536);
        shaper.tick(10000);
        shaper.enqueue(vec![1u8; 50], ServicePriority::BestEffort);
        shaper.enqueue(vec![2u8; 50], ServicePriority::Voice);
        shaper.enqueue(vec![3u8; 50], ServicePriority::Control);
        // Control (3) should be dequeued first
        let dequeued = shaper.dequeue(200);
        assert_eq!(dequeued.len(), 3);
        assert_eq!(dequeued[0][0], 3); // Control first
        assert_eq!(dequeued[1][0], 2); // Voice second
        assert_eq!(dequeued[2][0], 1); // BestEffort last
    }

    #[test]
    fn test_shaper_queue_report_generation() {
        let mut shaper = LlidShaper::new(5, 10.0, 65536);
        shaper.enqueue(vec![0u8; 1000], ServicePriority::Video);
        shaper.enqueue(vec![0u8; 500], ServicePriority::Voice);
        let reports = shaper.build_queue_reports();
        assert_eq!(reports.len(), 2);
        assert!(reports.iter().any(|r| r.queue_id == 1 && r.report_bytes == 1000));
        assert!(reports.iter().any(|r| r.queue_id == 2 && r.report_bytes == 500));
    }

    #[test]
    fn test_shaper_token_bucket_rate_limiting() {
        let mut shaper = LlidShaper::new(1, 10.0, 100); // 10 bytes/TQ, 100 byte bucket
        // Token bucket starts at capacity (100)
        let frame = vec![0u8; 80];
        assert!(shaper.enqueue(frame.clone(), ServicePriority::BestEffort));
        let dequeued = shaper.dequeue(200);
        assert_eq!(dequeued.len(), 1);
        // Now tokens = 100 - 80 = 20, next frame of 80 can't be dequeued
        shaper.enqueue(frame, ServicePriority::BestEffort);
        let dequeued2 = shaper.dequeue(200);
        assert_eq!(dequeued2.len(), 0); // insufficient tokens
    }

    // ── Burst-mode reception ───────────────────────────────────────────────

    #[test]
    fn test_burst_mode_level_recovery() {
        let mut meta = BurstModeMetadata::new(5, 1000);
        meta.estimate_level_dbm(0.5); // 0.5 amplitude
        assert!(meta.input_level_dbm < 0.0); // negative dBm
        assert!(meta.cdr_locked);
    }

    #[test]
    fn test_burst_mode_cdr_lock_check() {
        let mut meta = BurstModeMetadata::new(1, 0);
        meta.cdr_locked = true;
        meta.clock_offset_ppm = 50.0;
        assert!(meta.check_cdr_lock());
        meta.clock_offset_ppm = 200.0;
        assert!(!meta.check_cdr_lock());
    }

    // ── ONU discovery / registration FSM ─────────────────────────────────

    #[test]
    fn test_onu_discovery_and_registration() {
        let mut proc = EponProcessor::new(EponConfig::default());
        let mac = [0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01];
        let llid = proc.begin_discovery(mac).unwrap();
        assert_ne!(llid, BROADCAST_LLID);
        {
            let record = proc.find_onu(llid).unwrap();
            assert_eq!(record.state, OnuState::PendingRegister);
        }
        let ok = proc.complete_registration(llid, 80);
        assert!(ok);
        {
            let record = proc.find_onu(llid).unwrap();
            assert_eq!(record.state, OnuState::Registered);
            assert_eq!(record.rtt_tq, 80);
        }
    }

    #[test]
    fn test_onu_deregistration() {
        let mut proc = EponProcessor::new(EponConfig::default());
        let llid = proc.begin_discovery([0x01; 6]).unwrap();
        proc.complete_registration(llid, 100);
        proc.deregister_onu(llid);
        let record = proc.find_onu(llid).unwrap();
        assert_eq!(record.state, OnuState::Deregistered);
    }

    #[test]
    fn test_onu_max_capacity() {
        let config = EponConfig { max_onus: 4, ..Default::default() };
        let mut proc = EponProcessor::new(config);
        for i in 0..4u8 {
            assert!(proc.begin_discovery([i, i, i, i, i, i]).is_some());
        }
        // Pool should be exhausted
        assert!(proc.begin_discovery([0xFFu8; 6]).is_none());
    }

    // ── Processor stats ───────────────────────────────────────────────────

    #[test]
    fn test_stats_accumulation() {
        let mut proc = EponProcessor::new(EponConfig::default());
        let data = vec![0u8; RS_1G_K];
        proc.rs_encode(&data);
        proc.rs_encode(&data);
        assert_eq!(proc.stats().fec_encodes, 2);
    }

    #[test]
    fn test_stats_reset() {
        let mut proc = EponProcessor::new(EponConfig::default());
        let data = vec![0u8; RS_1G_K];
        proc.rs_encode(&data);
        proc.reset_stats();
        assert_eq!(proc.stats().fec_encodes, 0);
    }

    // ── Full frame encapsulation roundtrip ───────────────────────────────

    #[test]
    fn test_full_frame_roundtrip() {
        let mut proc = EponProcessor::new(EponConfig::default());
        let dst = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
        let payload = b"Test payload for EPON".to_vec();
        let raw = proc.encapsulate_frame(dst, 0x000A, 0x0800, payload.clone());
        let frame = proc.decapsulate_frame(&raw).unwrap();
        assert_eq!(frame.llid, 0x000A);
        assert_eq!(frame.ethertype, 0x0800);
        // payload may have been padded to 46 bytes
        assert!(frame.payload.starts_with(&payload));
    }

    #[test]
    fn test_processor_discovery_gate_roundtrip() {
        let mut proc = EponProcessor::new(EponConfig::default());
        proc.advance_timestamp(500);
        let pdu = proc.build_discovery_gate();
        let gate = proc.parse_gate(&pdu).unwrap();
        assert_eq!(gate.llid, BROADCAST_LLID);
        assert_eq!(gate.timestamp, 500);
    }

    // ── tq_to_bytes ───────────────────────────────────────────────────────

    #[test]
    fn test_tq_to_bytes_1g() {
        assert_eq!(tq_to_bytes(10, EponRate::Rate1G), 20);
    }

    #[test]
    fn test_tq_to_bytes_10g() {
        assert_eq!(tq_to_bytes(10, EponRate::Rate10GAsym), 160);
    }
}
