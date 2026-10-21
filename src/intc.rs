//! SH7269 interrupt controller: vector -> interrupt priority register field.
//! Only vectors the firmware has been observed to use are listed; an unknown vector is an error, not a guess.

pub const IBNR: u32 = 0xFFFE_080E;

/// (IPR register address, bit shift of the 4-bit level field) for an interrupt vector.
pub fn ipr_field(vector: u32) -> Option<(u32, u32)> {
    Some(match vector {
        171..=173 => (0xFFFE_0C08, 8),               // VDC4 signals 0..2
        174 | 175 => (0xFFFE_0C08, 4),               // VDC4 signals 3..4 (174 = VLINE)
        176..=178 => (0xFFFE_0C08, 0),               // VDC4 signals 5..
        181 | 182 => (0xFFFE_0C0A, 8),               // JCU JEDI/JDTI
        186 => (0xFFFE_0C0A, 0),                     // RGPVG INT1
        188 => (0xFFFE_0C0C, 12),                    // CMT CMI0
        189 => (0xFFFE_0C0C, 8),                     // CMT CMI1
        199 => (0xFFFE_0C0E, 4),                     // MTU1 TGI1A
        201 => (0xFFFE_0C0E, 0),                     // MTU1 TCI1V
        224 => (0xFFFE_0C16, 8),                     // SSIF0
        286..=289 => (0xFFFE_0C1E, 0),               // SCIF7 BRI/ERI/RXI/TXI
        282..=285 => (0xFFFE_0C1E, 4),               // SCIF6
        262..=265 => (0xFFFE_0C1C, 8),               // SCIF1
        108..=170 => { let c = (vector - 108) / 4; (0xFFFE_0C00 + 2 * (c / 4), 12 - 4 * (c % 4)) } // DMAC DEI/HEI
        332 => (0xFFFE_0C26, 12),                    // SDHI0_3
        341..=345 => (0xFFFE_0C28, 12),               // SRC0 OVF/UDF/CEF/ODFI/IDEI
        _ => return None,
    })
}
