//! Serial NOR flash on the controller's CSI20 (chip select P1.6, active low): 4 MiB, 3-byte addresses, standard
//! command set (03 READ, 0B FAST READ, 02 PP, 20 SE 4K, 52/D8 block erase, 60/C7 chip erase, 05 RDSR, 01 WRSR, 06 WREN,
//! 04 WRDI, 9F RDID). Program/erase complete immediately (WIP never observed set).

pub const SIZE: usize = 4 << 20;

#[derive(serde::Serialize, serde::Deserialize)]
pub struct SpiFlash {
    pub mem: Vec<u8>,
    selected: bool,
    cmd: Option<u8>,
    addr: u32,
    nbytes: u32,
    wel: bool,
    status: u8,
    pub dirty: bool,
    pub ops: u64,
    /// Read commands (start address, bytes read), when `trace_reads` (diagnostics).
    #[serde(skip)]
    pub reads: Vec<(u32, u32)>,
    #[serde(skip)]
    pub trace_reads: bool,
}

impl SpiFlash {
    pub fn blank() -> SpiFlash { SpiFlash { mem: vec![0xFF; SIZE], selected: false, cmd: None, addr: 0, nbytes: 0, wel: false, status: 0, dirty: false, ops: 0, reads: Vec::new(), trace_reads: false } }
    /// A unit that has been through factory provisioning. Fixture (no dump of a real chip exists): the controller keeps
    /// the installed GPS-receiver firmware version as a 16-byte record at 0x000000 (10 ASCII digits, NUL, 5 more bytes;
    /// read by the update check at RL78 0x4DE9A). An erased record (FF) means "never installed" and forces a GPS update.
    pub fn provisioned() -> SpiFlash {
        let mut f = Self::blank();
        f.mem[..16].copy_from_slice(b"0000000000\0\0\0\0\0\0");
        f
    }
    pub fn from_image(mut d: Vec<u8>) -> SpiFlash { d.resize(SIZE, 0xFF); SpiFlash { mem: d, ..Self::blank() } }

    /// Chip select level change (true = asserted / low).
    pub fn select(&mut self, on: bool) {
        if on && !self.selected { self.cmd = None; self.nbytes = 0; self.addr = 0; }
        if !on && self.selected && self.trace_reads && matches!(self.cmd, Some(0x03) | Some(0x0B)) && self.nbytes > 4 && self.reads.len() < 1_000_000 {
            let n = self.nbytes - if self.cmd == Some(0x0B) { 5 } else { 4 };
            self.reads.push((self.addr.wrapping_sub(n), n));
        }
        if !on && self.selected {
            // Erase commands execute on deselect.
            match self.cmd {
                Some(0x20) if self.nbytes == 4 && self.wel => self.erase(self.addr & !0xFFF, 0x1000),
                Some(0x52) if self.nbytes == 4 && self.wel => self.erase(self.addr & !0x7FFF, 0x8000),
                Some(0xD8) if self.nbytes == 4 && self.wel => self.erase(self.addr & !0xFFFF, 0x10000),
                Some(0x60) | Some(0xC7) if self.wel => self.erase(0, SIZE as u32),
                _ => {}
            }
            if matches!(self.cmd, Some(0x02) | Some(0x20) | Some(0x52) | Some(0xD8) | Some(0x60) | Some(0xC7) | Some(0x01)) { self.wel = false; }
        }
        self.selected = on;
    }
    fn erase(&mut self, a: u32, n: u32) { for b in &mut self.mem[a as usize..(a + n) as usize] { *b = 0xFF; } self.dirty = true; self.ops += 1; }

    /// One full-duplex byte exchange.
    pub fn xfer(&mut self, v: u8) -> u8 {
        if !self.selected { return 0xFF; }
        let k = self.nbytes;
        self.nbytes += 1;
        let Some(cmd) = self.cmd else {
            self.cmd = Some(v);
            match v { 0x06 => self.wel = true, 0x04 => self.wel = false, _ => {} }
            return 0xFF;
        };
        match cmd {
            0x05 => self.status | if self.wel { 2 } else { 0 },
            0x9F => [0xC2, 0x20, 0x16][((k - 1) % 3) as usize], // Macronix MX25L3206E-style ID (32 Mbit)
            0x01 => { if k == 1 { self.status = v & 0xBC; } 0xFF }
            0x03 | 0x0B | 0x02 | 0x20 | 0x52 | 0xD8 => {
                if k <= 3 { self.addr = (self.addr << 8) | v as u32; return 0xFF; }
                if cmd == 0x0B && k == 4 { return 0xFF; }
                match cmd {
                    0x03 | 0x0B => { let r = self.mem[self.addr as usize % SIZE]; self.addr = (self.addr + 1) % SIZE as u32; r }
                    0x02 if self.wel => {
                        let page = self.addr & !0xFF;
                        let off = (self.addr + (k - 4)) & 0xFF;
                        let a = (page | off) as usize % SIZE;
                        self.mem[a] &= v; self.dirty = true; if k == 4 { self.ops += 1; }
                        0xFF
                    }
                    _ => 0xFF,
                }
            }
            _ => 0xFF,
        }
    }
}
