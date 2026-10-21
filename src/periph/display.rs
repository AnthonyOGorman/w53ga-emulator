//! VDC4 register/event model and the board display clock, after the SH7269 hardware manual: shadow/active register latching on update requests and Vsync,
//! pixel clock from the selected source and PANEL_CLK divisor, H/V counters from FRC4, internal Vsync delayed by
//! FRC5 lines, output line counter and the VDC4 interrupt signals (VLINE, LO_VSYNC, ...).
//! No pixels are generated here; scanout reads the active GR layer registers.

use serde::Deserialize;
use std::collections::{BTreeMap, HashMap};

pub const SIGNALS: [&str; 9] = ["VI_VSYNC", "LO_VSYNC", "VSYNCERR", "VLINE", "VFIELD", "VBUFERR1", "VBUFERR2", "VBUFERR3", "VBUFERR4"];
pub const GR: [u32; 3] = [0xFFFF_7600, 0xFFFF_7700, 0xFFFF_7780];
const PANEL_CLK: u32 = 0xFFFF_7A90;
const SCL0_UPDATE: u32 = 0xFFFF_7500;
const FRC3: u32 = 0xFFFF_750C;
const FRC4: u32 = 0xFFFF_7510;
const FRC5: u32 = 0xFFFF_7514;

#[derive(Deserialize, Clone, Debug, serde::Serialize)]
pub struct VdcReg { pub address: u64, pub mask: u64, pub ones: u64, pub size: u32, pub group: String, pub name: String, pub shadow: u64, pub active: u64 }

#[derive(Deserialize, Clone)]
#[serde(rename_all = "camelCase")]
pub struct RawVdc {
    registers: BTreeMap<String, VdcReg>,
    clut_selection_known: [bool; 3],
    palette: Vec<Vec<Vec<Option<u64>>>>,
    input_clocks: [Option<bool>; 3],
    scl_shared: BTreeMap<String, [u64; 4]>,
    armed: u64, latched: u64, enabled: u64, vsyncs: u64, clock_edges: u64, events_accepted: u64, module_enabled: bool,
}

#[derive(Deserialize, Clone)]
#[serde(rename_all = "camelCase")]
pub struct RawDisplayClock {
    vdc: RawVdc,
    numerator: [u64; 4], denominator: [u64; 4], remainder: [u64; 4], source_edges: [u64; 4], present: [Option<bool>; 4],
    input_queue: Vec<[u64; 2]>, ready_inputs: Vec<[u64; 2]>, output_queue: Vec<[u64; 2]>,
    ticks: u64, pixels: u64, divider_remainder: u64, hsyncs: u64, input_vsyncs: u64, output_vsyncs: u64, raw_internal_vsyncs: u64,
    retimings: u64, cancelled_outputs: u64, discarded_inputs: u64, hpos: u32, vpos: u32, output_line: u32, input_select: u32,
    old_input_select: u32, output_line_known: bool, vdec_enabled: bool, old_panel: u64, old_frc3: u64, old_frc4: u64,
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Vdc4 {
    pub regs: Vec<VdcReg>,
    index: HashMap<u32, usize>,
    clut_known: [bool; 3],
    palette: Vec<Vec<Vec<Option<u64>>>>,
    input_clocks: [Option<bool>; 3],
    scl_shared: BTreeMap<u32, [u64; 4]>,
    pub armed: u64, pub latched: u64, pub enabled: u64, pub vsyncs: u64, pub clock_edges: u64, pub events_accepted: u64,
    pub module_enabled: bool,
    /// Set whenever an active register may have changed (write, latch copy); cleared by DisplayClock::sample_active.
    pub changed: bool,
    i_panel: usize, i_scl0: usize, i_frc3: usize, i_frc4: usize, i_frc5: usize, i_gr2clut: usize,
    /// Rectangle alpha fade per GR layer (graphics 2/3 processes): current alpha and Vsync count toward ARC_RATE+1.
    #[serde(skip)]
    pub arc: [(u8, u32); 3],
    /// SCL1_WR7.RES_FLM_CNT: frame buffer written before the current one.
    #[serde(skip)]
    pub wr_frame: u32,
}

pub type VResult<T> = Result<T, String>;

impl Vdc4 {
    fn from_raw(r: RawVdc) -> VResult<Vdc4> {
        let mut regs: Vec<VdcReg> = r.registers.into_values().collect();
        regs.sort_by_key(|x| x.address);
        let index: HashMap<u32, usize> = regs.iter().enumerate().map(|(i, x)| (x.address as u32, i)).collect();
        let g = |a: u32| index.get(&a).copied().ok_or(format!("VDC register {a:08x} missing"));
        let scl_shared = r.scl_shared.into_iter().map(|(k, v)| Ok((k.parse::<u64>().map_err(|e| e.to_string())? as u32, v))).collect::<VResult<_>>()?;
        Ok(Vdc4 {
            i_panel: g(PANEL_CLK)?, i_scl0: g(SCL0_UPDATE)?, i_frc3: g(FRC3)?, i_frc4: g(FRC4)?, i_frc5: g(FRC5)?, i_gr2clut: g(GR[2] + 80)?,
            regs, index, clut_known: r.clut_selection_known, palette: r.palette, input_clocks: r.input_clocks, scl_shared,
            armed: r.armed, latched: r.latched, enabled: r.enabled, vsyncs: r.vsyncs, clock_edges: r.clock_edges, events_accepted: r.events_accepted,
            module_enabled: r.module_enabled, changed: true, arc: [(0, 0); 3], wr_frame: 0,
        })
    }

    /// Registers of the rectangle alpha-blending function (graphics 2/3 processes = GR1/GR2 here) that the base
    /// register list lacks: GRn_AB6 (ARC_MODE/COEF/RATE), GRn_MON (ARC_ST) and the ARC bits of GRn_AB1. Appended, so
    /// the indices of existing registers do not move.
    pub fn ensure_arc_registers(&mut self) {
        for (k, g) in [(1usize, "G1P"), (2, "G2P")] {
            let base = GR[k] as u64;
            if let Some(&i) = self.index.get(&(GR[k] + 0x20)) { self.regs[i].mask |= 0x1110; }
            for (off, mask, group, name) in [(0x34u64, 0x01FF_00FFu64, g, "AB6"), (0x54, 0, "arcmon", "MON")] {
                let a = base + off;
                if self.index.contains_key(&(a as u32)) { continue; }
                self.regs.push(VdcReg { address: a, mask, ones: 0, size: 4, group: group.into(), name: name.into(), shadow: 0, active: 0 });
                self.index.insert(a as u32, self.regs.len() - 1);
            }
        }
    }

    /// Video input path registers the base register list lacks (manual sections 31-33): input controller INP_*, image
    /// quality IMGCNT_*, scaler-0 Vsync detection SCL0_FRC9 and the frame-buffer write-back SCL1_WR1..WR7. Update
    /// requests (INP_UPDATE, IMGCNT_UPDATE) take effect at the next Vsync; SCL1_WR* latch on input Vsync as before.
    pub fn ensure_video_registers(&mut self) {
        let add = |regs: &mut Vec<VdcReg>, index: &mut HashMap<u32, usize>, a: u32, mask: u64, init: u64, group: &str, name: &str| {
            if let Some(&i) = index.get(&a) { regs[i].mask = mask; regs[i].group = group.into(); return; }
            regs.push(VdcReg { address: a as u64, mask, ones: 0, size: 4, group: group.into(), name: name.into(), shadow: init, active: init });
            index.insert(a, regs.len() - 1);
        };
        let (r, i) = (&mut self.regs, &mut self.index);
        add(r, i, 0xFFFF_7400, 0x11, 0, "request", "INP_UPDATE");
        for (a, init, n) in [(0x7404u32, 0u64, "INP_SEL_CNT"), (0x7408, 0, "INP_EXT_SYNC_CNT"), (0x740C, 0x035A_01AD, "INP_VSYNC_PH_ADJ"), (0x7410, 0, "INP_DLY_ADJ")] {
            add(r, i, 0xFFFF_0000 | a, 0xFFFF_FFFF, init, "inp", n);
        }
        add(r, i, 0xFFFF_7480, 0x11, 0, "request", "IMGCNT_UPDATE");
        for a in (0x7484u32..0x7500).step_by(4) {
            let init = match a { 0x7484 => 0x0010_0803, 0x7488 => 0x0803_0803, 0x74A0 => 3, 0x74A4 => 0x0080_0100, 0x74AC => 0x0080_0000, 0x74B0 => 0x0100_0000, 0x74B4 => 0x0080_0000, 0x74B8 => 0x0000_0100, _ => 0 };
            add(r, i, 0xFFFF_0000 | a, 0xFFFF_FFFF, init, "imgcnt", "IMGCNT");
        }
        add(r, i, 0xFFFF_7524, 0, 0, "frc9", "SCL0_FRC9");
        add(r, i, 0xFFFF_7588, 0x1F7, 0, "scl1B", "SCL1_WR1");
        add(r, i, 0xFFFF_758C, 0xFFFF_FFFF, 0, "scl1B", "SCL1_WR2");
        add(r, i, 0xFFFF_7590, 0x7FFF_03FF, 0x0800_0001, "scl1B", "SCL1_WR3");
        add(r, i, 0xFFFF_7594, 0x7F_FFFF, 0x0008_0000, "scl1B", "SCL1_WR4");
        add(r, i, 0xFFFF_759C, 0x1311, 0x1000, "scl1A", "SCL1_WR5");
        add(r, i, 0xFFFF_75A0, 0xFFFF_FFFF, 0, "scl1B", "SCL1_WR6");
        add(r, i, 0xFFFF_75A4, 0, 0, "wr7", "SCL1_WR7");
    }

    /// Alpha fade of a layer's rectangle (GR_ARC_*), stepped on each output Vsync (manual 35.1.10).
    fn arc_vsync(&mut self) {
        for k in 1..3 {
            let (ab1, ab6, ab7) = (self.active(GR[k] + 0x20), self.active(GR[k] + 0x34), self.active(GR[k] + 0x38));
            let def = ((ab7 >> 16) & 0xFF) as u8;
            let (alpha, count) = &mut self.arc[k];
            if ab1 & 0x1000 == 0 { *alpha = def; *count = 0; continue; }
            let (sub, coef, rate) = (ab6 & 0x0100_0000 != 0, ((ab6 >> 16) & 0xFF) as i32, (ab6 & 0xFF) as u32);
            if coef == 0 { continue; }
            *count += 1;
            if *count >= rate + 1 {
                *count = 0;
                *alpha = (*alpha as i32 + if sub { -coef } else { coef }).clamp(0, 255) as u8;
            }
        }
    }

    /// Displayed CLUT entry (A, R, G, B in bits 31..0) of a layer: the table selected by GR_CLT_SEL.
    pub fn clut(&self, layer: usize, idx: usize) -> u32 {
        let bank = ((self.active(GR[layer] + 80) >> 16) & 1) as usize;
        self.palette.get(layer).and_then(|l| l.get(bank)).and_then(|b| b.get(idx)).copied().flatten().unwrap_or(0) as u32
    }

    /// GRn_ARC_ST: the fade is still adding or subtracting.
    pub fn arc_busy(&self, k: usize) -> bool {
        if k == 0 { return false; }
        let (ab1, ab6) = (self.active(GR[k] + 0x20), self.active(GR[k] + 0x34));
        if ab1 & 0x1000 == 0 || (ab6 >> 16) & 0xFF == 0 { return false; }
        let a = self.arc[k].0;
        if ab6 & 0x0100_0000 != 0 { a > 0 } else { a < 255 }
    }

    #[inline] pub fn active(&self, a: u32) -> u64 { self.regs[self.index[&a]].active }
    pub fn handles(a: u32, n: usize) -> bool { n > 0 && a >= 0xFFFF_6000 && a as u64 + n as u64 <= 0xFFFF_7A94 }
    fn palette_layer(a: u32) -> Option<usize> { if (0xFFFF_6000..0xFFFF_6C00).contains(&a) { Some(((a - 0xFFFF_6000) / 0x400) as usize) } else { None } }

    fn register(&self, a: u32, size: usize) -> VResult<usize> {
        for (i, r) in self.regs.iter().enumerate() {
            let (ra, rs) = (r.address as u32, r.size);
            if a >= ra && a as u64 + size as u64 <= ra as u64 + rs as u64 {
                if !matches!(size, 1 | 2 | 4) || size as u32 > rs || (size > 1 && a as usize % size != 0) { break; }
                if (ra == 0xFFFF_7680 || r.group == "adj") && size == 1 { break; }
                return Ok(i);
            }
        }
        Err(format!("unmodelled VDC access {a:08x}/{size}"))
    }

    fn clock_running(&self) -> VResult<bool> {
        let panel = self.regs[self.i_panel].active;
        if !self.module_enabled || panel & 0x100 == 0 { return Ok(false); }
        let (source, divider) = (((panel >> 12) & 3) as usize, panel & 63);
        if source == 3 || ![1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 16, 24, 32].contains(&divider) || (source == 2 && divider == 1) { return Err("prohibited panel clock source/divider".into()); }
        self.input_clocks[source].ok_or_else(|| "panel clock input availability not supplied".to_string())
    }

    pub fn read(&self, a: u32, size: usize) -> VResult<u64> {
        if let Some(layer) = Self::palette_layer(a) {
            if size != 4 || a % 4 != 0 || !self.clut_known[layer] { return Err("CLUT access width/selection unknown".into()); }
            let bank = 1 - ((self.active(GR[layer] + 80) >> 16) & 1) as usize;
            let idx = (((a - 0xFFFF_6000) % 0x400) / 4) as usize;
            return self.palette[layer][bank][idx].ok_or_else(|| "uninitialized CLUT entry".to_string());
        }
        let i = self.register(a, size)?;
        let r = &self.regs[i];
        let mut value = r.shadow;
        if r.group == "interrupt" { value = self.interrupt_value(r.address as u32); }
        if r.group == "wr7" { value = self.wr_frame as u64; }
        if r.group == "frc9" { value = 0; }
        if r.group == "arcmon" { value = self.arc_busy(GR.iter().position(|&g| g + 0x54 == r.address as u32).unwrap_or(0)) as u64; }
        if r.group == "monitor" {
            if !self.clut_known.iter().all(|&k| k) { return Err("CLUT monitor undefined".into()); }
            value = (0..3).fold(0, |v, k| v | (((self.active(GR[k] + 80) >> 16) & 1) << (k * 4)));
        }
        let shift = (r.size as i64 - (a as i64 - r.address as i64) - size as i64) * 8;
        Ok((value >> shift) & if size == 4 { 0xFFFF_FFFF } else { (1u64 << (size * 8)) - 1 })
    }

    pub fn write(&mut self, a: u32, size: usize, value: u64) -> VResult<()> {
        self.changed = true;
        if let Some(layer) = Self::palette_layer(a) {
            if size != 4 || a % 4 != 0 || !self.clut_known[layer] { return Err("CLUT write width/selection unknown".into()); }
            let bank = 1 - ((self.active(GR[layer] + 80) >> 16) & 1) as usize;
            let idx = (((a - 0xFFFF_6000) % 0x400) / 4) as usize;
            self.palette[layer][bank][idx] = Some(value);
            return Ok(());
        }
        let i = self.register(a, size)?;
        let (raddr, rsize, mask, ones) = { let r = &self.regs[i]; (r.address as u32, r.size, r.mask, r.ones) };
        let group = self.regs[i].group.clone();
        if matches!(group.as_str(), "monitor" | "arcmon" | "wr7" | "frc9") { return Err(format!("read-only VDC register {}", self.regs[i].name)); }
        let shift = (rsize as i64 - (a as i64 - raddr as i64) - size as i64) * 8;
        let lane = (if size == 4 { 0xFFFF_FFFFu64 } else { (1u64 << (size * 8)) - 1 }) << shift;
        let mut shifted = value << shift;
        // GRn_FLM2 (frame buffer base): bits 2-0 are reserved and the base is used 8-byte aligned; the firmware
        // sometimes passes an unaligned address (seen on the wireless LAN connection-method screen).
        if self.regs[i].name == "FLM2" { shifted &= !7; }
        if shifted & lane & !(mask | ones) != 0 { return Err(format!("unmodelled/reserved VDC bits {}", self.regs[i].name)); }
        if raddr == 0xFFFF_7688 && shifted & lane & 1 != 0 { return Err("ADJ enhancer overlay unsupported".into()); }
        if raddr == 0xFFFF_76B4 && shifted & lane & 3 != 0 { return Err("ADJ matrix modes unsupported".into()); }
        if (0xFFFF_76B8..=0xFFFF_76CC).contains(&raddr) {
            let id = [0x0080_0100u64, 0, 0x0080_0000, 0x0100_0000, 0x0080_0000, 0x0000_0100][((raddr - 0xFFFF_76B8) / 4) as usize];
            if shifted & mask & lane != id & mask & lane { return Err("ADJ non-identity coefficient unsupported".into()); }
        }
        if group == "interrupt" { self.write_interrupt(raddr, shifted, lane); return Ok(()); }
        let selected = mask & lane;
        if group == "request" {
            self.regs[i].shadow |= shifted & selected;
            if raddr == SCL0_UPDATE {
                let sh = self.regs[i].shadow;
                for f in self.scl_shared.values_mut() { if sh & f[0] == f[0] && shifted & f[0] != 0 { f[2] = 1; f[3] = 0; } }
            }
        } else {
            let r = &mut self.regs[i];
            r.shadow = ((r.shadow & !selected) | (shifted & selected) | r.ones) & 0xFFFF_FFFF;
        }
        if group == "immediate" { let r = &mut self.regs[i]; r.active = r.shadow; }
        Ok(())
    }

    fn interrupt_value(&self, a: u32) -> u64 {
        if a == 0xFFFF_7A80 { return (self.latched >> 8) & 1; }
        if a == 0xFFFF_7A88 { return (self.enabled >> 8) & 1; }
        let src = if a == 0xFFFF_7A84 { self.latched } else { self.enabled };
        (0..8).fold(0, |v, k| v | (((src >> k) & 1) << (k * 4)))
    }

    fn write_interrupt(&mut self, a: u32, value: u64, lane: u64) {
        let status = a == 0xFFFF_7A80 || a == 0xFFFF_7A84;
        let (first, last) = if a == 0xFFFF_7A80 || a == 0xFFFF_7A88 { (8, 9) } else { (0, 8) };
        for k in first..last {
            let bit = if k == 8 { 0 } else { k * 4 };
            if lane & (1u64 << bit) == 0 { continue; }
            let m = 1u64 << k;
            if status { if value & (1u64 << bit) != 0 { self.armed |= m } else { self.armed &= !m; self.latched &= !m; } }
            else if value & (1u64 << bit) != 0 { self.enabled |= m } else { self.enabled &= !m }
        }
    }

    pub fn signal(&mut self, index: usize) {
        let bit = 1u64 << index;
        if self.armed & bit != 0 { self.latched |= bit; self.armed &= !bit; self.events_accepted += 1; }
    }

    /// Pending signal indices (latched & enabled).
    pub fn pending(&self) -> u64 { self.latched & self.enabled & 0x1FF }

    fn copy_group(&mut self, group: &str) { self.changed = true; for r in self.regs.iter_mut() { if r.group == group { r.active = (r.active & !r.mask) | (r.shadow & r.mask); } } }
    fn copy(&mut self, i: usize, mask: u64) { self.changed = true; let r = &mut self.regs[i]; r.active = (r.active & !mask) | (r.shadow & mask); }

    pub fn clock_edge(&mut self) -> VResult<()> {
        if !self.clock_running()? { return Ok(()); }
        self.clock_edges += 1;
        if self.regs[self.i_scl0].shadow & 0x100 != 0 {
            self.copy_group("sync");
            self.regs[self.i_scl0].shadow &= !0x100;
        }
        Ok(())
    }

    /// clock_edge when the caller has established that the panel clock is running.
    #[inline]
    pub fn clock_edge_running(&mut self) {
        self.clock_edges += 1;
        if self.regs[self.i_scl0].shadow & 0x100 != 0 {
            self.copy_group("sync");
            self.regs[self.i_scl0].shadow &= !0x100;
        }
    }

    pub fn vsync_edge(&mut self) -> VResult<()> {
        if !self.clock_running()? { return Ok(()); }
        self.vsyncs += 1;
        for (req, group) in [(0xFFFF_7980u32, "tcon"), (0xFFFF_7A00, "output")] {
            let i = self.index[&req];
            if self.regs[i].shadow & 1 != 0 { self.copy_group(group); self.regs[i].shadow &= !1; }
        }
        let adj = self.index[&0xFFFF_7680];
        if self.regs[adj].shadow & 1 != 0 { self.copy_group("adj"); self.regs[adj].shadow &= !1; }
        self.scl_event(0x2010);
        for k in 0..3 {
            let ri = self.index[&GR[k]];
            let (i_req, p_req) = (self.regs[ri].shadow & 1 != 0, self.regs[ri].shadow & 16 != 0);
            let g = format!("G{k}");
            for j in 0..self.regs.len() {
                let grp = self.regs[j].group.clone();
                if (grp == format!("{g}I") && i_req) || (grp == format!("{g}P") && p_req) || (grp == format!("{g}both") && i_req && p_req) { let m = self.regs[j].mask; self.copy(j, m); }
                if grp == format!("{g}mixed1") { if i_req { self.copy(j, 0x10300); } if i_req && p_req { self.copy(j, 1); } }
                if grp == format!("{g}mixed6") { if p_req { self.copy(j, 0x103F); } if i_req && p_req { self.copy(j, 0xF3FF_0000); } }
            }
            if p_req { self.clut_known[k] = true; }
            self.regs[ri].shadow &= !0x11;
        }
        for (req, group) in [(0xFFFF_7400u32, "inp"), (0xFFFF_7480, "imgcnt")] {
            if let Some(&i) = self.index.get(&req) { if self.regs[i].shadow & 0x11 != 0 { self.copy_group(group); self.regs[i].shadow &= !0x11; } }
        }
        self.arc_vsync();
        self.signal(1); // LO_VSYNC
        Ok(())
    }

    fn scl_event(&mut self, events: u64) {
        let accepted = self.regs[self.i_scl0].shadow & events;
        if accepted & 0x10 != 0 { self.copy_group("syncB"); let i = self.index[&0xFFFF_7564]; self.copy(i, 1); }
        let keys: Vec<u32> = self.scl_shared.keys().copied().collect();
        for k in keys {
            let f = self.scl_shared[&k];
            if f[2] == 0 { continue; }
            let seen = f[3] | (accepted & f[0]);
            if seen & f[0] == f[0] { let i = self.index[&k]; self.copy(i, f[1]); self.scl_shared.insert(k, [f[0], f[1], 0, seen]); }
            else { self.scl_shared.insert(k, [f[0], f[1], f[2], seen]); }
        }
        self.regs[self.i_scl0].shadow &= !accepted;
    }

    pub fn input_vsync_edge(&mut self) -> VResult<()> {
        if self.clock_running()? {
            self.scl_event(0x1001);
            let i = self.index[&0xFFFF_7580];
            if self.regs[i].shadow & 0x10 != 0 { self.copy_group("scl1B"); }
            if self.regs[i].shadow & 0x01 != 0 { self.copy_group("scl1A"); }
            self.regs[i].shadow &= !0x11;
        }
        Ok(())
    }

    pub fn line_edge(&mut self, line: u32) -> VResult<()> {
        if self.clock_running()? && line as u64 == self.regs[self.i_gr2clut].active & 0x7FF { self.signal(3); } // VLINE
        Ok(())
    }

    pub fn supply_clock(&mut self, source: usize, present: bool) { self.input_clocks[source] = Some(present); }
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct DisplayClock {
    pub vdc: Vdc4,
    numerator: [u64; 4], denominator: [u64; 4], remainder: [u64; 4], pub source_edges: [u64; 4], present: [Option<bool>; 4],
    input_queue: Vec<[u64; 2]>, ready_inputs: Vec<[u64; 2]>, output_queue: Vec<[u64; 2]>,
    pub ticks: u64, pub pixels: u64, divider_remainder: u64, pub hsyncs: u64, pub input_vsyncs: u64, pub output_vsyncs: u64, pub raw_internal_vsyncs: u64,
    pub retimings: u64, pub cancelled_outputs: u64, pub discarded_inputs: u64, hpos: u32, vpos: u32, pub output_line: u32, input_select: u32,
    old_input_select: u32, output_line_known: bool, vdec_enabled: bool, old_panel: u64, old_frc3: u64, old_frc4: u64,
    /// Cached (panel, input_select, module_enabled) -> (running, selected source, divisor).
    clock_key: Option<(u64, u32, bool)>,
    clock_cfg: (bool, i32, u64),
    /// Line count (hsyncs) at the last accepted input Vsync, for the SCL0_FRC1/FRC2 Vsync filters.
    #[serde(skip)]
    last_input_line: Option<u64>,
    /// Input Vsyncs ignored by repeated-Vsync masking, and compensating Vsyncs generated for missing ones.
    #[serde(skip)]
    pub masked_inputs: u64,
    #[serde(skip)]
    pub compensated: u64,
}

impl DisplayClock {
    pub fn from_raw(r: RawDisplayClock) -> VResult<DisplayClock> {
        Ok(DisplayClock {
            vdc: Vdc4::from_raw(r.vdc)?, numerator: r.numerator, denominator: r.denominator, remainder: r.remainder, source_edges: r.source_edges,
            present: r.present, input_queue: r.input_queue, ready_inputs: r.ready_inputs, output_queue: r.output_queue, ticks: r.ticks, pixels: r.pixels,
            divider_remainder: r.divider_remainder, hsyncs: r.hsyncs, input_vsyncs: r.input_vsyncs, output_vsyncs: r.output_vsyncs,
            raw_internal_vsyncs: r.raw_internal_vsyncs, retimings: r.retimings, cancelled_outputs: r.cancelled_outputs, discarded_inputs: r.discarded_inputs,
            hpos: r.hpos, vpos: r.vpos, output_line: r.output_line, input_select: r.input_select, old_input_select: r.old_input_select,
            output_line_known: r.output_line_known, vdec_enabled: r.vdec_enabled, old_panel: r.old_panel, old_frc3: r.old_frc3, old_frc4: r.old_frc4,
            clock_key: None, clock_cfg: (false, -1, 1), last_input_line: None, masked_inputs: 0, compensated: 0,
        })
    }

    pub fn set_vdec_enabled(&mut self, v: bool) { self.vdec_enabled = v; }
    pub fn vdec_enabled(&self) -> bool { self.vdec_enabled }

    /// An input Vsync from the video decoder (camera field) at the next tick; it goes through the same path as the
    /// input events (VI_VSYNC, scaler input latching, input-synchronised output Vsync).
    pub fn queue_input_vsync(&mut self) {
        let e = [self.ticks + 1, 0];
        let pos = self.input_queue.partition_point(|x| x[0] <= e[0]);
        self.input_queue.insert(pos, e);
    }

    /// Clock summary for diagnostics.
    pub fn debug(&self) -> String {
        format!("panel {:x} input_select {} present {:?} num {:?} den {:?} ticks {} pixels {} out_vsyncs {} in_vsyncs {} line {} cfg {:?} vline {} enabled {:x}",
            self.vdc.regs[self.vdc.i_panel].active, self.input_select, self.present, self.numerator, self.denominator, self.ticks, self.pixels,
            self.output_vsyncs, self.input_vsyncs, self.output_line, self.clock_cfg, self.vdc.regs[self.vdc.i_gr2clut].active & 0x7FF, self.vdc.enabled)
            + &format!(" armed {:x} latched {:x} accepted {} frc4 {:x} frc5 {:x} frc3 {:x}", self.vdc.armed, self.vdc.latched, self.vdc.events_accepted,
            self.vdc.regs[self.vdc.i_frc4].active, self.vdc.regs[self.vdc.i_frc5].active, self.vdc.regs[self.vdc.i_frc3].active)
    }

    fn source(&self) -> VResult<usize> {
        let sel = ((self.vdc.regs[self.vdc.i_panel].active >> 12) & 3) as usize;
        if sel == 3 { return Err("prohibited panel source".into()); }
        Ok(if sel == 0 { self.input_select as usize } else { sel + 1 })
    }
    fn divisor(&self) -> VResult<u64> {
        let p = self.vdc.regs[self.vdc.i_panel].active;
        let d = p & 63;
        if ![1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 16, 24, 32].contains(&d) || ((p >> 12) & 3 == 2 && d == 1) { return Err("prohibited panel divisor".into()); }
        Ok(d)
    }
    fn running(&mut self) -> VResult<bool> {
        if !self.vdc.module_enabled || self.vdc.regs[self.vdc.i_panel].active & 0x100 == 0 { return Ok(false); }
        let s = self.source()?;
        self.divisor()?;
        let Some(p) = self.present[s] else { return Err("unknown selected source clock presence".into()) };
        if let Some(x) = self.present[self.input_select as usize] { self.vdc.supply_clock(0, x); }
        if let Some(x) = self.present[2] { self.vdc.supply_clock(1, x); }
        if let Some(x) = self.present[3] { self.vdc.supply_clock(2, x); }
        Ok(p)
    }

    fn sample_active(&mut self) {
        // Active registers change only through VDC writes/latch copies (flagged by `changed`) or input select.
        if !self.vdc.changed && self.input_select == self.old_input_select { return; }
        self.vdc.changed = false;
        let (panel, f3, f4) = (self.vdc.regs[self.vdc.i_panel].active, self.vdc.regs[self.vdc.i_frc3].active, self.vdc.regs[self.vdc.i_frc4].active);
        if panel & 0x303F != self.old_panel & 0x303F || self.input_select != self.old_input_select { self.divider_remainder = 0; self.retimings += 1; }
        if f3 != self.old_frc3 || f4 != self.old_frc4 {
            self.hpos = 0; self.vpos = 0; self.output_line = 0; self.output_line_known = false;
            self.cancelled_outputs += self.output_queue.len() as u64; self.output_queue.clear(); self.retimings += 1;
        }
        self.old_panel = panel; self.old_frc3 = f3; self.old_frc4 = f4; self.old_input_select = self.input_select;
    }

    fn schedule_output(&mut self, origin: u64) {
        let delay = self.vdc.regs[self.vdc.i_frc5].active & 255;
        let h = (self.vdc.regs[self.vdc.i_frc4].active & 2047) + 1;
        self.output_queue.push([self.pixels + delay * h, origin]);
        self.output_queue.sort_by_key(|x| x[0]);
    }

    fn pixel(&mut self) -> VResult<()> {
        self.pixels += 1;
        // `pixel` only runs while the selected clock is running, which is exactly Vdc4::clock_running.
        self.vdc.clock_edge_running();
        self.sample_active();
        let f4 = self.vdc.regs[self.vdc.i_frc4].active;
        let (hp, vp) = ((f4 & 2047) as u32 + 1, ((f4 >> 16) & 2047) as u32 + 1);
        let external = self.vdc.regs[self.vdc.i_frc3].active & 1 == 0;
        let mut line = false;
        self.hpos += 1;
        if self.hpos == hp {
            self.hpos = 0; self.hsyncs += 1; line = true; self.vpos = (self.vpos + 1) % vp;
            if !external && self.vpos == 0 { self.raw_internal_vsyncs += 1; self.schedule_output(0); }
        }
        let accepted = self.ready_inputs.iter().filter(|e| e[1] as u32 == self.input_select && (e[1] != 0 || self.vdec_enabled)).count();
        if accepted > 1 { return Err("multiple input edges within one panel sampling interval".into()); }
        let ready = std::mem::take(&mut self.ready_inputs);
        for e in ready {
            if e[1] as u32 != self.input_select || (e[1] == 0 && !self.vdec_enabled) { self.discarded_inputs += 1; continue; }
            // SCL0_FRC1 (RES_VMASK_ON, RES_VMASK lines): an input Vsync repeated within the masking period is ignored.
            let frc1 = self.vdc.active(0xFFFF_7504);
            if frc1 & 1 != 0 && self.last_input_line.is_some_and(|l| self.hsyncs - l < frc1 >> 16) { self.masked_inputs += 1; continue; }
            self.last_input_line = Some(self.hsyncs);
            self.input_vsyncs += 1;
            self.vdc.input_vsync_edge()?;
            self.vdc.signal(0); // VI_VSYNC
            if external { self.schedule_output(1); }
        }
        let mut output = false;
        while self.output_queue.first().is_some_and(|e| e[0] <= self.pixels) {
            self.output_queue.remove(0);
            self.vdc.vsync_edge()?;
            self.output_vsyncs += 1; self.output_line = 0; self.output_line_known = true; output = true;
            self.vdc.line_edge(0)?;
        }
        // SCL0_FRC2 (RES_VLACK_ON, RES_VLACK lines): with the output following the input, a missing input Vsync is
        // compensated by a Vsync generated RES_VLACK lines after the previous one.
        let frc2 = self.vdc.active(0xFFFF_7508);
        if line && external && frc2 & 1 != 0 && frc2 >> 16 != 0 {
            let last = *self.last_input_line.get_or_insert(self.hsyncs);
            if self.hsyncs - last >= frc2 >> 16 { self.last_input_line = Some(self.hsyncs); self.compensated += 1; self.schedule_output(2); }
        }
        if line && !output && self.output_line_known {
            self.output_line += 1;
            if self.output_line > 2047 { return Err("output line counter exceeded supported no-Vsync interval".into()); }
            self.vdc.line_edge(self.output_line)?;
        }
        Ok(())
    }

    /// `n` pixel clocks: runs that cannot reach a line end, an output event, a pending input edge or a register
    /// update only advance counters; everything else goes through the exact per-pixel path.
    fn pixels_bulk(&mut self, mut n: u64) -> VResult<()> {
        while n > 0 {
            let slow = !self.ready_inputs.is_empty() || self.vdc.changed || self.input_select != self.old_input_select
                || self.vdc.regs[self.vdc.i_scl0].shadow & 0x100 != 0;
            let quiet = if slow { 0 } else {
                let hp = (self.vdc.regs[self.vdc.i_frc4].active & 2047) as u64 + 1;
                let to_line = hp.saturating_sub(1 + self.hpos as u64);
                let to_out = self.output_queue.first().map_or(u64::MAX, |e| e[0].saturating_sub(self.pixels + 1));
                n.min(to_line).min(to_out)
            };
            if quiet == 0 { self.pixel()?; n -= 1; continue; }
            self.pixels += quiet; self.vdc.clock_edges += quiet; self.hpos += quiet as u32; n -= quiet;
        }
        Ok(())
    }

    fn clock_config(&mut self) -> VResult<(bool, i32, u64)> {
        let key = (self.vdc.regs[self.vdc.i_panel].active, self.input_select, self.vdc.module_enabled);
        if self.clock_key != Some(key) {
            let on = self.running()?;
            let selected = if on { self.source()? as i32 } else { -1 };
            let div = if on { self.divisor()? } else { 1 };
            self.clock_cfg = (on, selected, div);
            self.clock_key = Some(key);
        }
        Ok(self.clock_cfg)
    }

    pub fn advance_p0(&mut self, count: u64) -> VResult<()> {
        // Runs of ticks without a due input event are processed together; the tick of an input event alone, so the
        // order (inputs become ready, then that tick's pixels) is the same as ticking one P0 at a time.
        let mut left = count;
        while left > 0 {
            let chunk = match self.input_queue.first() {
                Some(e) if e[0] > self.ticks + 1 => (e[0] - self.ticks - 1).min(left),
                Some(_) => 1,
                None => left,
            };
            left -= chunk;
            self.ticks += chunk;
            self.sample_active();
            let (on, selected, div) = self.clock_config()?;
            while self.input_queue.first().is_some_and(|e| e[0] <= self.ticks) {
                let e = self.input_queue.remove(0);
                if on && e[1] as u32 == self.input_select && (e[1] != 0 || self.vdec_enabled) { self.ready_inputs.push(e); } else { self.discarded_inputs += 1; }
            }
            for s in 0..4 {
                if self.present[s] != Some(true) { continue; }
                let edges = if self.denominator[s] == 1 { self.numerator[s] * chunk } else {
                    self.remainder[s] += self.numerator[s] * chunk;
                    let e = self.remainder[s] / self.denominator[s];
                    self.remainder[s] %= self.denominator[s];
                    e
                };
                self.source_edges[s] += edges;
                if s as i32 == selected && edges > 0 {
                    let total = self.divider_remainder + edges;
                    self.divider_remainder = total % div;
                    self.pixels_bulk(total / div)?;
                }
            }
        }
        Ok(())
    }
}
