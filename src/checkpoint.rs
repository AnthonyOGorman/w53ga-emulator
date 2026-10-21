//! Machine start states in JSON form: CPU registers, register banks, MMIO register latches (address, size, write and
//! read masks, access behaviour, reset value) and the initial state of the peripheral models.
//!
//! The power-on reset state (`data/power-on.json.gz`, embedded by `build.rs`) is one of these with every latch at its
//! documented reset value, zeroed RAM and no memory pages; the program counter and stack pointer are not part of it
//! but come from the flash image's vector table (`Machine::power_on`).

use crate::cpu::{Bank, Cpu};
use crate::mem::{region, Region};
use base64::Engine;
use serde::Deserialize;
use std::collections::HashMap;
use std::io::Read;
use std::path::Path;

#[derive(Deserialize)]
struct RawPage {
    index: u64,
    data: String,
    #[serde(default)]
    mask: Option<String>,
}

/// Generic register latch: value, write mask, read mask and access behaviour.
#[derive(Deserialize, Clone, Debug)]
#[serde(rename_all = "camelCase")]
pub struct MmioLatch {
    pub addr: u64,
    pub value: u64,
    pub size: u32,
    pub name: String,
    #[serde(default)]
    pub known: u64,
    #[serde(default)]
    pub mask: u64,
    #[serde(default)]
    pub read_mask: u64,
    #[serde(default)]
    pub behavior: String,
    #[serde(default)]
    pub cmf_read: bool,
    #[serde(default)]
    pub status_read: u64,
    #[serde(default)]
    pub direction: u64,
    #[serde(default)]
    pub pin_mask: u64,
    #[serde(default)]
    pub pin_value: u64,
}

#[derive(Deserialize)]
#[allow(non_snake_case)]
struct RawFields {
    physicalBanks: Vec<HashMap<String, u64>>,
    regs: Vec<MmioLatch>,
    #[serde(default)]
    cmtRemainder: Vec<u64>,
    #[serde(default)]
    mtuRemainder: Vec<u64>,
    #[serde(default)]
    displayClock: Option<crate::periph::display::RawDisplayClock>,
    #[serde(default)]
    iicInit: Option<crate::periph::iic::RawIic>,
    #[serde(default)]
    iic1Transfer: Option<crate::periph::iic::RawIic>,
    #[serde(default)]
    rtc: Option<crate::periph::rtc::RawRtc>,
    #[serde(default)]
    scif7: Option<crate::periph::scif::RawScif7>,
    #[serde(default)]
    scif7TransportActive: bool,
    #[serde(default)]
    vdecInit: Option<RawVdec>,
    #[serde(default)]
    scif1: Option<crate::periph::scif::RawAdapter>,
    #[serde(default)]
    scif1Active: bool,
    #[serde(default)]
    scif6: Option<crate::periph::scif::RawAdapter>,
    #[serde(default)]
    scif6Active: bool,
    #[serde(default)]
    dma1Init: Option<RawDmaInit>,
    #[serde(default)]
    dma2Init: Option<RawDmaInit>,
    #[serde(default)]
    dma9: Option<RawDmaTx>,
    #[serde(default)]
    dma10: Option<RawDmaTx>,
    #[serde(default)]
    jcuEngine: Option<crate::periph::jcu::RawJcu>,
    #[serde(default)]
    ssiTx: Option<RawSsiTx>,
    #[serde(default)]
    srcControl: Option<RawSrc>,
    #[serde(default)]
    sdhi: Option<crate::periph::sdhi::RawSdhi>,
    #[serde(default)]
    sdhi1: Option<crate::periph::sdhi::RawSdhi1>,
}

#[derive(Deserialize, Clone)]
pub struct RawSsiClock { pub ten: bool, #[serde(default)] pub fifo: Vec<serde_json::Value> }
#[derive(Deserialize, Clone)]
pub struct RawSsiTx { pub cr: i64, pub fcr: i64, pub clock: RawSsiClock }
#[derive(Deserialize, Clone)]
#[serde(rename_all = "camelCase")]
pub struct RawSrc { #[serde(default)] pub input_control: i64, #[serde(default)] pub output_control: i64 }

/// DMA1/DMA2 channel state: registers r = [SAR, DAR, DMATCR, CHCR] and their DMARS word.
#[derive(Deserialize, Clone)]
#[serde(rename_all = "camelCase")]
pub struct RawDmaInit { pub r: [u64; 4], #[serde(default)] pub dmars0: Option<u64>, #[serde(default)] pub dmars1: Option<u64>, #[serde(default)] pub reload_destination: Option<u64> }
/// SCIF transmit DMA channel state (DMA9/DMA10).
#[derive(Deserialize, Clone)]
#[serde(rename_all = "camelCase")]
pub struct RawDmaTx { pub regs: [u64; 4], pub dmars: u64, pub arm_count: u64, pub units: u64 }

#[derive(Deserialize, Clone)]
pub struct RawVdec { #[serde(default)] pub values: HashMap<String, u64>, #[serde(default)] pub active: HashMap<String, u64> }

#[derive(Deserialize)]
struct RawFull {
    schema: String,
    steps: u64,
    script_sha256: String,
    cpu_registers: HashMap<String, String>,
    memory_pages: Vec<RawPage>,
    page_size: usize,
    fields: RawFields,
}

/// Power-on state supplied at run time (takes precedence over the one embedded at build time).
static POWER_ON_OVERRIDE: std::sync::Mutex<Option<Vec<u8>>> = std::sync::Mutex::new(None);

/// Use this power-on state (gzip-compressed JSON) from now on. It is parsed once to check it.
pub fn set_power_on_state(gz: Vec<u8>) -> Result<(), String> {
    Checkpoint::from_gz(&gz[..])?;
    *POWER_ON_OVERRIDE.lock().unwrap() = Some(gz);
    Ok(())
}

/// Whether a unit can be switched on from reset (a power-on state was embedded or supplied).
pub fn power_on_available() -> bool { crate::embedded::POWER_ON.is_some() || POWER_ON_OVERRIDE.lock().unwrap().is_some() }

/// Accepted schema names (the second is an earlier name of the same format).
const SCHEMAS: [&str; 2] = ["w53ga-power-on-v1", "cellstar-full-state-checkpoint-v1"];

pub struct Checkpoint {
    pub steps: u64,
    pub script_sha256: String,
    pub regs: HashMap<String, u32>,
    pub banks: [Bank; 15],
    pub latches: Vec<MmioLatch>,
    pub cmt_remainder: [u64; 2],
    pub mtu_remainder: Vec<u64>,
    pub display: Option<crate::periph::display::RawDisplayClock>,
    pub iic2: Option<crate::periph::iic::RawIic>,
    pub iic1: Option<crate::periph::iic::RawIic>,
    pub rtc: Option<crate::periph::rtc::RawRtc>,
    pub scif7: Option<(crate::periph::scif::RawScif7, bool)>,
    pub vdec: Option<RawVdec>,
    pub scif1: Option<(crate::periph::scif::RawAdapter, bool)>,
    pub scif6: Option<(crate::periph::scif::RawAdapter, bool)>,
    pub dma: Vec<(usize, [u64; 4], u64)>,
    pub dmars: Vec<(usize, u64)>,
    pub jcu: Option<crate::periph::jcu::RawJcu>,
    pub ssi: Option<RawSsiTx>,
    pub src: Option<RawSrc>,
    pub dma1_rdar: Option<u64>,
    pub sdhi: Option<crate::periph::sdhi::RawSdhi>,
    pub sdhi1: Option<crate::periph::sdhi::RawSdhi1>,
    pub pages: Vec<(u32, Vec<u8>)>,
}

pub struct LoadReport {
    pub flash_pages: usize,
    pub flash_mismatch_bytes: usize,
    pub sdram_pages: usize,
    pub sdram_mirror_conflict_bytes: usize,
    pub oram_pages: usize,
    pub other_pages: usize,
}

/// DMAC channel registers held per channel: (channel, [SAR, DAR, DMATCR, CHCR], arm count).
fn dma_channels(d1: &Option<RawDmaInit>, d2: &Option<RawDmaInit>, d9: &Option<RawDmaTx>, d10: &Option<RawDmaTx>) -> Vec<(usize, [u64; 4], u64)> {
    let mut v = Vec::new();
    if let Some(x) = d1 { v.push((1, x.r, x.r[2])); }
    if let Some(x) = d2 { v.push((2, x.r, x.r[2])); }
    if let Some(x) = d9 { v.push((9, x.regs, x.arm_count)); }
    if let Some(x) = d10 { v.push((10, x.regs, x.arm_count)); }
    v
}
/// DMARS words (index k = channels 2k/2k+1) held by those models.
fn dmars_words(d1: &Option<RawDmaInit>, d2: &Option<RawDmaInit>, d9: &Option<RawDmaTx>, d10: &Option<RawDmaTx>) -> Vec<(usize, u64)> {
    let mut v = Vec::new();
    if let Some(w) = d1.as_ref().and_then(|x| x.dmars0) { v.push((0, w)); }
    if let Some(w) = d2.as_ref().and_then(|x| x.dmars1) { v.push((1, w)); }
    if let Some(x) = d9 { v.push((4, x.dmars)); }
    if let Some(x) = d10 { v.push((5, x.dmars)); }
    v
}

impl Checkpoint {
    /// A start state from a gzip-compressed JSON file.
    pub fn load(path: &Path) -> Result<Checkpoint, String> {
        let f = std::fs::File::open(path).map_err(|e| format!("{}: {e}", path.display()))?;
        Self::from_gz(std::io::BufReader::new(f))
    }

    /// The chip at the moment power is applied: every register latch at its documented reset value, fresh peripheral
    /// models, zeroed RAM. PC and SP are not included (see `Machine::power_on`).
    pub fn power_on() -> Result<Checkpoint, String> {
        if let Some(d) = POWER_ON_OVERRIDE.lock().unwrap().as_ref() { return Self::from_gz(&d[..]); }
        match crate::embedded::POWER_ON {
            Some(d) => Self::from_gz(d),
            None => Err("no power-on state: supply one (--power-on-state FILE, or the web page's power-on state picker) or build with data/power-on.json.gz (see README)".into()),
        }
    }

    /// A start state from gzip-compressed JSON bytes.
    pub fn from_gz(r: impl Read) -> Result<Checkpoint, String> {
        let mut text = String::new();
        flate2::read::GzDecoder::new(r).read_to_string(&mut text).map_err(|e| e.to_string())?;
        let raw: RawFull = serde_json::from_str(&text).map_err(|e| format!("parse: {e}"))?;
        if !SCHEMAS.contains(&raw.schema.as_str()) { return Err(format!("unknown start-state schema {}", raw.schema)); }
        let regs = raw.cpu_registers.iter()
            .map(|(k, v)| Ok((k.clone(), u64::from_str_radix(v, 16).map_err(|e| e.to_string())? as u32)))
            .collect::<Result<HashMap<_, _>, String>>()?;
        if raw.fields.physicalBanks.len() != 15 { return Err("physical bank count".into()); }
        let mut banks = [Bank::default(); 15];
        for (i, b) in raw.fields.physicalBanks.iter().enumerate() {
            let g = |k: &str| b.get(k).copied().unwrap_or(0) as u32;
            for r in 0..15 { banks[i].r[r] = g(&format!("r{r}")); }
            banks[i].gbr = g("gbr"); banks[i].mach = g("mach"); banks[i].macl = g("macl"); banks[i].pr = g("pr"); banks[i].vto = g("vto");
        }
        let b64 = base64::engine::general_purpose::STANDARD;
        let mut pages = Vec::with_capacity(raw.memory_pages.len());
        for p in raw.memory_pages {
            let data = b64.decode(p.data.as_bytes()).map_err(|e| e.to_string())?;
            if data.len() < raw.page_size { return Err("page size".into()); }
            // A page may carry a larger backing array; only the first page_size bytes are addressed.
            let data = data[..raw.page_size].to_vec();
            if let Some(m) = p.mask {
                // Partially initialised page: keep only initialised bytes (the others read as 0).
                let mask = b64.decode(m.as_bytes()).map_err(|e| e.to_string())?;
                let mut d = data;
                for (i, byte) in d.iter_mut().enumerate() { if mask.get(i / 8).map_or(true, |mb| mb & (1 << (i % 8)) == 0) { *byte = 0; } }
                pages.push((p.index as u32, d));
            } else {
                pages.push((p.index as u32, data));
            }
        }
        let cr = &raw.fields.cmtRemainder;
        let cmt_remainder = [cr.first().copied().unwrap_or(0), cr.get(1).copied().unwrap_or(0)];
        Ok(Checkpoint { steps: raw.steps, script_sha256: raw.script_sha256, regs, banks, latches: raw.fields.regs, cmt_remainder,
            mtu_remainder: raw.fields.mtuRemainder, display: raw.fields.displayClock, iic2: raw.fields.iicInit, iic1: raw.fields.iic1Transfer, rtc: raw.fields.rtc,
            scif7: raw.fields.scif7.map(|s| (s, raw.fields.scif7TransportActive)), vdec: raw.fields.vdecInit,
            scif1: raw.fields.scif1.map(|s| (s, raw.fields.scif1Active)), scif6: raw.fields.scif6.map(|s| (s, raw.fields.scif6Active)),
            dma: dma_channels(&raw.fields.dma1Init, &raw.fields.dma2Init, &raw.fields.dma9, &raw.fields.dma10),
            dmars: dmars_words(&raw.fields.dma1Init, &raw.fields.dma2Init, &raw.fields.dma9, &raw.fields.dma10), jcu: raw.fields.jcuEngine,
            ssi: raw.fields.ssiTx, src: raw.fields.srcControl, dma1_rdar: raw.fields.dma1Init.as_ref().and_then(|d| d.reload_destination), sdhi: raw.fields.sdhi, sdhi1: raw.fields.sdhi1, pages })
    }

    /// Install memory (flash must already be the original dump) and CPU state.
    pub fn apply(&self, cpu: &mut Cpu) -> LoadReport {
        let mut rep = LoadReport { flash_pages: 0, flash_mismatch_bytes: 0, sdram_pages: 0, sdram_mirror_conflict_bytes: 0, oram_pages: 0, other_pages: 0 };
        // Canonical SDRAM pages first (0x1Cxx_xxxx), then compare uncached-mirror pages (0x3Cxx_xxxx) against them.
        let mut sorted: Vec<&(u32, Vec<u8>)> = self.pages.iter().collect();
        sorted.sort_by_key(|(a, _)| if (0x3C00_0000..0x3C28_0000).contains(a) { 1 } else { 0 });
        let mut sdram_seen = std::collections::HashSet::new();
        for (addr, data) in sorted {
            match region(*addr) {
                Region::Flash(o) => {
                    rep.flash_pages += 1;
                    rep.flash_mismatch_bytes += cpu.mem.flash[o..o + data.len()].iter().zip(data).filter(|(a, b)| a != b).count();
                }
                Region::Sdram(o) => {
                    rep.sdram_pages += 1;
                    if sdram_seen.insert(o) { cpu.mem.sdram[o..o + data.len()].copy_from_slice(data); }
                    else { rep.sdram_mirror_conflict_bytes += cpu.mem.sdram[o..o + data.len()].iter().zip(data).filter(|(a, b)| a != b).count(); }
                }
                Region::Oram(o) => { rep.oram_pages += 1; cpu.mem.oram[o..o + data.len()].copy_from_slice(data); }
                Region::Mmio => rep.other_pages += 1,
            }
        }
        let r = &self.regs;
        for i in 0..16 { cpu.r[i] = r[&format!("r{i}")]; cpu.fr[i] = r[&format!("fr{i}")]; }
        cpu.sr = r["sr"]; cpu.gbr = r["gbr"]; cpu.vbr = r["vbr"]; cpu.mach = r["mach"]; cpu.macl = r["macl"];
        cpu.pr = r["pr"]; cpu.pc = r["pc"]; cpu.fpscr = r["fpscr"]; cpu.fpul = r["fpul"]; cpu.tbr = r["tbr"];
        cpu.banks = self.banks;
        rep
    }

    pub fn latch(&self, addr: u32) -> Option<&MmioLatch> { self.latches.iter().find(|l| l.addr as u32 == addr) }
}
