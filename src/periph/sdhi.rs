//! SD host interface 0 (0xE803_E000) with an SDHC card. The card is a virtual FAT32 volume (`crate::fat32`):
//! synthetic metadata sectors (MBR, boot sector, FATs, directories) plus the files of a host folder, served from the
//! folder without changing it. Command completion is synchronous (no electrical timing). Written sectors (CMD24/25)
//! go to an overlay over the card (kept in saved states); `sdsync` turns the result back into file changes for the
//! folder (unless `keep_folder`). A `read_only` card refuses writes with a write-protect error.

use serde::Deserialize;
use std::collections::BTreeMap;
use std::fs::File;
use std::io::{Read, Seek, SeekFrom};
use std::path::{Path, PathBuf};

pub const BASE: u32 = 0xE803_E000;
const I1_EVENTS: u32 = 0x031D;
const I2_EVENTS: u32 = 0x837F;
const ILLEGAL: u64 = 1 << 22;
const WP: u64 = 1 << 26;
const RANGE: u64 = 1 << 31;
const BLOCK: u64 = 1 << 29;

/// Reads `buf.len()` bytes at `offset` from the card file named by `path` (host path, or the card-relative name a
/// front end registered). Installed by front ends without a file system (web); otherwise files are opened directly.
pub type FileReader = Box<dyn FnMut(&str, u64, &mut [u8]) -> Result<(), String> + Send>;
static FILE_READER: std::sync::Mutex<Option<FileReader>> = std::sync::Mutex::new(None);

/// Route all SD card file reads through `reader` (None restores direct file access).
pub fn set_file_reader(reader: Option<FileReader>) { *FILE_READER.lock().unwrap() = reader; }

/// Read part of a card file through the installed reader, if any (Some(result)), else None.
pub fn read_via_hook(path: &Path, offset: u64, buf: &mut [u8]) -> Option<Result<(), String>> {
    FILE_READER.lock().unwrap().as_mut().map(|f| f(&path.to_string_lossy(), offset, buf))
}

#[derive(serde::Serialize, serde::Deserialize)]
struct Extent { start: u64, count: u64, meta_offset: Option<u64>, file: Option<(PathBuf, u64)> }

#[derive(serde::Serialize, serde::Deserialize)]
pub struct VirtualSd {
    pub sector_count: u64,
    pub cid: [u8; 16],
    metadata: Vec<u8>,
    extents: Vec<Extent>,
    #[serde(skip)]
    open: Option<(PathBuf, File)>,
    /// Sectors the firmware wrote (LBA -> 512 bytes), served instead of the original content.
    #[serde(default)]
    pub overlay: BTreeMap<u64, Vec<u8>>,
    /// Refuse writes (write-protect error).
    #[serde(default)]
    pub read_only: bool,
    /// The SD folder a synthesised card was built from (where `sdsync` writes changes back).
    #[serde(default)]
    pub root: Option<PathBuf>,
    /// Keep the firmware's writes in this session only (`--sd-readonly`): the card accepts them (a real card does,
    /// and the firmware's driver waits for ever on a refused write) but `sdsync` leaves the folder alone.
    #[serde(default)]
    pub keep_folder: bool,
}

impl VirtualSd {
    /// Card synthesised by `fat32::Volume` (any SD folder, optional extra files), with the emulator's default CID
    /// (`DEFAULT_CID`) until another is set.
    pub fn from_volume(v: crate::fat32::Volume) -> VirtualSd {
        let mut extents: Vec<Extent> = v.meta.iter().map(|&(start, count, off)| Extent { start, count, meta_offset: Some(off), file: None }).collect();
        extents.extend(v.files.into_iter().map(|(start, count, f)| Extent { start, count, meta_offset: None, file: Some((f.path, f.size)) }));
        extents.sort_by_key(|e| e.start);
        VirtualSd { sector_count: v.sector_count, cid: DEFAULT_CID, metadata: v.metadata, extents, open: None, overlay: BTreeMap::new(), read_only: false, root: None, keep_folder: false }
    }

    pub fn read_sector(&mut self, lba: u64) -> Result<[u8; 512], String> {
        let mut out = [0u8; 512];
        if lba >= self.sector_count { return Err("LBA outside card".into()); }
        if let Some(s) = self.overlay.get(&lba) { out.copy_from_slice(s); return Ok(out); }
        self.read_original(lba)
    }

    /// A sector as the card was built (without the firmware's writes).
    pub fn read_original(&mut self, lba: u64) -> Result<[u8; 512], String> {
        let mut out = [0u8; 512];
        if lba >= self.sector_count { return Err("LBA outside card".into()); }
        let Some(e) = self.extents.iter().find(|e| lba >= e.start && lba < e.start + e.count) else { return Ok(out) };
        let off = (lba - e.start) * 512;
        if let Some(m) = e.meta_offset { out.copy_from_slice(&self.metadata[(m + off) as usize..(m + off) as usize + 512]); }
        else if let Some((p, size)) = &e.file {
            let n = 512.min(size.saturating_sub(off)) as usize;
            if n > 0 {
                if let Some(r) = read_via_hook(p, off, &mut out[..n]) { r?; return Ok(out); }
                if self.open.as_ref().is_none_or(|(q, _)| q != p) { self.open = Some((p.clone(), File::open(p).map_err(|err| err.to_string())?)); }
                let f = &mut self.open.as_mut().unwrap().1;
                f.seek(SeekFrom::Start(off)).map_err(|e| e.to_string())?;
                f.read_exact(&mut out[..n]).map_err(|e| e.to_string())?;
            }
        }
        Ok(out)
    }
}

#[derive(Deserialize, Clone)]
#[serde(rename_all = "camelCase")]
pub struct RawSdhi {
    regs: BTreeMap<String, u32>, state: u32, ocr_polls: u32, bus_width: u32, fifo_offset: usize, blocks_left: u32, app: bool, ifcond: bool,
    multi: bool, cd_pullup: bool, next_lba: u64, sectors_read: u64, fifo: Vec<i32>,
}

/// CRC7 (polynomial x^7 + x^3 + 1) as used by SD card registers and commands.
pub fn crc7(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0u8, |mut c, &byte| {
        for i in 0..8 { let bit = (byte >> (7 - i)) & 1; let top = (c >> 6) & 1; c = (c << 1) & 0x7F; if bit ^ top != 0 { c ^= 0x09; } }
        c
    })
}

/// CID of the emulated card unless another is given: manufacturer 0x00, OEM "EM", product "USDCD", revision 1.0,
/// serial 1, made January 2026, with a valid CRC7.
pub const DEFAULT_CID: [u8; 16] = cid_with_crc([0x00, b'E', b'M', b'U', b'S', b'D', b'C', b'D', 0x10, 0, 0, 0, 1, 0x01, 0xA1, 0]);

const fn cid_with_crc(mut c: [u8; 16]) -> [u8; 16] {
    let mut crc = 0u8;
    let mut k = 0;
    while k < 15 {
        let mut i = 0;
        while i < 8 { let bit = (c[k] >> (7 - i)) & 1; let top = (crc >> 6) & 1; crc = (crc << 1) & 0x7F; if bit ^ top != 0 { crc ^= 0x09; } i += 1; }
        k += 1;
    }
    c[15] = (crc << 1) | 1;
    c
}

/// Parse a CID given as 32 hex digits (spaces, colons and dashes ignored), as Linux shows it in
/// `/sys/block/mmcblk0/device/cid`. 30 digits (no CRC byte) are completed with the CRC7.
pub fn parse_cid(s: &str) -> Result<[u8; 16], String> {
    let hex: String = s.chars().filter(|c| !matches!(c, ' ' | ':' | '-')).collect();
    if !(hex.len() == 32 || hex.len() == 30) || !hex.chars().all(|c| c.is_ascii_hexdigit()) { return Err("CID: expected 32 hex digits".into()); }
    let mut cid = [0u8; 16];
    for (i, b) in cid.iter_mut().enumerate().take(hex.len() / 2) { *b = u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).unwrap(); }
    if hex.len() == 30 { cid[15] = (crc7(&cid[..15]) << 1) | 1; }
    Ok(cid)
}

/// SDHI register file (offsets 0..0x100), held flat for the per-clock DMA and interrupt checks; saved as the map of the
/// offsets present (the same form as before).
#[derive(Clone)]
pub struct RegFile { v: [u32; 256], present: [bool; 256] }

impl Default for RegFile { fn default() -> Self { RegFile { v: [0; 256], present: [false; 256] } } }

impl RegFile {
    #[inline] pub fn get(&self, o: &u32) -> Option<&u32> { let i = *o as usize; if i < 256 && self.present[i] { Some(&self.v[i]) } else { None } }
    #[inline] pub fn insert(&mut self, o: u32, val: u32) { let i = o as usize; assert!(i < 256, "SDHI offset {o:x}"); self.v[i] = val; self.present[i] = true; }
    pub fn clear(&mut self) { *self = RegFile::default(); }
    fn map(&self) -> BTreeMap<u32, u32> { (0..256).filter(|&i| self.present[i]).map(|i| (i as u32, self.v[i])).collect() }
}

impl FromIterator<(u32, u32)> for RegFile {
    fn from_iter<I: IntoIterator<Item = (u32, u32)>>(it: I) -> Self { let mut r = RegFile::default(); for (o, v) in it { r.insert(o, v); } r }
}
impl serde::Serialize for RegFile {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> { self.map().serialize(s) }
}
impl<'de> serde::Deserialize<'de> for RegFile {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> { Ok(BTreeMap::<u32, u32>::deserialize(d)?.into_iter().collect()) }
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct Sdhi {
    regs: RegFile,
    state: u32, ocr_polls: u32, bus_width: u32, fifo_offset: usize, blocks_left: u32, app: bool, ifcond: bool, multi: bool, cd_pullup: bool,
    next_lba: u64, pub sectors_read: u64, fifo: Vec<u8>,
    pub card: VirtualSd,
}

const CSD: [u8; 16] = [0x40, 0x0E, 0x00, 0x32, 0x50, 0x59, 0x00, 0x00, 0x1F, 0xFF, 0x7F, 0x80, 0x0A, 0x40, 0x20, 0x29];
const SCR: [u8; 8] = [0x02, 0x05, 0, 0, 0, 0, 0, 0];

type R<T> = Result<T, String>;

impl Sdhi {
    pub fn from_raw(x: RawSdhi, card: VirtualSd) -> R<Sdhi> {
        let regs = x.regs.into_iter().map(|(k, v)| Ok((k.parse::<u32>().map_err(|e| e.to_string())?, v))).collect::<R<_>>()?;
        Ok(Sdhi { regs, state: x.state, ocr_polls: x.ocr_polls, bus_width: x.bus_width, fifo_offset: x.fifo_offset, blocks_left: x.blocks_left, app: x.app,
            ifcond: x.ifcond, multi: x.multi, cd_pullup: x.cd_pullup, next_lba: x.next_lba, sectors_read: x.sectors_read,
            fifo: x.fifo.into_iter().map(|b| b as u8).collect(), card })
    }
    pub fn handles(a: u32, n: usize) -> bool { a >= BASE && a as u64 + n as u64 <= BASE as u64 + 0x100 }
    fn r(&self, o: u32) -> R<u32> { self.regs.get(&o).copied().ok_or(format!("unmodelled SDHI offset {o:02x}")) }
    fn set(&mut self, o: u32, v: u32) { self.regs.insert(o, v & 0xFFFF); }
    fn status(&self, error: u64, application: bool) -> u64 { error | ((self.state as u64) << 9) | if self.state == 3 || self.state == 4 { 256 } else { 0 } | if application { 32 } else { 0 } }
    fn arg(&self) -> R<u64> { Ok(((self.r(6)? as u64) << 16) | self.r(4)? as u64) }
    fn response32(&mut self, v: u64) { for o in (12..=26).step_by(2) { self.set(o, 0); } self.set(12, v as u32); self.set(14, (v >> 16) as u32); }
    fn response128(&mut self, full: &[u8; 16]) {
        // 128-bit register shifted right by 8 into eight 16-bit response words.
        let v = full.iter().fold(0u128, |a, &b| (a << 8) | b as u128) >> 8;
        for i in 0..8 { self.set(12 + i * 2, (v >> (i * 16)) as u32); }
    }
    fn clear_data(&mut self) -> R<()> { self.fifo.clear(); self.fifo_offset = 0; self.blocks_left = 0; self.multi = false; let v = self.r(0x1E)?; self.set(0x1E, v & !0x100); Ok(()) }
    fn data(&mut self, b: Vec<u8>) -> R<()> { self.fifo = b; self.fifo_offset = 0; let v = self.r(0x1E)?; self.set(0x1E, v | 0x100); self.state = 5; Ok(()) }
    fn load_sector(&mut self) -> R<()> {
        if self.next_lba >= self.card.sector_count { return Err("multiblock read beyond card".into()); }
        let s = self.card.read_sector(self.next_lba)?;
        self.next_lba += 1;
        self.sectors_read += 1;
        self.data(s.to_vec())
    }
    /// Host reset (SOFT_RST = 0): register file back to its reset values; card state is kept.
    fn reset_host(&mut self) {
        self.regs.clear();
        for o in [0, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 44, 46, 52, 54, 56, 0xD8, 0xE0, 0xF0, 0xFC, 0xFE] { self.regs.insert(o, 0); }
        self.regs.insert(0x1C, 0x20); // present, write protected
        for o in [0x20, 0x22, 0x38] { self.regs.insert(o, 0xFFFF); }
        self.regs.insert(0xFC, 0x00FF); self.regs.insert(0xFE, 0x00FF);
        self.fifo.clear(); self.fifo_offset = 0; self.blocks_left = 0;
    }
    fn reject(&mut self) { let s = self.status(ILLEGAL, false); self.response32(s); }

    fn execute(&mut self, raw: u32) -> R<()> {
        let cmd = raw & 63;
        let arg = self.arg()?;
        let application = self.app;
        self.app = false;
        if raw & 0x8080 != 0 { return Err(format!("SDHI command flags {raw:04x}")); }
        let mut timeout = false;
        match (application, cmd) {
            (_, 0) => { self.state = 0; self.ifcond = false; self.ocr_polls = 0; self.bus_width = 1; self.cd_pullup = true; self.clear_data()?; self.response32(0); }
            (_, 55) => { if (self.state == 0 && arg == 0) || ((self.state == 3 || self.state == 4) && arg >> 16 == 0x1234) { self.app = true; let s = self.status(0, true); self.response32(s); } else { self.reject(); } }
            (true, 41) => {
                if self.state != 0 { self.reject(); }
                else if !self.ifcond || arg & 0x4000_0000 == 0 || arg & 0xFF_8000 == 0 { self.response32(0x00FF_8000); }
                else { self.ocr_polls += 1; if self.ocr_polls >= 2 { self.state = 1; self.response32(0xC0FF_8000); } else { self.response32(0x00FF_8000); } }
            }
            (true, _) if self.state != 4 => self.reject(),
            (true, 42) => { if arg > 1 { self.reject(); } else { self.cd_pullup = arg != 0; let s = self.status(0, false); self.response32(s); } }
            (true, 6) => { if arg != 0 && arg != 2 { self.reject(); } else { self.bus_width = if arg == 2 { 4 } else { 1 }; let s = self.status(0, false); self.response32(s); } }
            (true, 51) | (true, 13) => {
                let s = self.status(0, false); self.response32(s); self.blocks_left = 1; self.multi = false;
                let mut b = if cmd == 51 { SCR.to_vec() } else { vec![0u8; 64] };
                if cmd == 13 { b[0] = if self.bus_width == 4 { 0x80 } else { 0 }; }
                self.data(b)?;
            }
            (true, c) => return Err(format!("unmodelled ACMD{c}")),
            (false, 8) => { if self.state != 0 { self.reject(); } else if arg & 0xFFFF_FF00 != 0x100 { timeout = true; } else { self.ifcond = true; self.response32(arg & 0xFFF); } }
            (false, 5) => timeout = true,
            (false, 2) => { if self.state != 1 { self.reject(); } else { self.state = 2; let cid = self.card.cid; self.response128(&cid); } }
            (false, 3) => { if self.state != 2 || arg != 0 { self.reject(); } else { self.state = 3; let s = (0x1234u64 << 16) | (self.status(0, false) & 0xFFFF); self.response32(s); } }
            (false, 9) => { if self.state != 3 || arg >> 16 != 0x1234 { self.reject(); } else { self.response128(&CSD); } }
            (false, 7) => {
                if self.state == 3 && arg >> 16 == 0x1234 { self.state = 4; let s = self.status(0, false); self.response32(s); }
                else if (self.state == 3 || self.state == 4) && arg == 0 { self.state = 3; timeout = true; } else { self.reject(); }
            }
            (false, 13) => { if !matches!(self.state, 3 | 4 | 5) || arg >> 16 != 0x1234 { self.reject(); } else { let s = self.status(0, false); self.response32(s); } }
            (false, 16) => { if self.state != 4 { self.reject(); } else { let s = self.status(if arg == 512 { 0 } else { BLOCK }, false); self.response32(s); } }
            (false, 17) | (false, 18) => {
                if self.state != 4 { self.reject(); }
                else if arg >= self.card.sector_count { let s = self.status(RANGE, false); self.response32(s); }
                else {
                    if self.r(0x26)? != 512 { return Err("SD read requires 512-byte host block length".into()); }
                    let s = self.status(0, false); self.response32(s);
                    self.next_lba = arg; self.multi = cmd == 18;
                    self.blocks_left = if self.multi { self.r(0x0A)? } else { 1 };
                    if self.blocks_left == 0 { return Err("SD multiblock count missing".into()); }
                    self.load_sector()?;
                }
            }
            (false, 12) => { if self.state != 5 && self.state != 6 { self.reject(); } else { self.clear_data()?; self.state = 4; let s = self.status(0, false); self.response32(s); let v = self.r(0x1C)?; self.set(0x1C, v | 4); } }
            (false, 24) | (false, 25) if self.card.read_only => { let s = self.status(if self.state == 4 { WP } else { ILLEGAL }, false); self.response32(s); }
            (false, 24) | (false, 25) => {
                // Receive-data state: the host fills SD_BUF block by block (BWE), see `buf_write`.
                if self.state != 4 { self.reject(); }
                else if arg >= self.card.sector_count { let s = self.status(RANGE, false); self.response32(s); }
                else {
                    if self.r(0x26)? != 512 { return Err("SD write requires 512-byte host block length".into()); }
                    let s = self.status(0, false); self.response32(s);
                    self.next_lba = arg; self.multi = cmd == 25;
                    self.blocks_left = if self.multi { self.r(0x0A)? } else { 1 };
                    if self.blocks_left == 0 { return Err("SD multiblock count missing".into()); }
                    self.state = 6; self.fifo.clear(); self.fifo_offset = 0;
                    let v = self.r(0x1E)?; self.set(0x1E, v | 0x200);
                }
            }
            (false, 6) => {
                if self.state != 4 || arg & 0x7F00_0000 != 0 { self.reject(); }
                else {
                    if self.r(0x26)? != 64 { return Err("CMD6 requires 64-byte block length".into()); }
                    let mut b = vec![0u8; 64]; b[1] = 100; b[11] = 1; b[13] = 1;
                    for g in 0..6 { let req = (arg >> (g * 4)) & 15; let sel = if req == 0 || req == 15 { 0 } else { 15 }; b[16 - g as usize / 2] |= (sel << ((g % 2) * 4)) as u8; }
                    let s = self.status(0, false); self.response32(s); self.blocks_left = 1; self.multi = false; self.data(b)?;
                }
            }
            (false, c) => return Err(format!("unmodelled CMD{c}")),
        }
        if timeout { let v = self.r(0x1E)?; self.set(0x1E, v | 0x40); } else { let v = self.r(0x1C)?; self.set(0x1C, v | 1); }
        Ok(())
    }

    pub fn read(&mut self, a: u32, n: usize) -> R<u64> {
        let o = a - BASE;
        if o == 0x30 {
            if n != 4 { return Err("SDHI FIFO requires 32-bit access".into()); }
            if self.state != 5 || self.fifo_offset + 4 > self.fifo.len() { return Err("SDHI FIFO read without data".into()); }
            let v = u32::from_be_bytes(self.fifo[self.fifo_offset..self.fifo_offset + 4].try_into().unwrap());
            self.fifo_offset += 4;
            if self.fifo_offset == self.fifo.len() {
                let x = self.r(0x1E)?; self.set(0x1E, x & !0x100);
                self.blocks_left -= 1;
                if self.multi && self.blocks_left > 0 { self.load_sector()?; } else { self.state = 4; self.clear_data()?; let x = self.r(0x1C)?; self.set(0x1C, x | 4); }
            }
            return Ok(v as u64);
        }
        if n != 2 || o & 1 != 0 { return Err(format!("SDHI register width +{o:02x}/{n}")); }
        Ok(self.r(o)? as u64)
    }

    pub fn write(&mut self, a: u32, n: usize, v: u64) -> R<()> {
        let o = a - BASE;
        if o == 0x30 {
            if n != 4 { return Err("SDHI FIFO requires 32-bit access".into()); }
            return self.buf_write(v as u32);
        }
        if n != 2 || o & 1 != 0 { return Err(format!("SDHI register width +{o:02x}/{n}")); }
        self.r(o)?;
        let v = v as u32;
        if (12..=26).contains(&o) { return Err("SDHI response registers are read-only".into()); }
        match o {
            0x1C => { let x = self.r(o)?; self.set(o, (x & !I1_EVENTS) | (x & v & I1_EVENTS)); return Ok(()); }
            0x1E | 0x36 => { let x = self.r(o)?; self.set(o, x & v); return Ok(()); }
            0xFC | 0xFE => { if v != 0x00FF { return Err("SDHI +FC/+FE only 00FF".into()); } self.set(o, v); return Ok(()); }
            0xE0 => { if v == 0 { self.reset_host(); } self.set(o, v); return Ok(()); }
            0x28 if v & 0x2000 != 0 => return Err("SDHI 8-bit bus unsupported".into()),
            0xD8 if v & !2 != 0 => return Err("SDHI DMA enable bits".into()),
            _ => {}
        }
        self.set(o, v);
        if o == 0 { self.execute(v)?; }
        if o == 8 && v & 1 != 0 && self.state == 5 { self.clear_data()?; self.state = 4; let x = self.r(0x1C)?; self.set(0x1C, x | 4); }
        Ok(())
    }

    /// One longword of write data; a full block goes to the card, then BWE again for the next or access end.
    fn buf_write(&mut self, v: u32) -> R<()> {
        if self.state != 6 { return Err("SDHI FIFO write without a write command".into()); }
        self.fifo.extend_from_slice(&v.to_be_bytes());
        if self.fifo.len() < 512 { return Ok(()); }
        let block = std::mem::take(&mut self.fifo);
        if self.next_lba >= self.card.sector_count { return Err("multiblock write beyond card".into()); }
        self.card.overlay.insert(self.next_lba, block);
        self.next_lba += 1;
        self.blocks_left -= 1;
        let x = self.r(0x1E)?; self.set(0x1E, x & !0x200);
        if self.multi && self.blocks_left > 0 { self.set(0x1E, x | 0x200); }
        else { self.state = 4; let x = self.r(0x1C)?; self.set(0x1C, x | 4); }
        Ok(())
    }

    /// DMA request for SD_BUF write (DMARS 0x11).
    pub fn dma_write_request(&self) -> bool { self.regs.get(&0xD8).copied().unwrap_or(0) & 2 != 0 && self.state == 6 }

    pub fn irq_pending(&self) -> bool {
        let g = |o| self.regs.get(&o).copied().unwrap_or(0);
        ((g(0x1C) & I1_EVENTS & !g(0x20)) | (g(0x1E) & I2_EVENTS & !g(0x22))) != 0
    }
    /// DMA request for SD_BUF read (DMARS 0x12).
    pub fn dma_read_request(&self) -> bool { self.regs.get(&0xD8).copied().unwrap_or(0) & 2 != 0 && self.state == 5 && self.fifo_offset < self.fifo.len() }
}

/// SD host 1 (0xE803_E800, SDIO for the wireless LAN module). Without a card (empty slot) it is a
/// TMIO register file where every command ends in a response timeout (INFO2 bit 6), seeded from the start state's
/// host-register fixture (`Sdhi1HostModel`, no endpoint). With the module attached (`card`) it runs the SDIO commands
/// the firmware's driver uses: CMD5/3/7, CMD52 and CMD53 block transfers through SD_BUF0 (PIO, BRE/BWE per block,
/// access end after the last), and the card interrupt (SDIO_INFO1.IOIRQ while SDIO_MODE.IOMOD is set).
#[derive(serde::Serialize, serde::Deserialize)]
pub struct SdhiEmpty {
    regs: RegFile,
    pub commands: u64,
    /// The wireless LAN module (not kept in saved states: the firmware probes it at boot).
    #[serde(skip)]
    pub card: Option<Box<crate::wlan::Wlan>>,
    #[serde(skip)]
    xfer: Option<Xfer>,
}

/// A CMD53 data phase: function, register address, block length, blocks left, and the bytes moved so far.
struct Xfer { write: bool, func: u8, addr: u32, block: usize, blocks: u32, data: Vec<u8>, pos: usize }

#[derive(Deserialize, Clone)]
pub struct RawSdhi1 { #[serde(default)] pub registers: BTreeMap<String, u32> }

pub const SDHI1_BASE: u32 = 0xE803_E800;

impl SdhiEmpty {
    pub fn new(seed: Option<&RawSdhi1>) -> SdhiEmpty {
        let mut regs = RegFile::default();
        if let Some(s) = seed { for (k, v) in &s.registers { if let Ok(o) = k.parse::<u32>() { regs.insert(o, *v); } } }
        SdhiEmpty { regs, commands: 0, card: None, xfer: None }
    }
    pub fn handles(a: u32, n: usize) -> bool { a >= SDHI1_BASE && a as u64 + n as u64 <= SDHI1_BASE as u64 + 0x100 }
    fn g(&self, o: u32) -> u32 { self.regs.get(&o).copied().unwrap_or(0) }
    fn or(&mut self, o: u32, bits: u32) { let v = self.g(o) | bits; self.regs.insert(o, v); }

    pub fn read(&mut self, a: u32, n: usize) -> R<u64> {
        let o = a - SDHI1_BASE;
        if o == 0x30 && n == 4 && self.card.is_some() { return Ok(self.buf_read() as u64); }
        if n != 2 || o & 1 != 0 { return Err(format!("SDHI1 width +{o:02x}/{n}")); }
        // With the module fitted: card present (CD) and not write-protected.
        if o == 0x1C && self.card.is_some() { return Ok((self.g(o) | 0xA0) as u64); }
        Ok(self.g(o) as u64)
    }
    pub fn write(&mut self, a: u32, n: usize, v: u64) -> R<()> {
        let o = a - SDHI1_BASE;
        if o == 0x30 && n == 4 && self.card.is_some() { self.buf_write(v as u32); return Ok(()); }
        if n != 2 || o & 1 != 0 { return Err(format!("SDHI1 width +{o:02x}/{n}")); }
        let v = v as u32 & 0xFFFF;
        let cur = self.g(o);
        match o {
            0x1C => { self.regs.insert(o, cur & (v | !I1_EVENTS)); }
            0x1E | 0x36 => { self.regs.insert(o, cur & v); }
            0x00 => {
                self.regs.insert(o, v); self.commands += 1;
                if self.card.is_some() { self.sdio_command(v); } else { self.or(0x1E, 0x40); }
            }
            _ => { self.regs.insert(o, v); }
        }
        Ok(())
    }
    pub fn irq_pending(&self) -> bool {
        ((self.g(0x1C) & I1_EVENTS & !self.g(0x20)) | (self.g(0x1E) & I2_EVENTS & !self.g(0x22)) | (self.g(0x36) & 0xC007 & !self.g(0x38))) != 0
    }

    /// Card time: the module's own activity, and its interrupt latched into SDIO_INFO1.IOIRQ while IOMOD is set.
    pub fn advance(&mut self, now: u64) {
        let Some(c) = &mut self.card else { return };
        c.advance(now);
        if c.irq_line() && self.g(0x34) & 1 != 0 { self.or(0x36, 1); }
    }

    fn response(&mut self, v: u32) {
        for o in (0x0C..=0x1A).step_by(2) { self.regs.insert(o, 0); }
        self.regs.insert(0x0C, v & 0xFFFF); self.regs.insert(0x0E, v >> 16);
        self.or(0x1C, 1);
    }

    fn sdio_command(&mut self, raw: u32) {
        let arg = (self.g(6) << 16) | self.g(4);
        let (size, count_reg) = (self.g(0x26), self.g(0x0A));
        let card = self.card.as_mut().unwrap();
        match raw & 63 {
            5 => self.response(0x90FF_8000), // R4: ready, one I/O function, no memory, 2.7-3.6 V
            3 => self.response(0x0001_0000), // R6: RCA 1
            7 | 0 => self.response(0),
            52 => {
                let (write, func, addr, data) = (arg >> 31 != 0, ((arg >> 28) & 7) as u8, (arg >> 9) & 0x1FFFF, arg as u8);
                let v = card.cmd52(func, addr, write, data);
                self.response(0x1000 | v as u32);
            }
            53 => {
                let (write, func, addr, count) = (arg >> 31 != 0, ((arg >> 28) & 7) as u8, (arg >> 9) & 0x1FFFF, arg & 0x1FF);
                let block_mode = arg & 0x0800_0000 != 0;
                let block = if block_mode { size as usize } else if count == 0 { 512 } else { count as usize };
                let blocks = if block_mode { count_reg.max(1) } else { 1 };
                let data = if write { Vec::new() } else { card.read_data(func, addr, block * blocks as usize) };
                self.xfer = Some(Xfer { write, func, addr, block: block.max(4), blocks, data, pos: 0 });
                self.response(0x1000);
                self.or(0x1E, if write { 0x200 } else { 0x100 });
            }
            c => { if std::env::var_os("AR_WLAN_LOG").is_some() { eprintln!("[sdhi1] CMD{c} arg {arg:08x}: no response"); } self.or(0x1E, 0x40); }
        }
    }

    fn block_done(&mut self) {
        let x = self.xfer.as_mut().unwrap();
        x.blocks -= 1;
        if x.blocks > 0 { let bit = if x.write { 0x200 } else { 0x100 }; self.or(0x1E, bit); return; }
        let x = self.xfer.take().unwrap();
        if x.write { if let Some(c) = &mut self.card { c.write_data(x.func, x.addr, &x.data); } }
        self.or(0x1C, 4); // access end
    }

    fn buf_read(&mut self) -> u32 {
        let Some(x) = &mut self.xfer else { return 0 };
        if x.write { return 0; }
        let v = u32::from_be_bytes(std::array::from_fn(|i| x.data.get(x.pos + i).copied().unwrap_or(0)));
        x.pos += 4;
        if x.pos % x.block == 0 { self.block_done(); }
        v
    }

    fn buf_write(&mut self, v: u32) {
        let Some(x) = &mut self.xfer else { return };
        if !x.write { return; }
        x.data.extend(v.to_be_bytes());
        x.pos += 4;
        if x.pos % x.block == 0 { self.block_done(); }
    }
}
