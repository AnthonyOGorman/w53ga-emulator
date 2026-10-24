//! Turns what the emulated software wrote to the SD card back into file changes for the folder the card was built from.
//!
//! The card (`sdhi::VirtualSd`) keeps written sectors in an overlay. Here both versions of the FAT32 volume, as built
//! and with the overlay, are read as file trees and compared (`changes`): files that are new or whose size, first
//! cluster or any sector changed are written, new directories are created, and files that are gone are deleted.
//! `sync_back` applies them to a host folder; the web front end applies them through the File System Access API.
//! Files added with `--sd-add` are not in the folder and are never touched there. If the written volume cannot be
//! read, or more than `MAX_DELETES` files would disappear, nothing is changed (a damaged FAT must not empty the folder).

use crate::periph::sdhi::VirtualSd;
use std::collections::{BTreeMap, BTreeSet};

const MAX_DELETES: usize = 20;

#[derive(Clone, PartialEq)]
struct Entry { dir: bool, cluster: u32, size: u32 }

struct Fat { spc: u64, fat_start: u64, data_start: u64, root: u32 }

type Reader<'a> = dyn FnMut(u64) -> Result<[u8; 512], String> + 'a;

fn geometry(rd: &mut Reader) -> Result<Fat, String> {
    let mbr = rd(0)?;
    let part = u32::from_le_bytes(mbr[454..458].try_into().unwrap()) as u64;
    let bs = rd(part)?;
    if &bs[82..87] != b"FAT32" { return Err("not a FAT32 volume".into()); }
    let spc = bs[13] as u64;
    let reserved = u16::from_le_bytes([bs[14], bs[15]]) as u64;
    let fats = bs[16] as u64;
    let fat_sectors = u32::from_le_bytes(bs[36..40].try_into().unwrap()) as u64;
    let root = u32::from_le_bytes(bs[44..48].try_into().unwrap());
    if spc == 0 || fats == 0 { return Err("bad boot sector".into()); }
    Ok(Fat { spc, fat_start: part + reserved, data_start: part + reserved + fats * fat_sectors, root })
}

fn next_cluster(f: &Fat, rd: &mut Reader, c: u32) -> Result<u32, String> {
    let s = rd(f.fat_start + c as u64 * 4 / 512)?;
    let o = (c as usize * 4) % 512;
    Ok(u32::from_le_bytes(s[o..o + 4].try_into().unwrap()) & 0x0FFF_FFFF)
}

/// LBAs of a cluster chain (at most `limit` clusters).
fn chain(f: &Fat, rd: &mut Reader, first: u32, limit: u64) -> Result<Vec<u64>, String> {
    let mut out = Vec::new();
    let mut c = first;
    let mut n = 0;
    while (2..0x0FFF_FFF8).contains(&c) {
        if n >= limit { return Err(format!("cluster chain from {first} too long")); }
        let lba = f.data_start + (c as u64 - 2) * f.spc;
        out.extend(lba..lba + f.spc);
        c = next_cluster(f, rd, c)?;
        n += 1;
    }
    Ok(out)
}

fn sfn(e: &[u8]) -> String {
    let lower = |s: &[u8], on: bool| -> String { let t = String::from_utf8_lossy(s).trim_end().to_string(); if on { t.to_lowercase() } else { t } };
    let base = lower(&e[0..8], e[12] & 0x08 != 0);
    let ext = lower(&e[8..11], e[12] & 0x10 != 0);
    if ext.is_empty() { base } else { format!("{base}.{ext}") }
}

/// Every file and directory: path (with '/') -> entry.
fn tree(rd: &mut Reader) -> Result<BTreeMap<String, Entry>, String> {
    let f = geometry(rd)?;
    let mut out = BTreeMap::new();
    let mut todo = vec![(String::new(), f.root)];
    while let Some((path, cluster)) = todo.pop() {
        if out.len() > 100_000 { return Err("too many entries".into()); }
        let mut lfn: Vec<(u8, Vec<u16>)> = Vec::new();
        'sectors: for lba in chain(&f, rd, cluster, 4096)? {
            let s = rd(lba)?;
            for e in s.chunks(32) {
                match e[0] {
                    0 => break 'sectors,
                    0xE5 => { lfn.clear(); continue; }
                    _ => {}
                }
                if e[11] == 0x0F {
                    let mut u: Vec<u16> = Vec::new();
                    for r in [1..11, 14..26, 28..32] { u.extend(e[r].chunks(2).map(|p| u16::from_le_bytes([p[0], p[1]]))); }
                    lfn.push((e[0] & 0x1F, u));
                    continue;
                }
                let long = if lfn.is_empty() { None } else {
                    lfn.sort_by_key(|x| x.0);
                    let u: Vec<u16> = lfn.iter().flat_map(|x| x.1.iter().copied()).take_while(|&c| c != 0 && c != 0xFFFF).collect();
                    Some(String::from_utf16_lossy(&u))
                };
                lfn.clear();
                if e[11] & 0x08 != 0 || e[0] == b'.' { continue; }
                let name = long.unwrap_or_else(|| sfn(e));
                let p = if path.is_empty() { name } else { format!("{path}/{name}") };
                let c = (u16::from_le_bytes([e[20], e[21]]) as u32) << 16 | u16::from_le_bytes([e[26], e[27]]) as u32;
                let dir = e[11] & 0x10 != 0;
                out.insert(p.clone(), Entry { dir, cluster: c, size: u32::from_le_bytes(e[28..32].try_into().unwrap()) });
                if dir { todo.push((p, c)); }
            }
        }
    }
    Ok(out)
}

/// One change to the card's folder.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Change {
    /// A directory that is new on the card.
    Mkdir(String),
    /// A file that is new (`existed` false) or changed, with its whole new content.
    Write { path: String, data: Vec<u8>, existed: bool },
    /// A file that was on the card and is gone.
    Delete(String),
}

/// The differences between the card as built and as the software left it (paths with '/', as on the card). Empty when
/// nothing was written or the card is read-only.
pub fn changes(card: &mut VirtualSd) -> Result<Vec<Change>, String> {
    if card.read_only || card.overlay.is_empty() { return Ok(Vec::new()); }
    let before = tree(&mut |l| card.read_original(l))?;
    let after = tree(&mut |l| card.read_sector(l)).map_err(|e| format!("written card not readable ({e}); SD folder left unchanged"))?;
    let key = |p: &String| p.to_lowercase();
    let old: BTreeMap<String, &Entry> = before.iter().map(|(p, e)| (key(p), e)).collect();
    let new_keys: BTreeSet<String> = after.keys().map(key).collect();
    let gone: Vec<&String> = before.iter().filter(|(p, e)| !e.dir && !new_keys.contains(&key(p))).map(|(p, _)| p).collect();
    if gone.len() > MAX_DELETES { return Err(format!("{} files would be deleted; SD folder left unchanged", gone.len())); }
    let f = geometry(&mut |l| card.read_sector(l))?;
    let mut out = Vec::new();
    for (p, e) in &after {
        let was = old.get(&key(p));
        if e.dir {
            if !was.is_some_and(|w| w.dir) { out.push(Change::Mkdir(p.clone())); }
            continue;
        }
        let sectors = if e.size == 0 { Vec::new() } else { chain(&f, &mut |l| card.read_sector(l), e.cluster, (e.size as u64).div_ceil(512 * f.spc) + 1)? };
        let touched = sectors.iter().any(|l| card.overlay.contains_key(l));
        if was.is_some_and(|w| **w == *e) && !touched { continue; }
        let mut data = Vec::with_capacity(e.size as usize);
        for l in &sectors { if data.len() >= e.size as usize { break; } data.extend_from_slice(&card.read_sector(*l)?); }
        data.truncate(e.size as usize);
        if data.len() != e.size as usize { return Err(format!("{p}: chain shorter than its size")); }
        out.push(Change::Write { path: p.clone(), data, existed: was.is_some() });
    }
    out.extend(gone.into_iter().map(|p| Change::Delete(p.clone())));
    Ok(out)
}

/// Bring the card's SD folder up to date with what the software wrote. Returns the changes made (empty when nothing
/// was written, the card is read-only or kept in the session, or it was not built from a folder).
pub fn sync_back(card: &mut VirtualSd) -> Result<Vec<String>, String> {
    let Some(root) = card.root.clone() else { return Ok(Vec::new()) };
    if card.keep_folder { return Ok(Vec::new()); }
    let mut done = Vec::new();
    for c in changes(card)? {
        match c {
            Change::Mkdir(p) => {
                let host = root.join(&p);
                if !host.is_dir() { std::fs::create_dir_all(&host).map_err(|err| format!("{}: {err}", host.display()))?; done.push(format!("created folder {p}")); }
            }
            Change::Write { path, data, existed } => {
                let host = root.join(&path);
                // A file that was on the card but not in the folder came from --sd-add: leave its host file alone.
                if existed && !host.is_file() { continue; }
                if std::fs::read(&host).is_ok_and(|d| d == data) { continue; }
                if let Some(d) = host.parent() { std::fs::create_dir_all(d).map_err(|err| err.to_string())?; }
                std::fs::write(&host, &data).map_err(|err| format!("{}: {err}", host.display()))?;
                done.push(format!("{} {path} ({} bytes)", if existed { "updated" } else { "added" }, data.len()));
            }
            Change::Delete(p) => {
                let host = root.join(&p);
                if !host.is_file() { continue; }
                std::fs::remove_file(&host).map_err(|err| format!("{}: {err}", host.display()))?;
                done.push(format!("deleted {p}"));
            }
        }
    }
    Ok(done)
}

/// Files added with `--sd-persist` (card path, host file): what the software left in them is written to their host
/// files, whatever the folder's mode. Returns the files written.
pub fn persist_back(card: &mut VirtualSd, files: &[(String, std::path::PathBuf)]) -> Result<Vec<String>, String> {
    if files.is_empty() { return Ok(Vec::new()); }
    let mut done = Vec::new();
    for c in changes(card)? {
        let Change::Write { path, data, .. } = c else { continue };
        let Some((_, host)) = files.iter().find(|(cp, _)| cp.trim_start_matches('/').eq_ignore_ascii_case(&path)) else { continue };
        if std::fs::read(host).is_ok_and(|d| d == data) { continue; }
        std::fs::write(host, &data).map_err(|err| format!("{}: {err}", host.display()))?;
        done.push(format!("{path} -> {} ({} bytes)", host.display(), data.len()));
    }
    Ok(done)
}

/// `persist_back` on a machine's card, printed.
pub fn persist_and_report(card: Option<&mut VirtualSd>, files: &[(String, std::path::PathBuf)]) {
    let Some(card) = card else { return };
    match persist_back(card, files) {
        Ok(c) if c.is_empty() => {}
        Ok(c) => { println!("kept for the next run:"); for l in c { println!("  {l}"); } }
        Err(e) => println!("--sd-persist files not written: {e}"),
    }
}

/// `sync_back` on a machine's card, printed (front ends call this when they stop).
pub fn sync_and_report(card: Option<&mut VirtualSd>) {
    let Some(card) = card else { return };
    let folder = card.root.as_ref().map(|r| r.display().to_string()).unwrap_or_default();
    match sync_back(card) {
        Ok(c) if c.is_empty() => {}
        Ok(c) => { println!("SD card changes written to {folder}:"); for l in c { println!("  {l}"); } }
        Err(e) => println!("SD card changes not written: {e}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fat32::Volume;

    fn temp_folder(tag: &str) -> std::path::PathBuf {
        let d = std::env::temp_dir().join(format!("w53ga-sdsync-{tag}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(d.join("DIR")).unwrap();
        std::fs::write(d.join("A.TXT"), b"hello").unwrap();
        std::fs::write(d.join("DIR").join("B.BIN"), vec![7u8; 1000]).unwrap();
        d
    }

    fn card(root: &std::path::Path) -> VirtualSd {
        let (vol, skipped) = Volume::build(root, &[]).unwrap();
        assert!(skipped.is_empty());
        let mut c = VirtualSd::from_volume(vol);
        c.root = Some(root.to_path_buf());
        c
    }

    #[test]
    fn built_card_reads_back_as_the_folder() {
        let root = temp_folder("read");
        let mut c = card(&root);
        let t = tree(&mut |l| c.read_sector(l)).unwrap();
        let names: Vec<String> = t.keys().map(|k| k.to_uppercase()).collect();
        assert!(names.contains(&"A.TXT".to_string()) && names.contains(&"DIR".to_string()) && names.contains(&"DIR/B.BIN".to_string()), "{names:?}");
        let a = t.iter().find(|(k, _)| k.eq_ignore_ascii_case("A.TXT")).unwrap().1;
        assert_eq!(a.size, 5);
        let f = geometry(&mut |l| c.read_sector(l)).unwrap();
        let lba = chain(&f, &mut |l| c.read_sector(l), a.cluster, 4).unwrap()[0];
        assert_eq!(&c.read_sector(lba).unwrap()[..5], b"hello");
        assert!(changes(&mut c).unwrap().is_empty());
        let _ = std::fs::remove_dir_all(&root);
    }

    #[test]
    fn written_sectors_become_file_changes() {
        let root = temp_folder("write");
        let mut c = card(&root);
        let t = tree(&mut |l| c.read_sector(l)).unwrap();
        let a = t.iter().find(|(k, _)| k.eq_ignore_ascii_case("A.TXT")).unwrap().1.clone();
        let f = geometry(&mut |l| c.read_sector(l)).unwrap();
        let lba = chain(&f, &mut |l| c.read_sector(l), a.cluster, 4).unwrap()[0];
        let mut s = c.read_sector(lba).unwrap().to_vec();
        s[..5].copy_from_slice(b"HELLO");
        c.overlay.insert(lba, s);
        let ch = changes(&mut c).unwrap();
        assert_eq!(ch.len(), 1, "{ch:?}");
        match &ch[0] { Change::Write { path, data, existed } => { assert!(path.eq_ignore_ascii_case("A.TXT")); assert_eq!(data, b"HELLO"); assert!(existed); } c => panic!("{c:?}") }
        // --sd-readonly keeps the folder as it was; otherwise the change is written back.
        c.keep_folder = true;
        assert!(sync_back(&mut c).unwrap().is_empty());
        assert_eq!(std::fs::read(root.join("A.TXT")).unwrap(), b"hello");
        c.keep_folder = false;
        assert_eq!(sync_back(&mut c).unwrap().len(), 1);
        assert_eq!(std::fs::read(root.join("A.TXT")).unwrap(), b"HELLO");
        let _ = std::fs::remove_dir_all(&root);
    }

    #[test]
    fn read_only_card_reports_nothing() {
        let root = temp_folder("ro");
        let mut c = card(&root);
        c.overlay.insert(0, vec![0; 512]);
        c.read_only = true;
        assert!(changes(&mut c).unwrap().is_empty());
        let _ = std::fs::remove_dir_all(&root);
    }
}
