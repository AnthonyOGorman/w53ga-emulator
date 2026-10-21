//! FAT32 volume synthesised from a folder, served read-only as the SD card (no copy of the files is made).
//!
//! Layout: MBR with one FAT32 (LBA, type 0x0C) partition at LBA 2048, 512-byte
//! sectors, 64 sectors per cluster, 32 reserved sectors, two FATs, root directory at cluster 2, FSInfo at sector 1
//! and backups at 6/7. Directories take the first clusters in breadth-first order, files follow contiguously in
//! depth-first alphabetical order. Names are 8.3 with the NT lower-case flags (no long names); attributes are
//! archive + read-only and timestamps are zero (1980-01-01). Extra files (e.g. update packages) can be added at
//! chosen card paths without touching the source folder.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

pub const PART_START: u64 = 2048;
const SPC: u64 = 64;
const RESERVED: u64 = 32;
const CLUSTER: u64 = SPC * 512;

/// A file on the card: host path and size.
#[derive(Clone)]
pub struct HostFile { pub path: PathBuf, pub size: u64, pub extra: bool }

#[derive(Default)]
struct Dir { dirs: BTreeMap<String, Dir>, files: BTreeMap<String, HostFile> }

/// Card image description: metadata sectors and file extents, in the form `sdhi::VirtualSd` serves.
pub struct Volume {
    pub sector_count: u64,
    /// (first LBA, sector count, offset into `metadata`)
    pub meta: Vec<(u64, u64, u64)>,
    pub metadata: Vec<u8>,
    /// (first LBA, sector count, file)
    pub files: Vec<(u64, u64, HostFile)>,
}

/// 8.3 directory name and NT case flags (0x08 base lower, 0x10 extension lower); None if not representable.
fn short_name(name: &str) -> Option<([u8; 11], u8)> {
    let (base, ext) = match name.rfind('.') { Some(i) if i > 0 => (&name[..i], &name[i + 1..]), _ => (name, "") };
    if base.is_empty() || base.len() > 8 || ext.len() > 3 { return None; }
    let ok = |s: &str| s.bytes().all(|b| b.is_ascii_alphanumeric() || b"!#$%&'()-@^_`{}~".contains(&b));
    if !ok(base) || !ok(ext) { return None; }
    let case = |s: &str| -> Option<bool> {
        let (lo, up) = (s.bytes().any(|b| b.is_ascii_lowercase()), s.bytes().any(|b| b.is_ascii_uppercase()));
        if lo && up { None } else { Some(lo) }
    };
    let (bl, el) = (case(base)?, case(ext)?);
    let mut n = [b' '; 11];
    for (i, b) in base.bytes().enumerate() { n[i] = b.to_ascii_uppercase(); }
    for (i, b) in ext.bytes().enumerate() { n[8 + i] = b.to_ascii_uppercase(); }
    Some((n, (bl as u8) << 3 | (el as u8) << 4))
}

fn scan(dir: &Path, out: &mut Dir, skipped: &mut Vec<String>) -> Result<(), String> {
    for e in std::fs::read_dir(dir).map_err(|e| format!("{}: {e}", dir.display()))? {
        let e = e.map_err(|e| e.to_string())?;
        let name = e.file_name().to_string_lossy().to_string();
        if short_name(&name).is_none() { skipped.push(e.path().display().to_string()); continue; }
        let md = e.metadata().map_err(|e| e.to_string())?;
        if md.is_dir() { scan(&e.path(), out.dirs.entry(name).or_default(), skipped)?; }
        else { out.files.insert(name, HostFile { path: e.path(), size: md.len(), extra: false }); }
    }
    Ok(())
}

fn dir_entry(name: [u8; 11], attr: u8, nt: u8, cluster: u32, size: u32) -> [u8; 32] {
    let mut e = [0u8; 32];
    e[..11].copy_from_slice(&name);
    e[11] = attr;
    e[12] = nt;
    if attr & 0x10 != 0 || attr & 0x20 != 0 { e[15] = 0x00; e[16] = 0x21; e[17] = 0x00; e[18] = 0x21; e[24] = 0x21; }
    e[20..22].copy_from_slice(&((cluster >> 16) as u16).to_le_bytes());
    e[26..28].copy_from_slice(&(cluster as u16).to_le_bytes());
    e[28..32].copy_from_slice(&size.to_le_bytes());
    e
}

fn insert(tree: &mut Dir, card: &str, file: HostFile) -> Result<(), String> {
    let parts: Vec<&str> = card.split(['/', '\\']).filter(|p| !p.is_empty()).collect();
    let (name, dirs) = parts.split_last().ok_or("empty card path")?;
    if parts.iter().any(|p| short_name(p).is_none()) { return Err(format!("card path {card} is not 8.3")); }
    let mut d = tree;
    for p in dirs { d = d.dirs.entry(p.to_string()).or_default(); }
    d.files.insert(name.to_string(), file);
    Ok(())
}

impl Volume {
    /// Volume from `root` plus `extra` files at card paths ("FOLDER/FILE.BIN" = host file).
    pub fn build(root: &Path, extra: &[(String, PathBuf)]) -> Result<(Volume, Vec<String>), String> {
        let mut tree = Dir::default();
        let mut skipped = Vec::new();
        scan(root, &mut tree, &mut skipped)?;
        for (card, host) in extra {
            let size = std::fs::metadata(host).map_err(|e| format!("{}: {e}", host.display()))?.len();
            insert(&mut tree, card, HostFile { path: host.clone(), size, extra: true })?;
        }
        Self::from_tree(tree, skipped)
    }

    /// Volume from a list of (card path, size, extra) without a file system; the card path doubles as the file key
    /// given to the installed `sdhi::FileReader`. Names that are not 8.3 are skipped and returned.
    pub fn from_list(files: &[(String, u64, bool)]) -> Result<(Volume, Vec<String>), String> {
        let mut tree = Dir::default();
        let mut skipped = Vec::new();
        for (card, size, extra) in files {
            if card.split(['/', '\\']).filter(|p| !p.is_empty()).any(|p| short_name(p).is_none()) { skipped.push(card.clone()); continue; }
            insert(&mut tree, card, HostFile { path: PathBuf::from(card), size: *size, extra: *extra })?;
        }
        Self::from_tree(tree, skipped)
    }

    fn from_tree(tree: Dir, skipped: Vec<String>) -> Result<(Volume, Vec<String>), String> {

        // Cluster allocation: directories depth-first (pre-order) from cluster 2, then files depth-first; extra files
        // last, so the folder's own files keep the same clusters whatever is added.
        struct Placed { entries: Vec<[u8; 32]>, cluster: u32 }
        let mut dir_clusters: Vec<Placed> = Vec::new();
        let mut next = 2u32;
        let mut order: Vec<Vec<String>> = Vec::new();
        let mut dir_cluster_of: BTreeMap<Vec<String>, u32> = BTreeMap::new();
        fn pre(path: &mut Vec<String>, d: &Dir, next: &mut u32, order: &mut Vec<Vec<String>>, of: &mut BTreeMap<Vec<String>, u32>) {
            of.insert(path.clone(), *next); *next += 1; order.push(path.clone());
            for (name, sub) in &d.dirs { path.push(name.clone()); pre(path, sub, next, order, of); path.pop(); }
        }
        pre(&mut Vec::new(), &tree, &mut next, &mut order, &mut dir_cluster_of);
        let get = |path: &[String]| -> &Dir { let mut d = &tree; for p in path { d = &d.dirs[p]; } d };
        // Files depth-first alphabetical.
        let mut file_cluster: BTreeMap<Vec<String>, u32> = BTreeMap::new();
        let mut files = Vec::new();
        fn walk(path: &mut Vec<String>, d: &Dir, extra: bool, next: &mut u32, file_cluster: &mut BTreeMap<Vec<String>, u32>, files: &mut Vec<(u32, HostFile)>) {
            for (name, f) in d.files.iter().filter(|f| f.1.extra == extra) {
                path.push(name.clone());
                let n = f.size.div_ceil(CLUSTER) as u32;
                let c = if n == 0 { 0 } else { let c = *next; *next += n; c };
                file_cluster.insert(path.clone(), c);
                if n > 0 { files.push((c, f.clone())); }
                path.pop();
            }
            for (name, sub) in &d.dirs { path.push(name.clone()); walk(path, sub, extra, next, file_cluster, files); path.pop(); }
        }
        walk(&mut Vec::new(), &tree, false, &mut next, &mut file_cluster, &mut files);
        walk(&mut Vec::new(), &tree, true, &mut next, &mut file_cluster, &mut files);
        let used = next as u64;

        // Geometry: at least 4 GiB, enough clusters for the content with headroom.
        let mut sectors_total = 8_388_608u64;
        while (sectors_total - PART_START) / SPC < used + 1024 { sectors_total *= 2; }
        let part_sectors = sectors_total - PART_START;
        let clusters = (part_sectors - RESERVED) / SPC; // upper bound; FAT space taken from it below
        let fat_sectors = ((clusters + 2) * 4).div_ceil(512);
        let data_start = PART_START + RESERVED + 2 * fat_sectors;
        let clusters = (sectors_total - data_start) / SPC;
        if clusters < used { return Err("card content too large".into()); }

        // Directory clusters.
        for path in &order {
            let d = get(path);
            let own = dir_cluster_of[path];
            let mut entries = Vec::new();
            if path.is_empty() { entries.push(dir_entry(*b"W53GA CARD ", 0x08, 0, 0, 0)); }
            else {
                let parent = if path.len() == 1 { 0 } else { dir_cluster_of[&path[..path.len() - 1].to_vec()] };
                entries.push(dir_entry(*b".          ", 0x10, 0, own, 0));
                entries.push(dir_entry(*b"..         ", 0x10, 0, parent, 0));
            }
            for name in d.dirs.keys() {
                let (n, nt) = short_name(name).unwrap();
                let mut p = path.clone(); p.push(name.clone());
                entries.push(dir_entry(n, 0x10, nt, dir_cluster_of[&p], 0));
            }
            for (name, f) in &d.files {
                let (n, nt) = short_name(name).unwrap();
                let mut p = path.clone(); p.push(name.clone());
                if f.size > u32::MAX as u64 { return Err(format!("{} exceeds 4 GiB", f.path.display())); }
                entries.push(dir_entry(n, 0x21, nt, file_cluster[&p], f.size as u32));
            }
            if entries.len() > (CLUSTER / 32) as usize { return Err(format!("directory /{} has too many entries", path.join("/"))); }
            dir_clusters.push(Placed { entries, cluster: own });
        }

        // Metadata sectors.
        let mut metadata = Vec::new();
        let mut meta = Vec::new();
        let mut push = |lba: u64, data: &[u8], metadata: &mut Vec<u8>| {
            let count = data.len().div_ceil(512) as u64;
            meta.push((lba, count, metadata.len() as u64));
            metadata.extend_from_slice(data);
            metadata.resize(metadata.len().div_ceil(512) * 512, 0);
        };
        let mut mbr = [0u8; 512];
        mbr[446..462].copy_from_slice(&[0x00, 0xFE, 0xFF, 0xFF, 0x0C, 0xFE, 0xFF, 0xFF, 0, 0, 0, 0, 0, 0, 0, 0]);
        mbr[454..458].copy_from_slice(&(PART_START as u32).to_le_bytes());
        mbr[458..462].copy_from_slice(&(part_sectors as u32).to_le_bytes());
        mbr[440..444].copy_from_slice(&0xCAA6_0128u32.to_le_bytes()); // disk signature (= volume serial)
        mbr[510] = 0x55; mbr[511] = 0xAA;
        push(0, &mbr, &mut metadata);
        let mut bs = [0u8; 512];
        bs[..3].copy_from_slice(&[0xEB, 0x58, 0x90]);
        bs[3..11].copy_from_slice(b"W53GAEMU");
        bs[11..13].copy_from_slice(&512u16.to_le_bytes());
        bs[13] = SPC as u8;
        bs[14..16].copy_from_slice(&(RESERVED as u16).to_le_bytes());
        bs[16] = 2;
        bs[21] = 0xF8;
        bs[24..26].copy_from_slice(&63u16.to_le_bytes());
        bs[26..28].copy_from_slice(&255u16.to_le_bytes());
        bs[28..32].copy_from_slice(&(PART_START as u32).to_le_bytes());
        bs[32..36].copy_from_slice(&(part_sectors as u32).to_le_bytes());
        bs[36..40].copy_from_slice(&(fat_sectors as u32).to_le_bytes());
        bs[44..48].copy_from_slice(&2u32.to_le_bytes());
        bs[48..50].copy_from_slice(&1u16.to_le_bytes());
        bs[50..52].copy_from_slice(&6u16.to_le_bytes());
        bs[64] = 0x80; bs[66] = 0x29;
        bs[67..71].copy_from_slice(&0xCAA6_0128u32.to_le_bytes());
        bs[71..82].copy_from_slice(b"W53GA CARD ");
        bs[82..90].copy_from_slice(b"FAT32   ");
        bs[510] = 0x55; bs[511] = 0xAA;
        let mut fsinfo = [0u8; 512];
        fsinfo[..4].copy_from_slice(b"RRaA");
        fsinfo[484..488].copy_from_slice(b"rrAa");
        fsinfo[488..492].copy_from_slice(&((clusters - (used - 2)) as u32).to_le_bytes());
        fsinfo[492..496].copy_from_slice(&(next).to_le_bytes());
        fsinfo[510] = 0x55; fsinfo[511] = 0xAA;
        push(PART_START, &bs, &mut metadata);
        push(PART_START + 1, &fsinfo, &mut metadata);
        push(PART_START + 6, &bs, &mut metadata);
        push(PART_START + 7, &fsinfo, &mut metadata);
        // FAT: media/EOC, one cluster per directory, contiguous chains for files.
        let mut fat = vec![0u8; (fat_sectors * 512) as usize];
        let mut set = |c: u32, v: u32| fat[c as usize * 4..c as usize * 4 + 4].copy_from_slice(&v.to_le_bytes());
        set(0, 0x0FFF_FFF8); set(1, 0x0FFF_FFFF);
        for d in &dir_clusters { set(d.cluster, 0x0FFF_FFFF); }
        for (c, f) in &files {
            let n = f.size.div_ceil(CLUSTER) as u32;
            for k in 0..n { set(c + k, if k + 1 == n { 0x0FFF_FFFF } else { c + k + 1 }); }
        }
        push(PART_START + RESERVED, &fat, &mut metadata);
        push(PART_START + RESERVED + fat_sectors, &fat, &mut metadata);
        for d in &dir_clusters {
            let mut sec = vec![0u8; CLUSTER as usize];
            for (i, e) in d.entries.iter().enumerate() { sec[i * 32..i * 32 + 32].copy_from_slice(e); }
            push(data_start + (d.cluster as u64 - 2) * SPC, &sec, &mut metadata);
        }
        let files = files.into_iter().map(|(c, f)| (data_start + (c as u64 - 2) * SPC, f.size.div_ceil(512).div_ceil(SPC) * SPC, f)).collect();
        Ok((Volume { sector_count: sectors_total, meta, metadata, files }, skipped))
    }
}
