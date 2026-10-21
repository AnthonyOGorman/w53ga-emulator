//! Data embedded at build time: the power-on reset state (shipped) and HLE profiles (not shipped; both can also be given at run time).
//!
//! - `data/power-on.json.gz`: the SH7269 power-on reset state (register reset values and the peripheral models'
//!   initial state; no memory contents, no PC/SP, which come from the flash image's vector table). Without it, a power-on
//!   state file must be supplied at run time (`--power-on-state FILE`, or the web page's file picker).
//! - `profiles/*.json`: HLE profiles (see `src/hle.rs`), chosen automatically when their checks match the flash image.

use std::path::{Path, PathBuf};

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed=data");
    println!("cargo:rerun-if-changed=profiles");
    let manifest = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let lit = |p: &Path| format!("{:?}", std::fs::canonicalize(p).unwrap_or_else(|_| p.to_path_buf()).display().to_string());

    let power_on = manifest.join("data").join("power-on.json.gz");
    let mut code = if power_on.is_file() {
        println!("cargo:rerun-if-changed={}", power_on.display());
        format!("pub const POWER_ON: Option<&[u8]> = Some(include_bytes!({}));\n", lit(&power_on))
    } else {
        "pub const POWER_ON: Option<&[u8]> = None;\n".to_string()
    };

    let mut profiles: Vec<PathBuf> = std::fs::read_dir(manifest.join("profiles")).map(|d| d.flatten().map(|e| e.path()).filter(|p| p.extension().is_some_and(|x| x == "json")).collect()).unwrap_or_default();
    profiles.sort();
    code += "pub const PROFILES: &[&str] = &[";
    for p in &profiles {
        println!("cargo:rerun-if-changed={}", p.display());
        code += &format!("include_str!({}), ", lit(p));
    }
    code += "];\n";

    std::fs::write(Path::new(&std::env::var("OUT_DIR").unwrap()).join("embedded.rs"), code).unwrap();
}
