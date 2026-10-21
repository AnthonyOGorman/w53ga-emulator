//! Data embedded at build time when present in the source tree (see build.rs): `POWER_ON` (the power-on reset
//! state, `data/power-on.json.gz`) and `PROFILES` (`profiles/*.json`). The repository ships neither.

include!(concat!(env!("OUT_DIR"), "/embedded.rs"));
