//! Hardware emulator for the AR-W53GA car GPS/radar unit.
//!
//! The board: a Renesas SH7269 (SH-2A with FPU) main CPU with its on-chip peripherals (VDC4 display controller,
//! JCU JPEG codec, SDHI, SCIF, MTU2, CMT, DMAC, RIIC, SSI/SRC audio, RTC, watchdog), 32 MiB of CS0 NOR flash and
//! SDRAM, plus an RL78/G13 companion controller (with its serial flash and GNSS receiver), the SDIO wireless LAN
//! module, the IR remote and the OBD-II and drive-recorder accessories.
//!
//! No software is included: the program running on the board comes from a flash image the user supplies, the
//! controller's program from its own image, and the SD card from a folder.

pub mod camera;
pub mod checkpoint;
pub mod cpu;
pub mod detect;
pub mod dvr;
pub mod embedded;
pub mod fat32;
pub mod gfx;
pub mod gnss;
pub mod hle;
pub mod intc;
pub mod machine;
pub mod mem;
pub mod obd2;
pub mod periph;
pub mod prof;
pub mod rl78;
pub mod sdsync;
pub mod session;
pub mod usernet;
pub mod wlan;
