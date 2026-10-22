//! RL78/G13 companion controller (R5F100LLA): RL78-S3 core, G13 peripherals, its external SPI NOR flash, and the
//! link to the main CPU. The controller runs whatever code flash image the user supplies.

pub mod cpu;
pub mod bus;
pub mod dis;
pub mod g13;
pub mod companion;
pub mod spiflash;
pub mod receiver;

pub use cpu::{Rl78, Rl78Bus, Rl78Fault};
