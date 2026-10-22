HLE profiles (`*.json`) placed in this folder are built into the emulator and chosen automatically when their checks
match the flash image (see the main README, "HLE profiles"). A profile is made from a particular flash image, so
nothing here except this file is committed (see `.gitignore`).

The folder always exists so that the build script can watch it: a path the build script watches but that is missing
makes cargo rebuild the crate on every build.
