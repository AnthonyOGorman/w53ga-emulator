# GPS loss (tunnels): what the unit does, and how the emulator reproduces it

Findings about the unit's own program (main firmware 1.15, controller I4PGM015) when the GPS fix is lost while the
car keeps driving, for example in a tunnel, and how the emulator's GPS cut and inertial-sensor model reproduce it.
Addresses are those of the main program (`I9PGM115/payload.elf` symbols; RAM variables in on-chip RAM `0xFFF8xxxx`).

Each statement is marked:
**[verified]** observed in the emulator running the original firmware (RAM read-outs with `--peek`, link log,
screenshots); **[code]** read from the disassembly/decompilation but not separately exercised; **[manual]** from the
user manual; **[inferred]** a conclusion that goes beyond what was checked.

## Short answers

| Question | Answer |
|---|---|
| Speed without GPS | OBD-II speed when the adapter is connected; otherwise the unit's own **G-sensor speed**: longitudinal acceleration integrated from the last GPS speed, kept within a band around a speed estimated from road vibration. [code, verified] |
| Speed on screen | With OBD-II the number stays. Without it, "- - -" from 11.5 s after the loss until about 6.5 s after the sky clears. [verified, manual p.17/35] |
| Does the map keep moving? | Yes: dead reckoning every 0.5 s from that speed and the **gyro** course, map-matched along the road. [verified] |
| Which inputs | Gyro yaw rate (MPU-6500 Y axis), accelerometer (longitudinal for speed, vertical vibration for the speed estimate and "moving or parked"), OBD-II speed. The barometer is used only for a special case (Shuto Expressway Yamate tunnel). [code, verified] |
| Other screen changes | The map turns to night colours at the next full minute of the clock once the fix has been gone 5 s, and back at the next full minute after it returns; the backlight goes to the night brightness setting. [verified timing, code for the trigger] |
| With the old stationary sensor | Without vibration the unit decides the car is **parked**: without OBD-II the dead-reckoned speed is forced to 0 and the position stops; the gyro course never changes, so turns are missed. [verified] |

## The pieces in the firmware

### Cadence and data flow [code, verified]

- `_SensorTask` (`0x10611C`) calls `_sensorScanAll` (`0xB0F62`) every 1 ms tick. The accelerometer and gyro are read
  every 50 calls (20 Hz, `_scan_GrvSensor` `0x69004`, `_scan_Gyro` `0x6A784`); about every 0.5 s it flips the sensor
  buffer and calls `_ComplementGPS_Data(g_stcGPSINFO_RAW 0xFFF8432A, m_complement_gps_data 0xFFF84311)` and
  `_update_GPS_INFO`. The 0.5 s cycle was confirmed in the emulator from the dead-reckoning distance counter.
- The controller forwards one GPS record (link command 0x11) per second. Without a fix it keeps sending it, with
  status 0x00, position and speed zero and the clock running on [verified, `AR_LINK_LOG`].
- `_ComplementGPS_Data` (`0x421B4`): `_GpsStatus` (`0xFFF82854`) becomes 1 only after the record's status has bits
  0 and 2 set (fix) for 6 cycles (`_use_gps_speed`, `0xFFF8B4AE`). Then it runs `_calc_Gsensor`, `_calc_Gyro`,
  `_checkStatusComplementGps`, `_calc_Location`, and `_overwrite_GpsData` (`0x42066`) copies the dead-reckoned
  position, course and speed into `_g_stcGPSINFO` (`0xFFF8B936`), the record the map, the warnings and the speed
  display read. So the rest of the program sees no difference between GPS and dead reckoning except the mode byte
  `g_stcGPSINFO[0x1A]`.

### Dead-reckoning mode (`m_complement_gps_data` bits 0-1) [code, verified]

`_checkStatusComplementGps` (`0x42952`): 0 = no position yet, 3 = GPS, **1 = GPS lost, dead reckoning**, 2 = the
Yamate-tunnel mode below. 3 → 1 when `_GpsStatus` drops, 1 → 3 when it returns, 0 → 3 at the first fix. In every
emulator run the cut produced 3 → 1 → 3. Nothing ends mode 1 while the fix stays away; the code that gives up after a
distance limit belongs to mode 2.

### Position: `_calc_Location` (`0x43628`) [code, verified]

Every cycle in modes 1-3: course = gyro yaw `_fGyrYaw` (`0xFFF838C8`, 1/8 degree) once `_gyro_enable`
(`0xFFF8B800`) is set, else the last GPS average course; speed = `_get_GrvSpd` (`0x6A356`): the OBD-II speed byte
`_m_OBD2_CARINFO+2` (`0xFFF8A3B8`, 0xFF = none) if available, else the G-sensor speed; distance = speed × 0.5 s
(constant 0.13889). Without GPS the new latitude/longitude (1/600000 degree) is stepped from that and then
**map-matched along the road by distance** (`_mm_CalculateMyCarLocationByDistance`, `0x910F4`); with GPS the fix is
map-matched (`_mm_CalculateMyCarLocation`, `0x90006`).

### Speed: `_calc_Gsensor` (`0x699AC`) [code, verified]

- The G-sensor speed `_GpsSpd` (`0xFFF83760`, 102 units per km/h) integrates the longitudinal acceleration (mg per
  cycle). While fixed it is pulled to the GPS speed, and an offset is learnt from acceleration minus 51 × the GPS speed
  change. Without a fix the offset is not applied and the speed is held within **[VtSpd − 10, VtSpd + 20] km/h**
  (`_VtSpd`, `0xFFF83764`).
- `VtSpd` is a speed **estimated from road vibration**: `_calc_nowVtSpd` (`0x68E44`) takes the spread of the
  sample-to-sample changes of the vertical acceleration (in mg, read directly as km/h) and `_expect_VtSpd` (`0x697AE`)
  the spread of its 0.5 s averages; `VtSpd` is the larger.
- **Parking**: when successive samples hardly differ (below the learnt thresholds `VtPark`/`TrPark`, 4-20 mg,
  `0xFFF8B5C4`), the parking flag (`0xFFF8B78D`) is set and, without GPS, the G-sensor speed is forced to **0**.
- At start-up the program zeroes the accelerometer (`_setZERO_G`, `0x68C42`; measured 2.5 to 10.5 s after power-on;
  offsets clamped to ±500 mg) as automatic mounting-tilt correction (manual p.10).
- Display: `_scr_mgr_UpdateSpeed` (`0x98EC0`) draws the number if the OBD-II speed byte is valid or the GPS icon
  status `_m_iGPSIconStatusID` (`0xFFF8BD10`) is 0, otherwise "- - -". `_scr_mgr_UpdateGpsIconStatus` (`0x9AA1C`)
  sets that status from the mode byte: 3 → 0; 3 → 1 keeps 0 for a hold period (`_g_iGPSIconKeepCounter`), then 2.

### Course: the gyro [code, verified]

- `_dev_gyro_init` (`0x58B18`) writes CONFIG 0x05, GYRO_CONFIG 0x00 (±250 °/s), SMPLRT_DIV 0x31 (20 Hz).
  `_accessGyroValue` (`0x6A5C8`) scales by 1/131.07. `_readGyroValue`/`_scan_Gyro` take the chip's **Y rate as yaw**
  and average 10 samples; `_calc_Gyro`/`_calc_GyroYaw` (`0x6AF76`/`0x6ABB4`) integrate it every 0.5 s (positive =
  turning right, as a compass course).
- The integration treats slow rates (below 2 °/s) as drift and lets them decay 1 % per cycle, so long gentle curves
  are under-read. While fixed, the gyro course is re-aligned with the GPS course (above 30 km/h when they differ by
  more than 15°, and on mode changes). The gyro is enabled after the first drive above 10 km/h with a fix.

### Accelerometer set-up and axes [code]

`_dev_g_sensor_init` (`0x58A80`) writes PWR_MGMT_2 0x00, ACCEL_CONFIG 0x00 (±2 g), ACCEL_CONFIG2 0x05 and checks
WHO_AM_I = 0x70. `_fixGAccValue_INVENSENSE` scales by 0.061 (mg). `_readSensorValue` (`0x68D88`) maps the axes to
vertical = −Y (the display-rotation check `_chk_ROTATION` expects more than +700 mg there; below −700 mg the screen
is turned over), lateral = −X and longitudinal = −Z (positive when accelerating). So the board stands upright, with the
chip's Y axis pointing down and Z towards the driver.

### Barometer [code]

`_dev_p_sensor_init` (`0x58BE4`) probes six pressure sensors on IIC2 (table at `0x136514`): BMP180 (0x77, ID 0x55),
BMP280 (0x76, 0x58), BME280 (0x76, 0x60), DPS310 (0x76, 0x10), LPS22HB (0x5C, 0xB1) and one more at 0x5D (ID 0x32 in
register 0x10). It is used for the pressure gauge and altitude, the downhill warning before cameras (取締機前下り坂警告)
and `_PresetCompYamateTunnel` (`0x75CA0`): preset positions and a pressure-based speed factor (`_m_fPressKeisu`) for
the Shuto Expressway Yamate tunnel / Ohashi junction, the only use of mode 2. Which part the real unit carries is not
known here. **The emulator fits none** (every probe is NACKed), so these features are inactive; dead reckoning in
ordinary tunnels does not need it.

### Night colours and dimming [code for the trigger, verified timing, inferred link]

`_system_set_GpsDimmerStatus` (`0xCB53A`): once a fix has been seen, a GPS status byte of 0 for 5 s sets
`_g_blGpsDimFlg` (`0xFFF8C27C`); a fix clears it. `_screen_set_backlight_dimmer` (`0xB0B74`) then uses the night
brightness setting. In the emulator the map changed to its night colours (`__$m_bNightMapFlg`, `0xFFF85318`) at the
next full minute of the GPS clock after the loss (12:02:00, 10-25 s into the cuts) and back at the next full minute
after the fix returned; runs without a cut never changed. The code that sets the night-map flag was not traced, so the
link to the dimmer flag is inferred from this timing. The backlight itself is not visible in the emulator.

### Tunnel guidance [manual, code]

The manual says that in-tunnel speed cameras (トンネル内取締機) and the tunnel-exit guidance (トンネル出口案内) rely on the
triple-sensor position (p.6); that tunnel entrance/exit guidance covers tunnels of 1 km or more, but not toll roads or
urban expressways (p.39-40, setting トンネル案内 p.58); that the speed-camera countdown and camera-position warnings are
not given in tunnels and the Shuto Expressway LH system in tunnels is announced (p.35). Display sequences exist for
them (`__$dspmode_WARNING_TUNNEL_LH_SEQ`, `__$dspmode_INFO_TUNNEL_ENTRANCEL_SEQ`,
`__$dspmode_STANDBY_DCLOCK_TUNNEL_EXIT_SEQ`). **None of them could be triggered here** (see below).

## Verification in the emulator

Original flash 1.15 and controller I4PGM015, SD card with its CID, power-on boot each time; `--peek` of the variables
above every 0.5 s; GPS cut with `--gps-cut`. The cut starts after the map is up and the car has been driving for
about 90 s.

| Run | Route, speed | Cut | Max position error during the cut | Error at the end of the cut | Speed used (avg) |
|---|---|---|---|---|---|
| New sensor + OBD-II | Tottori town streets with turns of 22-95°, 50 km/h | 110-180 s | 315 m | 208 m | 50.0 (OBD) |
| New sensor, no OBD-II | same | 110-180 s | 315 m | 208 m | 50.9 (G-sensor) |
| Old stationary sensor + OBD-II | same | 110-180 s | 661 m | 661 m | 50.0 (OBD), course frozen |
| Old stationary sensor, no OBD-II | same | 110-180 s | 613 m | 613 m | 0.4 (parked: position stops) |
| New sensor + OBD-II | Tottori Expressway (E29) southbound, through 用瀬第２トンネル (2.83 km), 80 km/h | 105-232.5 s (portal to portal) | 252 m | 252 m | 80.0 (OBD) |
| New sensor, no OBD-II | same | same | 227 m | 216 m | 80.1 (G-sensor, 79-82) |

The error is between the unit's position (`g_stcGPSINFO`) and the emulated car. Before the cut it was 15-40 m (map
matching against the unit's map; the expressway route is OpenStreetMap geometry).

What was seen:

- **Mode and icon**: 3 → 1 half a second after the loss. The GPS icon status went 0 → 2 after 11.5 s, and back to 0
  6.5-7 s after the sky cleared (2 s receiver re-acquisition, then 6 settle cycles). This screen layout has no GPS
  icon in its title bar, so only the variable shows it.
- **Speed display**: without OBD-II the speed stayed on screen for 11.5 s, then "- - -" until 6.5-7 s after the exit.
  With OBD-II the number stayed on screen throughout.
- **Map**: kept moving and turning with the car. On the expressway the arrow followed the expressway through the
  tunnel; the error grew mainly because the gyro under-reads the tunnel's long gentle curve (course −45° by the gyro
  against −49° true, from the firmware's drift decay; on straight stretches within 4.3°). On the street route the gyro
  course followed the turns but settled about 10° off after the sharpest ones, and at one junction the map
  matching kept snapping the position back to the same road segment for about 7 s,
  which left a 130-300 m lag until the fix returned. After the exit the position was back within 30 m after 6 s
  (streets) and 8.5 s (expressway).
- **G-sensor speed without OBD-II**: 79-82 km/h at a true 80, 50-52 km/h at a true 50.
- **Old stationary sensor**: parking flag set all the time; the gyro course stayed at its value from the cut's start,
  so every turn was missed. Without OBD-II the dead-reckoned speed was 0: the map stood still, the speed display fell
  to 0 and then "- - -".
- **Night colours**: as described above, at 12:02:00 on the GPS clock in every run with a cut.
- **Tunnel guidance**: none was shown or heard, also with the controller's warning database
  (`--rl78-spi` with a provisioned database). The two tunnels on the street route are under 1 km, and the expressway
  is not covered by the entrance/exit guidance (manual p.39-40). The only warning icon in the tunnel was a police point
  ahead. The in-tunnel camera and tunnel-exit cases could not be tried without knowing where such points are in the
  database.

## What the emulator now does

- **GPS cut** (`src/gnss.rs`): with the sky blocked the receiver keeps its once-a-second output with the time running
  on, but RMC status V, GGA quality 0 and 0 satellites, GNS mode NN, and GSV still listing the satellites without
  signal levels. The controller program turns that into status-0 records, as above. After the sky clears it tracks
  for 2 s, then reports a fix. Meanwhile the car drives on along the route, so it comes out further along the road,
  and the OBD-II speed keeps following the car. Controls: `--gps-cut FROM-TO[;FROM-TO]` (emulated seconds, all front
  ends through `session.rs`), key **T** in the window, **Enter tunnel / Leave tunnel** (or **T**) on the web page,
  whose GPS status shows "no fix (tunnel)" or "acquiring".
- **Vehicle** (`src/gnss.rs`): stands for the first 15 s after power-on (the unit zeroes its accelerometer then;
  driving away earlier made it learn the 0.2 g pull-away as tilt and integrate it as braking in tunnels), then
  accelerates at 2 m/s² and brakes at 3 m/s² towards the set speed. Corners are rounded (radius for 2.5 m/s² lateral
  at the set speed, 15-500 m), so course and gyro agree. Routes may give altitudes (`lat,lon,alt`) for grades.
- **Inertial sensor** (`src/imu.rs`): samples at the rate the program sets (20 Hz), in its ranges and axes: gravity
  on −Y, speed changes and grade on Z, cornering on X, yaw rate on the gyro's Y axis, pitch rate on X, plus vertical
  road vibration of 0.8 mg + 1.0 mg per km/h (high-passed noise, calibrated so that the program's vibration speed
  estimate comes out at about 0.85-1.1 times the true speed). `AR_IMU=legacy` restores the old stationary sensor,
  `AR_IMU=off` writes no samples, `AR_IMU_VIB=IDLE,PER_KMH` changes the vibration.
- **Diagnostics**: `w53ga-headless --peek ADDR,LEN[;...] --peek-every S` prints memory and the GPS state, as used above.

## Limits and open points

- No barometer is emulated, so the Yamate-tunnel mode (2), the pressure gauge and the downhill warning cannot be
  exercised. Which sensor the real unit carries would have to be read from the board.
- The vibration model is synthetic. Real road vibration differs and so would the unit's speed estimate without
  OBD-II. Gyro bias, temperature drift and mounting tilt are not modelled (the program corrects for them).
- The tunnel guidance was not triggered: it needs a 1 km+ tunnel on an ordinary road, an in-tunnel camera or an LH
  point in the warning database; none was on the routes tried.
- The link between the GPS-dimmer flag and the night map, and why the switch waits for the full minute, were not
  traced.
- Saved states made before this change still load, but their accelerometer zero was learnt with the old sensor
  (+1 g on Z), so tunnel tests need a state made from power-on with this build.
