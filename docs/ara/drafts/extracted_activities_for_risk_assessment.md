# Extracted Activities for Risk Assessment

> **Source Documents:**
> - [`docs/equipment/htc_vive_base_station_user_guide.txt`](htc_vive_base_station_user_guide.txt)
> - [`docs/equipment/htc_vive_tracker_v1.txt`](htc_vive_tracker_v1.txt)
> - [`docs/equipment/htc_vive_tracker_user_manual.txt`](htc_vive_tracker_user_manual.txt)
> - [`docs/ara/.minutes_7_5_26.md`](../ara/.minutes_7_5_26.md) — Lab notes from Thursday 8/5/2026
>
> **Project Context:** RoboCup Soccer — Booster K1 (HSL 2026) — External pose estimation using HTC Vive motion tracking (base stations + trackers)
> **Reference:** [`docs/assignment_spec.txt`](../assignment_spec.txt:135-140) — Section 2.8 requires an Activity Risk Assessment (ARA) before development activities or use of the robot may commence.

---

## 1. HTC Vive Base Station 2.0 — Activities & Hazards

> **Note:** The lab has **4× v1.0** and **2× v2.0** base station models. Which model is used depends on the tracker model in use. The v1.0 base stations are required when using v1.0 trackers and provide a smaller tracking area (~5 m²). The v2.0 base stations support a larger tracking area and are used with v2.0/v3.0 trackers.

### 1.1 Mounting / Installation

| Activity | Hazard / Risk | Source Reference |
|----------|---------------|------------------|
| Mounting base stations on walls using power tools (screwdriver/drill) | Physical injury from power tools; falling objects if mounts are insecure | [`htc_vive_base_station_user_guide.txt:149-150`](htc_vive_base_station_user_guide.txt:149-150) |
| Drilling holes in concrete or drywall for mounting anchors | Dust inhalation; debris; electric shock if drilling into hidden wiring | [`htc_vive_base_station_user_guide.txt:155`](htc_vive_base_station_user_guide.txt:155) |
| Installing base stations at height (>2 m / 6.5 ft) | Fall from height; dropped equipment causing injury | [`htc_vive_base_station_user_guide.txt:62-63`](htc_vive_base_station_user_guide.txt:62-63) |
| Using tripods, light stands, or cargo poles as mounting solutions | Tip-over hazard; unstable mounting leading to equipment damage or injury | [`htc_vive_base_station_user_guide.txt:40-42`](htc_vive_base_station_user_guide.txt:40-42) |
| Mounting base stations on tripods (lab setup) | Tripod tip-over; base station falling and shattering; tripod legs creating tripping hazard | [`docs/ara/.minutes_7_5_26.md`](../ara/.minutes_7_5_26.md:6) |
| Adjusting base station angle (loosening clamping ring while holding the station) | Dropping the base station during adjustment; pinch hazard from clamping ring | [`htc_vive_base_station_user_guide.txt:162-165`](htc_vive_base_station_user_guide.txt:162-165) |

### 1.2 Electrical / Power

| Activity | Hazard / Risk | Source Reference |
|----------|---------------|------------------|
| Connecting power cables and plugging adapters into power outlets | Electric shock; tripping hazard from cables; fire risk from damaged cables | [`htc_vive_base_station_user_guide.txt:45-47`](htc_vive_base_station_user_guide.txt:45-47) |
| Using non-supplied power cables or adapters | Fire hazard; equipment damage; electric shock | [`htc_vive_base_station_user_guide.txt:47`](htc_vive_base_station_user_guide.txt:47) |

### 1.3 Operation

| Activity | Hazard / Risk | Source Reference |
|----------|---------------|------------------|
| Operating base stations with cracked or damaged front panel | Risk of injury from exposed components; electrical hazard | [`htc_vive_base_station_user_guide.txt:21-22`](htc_vive_base_station_user_guide.txt:21-22) |
| Attempting to pry open or disassemble base stations | Physical injury; electric shock; damage to product | [`htc_vive_base_station_user_guide.txt:19-20`](htc_vive_base_station_user_guide.txt:19-20) |
| Infrared signals from base stations affecting nearby IR sensors (e.g., TV remote controls) | Interference with other equipment | [`htc_vive_base_station_user_guide.txt:12-13`](htc_vive_base_station_user_guide.txt:12-13) |
| Base stations accidentally struck, dropped, or bumped during operation | Equipment damage; compromised tracking performance; falling hazard | [`htc_vive_base_station_user_guide.txt:31-33`](htc_vive_base_station_user_guide.txt:31-33) |
| Setting up in areas with bright light | Reduced tracking performance (not a safety hazard, but an operational risk) | [`htc_vive_base_station_user_guide.txt:74-75`](htc_vive_base_station_user_guide.txt:74-75) |

### 1.4 Cleaning / Maintenance

| Activity | Hazard / Risk | Source Reference |
|----------|---------------|------------------|
| Cleaning base stations with liquids or cleaning chemicals | Electric shock if liquid enters device; damage to front panel | [`htc_vive_base_station_user_guide.txt:169-176`](htc_vive_base_station_user_guide.txt:169-176) |
| Scratching the front panel during cleaning | Reduced optical performance; potential for cracking | [`htc_vive_base_station_user_guide.txt:175-176`](htc_vive_base_station_user_guide.txt:175-176) |
| Unplugging and unmounting before cleaning | Dropping equipment during handling | [`htc_vive_base_station_user_guide.txt:172`](htc_vive_base_station_user_guide.txt:172) |

### 1.5 Firmware Update

| Activity | Hazard / Risk | Source Reference |
|----------|---------------|------------------|
| Updating firmware via Bluetooth or USB cable | Bricking the device if power is disconnected during update | [`htc_vive_base_station_user_guide.txt:275-277`](htc_vive_base_station_user_guide.txt:275-277) |
| Connecting base station to computer via micro-USB for firmware update | USB cable tripping hazard | [`htc_vive_base_station_user_guide.txt:268-269`](htc_vive_base_station_user_guide.txt:268-269) |

---

## 2. HTC Vive Tracker — Activities & Hazards

> **Note:** The lab has access to **2× v1.0 trackers** (currently), which provide ~5 m² tracking area and require v1.0 base stations. The team is attempting to acquire a **v2.0 tracker** which would allow use of v2.0 base stations and a larger tracking area. Tracker v3.0 is also available on the market but not yet acquired. This section covers activities common to all tracker versions, drawing from both the v1.0 technical reference and the generic tracker user manual.

### 2.1 Mechanical / Docking

| Activity | Hazard / Risk | Source Reference |
|----------|---------------|------------------|
| Attaching/detaching the tracker to/from an accessory | Pinch hazard from docking mechanism; dropping the tracker | [`htc_vive_tracker_v1.txt:185-189`](htc_vive_tracker_v1.txt:185-189) |
| Using the 1/4" screw mounting mechanism (standard camera mount) | Over-tightening causing damage; stripping threads | [`htc_vive_tracker_v1.txt:269-270`](htc_vive_tracker_v1.txt:269-270); [`htc_vive_tracker_user_manual.txt:50`](htc_vive_tracker_user_manual.txt:50) |
| Aligning stabilizing pin with accessory recess during docking | Misalignment causing damage to pin or recess; difficulty detaching | [`htc_vive_tracker_user_manual.txt:63-65`](htc_vive_tracker_user_manual.txt:63-65) |
| Attaching tracker to objects/surfaces using adhesive tape (e.g., 3M VHB) | Adhesive failure causing tracker to fall and be damaged; residue on surfaces | [`htc_vive_tracker_v1.txt:211-212`](htc_vive_tracker_v1.txt:211-212) |
| Attaching tracker to rough/soft surfaces using straps | Insufficient fastening causing tracker detachment during movement | [`htc_vive_tracker_v1.txt:213-214`](htc_vive_tracker_v1.txt:213-214) |
| Accessory body obstructing tracking sensors (improper placement) | Tracking failure; potential collision if used in VR | [`htc_vive_tracker_v1.txt:221-223`](htc_vive_tracker_v1.txt:221-223) |
| Accessory extending beyond recommended placing cone (270° FOV) | Blocked sensor view; degraded tracking performance | [`htc_vive_tracker_v1.txt:168-173`](htc_vive_tracker_v1.txt:168-173) |

### 2.2 Electrical / Power

| Activity | Hazard / Risk | Source Reference |
|----------|---------------|------------------|
| Charging via micro USB connector (AC adapter: 5V ±5%, 1000 mA) | Overheating if using incorrect charger; fire risk | [`htc_vive_tracker_v1.txt:158-159`](htc_vive_tracker_v1.txt:158-159); [`htc_vive_tracker_user_manual.txt:55-57`](htc_vive_tracker_user_manual.txt:55-57) |
| Charging via PC USB port (500 mA) | Slow charging; USB port overload if multiple devices connected | [`htc_vive_tracker_v1.txt:160`](htc_vive_tracker_v1.txt:160); [`htc_vive_tracker_user_manual.txt:60`](htc_vive_tracker_user_manual.txt:60) |
| Charging via Pogo pin (5V ±5%, 500 mA) | Electric shock if pogo pins are damaged or shorted | [`htc_vive_tracker_v1.txt:161-162`](htc_vive_tracker_v1.txt:161-162) |
| Electrostatic discharge (ESD) — Human Body Model up to 4000 V | Damage to sensitive electronic components if not properly grounded | [`htc_vive_tracker_v1.txt:107`](htc_vive_tracker_v1.txt:107) |

### 2.3 Radio Frequency / Wireless (Dongle)

| Activity | Hazard / Risk | Source Reference |
|----------|---------------|------------------|
| Wireless operation with dongle (OTA performance) | RF interference with other equipment if metal parts are within 30 mm of antenna | [`htc_vive_tracker_v1.txt:140-149`](htc_vive_tracker_v1.txt:140-149) |
| Pairing tracker with dongle or headset | Potential RF interference in lab environment | [`htc_vive_tracker_v1.txt:585-591`](htc_vive_tracker_v1.txt:585-591); [`htc_vive_tracker_user_manual.txt:85-92`](htc_vive_tracker_user_manual.txt:85-92) |
| Placing dongle less than 45 cm (18 in) from computer | RF interference causing tracking dropout or degraded performance | [`htc_vive_tracker_user_manual.txt:93-94`](htc_vive_tracker_user_manual.txt:93-94) |
| Dongle cradle and USB cable routing across workspace | Tripping hazard from USB cable; dongle being accidentally knocked or moved | [`htc_vive_tracker_user_manual.txt:82-83`](htc_vive_tracker_user_manual.txt:82-83) |

### 2.4 Operation

| Activity | Hazard / Risk | Source Reference |
|----------|---------------|------------------|
| Using tracker with accessories that have reflective surfaces | Faulty sensor readings due to reflection interference | [`htc_vive_tracker_v1.txt:195-196`](htc_vive_tracker_v1.txt:195-196) |
| Continuous vibration affecting IMU performance (IMU drift) | Degraded tracking accuracy; potential for erratic robot behaviour | [`htc_vive_tracker_v1.txt:334-338`](htc_vive_tracker_v1.txt:334-338) |
| Tracker automatically turning off due to low battery, pairing timeout (>30 s idle), or inactivity (>5 min no movement) | Unexpected loss of tracking during operation | [`htc_vive_tracker_v1.txt:806-813`](htc_vive_tracker_v1.txt:806-813); [`htc_vive_tracker_user_manual.txt:72-76`](htc_vive_tracker_user_manual.txt:72-76) |
| Using more than one tracker (up to 11 trackers + 2 controllers) | RF congestion; channel interference | [`htc_vive_tracker_v1.txt:814-819`](htc_vive_tracker_v1.txt:814-819); [`htc_vive_tracker_user_manual.txt:98-100`](htc_vive_tracker_user_manual.txt:98-100) |
| Operating tracker on a moving object (robot) without clearing the play area | Collision with obstacles or other individuals in the tracking space | [`htc_vive_tracker_user_manual.txt:95-96`](htc_vive_tracker_user_manual.txt:95-96) |
| Using Vive ROS2 integration instead of standard SteamVR software | Software misconfiguration causing unexpected robot behaviour; loss of tracking data stream | [`docs/ara/.minutes_7_5_26.md`](../ara/.minutes_7_5_26.md:8) |

### 2.5 Firmware Update

| Activity | Hazard / Risk | Source Reference |
|----------|---------------|------------------|
| Updating firmware via USB cable (MCU, FPGA, RF firmware) | Bricking the device if USB cable is disconnected during update | [`htc_vive_tracker_v1.txt:725-738`](htc_vive_tracker_v1.txt:725-738); [`htc_vive_tracker_user_manual.txt:111-112`](htc_vive_tracker_user_manual.txt:111-112) |
| Running firmware update commands (`lighthouse_watchman_update`) | Incorrect firmware file causing device malfunction | [`htc_vive_tracker_v1.txt:733-738`](htc_vive_tracker_v1.txt:733-738) |
| Updating firmware via SteamVR app (automatic detection) | Interrupted update if SteamVR crashes or USB is disconnected | [`htc_vive_tracker_user_manual.txt:113-119`](htc_vive_tracker_user_manual.txt:113-119) |

### 2.6 LED Status Indicators (Operational Awareness)

| Activity | Hazard / Risk | Source Reference |
|----------|---------------|------------------|
| Interpreting LED status (green = normal, blinking red = low battery, blinking blue = pairing, blue = connecting, white = fully charged and off) | Misinterpreting status could lead to operating with low battery, in pairing mode unexpectedly, or assuming device is off when still active | [`htc_vive_tracker_v1.txt:779-788`](htc_vive_tracker_v1.txt:779-788); [`htc_vive_tracker_user_manual.txt:105-109`](htc_vive_tracker_user_manual.txt:105-109) |

---

## 3. Summary of Risk Categories

| Risk Category | Count | Key Sources |
|---------------|-------|-------------|
| **Electrical Shock / Fire** | 5 | Power connections, charging, damaged front panels |
| **Physical Injury** | 7 | Mounting at height, power tools, pinch hazards, dropped equipment, tripod tip-over, play area collision |
| **Equipment Damage** | 9 | Dropping, over-tightening, adhesive failure, firmware update interruption, misalignment damage |
| **Tracking / Performance Degradation** | 9 | Bright light, reflective surfaces, vibration, improper placement, RF interference, dongle proximity, ROS2 misconfiguration |
| **RF Interference** | 4 | IR signals, antenna proximity, multi-device congestion, dongle placement |
| **Trip Hazards** | 3 | Power cables, USB cables, tripod legs |

---

## 4. Applicability to RoboCup Soccer Project

For the RoboCup Soccer project using Booster K1 robots, the following activities from these equipment guides are most relevant:

1. **Mounting Vive Trackers to the Booster K1** — The docking mechanism (1/4" screw, stabilizing pin) and ensuring the tracker does not obstruct the robot's movement or sensors. The tracker will be used for external pose estimation on the robot.

2. **Power management** — Charging trackers and base stations between test sessions; ensuring batteries are not depleted during runs. The team has 2× v1.0 trackers (and potentially a v2.0) plus 6× base stations to manage.

3. **RF coordination** — Managing multiple wireless devices (trackers, dongles, controllers) in the lab to avoid interference during competition-like conditions. Dongle placement ≥45 cm from computer is critical.

4. **Firmware updates** — Keeping tracker and base station firmware up-to-date without bricking devices. Updates may be performed via SteamVR app or command-line tools.

5. **Lab setup** — Mounting base stations on tripods in the VXLab or testing area for tracking coverage of the RoboCup field. Tripod stability and leg placement are key safety considerations.

6. **ROS2 integration** — The team will use the Vive ROS2 packaged integration for external pose estimation rather than standard HTC Vive/SteamVR software. This introduces software-related risks including misconfiguration, data stream loss, and unexpected robot behaviour.
