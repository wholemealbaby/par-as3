# Extracted Activities for Risk Assessment

> **Source Documents:**
> - [`docs/equipment/htc_vive_base_station_user_guide.txt`](htc_vive_base_station_user_guide.txt)
> - [`docs/equipment/htc_vive_tracker_v1.txt`](htc_vive_tracker_v1.txt)
>
> **Project Context:** RoboCup Soccer — Booster K1 (HSL 2026)
> **Reference:** [`docs/assignment_spec.txt`](../assignment_spec.txt:135-140) — Section 2.8 requires an Activity Risk Assessment (ARA) before development activities or use of the robot may commence.

---

## 1. HTC Vive Base Station 2.0 — Activities & Hazards

### 1.1 Mounting / Installation

| Activity | Hazard / Risk | Source Reference |
|----------|---------------|------------------|
| Mounting base stations on walls using power tools (screwdriver/drill) | Physical injury from power tools; falling objects if mounts are insecure | [`htc_vive_base_station_user_guide.txt:149-150`](htc_vive_base_station_user_guide.txt:149-150) |
| Drilling holes in concrete or drywall for mounting anchors | Dust inhalation; debris; electric shock if drilling into hidden wiring | [`htc_vive_base_station_user_guide.txt:155`](htc_vive_base_station_user_guide.txt:155) |
| Installing base stations at height (>2 m / 6.5 ft) | Fall from height; dropped equipment causing injury | [`htc_vive_base_station_user_guide.txt:62-63`](htc_vive_base_station_user_guide.txt:62-63) |
| Using tripods, light stands, or cargo poles as mounting solutions | Tip-over hazard; unstable mounting leading to equipment damage or injury | [`htc_vive_base_station_user_guide.txt:40-42`](htc_vive_base_station_user_guide.txt:40-42) |
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

## 2. HTC Vive Tracker (2018) — Activities & Hazards

### 2.1 Mechanical / Docking

| Activity | Hazard / Risk | Source Reference |
|----------|---------------|------------------|
| Attaching/detaching the tracker to/from an accessory | Pinch hazard from docking mechanism; dropping the tracker | [`htc_vive_tracker_v1.txt:185-189`](htc_vive_tracker_v1.txt:185-189) |
| Using the 1/4" screw mounting mechanism | Over-tightening causing damage; stripping threads | [`htc_vive_tracker_v1.txt:269-270`](htc_vive_tracker_v1.txt:269-270) |
| Attaching tracker to objects/surfaces using adhesive tape (e.g., 3M VHB) | Adhesive failure causing tracker to fall and be damaged; residue on surfaces | [`htc_vive_tracker_v1.txt:211-212`](htc_vive_tracker_v1.txt:211-212) |
| Attaching tracker to rough/soft surfaces using straps | Insufficient fastening causing tracker detachment during movement | [`htc_vive_tracker_v1.txt:213-214`](htc_vive_tracker_v1.txt:213-214) |
| Accessory body obstructing tracking sensors (improper placement) | Tracking failure; potential collision if used in VR | [`htc_vive_tracker_v1.txt:221-223`](htc_vive_tracker_v1.txt:221-223) |
| Accessory extending beyond recommended placing cone (270° FOV) | Blocked sensor view; degraded tracking performance | [`htc_vive_tracker_v1.txt:168-173`](htc_vive_tracker_v1.txt:168-173) |

### 2.2 Electrical / Power

| Activity | Hazard / Risk | Source Reference |
|----------|---------------|------------------|
| Charging via micro USB connector (AC adapter: 5V ±5%, 1000 mA) | Overheating if using incorrect charger; fire risk | [`htc_vive_tracker_v1.txt:158-159`](htc_vive_tracker_v1.txt:158-159) |
| Charging via PC USB port (500 mA) | Slow charging; USB port overload if multiple devices connected | [`htc_vive_tracker_v1.txt:160`](htc_vive_tracker_v1.txt:160) |
| Charging via Pogo pin (5V ±5%, 500 mA) | Electric shock if pogo pins are damaged or shorted | [`htc_vive_tracker_v1.txt:161-162`](htc_vive_tracker_v1.txt:161-162) |
| Electrostatic discharge (ESD) — Human Body Model up to 4000 V | Damage to sensitive electronic components if not properly grounded | [`htc_vive_tracker_v1.txt:107`](htc_vive_tracker_v1.txt:107) |

### 2.3 Radio Frequency / Wireless

| Activity | Hazard / Risk | Source Reference |
|----------|---------------|------------------|
| Wireless operation with dongle (OTA performance) | RF interference with other equipment if metal parts are within 30 mm of antenna | [`htc_vive_tracker_v1.txt:140-149`](htc_vive_tracker_v1.txt:140-149) |
| Pairing tracker with dongle or headset | Potential RF interference in lab environment | [`htc_vive_tracker_v1.txt:585-591`](htc_vive_tracker_v1.txt:585-591) |

### 2.4 Operation

| Activity | Hazard / Risk | Source Reference |
|----------|---------------|------------------|
| Using tracker with accessories that have reflective surfaces | Faulty sensor readings due to reflection interference | [`htc_vive_tracker_v1.txt:195-196`](htc_vive_tracker_v1.txt:195-196) |
| Continuous vibration affecting IMU performance (IMU drift) | Degraded tracking accuracy; potential for erratic robot behaviour | [`htc_vive_tracker_v1.txt:334-338`](htc_vive_tracker_v1.txt:334-338) |
| Tracker automatically turning off due to low battery or inactivity | Unexpected loss of tracking during operation | [`htc_vive_tracker_v1.txt:806-813`](htc_vive_tracker_v1.txt:806-813) |
| Using more than one tracker (up to 11 trackers + 2 controllers) | RF congestion; channel interference | [`htc_vive_tracker_v1.txt:814-819`](htc_vive_tracker_v1.txt:814-819) |

### 2.5 Firmware Update

| Activity | Hazard / Risk | Source Reference |
|----------|---------------|------------------|
| Updating firmware via USB cable (MCU, FPGA, RF firmware) | Bricking the device if USB cable is disconnected during update | [`htc_vive_tracker_v1.txt:725-738`](htc_vive_tracker_v1.txt:725-738) |
| Running firmware update commands (`lighthouse_watchman_update`) | Incorrect firmware file causing device malfunction | [`htc_vive_tracker_v1.txt:733-738`](htc_vive_tracker_v1.txt:733-738) |

### 2.6 LED Status Indicators (Operational Awareness)

| Activity | Hazard / Risk | Source Reference |
|----------|---------------|------------------|
| Interpreting LED status (green = normal, blinking red = low battery, blinking blue = pairing, orange = charging) | Misinterpreting status could lead to operating with low battery or in pairing mode unexpectedly | [`htc_vive_tracker_v1.txt:779-788`](htc_vive_tracker_v1.txt:779-788) |

---

## 3. Summary of Risk Categories

| Risk Category | Count | Key Sources |
|---------------|-------|-------------|
| **Electrical Shock / Fire** | 5 | Power connections, charging, damaged front panels |
| **Physical Injury** | 6 | Mounting at height, power tools, pinch hazards, dropped equipment |
| **Equipment Damage** | 8 | Dropping, over-tightening, adhesive failure, firmware update interruption |
| **Tracking / Performance Degradation** | 7 | Bright light, reflective surfaces, vibration, improper placement, RF interference |
| **RF Interference** | 3 | IR signals, antenna proximity, multi-device congestion |
| **Trip Hazards** | 2 | Power cables, USB cables |

---

## 4. Applicability to RoboCup Soccer Project

For the RoboCup Soccer project using Booster K1 robots, the following activities from these equipment guides are most relevant:

1. **Mounting Vive Trackers to the Booster K1** — The docking mechanism (1/4" screw, stabilizing pin) and ensuring the tracker does not obstruct the robot's movement or sensors.
2. **Power management** — Charging trackers and base stations between test sessions; ensuring batteries are not depleted during runs.
3. **RF coordination** — Managing multiple wireless devices (trackers, dongles, controllers) in the lab to avoid interference during competition-like conditions.
4. **Firmware updates** — Keeping tracker and base station firmware up-to-date without bricking devices.
5. **Lab setup** — Mounting base stations in the VXLab or testing area for tracking coverage of the RoboCup field.
