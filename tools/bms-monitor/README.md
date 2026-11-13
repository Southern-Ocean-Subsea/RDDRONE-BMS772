# BMS772 Monitor

A standalone program that shows live data from an RDDRONE-BMS772 battery on a
Linux PC, through a USB-to-CAN adapter such as the Innomaker USB2CAN.

It is one Python file and one web page. It uses only what ships with Python 3,
so there is nothing to install with `pip`.

## Quick start

1. Connect the adapter to the BMS CAN bus and plug it into the PC.
2. Run:

   ```
   python3 bms_monitor.py
   ```

3. The dashboard opens in your browser at `http://localhost:8772/`.

The first time, the adapter has to be started at the BMS bitrate, which needs
administrator rights. The page shows a **Start adapter** button that asks for
your password. To avoid that prompt for good, run `./install.sh` once: it adds
a udev rule that starts the adapter whenever it is plugged in, and puts
"BMS772 Monitor" in the applications menu.

To see the dashboard without any hardware:

```
python3 bms_monitor.py --demo
```

## What it shows

| Area | Contents |
|---|---|
| Top row | State of charge, pack voltage, output voltage and whether the output is on, current and power, temperature, state of health, fault flags |
| Cells | Every cell voltage, lowest, highest, spread and distance from the average, on a scale between the BMS undervoltage and overvoltage limits |
| Faults | Status flags, the last fault code stored in the BMS decoded into names, and a log of every change since the monitor started |
| Trends | Voltage, current, state of charge and temperature for the last 15 minutes |
| Battery details | Identity, capacity, cycle count, and the current and voltage limits set in the BMS |
| Diagnostics | Message rates, raw payloads, registers, bus errors, other traffic on the bus, and every decoded field |

**Save snapshot** downloads everything currently known as a JSON file.

## What the BMS sends over CAN

The firmware in this repository publishes four Cyphal messages and answers one
readable register. The monitor decodes all of them.

| Message | Default subject ID | How often | Contents |
|---|---|---|---|
| Energy source | 4096 | every measurement (1 s by default) | Current, output voltage, remaining and full energy, time since the BMS booted |
| Battery status | 4097 | 1 s | Cell voltages, remaining charge, coldest and hottest sensor |
| Battery parameters | 4098 | 5 s | Model ID, design capacity, cell limits, current limits, cycle count, state of health, technology, mass |
| Battery info | 4099 | 1 s | Temperature, output voltage, current, average power, remaining and full energy, state of charge and health, status flags, time to full, battery ID, model name |
| Register `bms_fault_code` | on request | read every 2 s | The last fault the BMS recorded |

These are **not** sent over CAN by this firmware, so the monitor cannot show
them: the BMS state (normal, charge, sleep, fault), the individual temperature
sensors, and the rest of the `bms get all` parameter list. Those are only
available on the serial CLI.

## Requirements

- Linux with Python 3.7 or newer and the `ip` command (part of every desktop
  distribution).
- A USB-to-CAN adapter that appears as a SocketCAN interface (`can0`). The
  Innomaker USB2CAN normally uses the `gs_usb` kernel driver, which ships with
  current Ubuntu, Debian and Raspberry Pi OS kernels.
- The BMS in Cyphal mode (`can-mode CYPHAL`) with a node ID set, at
  1 Mbit/s classic CAN. These are the values `BMS_programming_script.py` in
  the NXP-BMS repository sets.
- CAN-H to CAN-H, CAN-L to CAN-L, and 120 ohm termination at both ends of the
  bus. Many adapters have a jumper or switch for their terminator.

## Options

| Option | Purpose |
|---|---|
| `--demo` | Simulated battery; no hardware |
| `--interface can1` | Use a specific interface instead of finding one |
| `--bitrate 500000` | BMS set to a bitrate other than 1 Mbit/s |
| `--node-id 125` | Node ID the monitor uses on the bus (default 126) |
| `--listen-only` | Never transmit. The stored fault code cannot be read in this mode |
| `--port 9000` | Web page port (default 8772) |
| `--host 0.0.0.0` | Let other computers on the network open the page |
| `--no-browser` | Do not open the browser |

## What it puts on the bus

The monitor only transmits while a BMS is talking, and only two things:

- a one-frame heartbeat each second, and
- read requests for five registers: `bms_fault_code` and the four subject-ID
  registers. Each request carries an empty value, which the firmware treats as
  a read. The monitor contains no code that writes a register or sends a
  command, and the tests check every transmitted frame for this.

If another device on the bus already uses the monitor's node ID, the monitor
stops transmitting and says so. Use `--listen-only` to keep it silent.

## Troubleshooting

| The page says | Check |
|---|---|
| Plug in the USB-to-CAN adapter | `ip link` should list `can0`. If not, the adapter or its driver is missing: `lsusb`, `sudo modprobe gs_usb` |
| Adapter found but not started | Press **Start adapter**, or run the command shown, or run `./install.sh` once |
| Adapter running at another bitrate | Another program configured it. Press **Restart adapter** to restart it at the BMS bitrate |
| Adapter is in bus-off | The adapter stopped after too many CAN errors, usually because nothing acknowledges its frames: BMS off or asleep, CAN-H and CAN-L swapped, missing terminator, or wrong bitrate. Fix that, then press **Restart adapter**. `--listen-only` stops the app transmitting at all |
| Could not start it: Device doesn't support restart from Bus Off | Fixed in version 1.0.1. Older versions asked the driver for automatic bus-off recovery, which gs_usb adapters such as the Innomaker refuse |
| Waiting for the BMS | The BMS sends nothing while asleep or off: press its button or connect a charger. Then wiring, termination, bitrate and `can-mode` |
| Bus errors: no acknowledgement | Nothing else on the bus is answering: BMS off, wiring, or wrong bitrate |
| BMS stopped sending | It went to sleep, was switched off, or the cable came out |
| Last fault code "not read yet" | The BMS is not answering register requests, or the monitor is in listen-only mode |

## Firmware behaviour worth knowing

The monitor follows `src/uavcan.c` and `src/UAVCAN/portid.c` as they are, including:

- **Current sign.** Positive is charging, negative is discharging.
- **Output voltage.** The reported voltage is measured after the power switch,
  so it drops to about zero when the output is off. The pack voltage on the
  dashboard is the sum of the cell voltages.
- **Fault flags latch.** The firmware sets status flags but has no path that
  clears them in normal running, so a flag can still be shown after the
  condition has passed.
- **Stored fault code.** `bms_fault_code` holds the most recent non-zero fault
  and is not cleared when the fault goes away.
- **Temperatures.** Battery info carries degrees Celsius directly. Battery
  status converts with 272.15 instead of 273.15; the monitor undoes it with
  the same constant so values match the BMS CLI.
- **Fixed fields.** Readiness, health and the error code in battery status are
  constants in this firmware.
- **Register replies.** The firmware numbers replies with its own counter, so
  the monitor sends one request at a time and checks the value type.

## Tests

```
python3 -m unittest discover -s tests
```

`tests/vectors.json` holds messages serialised by the firmware's own generated
headers in `inc/UAVCAN` (`tests/gen_vectors.c` rebuilds it), so the decoder is
checked against the real wire format. The transport was also cross-checked in
both directions against the Cyphal code in apama-control-system, which has
read a real BMS on the ROV.

The tests cannot open a real CAN interface. The adapter handling around
SocketCAN is covered with a stand-in socket; the first run against the real
adapter and battery is still to be done.
