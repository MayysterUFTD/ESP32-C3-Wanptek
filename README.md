# Wanptek PSU Controller

An ESP32-C3 firmware project with a local web panel for monitoring and
controlling a compatible Wanptek power supply. The ESP connects to a 2.4 GHz
Wi-Fi network and starts a password-protected fallback access point for setup
and recovery.

## Features

- Live voltage, current, power, and regulation-mode readings
- Voltage and current setpoint control, with confirmation from PSU readback
- Remote-control, output, and over-current-protection (OCP) toggles
- Live charts for the latest 600 measurements
- Configurable measurement polling interval
- Web-based Wi-Fi setup, saved credentials, and fallback access point

## Requirements

- An ESP32-C3 development board supported by the PlatformIO
  `esp32-c3-devkitm-1` board definition
- PlatformIO Core (`pio`) or the PlatformIO IDE extension for Visual Studio
  Code
- A compatible Wanptek power supply and an electrical interface appropriate
  for its communication port
- A 2.4 GHz Wi-Fi network (optional if you only use the fallback access point)

The firmware uses the ESP32-C3 native USB Serial/JTAG port for logs. Some
boards may need a lower Wi-Fi transmit power; see
[Troubleshooting](#troubleshooting).

## Build and upload

1. Clone this repository and open its root directory.
2. Create `config.local.ini` in the project root with a private access point
   password:

   ```ini
   [env]
   build_flags = -DWANPTEK_AP_PASSWORD=\"choose-a-password\"
   ```

   The password must be 8–63 characters. This local file is ignored by Git;
   keep it private. The firmware will not build without this setting.
3. Connect the ESP32-C3 board over USB and run:

   ```sh
   pio run -t upload
   pio run -t uploadfs
   pio device monitor
   ```

   The firmware and web panel are uploaded separately. Upload the filesystem
   image again whenever files in `data/` change. The serial monitor runs at
   19200 baud and prints the access point name and panel URL, or the address
   assigned by your Wi-Fi network.

## Connect to Wi-Fi

1. Connect your phone or computer to the fallback network. Its name is
   `Wanptek-` followed by the last eight characters of the board's MAC
   address; the serial monitor prints the exact name.
2. Open [http://192.168.4.1](http://192.168.4.1).
3. Open **Settings**, enter your 2.4 GHz network name and password, then select
   **Connect and save**. Leave the password empty for an open network.
4. Once connected, the fallback access point shuts down after 15 seconds.
   Switch your phone or computer to the same Wi-Fi network and open the panel
   at the address shown in the panel or serial monitor.

Credentials are saved in device flash and retried after a restart or
connection loss. If the connection fails, the fallback access point starts
again so you can correct the settings. Select **Forget network** to delete
the saved credentials and return to the fallback access point.

## Connect the power supply

The firmware follows the Modbus RTU framing and register layout used by
[flkoliv/sw-wanptek-psu](https://github.com/flkoliv/sw-wanptek-psu). The
defaults are:

| Setting | Default |
| --- | --- |
| Modbus address | `0` |
| Serial format | `19200` baud, `8N1` |
| ESP32-C3 RX / TX | GPIO `3` / GPIO `4` |
| Measurement interval | `100 ms` |

Connect PSU TX to ESP32 RX, PSU RX to ESP32 TX, and signal ground where
required by the interface. **Do not connect unknown PSU pins directly to
ESP32 GPIO.** Use an appropriate level shifter or isolated interface for the
exact PSU communication port. Wanptek models and interfaces may differ;
confirm the model, port, address, baud rate, and electrical levels before
connecting.

After connecting to the panel:

1. Enable **Remote control**.
2. Enter voltage and current setpoints and select **Apply setpoints**. The
   panel confirms them only after a fresh PSU readback matches.
3. Enable **Output** only when ready; the panel asks for confirmation before
   switching it on.

The **Setpoints** and **Live charts** sections can be collapsed using their
headings. Both are open by default.

Disabling the output does not require confirmation. Setpoints are limited to
the maximum values reported by the power supply. The dashboard includes
optional quick presets; selecting one fills in the values but does not apply
them. Expand **Battery presets** to view the chemistry groups. Battery presets
change the voltage only; set the current limit separately. The flooded
lead-acid presets use 2.35 V per cell, up to 6S; verify the voltage for the
battery, charging stage, and temperature. The NiMH presets are nominal values,
not charging settings, up to 10S (12 V nominal). Verify the battery chemistry,
cell count, charging voltage, and charge-current specifications, and use an
appropriate charger and protection circuit.

The requested polling interval can be set from 10 to 1000 ms on the **Settings**
page and is saved in NVS, so it persists across restarts. A new device uses the
100 ms default. The actual interval may be longer because serial frames are sent
sequentially and the PSU and network need time to respond.

## Security

The web panel uses HTTP and has no separate authentication. Anyone with access
to the connected Wi-Fi network can control the power supply. Wi-Fi credentials
are sent without transport encryption and the saved network password is stored
in device flash. Use a trusted network, set a private access point password,
and protect physical access to the board. Do not expose the fallback access
point or panel to untrusted users.

## Troubleshooting

- **The panel is missing or SPIFFS fails to mount:** run `pio run -t uploadfs`
  after uploading the firmware.
- **The fallback network is not visible:** some boards require reduced
  transmit power. Replace the `build_flags` line in `config.local.ini` with
  the following, then rebuild and upload the firmware:

  ```ini
  [env]
  build_flags = -DWANPTEK_AP_PASSWORD=\"choose-a-password\" -DWANPTEK_WIFI_TX_POWER=WIFI_POWER_8_5dBm
  ```

- **The serial monitor shows no logs:** use the board's native USB Serial/JTAG
  port. If you use an external UART-to-USB adapter, remove the USB build flags
  from `platformio.ini` and rebuild.
- **The power supply does not respond:** verify the PSU model and port,
  interface and signal levels, TX/RX wiring, ground, address, and serial
  settings. Disconnect the load before investigating unexpected setpoints.

## Tests

Run the protocol unit tests with PlatformIO:

```sh
pio test
```
