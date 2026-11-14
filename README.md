# ESP32 Wanptek PSU controller

The ESP32-C3 connects to a configured 2.4 GHz Wi-Fi network and serves a local
web panel for monitoring and controlling a compatible Wanptek power supply.
When the network is unavailable, it creates a password-protected fallback AP.

## Build and upload

From this directory, build and upload the firmware and the SPIFFS web files:

```powershell
pio run -t upload
pio run -t uploadfs
pio device monitor
```

The firmware upload does not upload the web files. Run `pio run -t uploadfs`
after uploading the firmware, and again whenever files in `data` change.
The filesystem image must be SPIFFS, as configured in `platformio.ini`.

The monitor runs at 19200 baud. It prints the active access-point name and
panel URL, or the address assigned by the home network.
The firmware uses the ESP32-C3 native USB Serial/JTAG port for `Serial`, not an
external UART-to-USB adapter. The AP status is repeated every 10 seconds. If the
board uses an external UART adapter instead, remove the USB build flags from
`platformio.ini` and rebuild. Some ESP32-C3 boards require reduced transmit
power; set `-DWANPTEK_WIFI_TX_POWER=WIFI_POWER_8_5dBm` in `config.local.ini`
if the access point is not visible at the default power.

## Home Wi-Fi

Connect to the fallback AP and open `http://192.168.4.1`. In **Wi-Fi
settings**, enter the SSID and password and select **Connect and save**.
The ESP32 saves the credentials in NVS and attempts to connect; failed
credentials remain saved so that they can be corrected via the fallback AP.
The AP is turned off 15 seconds after a successful connection. Switch the
phone or computer to the home network and open the IP shown in the panel or
serial monitor. After a restart or connection loss, the ESP32 retries the
saved network and activates the fallback AP if it cannot connect. **Forget
network** deletes the saved credentials and returns to the fallback AP.

The web panel has no separate authentication or HTTPS: anyone on the connected
network can control the PSU, and the Wi-Fi password is transmitted unencrypted.
Use only a trusted network, and configure Wi-Fi via the protected fallback AP.
The saved Wi-Fi password is stored in device flash; protect physical access.

## PSU serial connection

The firmware follows the Modbus RTU framing and register layout used by
[flkoliv/sw-wanptek-psu](https://github.com/flkoliv/sw-wanptek-psu):

- address `0`, `19200` baud, `8N1`
- FC03 reads eight registers beginning at address `0`
- FC16 writes the status, voltage setpoint, and current setpoint
- FC16 setpoint byte order follows the endianness bit reported by FC03
- voltage/current setpoints are limited to the maximum values reported by the PSU

Defaults in `src/main.cpp` are RX GPIO 3, TX GPIO 4, and the serial settings
above. Connect PSU TX to ESP32 RX, PSU RX to ESP32 TX, and signal ground where
required by the interface. **Use a level shifter or isolated interface suited
to the exact PSU communication port. Do not connect unknown PSU pins directly
to ESP32 GPIO.** Confirm the PSU model, port, address, and baud rate before
connecting; Wanptek models and communication interfaces may differ.

After joining the ESP32 access point, open the panel URL shown in the serial
monitor. Enable **Remote control** before changing setpoints. Output enable
requires confirmation in the page; disabling the output does not.
The panel reports setpoints as confirmed only after a fresh PSU readback matches
the requested voltage and current. If verification fails, the serial monitor
shows the FC16 request, response, and decoded/raw setpoint registers. Disconnect
the load before troubleshooting unexpected settings.

The panel plots the most recent 600 received voltage and current samples.
The polling interval can be set between 10 and 1000 ms, with a default of
100 ms after each restart. The displayed actual interval can be longer than
requested: Modbus frames are serialized, require inter-frame silence, and the
PSU and network have their own response times. At 19200 baud, the eight-byte
request and 21-byte response alone take about 15 ms on the wire.

The AP password is set at build time in the untracked `config.local.ini` file:

```ini
[env]
build_flags = -DWANPTEK_AP_PASSWORD=\"your-password\"
```

The password must be 8–63 characters long. Rebuild and upload the firmware
after changing it. Keep this file private and do not expose the AP to untrusted
users; anyone connected to it can control the power supply.
