# ConfigTool — MT7628N AT Command Daemon

AT-command daemon for **MT7628N** (OpenWrt) that communicates over UART with an **STM32** master.
Implements the STM32 ↔ MT7628 UART AT protocol for WiFi AP/STA, Ethernet, reset, and configuration management.

- **Full protocol spec:** `others/stm32_to_mt7628_uart.txt`
- **Architecture & design:** [`DESIGN.md`](DESIGN.md)

---

## Protocol basics

| Property | Value |
|----------|-------|
| Transport | UART `ttyS1`, 115200 8N1 |
| Encoding | ASCII, line ending `\r\n` (CRLF) |
| Master | STM32 — sends commands, waits for response |
| Slave | MT7628N — executes, replies `OK` or `ERROR:N` |
| Max line length | 255 characters (longer lines → `ERROR:6`) |

After boot/reset the daemon sends **`+SYS:BOOT,READY`**; the STM32 must wait for this before sending commands.

---

## Build (OpenWrt cross-compile)

Requires an OpenWrt SDK/staging directory for mipsel.

```bash
# Default — uses STAGING_DIR path set in Makefile
make

# Override SDK path on the command line
make STAGING_DIR=/opt/openwrt/staging_dir

# Override firmware version string
make VERSION=1.3.0

# Remove objects and binary
make clean
```

Output binary: **`bin/Sample`**.  
Copy to the target device (e.g. `/usr/bin/configtool`) and run it as a daemon on startup.

---

## AT Command Reference

### Basic

| Command | Description | Response |
|---------|-------------|----------|
| `AT` | Alive check | `OK` |
| `AT+VER?` | Firmware version | `+VER:MT7628N-AT-1.2.0` then `OK` |

### Reset & Persistence

| Command | Description | Response |
|---------|-------------|----------|
| `AT+RST` | Software reboot | `OK`, then Linux reboots → `+SYS:BOOT,READY` |
| `AT+FACTORY` | Factory reset + reboot | `OK`, clears all config → `+SYS:BOOT,READY` |
| `AT+SAVE` | Commit UCI config to flash | `OK` or `ERROR:2` |

### WiFi Mode

Mode values: `0` = OFF, `1` = STA only, `2` = AP only, `3` = AP + STA

| Command | Description | Response |
|---------|-------------|----------|
| `AT+WIFIMODE=<0-3>` | Set WiFi mode | `OK` or `ERROR:N` |
| `AT+WIFIMODE?` | Query current mode | `+WIFIMODE:<0\|1\|2\|3>` then `OK` |

### WiFi AP (UCI section `wifinet0`)

| Command | Description | Response |
|---------|-------------|----------|
| `AT+WIFIAPCFG=<SSID>,<PWD>,<SEC>` | Set AP config | `OK` or `ERROR:N` |
| `AT+WIFIAPCFG?` | Query AP config | `+WIFIAPCFG:SSID=…,SEC=…,CLIENTS=n` then `OK` |
| `AT+WIFIAP?` | List connected clients | `+WIFIAP:CLIENT,<MAC>,<IP>` × n then `OK` |

`SEC` values: `OPEN`, `WPA`, `WPA2`, `WPA_WPA2`  
`SSID`: 1–32 printable ASCII characters  
`PWD`: 8–63 printable ASCII characters (ignored for `OPEN`)

### WiFi STA (UCI section `wifinet1`, network `wwan`)

| Command | Description | Response |
|---------|-------------|----------|
| `AT+WIFISTACFG=<SSID>,<PWD>,<SEC>` | Set STA credentials | `OK` or `ERROR:N` |
| `AT+WIFISTACFG?` | Query STA credentials | `+WIFISTACFG:SSID=…,SEC=…,PASSWORD=…` then `OK` |
| `AT+WIFISTA=1` | Connect STA | `OK` |
| `AT+WIFISTA=0` | Disconnect STA | `OK` |
| `AT+WIFISTA?` | Query STA status | `+WIFISTA:CONNECTED,IP=…,RSSI=…,SEC=…` or `+WIFISTA:DISCONNECTED,REASON=NOT_CONNECTED` then `OK` |

### Ethernet

| Command | Description | Response |
|---------|-------------|----------|
| `AT+ETH?` | Link status, IP, clients | `+ETH:DOWN` or `+ETH:UP,IP=…,CLIENTS=n` + `+ETH:CLIENT,1,<MAC>,<IP>` × n then `OK` |

---

## Async Events (unsolicited)

The daemon emits these at any time when state changes. No command needed.

### WiFi events

| Event | Trigger |
|-------|---------|
| `+WIFI:APJOIN,<MAC>,<IP>` | Client joined the AP |
| `+WIFI:APLEAVE,<MAC>` | Client left the AP |
| `+WIFI:STACONN,<IP>,<SEC>` | STA connected and got IP |
| `+WIFI:STADISCONN,NOT_CONNECTED` | STA disconnected |

> **Note:** `APJOIN` uses a non-blocking DHCP wait (up to ~3 poll ticks ≈ 3 s). If no lease
> appears within that window the event is fired with `IP=0.0.0.0`.

### Ethernet events (PORT = `1` for the single LAN port)

| Event | Trigger |
|-------|---------|
| `+ETH:UP,1,IP=<ip>` | Ethernet link came up |
| `+ETH:DOWN,1` | Ethernet link went down |
| `+ETH:CLIENT,1,<MAC>,<IP>` | New LAN client appeared |
| `+ETH:CLIENT_LEAVE,1,<MAC>` | LAN client disappeared |

> **Note:** `+ETH:UP` uses a non-blocking DHCP wait (up to ~5 poll ticks ≈ 5 s).

---

## Error Codes

| Response | Meaning |
|----------|---------|
| `ERROR:1` | Invalid or out-of-range parameter |
| `ERROR:2` | Command executed but system call / UCI returned an error |
| `ERROR:6` | Input line too long (≥ 256 bytes) or unrecognised command |

Full error code definitions: `others/stm32_to_mt7628_uart.txt`

---

## Source Layout

| Path | Role |
|------|------|
| `source/main.c` | Entry point, signal handling, `select()` event loop |
| `source/atcmd.c` | All AT command handlers, UCI helpers, async event polling |
| `source/uart.c` | UART open/configure/read/write, line extraction, callback dispatch |
| `include/uart.h` | UART abstraction API |
| `include/atcmd.h` | AT command public API (`atcmd_init`, poll functions) |
| `others/stm32_to_mt7628_uart.txt` | Full protocol specification |
| `DESIGN.md` | Architecture, data flow, state machine diagrams |

---

## Runtime Dependencies

| Dependency | Purpose |
|------------|---------|
| `libuci` | Read/write OpenWrt UCI wireless configuration |
| `libubox` | OpenWrt utility library |
| `iw` | AP station dump; STA link/signal query |
| `ip neigh` | Ethernet ARP neighbour table |
| `/tmp/dhcp.leases` | Map MAC addresses to DHCP-assigned IPs |
| `/sys/class/net/eth0/carrier` | Ethernet link carrier state |
| `wifi` (OpenWrt) | Apply wireless configuration changes |
| `reboot` / `firstboot` | Software / factory reset |
