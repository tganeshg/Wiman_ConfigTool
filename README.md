# ConfigTool — MT7628N AT Command Daemon

AT-command daemon for **MT7628N** (OpenWrt) that communicates over UART with an **STM32** master. Implements the STM32 ↔ MT7628 UART AT protocol for WiFi (AP/STA), Ethernet, reset, and configuration.

## Protocol

- **ASCII** AT commands over UART; line ending **CRLF** (`\r\n`).
- **STM32 = master**, MT7628 = slave; one command at a time.
- Responses end with **OK** or **ERROR**; async events can be sent at any time.
- Full specification: **`others/stm32_to_mt7628_uart.txt`**

After boot/reset the daemon sends **`+SYS:BOOT,READY`**; the STM32 must wait for this before sending commands.

---

## Build (OpenWrt cross-compile)

Requires OpenWrt staging/toolchain (see `Makefile` for `STAGING_DIR`).

```bash
make
```

Output: **`bin/Sample`**. Copy to the device (e.g. `/usr/bin/` or your firmware path).

```bash
make clean   # remove objects and binary
```

---

## Implemented AT Commands

### Basic
| Command    | Description        | Response / behavior        |
|-----------|--------------------|----------------------------|
| `AT`      | Check alive        | OK                         |
| `AT+VER?` | Protocol version   | +VER:MT7628N-AT-1.2.0, OK  |

### Reset & config
| Command       | Description           | Response / behavior                    |
|---------------|-----------------------|----------------------------------------|
| `AT+RST`      | Software reset        | OK, then Linux reboot → +SYS:BOOT,READY |
| `AT+FACTORY`  | Factory reset         | Clears WiFi config, reboot → +SYS:BOOT,READY |
| `AT+SAVE`     | Save to flash         | OK (runs `uci commit`)                 |

### WiFi mode (0=OFF, 1=STA, 2=AP, 3=AP+STA)
| Command            | Description        | Response / behavior                          |
|--------------------|--------------------|----------------------------------------------|
| `AT+WIFIMODE=<n>`  | Set mode           | OK; ensures wifinet0/wifinet1, network.wwan |
| `AT+WIFIMODE?`     | Query mode         | +WIFIMODE:&lt;0\|1\|2\|3&gt;, OK             |

### WiFi AP (LUCI: named section `wifinet0`)
| Command               | Description              | Response / behavior                    |
|-----------------------|--------------------------|----------------------------------------|
| `AT+WIFIAPCFG=SSID,PWD,SEC` | Set AP config      | OK (SEC: OPEN, WPA, WPA2, WPA_WPA2)   |
| `AT+WIFIAPCFG?`       | Query AP config          | +WIFIAPCFG:SSID=…,SEC=…,CLIENTS=n, OK |
| `AT+WIFIAP?`          | AP client list           | +WIFIAP:CLIENT,MAC,IP … OK             |

### WiFi STA (LUCI: named section `wifinet1`, network `wwan`)
| Command                | Description          | Response / behavior                          |
|------------------------|---------------------|----------------------------------------------|
| `AT+WIFISTACFG=SSID,PWD,SEC` | Set STA config | OK; creates wifinet1/network.wwan if needed |
| `AT+WIFISTACFG?`       | Query STA config     | +WIFISTACFG:SSID=…,SEC=…,PASSWORD=…, OK      |
| `AT+WIFISTA=1` / `=0`  | Connect / disconnect | OK                                          |
| `AT+WIFISTA?`          | STA status           | +WIFISTA:CONNECTED,IP=…,RSSI=…,SEC=… or DISCONNECTED, OK |

### Ethernet
| Command   | Description              | Response / behavior                                      |
|-----------|--------------------------|----------------------------------------------------------|
| `AT+ETH?` | Link, IP, client count   | +ETH:DOWN, OK or +ETH:UP,IP=…,CLIENTS=n + +ETH:CLIENT,1,MAC,IP … OK |

---

## Async events (unsolicited, sent when state changes)

### WiFi
| Event                    | When                         |
|--------------------------|------------------------------|
| `+WIFI:APJOIN,MAC,IP`    | Client joined AP             |
| `+WIFI:APLEAVE,MAC`      | Client left AP               |
| `+WIFI:STACONN,IP,SEC`   | STA connected (got IP)       |
| `+WIFI:STADISCONN,REASON`| STA disconnected             |

### Ethernet (PORT = 1 for single LAN)
| Event                      | When                     |
|----------------------------|--------------------------|
| `+ETH:UP,1,IP=<ip>`        | Link up                  |
| `+ETH:DOWN,1`              | Link down                |
| `+ETH:CLIENT,1,MAC,IP`     | New client got IP         |
| `+ETH:CLIENT_LEAVE,1,MAC`  | Client left              |

---

## Source layout

| Path            | Role                                      |
|-----------------|-------------------------------------------|
| `source/main.c` | Entry, UART init, select loop, event poll |
| `source/atcmd.c`| All AT handlers, UCI, WiFi/ETH event poll |
| `source/uart.c` | UART open/read/write, line callback       |
| `include/*.h`   | atcmd.h, uart.h                           |
| `others/stm32_to_mt7628_uart.txt` | Full protocol spec              |

---

## Dependencies

- OpenWrt toolchain (mipsel, musl); **libuci** (UCI), **libubox**.
- Runtime: **eth0** for Ethernet; **phy0-ap0** (AP) and discovered STA interface for WiFi; **/tmp/dhcp.leases** for client lists.

---

## Error responses

- **ERROR** — generic failure  
- **ERROR:1** — INVALID_PARAM (e.g. bad AT+WIFIAPCFG / AT+WIFISTACFG parameters)

Full error code list is in **`others/stm32_to_mt7628_uart.txt`**.
