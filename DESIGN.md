# ConfigTool — Design & Architecture

## 1. System Context

```
  ┌─────────────────────────────────────────────┐
  │               Hardware                       │
  │                                              │
  │   STM32 (Master)  ←── UART ttyS1 ──►  MT7628N (Slave)
  │   PLC / HMI                              OpenWrt Linux
  │   AT command issuer                     configTool daemon
  └─────────────────────────────────────────────┘
```

The **STM32 is the master**: it sends AT commands and waits for `OK`/`ERROR:N`.  
The **MT7628N is the slave**: it executes each command (UCI config, `wifi`, `iw`, etc.) and sends back responses.  
The **configTool daemon** (`bin/Sample`) runs on the MT7628N and is the sole process handling this protocol.

---

## 2. Software Layers

```
┌──────────────────────────────────────────────────────────────┐
│  main.c  — Entry point, signal handling, select() event loop │
│            Calls wifi_events_poll() + eth_events_poll() each  │
│            loop tick for async unsolicited events             │
├──────────────────────────────────────────────────────────────┤
│  atcmd.c — AT command handlers + async event polling         │
│            • UCI C-API calls (no shell for user data)         │
│            • Non-blocking DHCP retry via pending tables       │
│            • WiFi (AP/STA) + Ethernet + Reset + Save          │
├──────────────────────────────────────────────────────────────┤
│  uart.c  — UART I/O abstraction                              │
│            • termios configuration                            │
│            • Ring-style RX buffer, line extraction           │
│            • Callback dispatch per complete line              │
│            • ERROR:6 on oversized lines (≥ 256 B)             │
└──────────────────────────────────────────────────────────────┘
```

---

## 3. Data Flow

### 3.1 Command path (STM32 → MT7628N → STM32)

```
UART HW RX
    │
    ▼
uart_process_events()            [uart.c]
    │  read() bytes into rx_buffer
    │  scan for \r / \n / \r\n
    │  extract line_buffer (null-terminated)
    │  if line ≥ 256 B → write "ERROR:6\r\n" immediately
    │
    ▼
command_callback()               [atcmd.c]
    │  strcmp / strncmp dispatch
    │
    ▼
cmd_wifi*/cmd_eth*/cmd_reset*()  [atcmd.c]
    │  validate params (sscanf, validate_wifi_string)
    │  uci_get_string / uci_set_string  (UCI C-API)
    │  system() for wifi restart / reboot (hardcoded only)
    │
    ▼
send_response()                  [atcmd.c]
    │  vsnprintf → uart_write() → write() to UART fd
    ▼
UART HW TX  →  STM32
```

### 3.2 Async event path (state-change → STM32)

```
main loop tick (select() 1 s timeout)
    │
    ├─► wifi_events_poll()        [atcmd.c]
    │       iw dev phy0-ap0 station dump  →  AP client list diff
    │       get_sta_connected_iface       →  STA connected/disconnected diff
    │       apjoin_pending[]              →  non-blocking DHCP retry
    │       send_event() on change        →  uart_write()
    │
    └─► eth_events_poll()         [atcmd.c]
            /sys/class/net/eth0/carrier   →  link up/down diff
            /tmp/dhcp.leases + ip neigh   →  client join/leave diff
            eth_up_pending                →  non-blocking DHCP retry
            send_event() on change        →  uart_write()
```

---

## 4. Module Responsibilities

### `main.c`
- POSIX signal handling (`SIGINT`/`SIGTERM` → clean shutdown).
- UART lifecycle: `uart_init` → `uart_configure` → `uart_open` → loop → `uart_destroy`.
- `select()` loop with 1 s timeout so async events are polled regularly.
- No business logic; delegates entirely to `uart.c` and `atcmd.c`.

### `uart.c`
- Owns the `uart_inst_t` opaque struct (fd, termios, RX ring buffer, callback table).
- `uart_process_events()` is the only place `read()` is called; it:
  1. Appends to `rx_buffer`.
  2. Finds line terminators (`\n`, `\r`, `\r\n`).
  3. Copies complete lines into `line_buffer` (255-char max) and dispatches all registered callbacks.
  4. Writes `ERROR:6\r\n` for lines that would overflow the buffer.
  5. Compacts the unprocessed tail of `rx_buffer` with `memmove`.
- Callback registration is open-ended (up to 16 slots); the `pattern` field is reserved for future prefix-based filtering — currently all callbacks fire on every line.

### `atcmd.c`
- All UCI interactions go through `uci_get_string()` / `uci_set_string()` which use the **UCI C-API** directly — no user-supplied data ever reaches a shell.
- `system()` calls are restricted to **hardcoded** OpenWrt commands (`wifi`, `reboot`, `firstboot`, `uci commit`).
- Input validation order: `sscanf` for numeric params, `validate_wifi_string` (printable ASCII 0x20–0x7E) for SSIDs/passwords, length bounds.
- Error response macros (`ERR_INVALID_PARAM` / `ERR_CMD_FAILED` / `ERR_NOT_SUPPORTED`) ensure a consistent `ERROR:N` protocol format throughout.

---

## 5. Key Design Decisions

### 5.1 Single-threaded `select()` loop
All I/O and polling happens on a single thread. There are no mutexes, no shared-memory races.  
Trade-off: long-running operations (UCI commit, `wifi` restart) block command processing briefly. `select()` resumes once they complete.

### 5.2 Non-blocking DHCP retry (pending tables)
Previously, APJOIN and ETH-UP events blocked the event loop with `sleep(1)` for up to 3 s / 5 s waiting for a DHCP lease.  
Now:
- **`apjoin_pending[]`** (8 slots): new AP clients with no IP are queued here; each poll tick retries the lease lookup; event fires when IP arrives or retries exhaust (3 ticks ≈ 3 s).
- **`eth_up_pending`**: link-up with no IP defers `+ETH:UP` for up to 5 poll ticks; the event fires immediately once an IP appears.

No `sleep()` anywhere in the hot path.

### 5.3 UCI C-API for configuration
WiFi SSID, password, and encryption type are written via `uci_set_string()` (UCI C library), not via `system("uci set ... value")`. This eliminates shell-injection risk for user-supplied values.

### 5.4 Named UCI sections (LuCI compatible)
AP interface: **`wifinet0`** (`radio0`, mode `ap`, network `lan`)  
STA interface: **`wifinet1`** (`radio0`, mode `sta`, network `wwan`)  
These match the LuCI web UI naming convention so both interfaces can be managed from LuCI and the AT command daemon simultaneously.

### 5.5 Input validation
| Check | Where |
|-------|-------|
| SSID length 1–32, printable ASCII | `cmd_wifiapcfg_set`, `cmd_wifista_cfg` |
| Password length 8–63, printable ASCII (WPA/WPA2) | same |
| Security string: OPEN / WPA / WPA2 / WPA_WPA2 only | same |
| WiFi mode 0–3 via `sscanf` | `cmd_wifimode_set` |
| STA enable 0–1 via `sscanf` | `cmd_wifista_set` |
| Line length < 256 B | `uart_process_events` |

---

## 6. State Machines

### 6.1 WiFi event state (`wifi_event_prev`)
```
Fields: ap_macs[], ap_ips[], ap_count, sta_connected, sta_ip, sta_sec

Each poll tick:
  current_ap_list  diff with  prev_ap_list
       │                            │
   new MAC found?             old MAC missing?
       │                            │
  → APJOIN event             → APLEAVE event

  current_sta_connected diff with prev_sta_connected
       │                               │
  newly connected?              just disconnected?
       │                               │
  → STACONN event              → STADISCONN event
```

### 6.2 APJOIN pending table
```
New client, no IP yet
    │
    ▼
apjoin_pending[i] = { mac, retries=3 }
    │
    ▼  (each poll tick)
ap_client_lookup_ip()
    ├── found?  → fire +WIFI:APJOIN,mac,ip  → remove from table
    └── retries == 0?  → fire +WIFI:APJOIN,mac,0.0.0.0  → remove
        else retries--  → keep in table
```

### 6.3 Ethernet event state (`eth_event_prev` + `eth_up_pending`)
```
carrier(t)  vs  carrier(t-1)
    │
  rising edge?
    ├── IP available? → +ETH:UP immediately
    └── no IP?  → eth_up_pending.active=1, retries=5
                     each tick: re-read IP
                     IP found or retries==0 → +ETH:UP (with IP or 0.0.0.0)
  falling edge?
    └── +ETH:DOWN; cancel any pending

client_list(t)  diff with  client_list(t-1)
    ├── new? → +ETH:CLIENT
    └── gone? → +ETH:CLIENT_LEAVE
```

---

## 7. Error Code Reference

| Code | Macro | Meaning |
|------|-------|---------|
| `ERROR:1` | `ERR_INVALID_PARAM` | Bad / out-of-range parameter |
| `ERROR:2` | `ERR_CMD_FAILED` | Command executed but system call / UCI returned error |
| `ERROR:6` | `ERR_NOT_SUPPORTED` | Oversized line (≥256 B) or unrecognised AT command |

---

## 8. Directory Layout

```
configTool/
├── Makefile                  Cross-compile for OpenWrt mipsel (STAGING_DIR configurable)
├── README.md                 Quick-start, protocol summary, command reference
├── DESIGN.md                 This file — architecture and design rationale
├── include/
│   ├── uart.h                UART abstraction API
│   └── atcmd.h               AT command public API (atcmd_init, wifi/eth poll)
├── source/
│   ├── main.c                Entry point, signal handler, select() loop
│   ├── uart.c                UART I/O, line parsing, callback dispatch
│   └── atcmd.c               AT handlers, UCI helpers, async event polling
├── bin/
│   └── Sample                Compiled binary (mipsel OpenWrt)
└── others/
    └── stm32_to_mt7628_uart.txt   Full UART AT protocol specification
```

---

## 9. Build Notes

```bash
# Default (uses STAGING_DIR from Makefile)
make

# Override SDK path
make STAGING_DIR=/opt/openwrt/staging_dir

# Override firmware version
make VERSION=1.3.0

# Clean
make clean
```

Compiler hardening flags enabled: `-fstack-protector-all`, `-D_FORTIFY_SOURCE=2`, `-Werror=format-security`.

---

## 10. Runtime Dependencies

| Resource | Used for |
|----------|----------|
| `/dev/ttyS1` (115200 8N1) | UART communication with STM32 |
| `libuci` | Read/write OpenWrt UCI wireless config |
| `libubox` | OpenWrt utility library (linked) |
| `iw` | AP station dump, STA link status |
| `ip neigh` | Ethernet neighbour (ARP) table |
| `/tmp/dhcp.leases` | DHCP lease → IP mapping for clients |
| `/sys/class/net/eth0/carrier` | Ethernet link carrier detection |
| `wifi` (OpenWrt) | Apply wireless config changes |
| `reboot` / `firstboot` | Software / factory reset |
