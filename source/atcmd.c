/*****************************************************************
*
*   Copyright (c) 2026
*   All rights reserved.
*
*   Project         :
*   Last Updated on :
*   Author          : Ganesh
*
*   Revision History
****************************************************************
*   Date            Version     Name        Description
****************************************************************
*   25/02/2026      1.0         Ganesh      Initial Split (atcmd)
*
*****************************************************************/

/*** Includes ***/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <stdarg.h>
#include <ctype.h>
#include <time.h>
#include <uci.h>

/****************************************************************
* Firmware version
****************************************************************/
#define AT_VERSION "MT7628N-AT-1.2.0"

/*============================================================================
 * ERROR CODES (match protocol spec)
 *============================================================================*/
#define ERR_INVALID_PARAM   "ERROR:1"   /* Bad / out-of-range parameter        */
#define ERR_CMD_FAILED      "ERROR:2"   /* Command executed but returned error  */
#define ERR_NOT_SUPPORTED   "ERROR:6"   /* Oversized / unrecognised input       */

#include "uart.h"
#include "atcmd.h"

/*============================================================================
 * RESPONSE/EVENT HELPERS
 *============================================================================*/
/****************************************************************
* send_response
*
* Formats and sends a solicited response line to the STM32 master.
* Appends \r\n after the formatted string.  Used for all command
* responses: "OK", "ERROR:N", "+CMD:data", etc.
*
* Parameters:
*   uart  - Open UART instance to write to
*   fmt   - printf-style format string
*   ...   - Format arguments
****************************************************************/
static void send_response(uart_inst_t *uart, const char *fmt, ...) {
    char buf[256] = {0};
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    uart_write(uart, buf, strlen(buf));
    uart_write(uart, "\r\n", 2);
}

/****************************************************************
* drain_popen
*
* Reads and discards all remaining output from a popen() FILE
* before calling pclose().  Prevents SIGPIPE ("Broken pipe") in
* the child process if we stopped reading early.
*
* Parameters:
*   fp  - FILE* returned by popen(); safe to call with NULL.
****************************************************************/
static void drain_popen(FILE *fp) {
    char buf[256] = {0};
    if (fp) while (fgets(buf, sizeof(buf), fp)) {}
}

/****************************************************************
* validate_wifi_string
*
* Validates that every character in a string is printable ASCII
* (0x20 space through 0x7E tilde).  Rejects empty strings, NUL
* pointers, and any byte with a control character or high bit set.
*
* Used to sanitise SSIDs and WPA passwords received over the UART
* AT interface before they are passed to the UCI configuration API.
*
* Parameters:
*   s  - Null-terminated string to validate
*
* Returns:
*   1 if the string is non-empty and all characters are printable
*   ASCII, 0 otherwise.
****************************************************************/
static int validate_wifi_string(const char *s) {
    if (!s || s[0] == '\0') return 0;
    while (*s) {
        unsigned char c = (unsigned char)*s;
        if (c < 0x20 || c > 0x7E) return 0;
        s++;
    }
    return 1;
}

/** Parse dnsmasq lease line "timestamp mac ip hostname client_id"; return 1 if mac matches (case-insensitive) and copy ip. */
static int lease_line_get_ip(const char *lease_line, const char *mac, char *ip_out, size_t ip_len) {
    unsigned int ts = 0;
    char mac_buf[18] = {0};
    char ip_buf[16] = {0};
    if (sscanf(lease_line, "%u %17s %15s", &ts, mac_buf, ip_buf) < 3)
        return 0;
    if (strcasecmp(mac_buf, mac) != 0)
        return 0;
    strncpy(ip_out, ip_buf, ip_len - 1);
    ip_out[ip_len - 1] = '\0';
    return 1;
}

/****************************************************************
* send_event
*
* Formats and sends an unsolicited (async) event line to the
* STM32 master.  Events are not paired with a command so no "OK"
* line follows.  Appends \r\n after the formatted string.
*
* Examples: "+WIFI:APJOIN,aa:bb:cc:dd:ee:ff,192.168.1.10"
*           "+ETH:UP,1,IP=192.168.1.1"
*
* Parameters:
*   uart  - Open UART instance to write to
*   fmt   - printf-style format string
*   ...   - Format arguments
****************************************************************/
static void send_event(uart_inst_t *uart, const char *fmt, ...) {
    char buf[256] = {0};
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    uart_write(uart, buf, strlen(buf));
    uart_write(uart, "\r\n", 2);
}

/*============================================================================
 * UCI HELPERS
 *============================================================================*/
/****************************************************************
* uci_get_string
*
* Reads a single UCI string option using the UCI C library API.
* Allocates a fresh UCI context per call (no global context state).
*
* Parameters:
*   pkg      - UCI package name (e.g. "wireless", "network")
*   section  - UCI section name or anonymous index (e.g. "wifinet0",
*              "@wifi-iface[0]")
*   option   - Option name (e.g. "ssid", "disabled")
*   out      - Output buffer to receive the value string
*   out_len  - Size of output buffer in bytes
*
* Returns:
*   0 on success with out filled (null-terminated, truncated to
*   out_len-1 if necessary).
*   -1 if the package/section/option is not found or any UCI
*   operation fails.
****************************************************************/
static int uci_get_string(const char *pkg, const char *section, const char *option,
                          char *out, size_t out_len) {
    struct uci_context *ctx = uci_alloc_context();
    struct uci_ptr ptr;
    char path[256] = {0};

    snprintf(path, sizeof(path), "%s.%s.%s", pkg, section, option);

    if (uci_lookup_ptr(ctx, &ptr, path, true) != UCI_OK) {
        uci_free_context(ctx);
        return -1;
    }

    if (!ptr.o || !ptr.o->v.string) {
        uci_free_context(ctx);
        return -1;
    }

    strncpy(out, ptr.o->v.string, out_len - 1);
    out[out_len - 1] = '\0';
    uci_free_context(ctx);
    return 0;
}

/****************************************************************
* uci_set_string
*
* Writes a single UCI string option and commits the change to disk
* using the UCI C library API.  User-supplied values (e.g. SSID,
* password) are passed directly to the UCI library — NOT through
* a shell — so no shell-injection escaping is needed here.
*
* Parameters:
*   pkg      - UCI package name (e.g. "wireless")
*   section  - UCI section name (e.g. "wifinet0")
*   option   - Option name (e.g. "ssid")
*   value    - Value string to set
*
* Returns:
*   0 on success (option written and committed).
*   -1 if uci_lookup_ptr, uci_set, or uci_commit fail.
*
* Notes:
*   Each call allocates and frees its own UCI context.
*   uci_commit is called with a NULL file argument so the change
*   is written to the default UCI config directory.
****************************************************************/
static int uci_set_string(const char *pkg, const char *section, const char *option,
                          const char *value) {
    struct uci_context *ctx = uci_alloc_context();
    struct uci_ptr ptr;
    char path[256] = {0};

    snprintf(path, sizeof(path), "%s.%s.%s", pkg, section, option);

    if (uci_lookup_ptr(ctx, &ptr, path, true) != UCI_OK) {
        uci_free_context(ctx);
        return -1;
    }

    ptr.value = value;

    if (uci_set(ctx, &ptr) != UCI_OK) {
        uci_free_context(ctx);
        return -1;
    }

    if (uci_commit(ctx, &ptr.p, false) != UCI_OK) {
        uci_free_context(ctx);
        return -1;
    }

    uci_free_context(ctx);
    return 0;
}

/*============================================================================
 * AP / STA / ETH / RESET / SAVE COMMANDS
 * LUCI expects named section "wifinet0" for AP, "wifinet1" for STA
 *============================================================================*/
#define AP_IFACE_NAME "wifinet0"
#define STA_IFACE_NAME "wifinet1"

/****************************************************************
* cmd_wifiapcfg_set
*
* Handles: AT+WIFIAPCFG=<SSID>,<PASSWORD>,<SECURITY>
*
* Configures the Access Point interface (UCI section wifinet0).
* Creates the section if it does not yet exist.
* Applies the configuration and restarts the radio with `wifi`.
*
* Parameters:
*   uart    - UART instance for sending the response
*   params  - Pointer to the substring after "AT+WIFIAPCFG="
*             Expected format: "SSID,PASSWORD,SECURITY"
*
* Validation:
*   SSID     : 1–32 printable ASCII characters
*   PASSWORD : 8–63 printable ASCII characters (WPA/WPA2 only)
*   SECURITY : OPEN | WPA | WPA2 | WPA_WPA2
*
* Response:
*   OK on success; ERROR:1 on invalid params; ERROR:2 on UCI failure.
****************************************************************/
static void cmd_wifiapcfg_set(uart_inst_t *uart, const char *params) {
    char ssid[33] = "";
    char password[64] = "";
    char security[16] = "";
    char tmp[8] = "";

    int parsed = sscanf(params, "%32[^,],%63[^,],%15s",
                        ssid, password, security);

    if (parsed != 3) {
        send_response(uart, "ERROR:1");  // INVALID_PARAM
        return;
    }

    /* Trim trailing CR/LF from UART line */
    ssid[strcspn(ssid, "\r\n")] = '\0';
    password[strcspn(password, "\r\n")] = '\0';
    security[strcspn(security, "\r\n")] = '\0';

    if (strlen(ssid) < 1 || strlen(ssid) > 32 || !validate_wifi_string(ssid)) {
        send_response(uart, ERR_INVALID_PARAM);
        return;
    }

    const char *encryption = NULL;
    if (strcmp(security, "OPEN") == 0) {
        encryption = "none";
    } else if (strcmp(security, "WPA") == 0) {
        encryption = "psk";
        if (strlen(password) < 8 || strlen(password) > 63 || !validate_wifi_string(password)) {
            send_response(uart, ERR_INVALID_PARAM);
            return;
        }
    } else if (strcmp(security, "WPA2") == 0) {
        encryption = "psk2";
        if (strlen(password) < 8 || strlen(password) > 63 || !validate_wifi_string(password)) {
            send_response(uart, ERR_INVALID_PARAM);
            return;
        }
    } else if (strcmp(security, "WPA_WPA2") == 0) {
        encryption = "psk-mixed";
        if (strlen(password) < 8 || strlen(password) > 63 || !validate_wifi_string(password)) {
            send_response(uart, ERR_INVALID_PARAM);
            return;
        }
    } else {
        send_response(uart, ERR_INVALID_PARAM);
        return;
    }

    /* Ensure named AP section wifinet0 exists (LUCI format) */
    if (uci_get_string("wireless", AP_IFACE_NAME, "device", tmp, sizeof(tmp)) < 0) {
        system("uci set wireless." AP_IFACE_NAME "=wifi-iface");
        system("uci set wireless." AP_IFACE_NAME ".device='radio0'");
        system("uci set wireless." AP_IFACE_NAME ".network='lan'");
        system("uci set wireless." AP_IFACE_NAME ".mode='ap'");
        system("uci set wireless." AP_IFACE_NAME ".disabled='0'");
        system("uci commit wireless");
    }

    if (uci_set_string("wireless", AP_IFACE_NAME, "mode", "ap") < 0) {
        send_response(uart, ERR_CMD_FAILED);
        return;
    }
    if (uci_set_string("wireless", AP_IFACE_NAME, "network", "lan") < 0) {
        send_response(uart, ERR_CMD_FAILED);
        return;
    }
    if (uci_set_string("wireless", AP_IFACE_NAME, "ssid", ssid) < 0) {
        send_response(uart, ERR_CMD_FAILED);
        return;
    }
    if (uci_set_string("wireless", AP_IFACE_NAME, "encryption", encryption) < 0) {
        send_response(uart, ERR_CMD_FAILED);
        return;
    }
    if (strcmp(security, "OPEN") != 0) {
        if (uci_set_string("wireless", AP_IFACE_NAME, "key", password) < 0) {
            send_response(uart, ERR_CMD_FAILED);
            return;
        }
    }
    uci_set_string("wireless", AP_IFACE_NAME, "disabled", "0");

    system("uci commit wireless");
    system("wifi >/dev/null 2>&1");

    send_response(uart, "OK");
}

/****************************************************************
* cmd_wifiapcfg_query
*
* Handles: AT+WIFIAPCFG?
*
* Reads the current AP configuration from UCI (prefers wifinet0,
* falls back to @wifi-iface[0]) and counts connected stations
* via `iw dev phy0-ap0 station dump`.
*
* Parameters:
*   uart  - UART instance for sending the response
*
* Response:
*   +WIFIAPCFG:SSID=<ssid>,SEC=<OPEN|WPA|WPA2|WPA_WPA2>,CLIENTS=<n>
*   OK
****************************************************************/
static void cmd_wifiapcfg_query(uart_inst_t *uart) {
    char ssid[33] = "";
    char encryption[16] = "";
    int clients = 0;
    const char *ap_sec = AP_IFACE_NAME;

    if (uci_get_string("wireless", ap_sec, "ssid", ssid, sizeof(ssid)) < 0) {
        ap_sec = "@wifi-iface[0]";
        if (uci_get_string("wireless", ap_sec, "ssid", ssid, sizeof(ssid)) < 0) {
            strcpy(ssid, "OpenWrt");
        }
    }
    if (uci_get_string("wireless", ap_sec, "encryption", encryption, sizeof(encryption)) < 0) {
        strcpy(encryption, "none");
    }

    const char *sec_str = "OPEN";
    if (strcmp(encryption, "psk") == 0) sec_str = "WPA";
    else if (strcmp(encryption, "psk2") == 0) sec_str = "WPA2";
    else if (strcmp(encryption, "psk-mixed") == 0) sec_str = "WPA_WPA2";

    FILE *fp = popen("iw dev phy0-ap0 station dump | grep -c 'Station' 2>/dev/null || echo 0", "r");
    char buf[16];
    if (fp) {
        if (fgets(buf, sizeof(buf), fp)) clients = atoi(buf);
        drain_popen(fp);
        pclose(fp);
    }

    send_response(uart, "+WIFIAPCFG:SSID=%s,SEC=%s,CLIENTS=%d",
                  ssid, sec_str, clients);
    send_response(uart, "OK");
}

/****************************************************************
* cmd_wifimode_set
*
* Handles: AT+WIFIMODE=<0|1|2|3>
*
* Switches the WiFi radio operating mode:
*   0 = OFF    — disables both wifinet0 and wifinet1
*   1 = STA    — disables AP (wifinet0), enables STA (wifinet1),
*                creates network.wwan if needed, restarts network
*   2 = AP     — enables AP (wifinet0), disables STA (wifinet1)
*   3 = AP+STA — enables both wifinet0 and wifinet1
*
* UCI section wifinet0 / wifinet1 are created if they do not exist.
* For STA modes, existing SSID/password/encryption values in the STA
* section are preserved; only device/mode/network/disabled are set.
*
* Parameters:
*   uart   - UART instance for sending the response
*   param  - Pointer to the mode digit string after "AT+WIFIMODE="
*
* Response:
*   OK on success; ERROR:1 on invalid mode; ERROR:2 on shell failure.
****************************************************************/
static void cmd_wifimode_set(uart_inst_t *uart, const char *param) {
    int mode = -1;
    char command[512] = {0};
    char buffer[256] = {0};
    char tmp[8] = {0};
    FILE *fp = NULL;
    int ret = 0;

    if (sscanf(param, "%d", &mode) != 1 || mode < 0 || mode > 3) {
        send_response(uart, ERR_INVALID_PARAM);
        return;
    }

    /* Ensure wifinet0 exists when switching to AP (mode 2 or 3); remove default_radio0 so wifinet0 gets phy0-ap0 */
    if (mode == 2 || mode == 3) {
        system("uci delete wireless.default_radio0 2>/dev/null");
        if (uci_get_string("wireless", AP_IFACE_NAME, "device", tmp, sizeof(tmp)) < 0) {
            system("uci set wireless." AP_IFACE_NAME "=wifi-iface");
            system("uci set wireless." AP_IFACE_NAME ".device='radio0'");
            system("uci set wireless." AP_IFACE_NAME ".network='lan'");
            system("uci set wireless." AP_IFACE_NAME ".mode='ap'");
            system("uci set wireless." AP_IFACE_NAME ".disabled='0'");
            system("uci commit wireless");
        }
    }

    fp = popen("uci get wireless." AP_IFACE_NAME ".disabled 2>/dev/null", "r");
    if (fp) {
        if (fgets(buffer, sizeof(buffer), fp)) buffer[strcspn(buffer, "\n")] = '\0';
        drain_popen(fp);
        pclose(fp);
    }

    fp = popen("uci get wireless." AP_IFACE_NAME ".mode 2>/dev/null", "r");
    if (fp) {
        if (fgets(buffer, sizeof(buffer), fp)) buffer[strcspn(buffer, "\n")] = 0;
        drain_popen(fp);
        pclose(fp);
    }

    if (mode == 2) { /* AP only */
        system("uci delete wireless.radio0.disabled 2>/dev/null");
        snprintf(command, sizeof(command),
            "uci set wireless." AP_IFACE_NAME ".mode='ap' 2>&1; "
            "uci set wireless." AP_IFACE_NAME ".disabled='0' 2>&1; "
            "uci set wireless." STA_IFACE_NAME ".disabled='1' 2>/dev/null; "
            "uci commit wireless 2>&1; "
            "wifi >/dev/null 2>&1");
    } else if (mode == 1) { /* STA only: radio0 on, network.wwan, wifinet1, wifi, network restart */
        system("uci delete wireless.radio0.disabled 2>/dev/null");
        system("uci set network.wwan=interface 2>/dev/null");
        system("uci set network.wwan.proto='dhcp' 2>/dev/null");
        system("uci commit network 2>/dev/null");
        if (uci_get_string("wireless", STA_IFACE_NAME, "device", tmp, sizeof(tmp)) < 0) {
            /* Newly create wifinet1; copy STA credentials only from a section that has mode=sta (never from AP) */
            system("uci set wireless." STA_IFACE_NAME "=wifi-iface");
            if (uci_get_string("wireless", "@wifi-iface[1]", "mode", tmp, sizeof(tmp)) == 0 && strcmp(tmp, "sta") == 0) {
                if (uci_get_string("wireless", "@wifi-iface[1]", "ssid", buffer, sizeof(buffer)) == 0 && buffer[0])
                    uci_set_string("wireless", STA_IFACE_NAME, "ssid", buffer);
                if (uci_get_string("wireless", "@wifi-iface[1]", "encryption", buffer, sizeof(buffer)) == 0 && buffer[0])
                    uci_set_string("wireless", STA_IFACE_NAME, "encryption", buffer);
                if (uci_get_string("wireless", "@wifi-iface[1]", "key", buffer, sizeof(buffer)) == 0 && buffer[0])
                    uci_set_string("wireless", STA_IFACE_NAME, "key", buffer);
            } else if (uci_get_string("wireless", "@wifi-iface[0]", "mode", tmp, sizeof(tmp)) == 0 && strcmp(tmp, "sta") == 0) {
                if (uci_get_string("wireless", "@wifi-iface[0]", "ssid", buffer, sizeof(buffer)) == 0 && buffer[0])
                    uci_set_string("wireless", STA_IFACE_NAME, "ssid", buffer);
                if (uci_get_string("wireless", "@wifi-iface[0]", "encryption", buffer, sizeof(buffer)) == 0 && buffer[0])
                    uci_set_string("wireless", STA_IFACE_NAME, "encryption", buffer);
                if (uci_get_string("wireless", "@wifi-iface[0]", "key", buffer, sizeof(buffer)) == 0 && buffer[0])
                    uci_set_string("wireless", STA_IFACE_NAME, "key", buffer);
            }
            system("uci commit wireless");
        }
        /* Only set device/mode/network/disabled; never overwrite ssid/encryption/key (keep WIFISTACFG values) */
        system("uci set wireless." STA_IFACE_NAME ".device='radio0'");
        system("uci set wireless." STA_IFACE_NAME ".mode='sta'");
        system("uci set wireless." STA_IFACE_NAME ".network='wwan'");
        system("uci set wireless." AP_IFACE_NAME ".disabled='1'");
        system("uci set wireless." STA_IFACE_NAME ".disabled='0'");
        system("uci commit wireless");
        system("wifi >/dev/null 2>&1");
        system("/etc/init.d/network restart >/dev/null 2>&1");
        ret = 0;
        goto send_wifimode_result;
    } else if (mode == 3) { /* AP + STA: ensure both wifinet0 and wifinet1 exist, enable both */
        system("uci delete wireless.radio0.disabled 2>/dev/null");
        system("uci set network.wwan=interface 2>/dev/null");
        system("uci set network.wwan.proto='dhcp' 2>/dev/null");
        system("uci commit network 2>/dev/null");
        if (uci_get_string("wireless", STA_IFACE_NAME, "device", tmp, sizeof(tmp)) < 0) {
            system("uci set wireless." STA_IFACE_NAME "=wifi-iface");
            if (uci_get_string("wireless", "@wifi-iface[1]", "mode", tmp, sizeof(tmp)) == 0 && strcmp(tmp, "sta") == 0) {
                if (uci_get_string("wireless", "@wifi-iface[1]", "ssid", buffer, sizeof(buffer)) == 0 && buffer[0])
                    uci_set_string("wireless", STA_IFACE_NAME, "ssid", buffer);
                if (uci_get_string("wireless", "@wifi-iface[1]", "encryption", buffer, sizeof(buffer)) == 0 && buffer[0])
                    uci_set_string("wireless", STA_IFACE_NAME, "encryption", buffer);
                if (uci_get_string("wireless", "@wifi-iface[1]", "key", buffer, sizeof(buffer)) == 0 && buffer[0])
                    uci_set_string("wireless", STA_IFACE_NAME, "key", buffer);
            } else if (uci_get_string("wireless", "@wifi-iface[0]", "mode", tmp, sizeof(tmp)) == 0 && strcmp(tmp, "sta") == 0) {
                if (uci_get_string("wireless", "@wifi-iface[0]", "ssid", buffer, sizeof(buffer)) == 0 && buffer[0])
                    uci_set_string("wireless", STA_IFACE_NAME, "ssid", buffer);
                if (uci_get_string("wireless", "@wifi-iface[0]", "encryption", buffer, sizeof(buffer)) == 0 && buffer[0])
                    uci_set_string("wireless", STA_IFACE_NAME, "encryption", buffer);
                if (uci_get_string("wireless", "@wifi-iface[0]", "key", buffer, sizeof(buffer)) == 0 && buffer[0])
                    uci_set_string("wireless", STA_IFACE_NAME, "key", buffer);
            }
            system("uci commit wireless");
        }
        snprintf(command, sizeof(command),
            "uci set wireless." AP_IFACE_NAME ".mode='ap'; "
            "uci set wireless." AP_IFACE_NAME ".disabled='0'; "
            "uci set wireless." STA_IFACE_NAME ".disabled='0'; "
            "uci commit wireless; "
            "wifi >/dev/null 2>&1");
    } else if (mode == 0) { /* OFF */
        snprintf(command, sizeof(command),
            "uci set wireless." AP_IFACE_NAME ".disabled='1' 2>&1; "
            "uci set wireless." STA_IFACE_NAME ".disabled='1' 2>/dev/null; "
            "uci commit wireless 2>&1; "
            "wifi >/dev/null 2>&1");
    }

    fp = popen(command, "r");
    if (!fp) {
        send_response(uart, ERR_CMD_FAILED);
        return;
    }

    while (fgets(buffer, sizeof(buffer), fp)) {
        buffer[strcspn(buffer, "\n")] = 0;
    }

    ret = pclose(fp);
    fp = popen("uci get wireless." AP_IFACE_NAME ".disabled 2>/dev/null", "r");
    if (fp) {
        if (fgets(buffer, sizeof(buffer), fp)) buffer[strcspn(buffer, "\n")] = 0;
        drain_popen(fp);
        pclose(fp);
    }

    fp = popen("uci get wireless." AP_IFACE_NAME ".mode 2>/dev/null", "r");
    if (fp) {
        if (fgets(buffer, sizeof(buffer), fp)) buffer[strcspn(buffer, "\n")] = 0;
        drain_popen(fp);
        pclose(fp);
    }

    fp = popen("ps | grep -E 'hostapd|wpa_supplicant' | grep -v grep", "r");
    if (fp) {
        while (fgets(buffer, sizeof(buffer), fp)) { buffer[strcspn(buffer, "\n")] = 0; }
        pclose(fp);
    }

send_wifimode_result:
    if (ret == 0) {
        send_response(uart, "OK");
    } else {
        send_response(uart, ERR_CMD_FAILED);
    }
}

/****************************************************************
* cmd_wifimode_query
*
* Handles: AT+WIFIMODE?
*
* Reads the disabled/mode flags for wifinet0 (AP) and wifinet1
* (STA) from UCI and maps them to a single mode number:
*   0 = both disabled, 1 = STA only, 2 = AP only, 3 = AP+STA
*
* Falls back to @wifi-iface[0] / @wifi-iface[1] if the named
* sections are not found.  If the "disabled" option is absent the
* interface is treated as enabled (OpenWrt convention).
*
* Parameters:
*   uart  - UART instance for sending the response
*
* Response:
*   +WIFIMODE:<0|1|2|3>
*   OK
****************************************************************/
static void cmd_wifimode_query(uart_inst_t *uart) {
    char mode0[8] = "";
    char disabled0[8] = "";
    char mode1[8] = "";
    char disabled1[8] = "";
    int current_mode = 0;
    int ap_has_disabled = 0;
    int sta_has_disabled = 0;

    /* Read AP (wifinet0, else @wifi-iface[0]) state */
    if (uci_get_string("wireless", AP_IFACE_NAME, "disabled", disabled0, sizeof(disabled0)) == 0) {
        ap_has_disabled = 1;
    } else if (uci_get_string("wireless", "@wifi-iface[0]", "disabled", disabled0, sizeof(disabled0)) == 0) {
        ap_has_disabled = 1;
    }
    if (uci_get_string("wireless", AP_IFACE_NAME, "mode", mode0, sizeof(mode0)) != 0) {
        uci_get_string("wireless", "@wifi-iface[0]", "mode", mode0, sizeof(mode0));
    }
    mode0[sizeof(mode0) - 1] = '\0';
    if (!ap_has_disabled) {
        /* No "disabled" option = enabled in OpenWrt */
        strcpy(disabled0, (strcmp(mode0, "ap") == 0) ? "0" : "1");
    }

    /* Read STA (wifinet1, else @wifi-iface[1]) state */
    if (uci_get_string("wireless", STA_IFACE_NAME, "disabled", disabled1, sizeof(disabled1)) == 0) {
        sta_has_disabled = 1;
    } else if (uci_get_string("wireless", "@wifi-iface[1]", "disabled", disabled1, sizeof(disabled1)) == 0) {
        sta_has_disabled = 1;
    }
    if (uci_get_string("wireless", STA_IFACE_NAME, "mode", mode1, sizeof(mode1)) != 0) {
        uci_get_string("wireless", "@wifi-iface[1]", "mode", mode1, sizeof(mode1));
    }
    mode1[sizeof(mode1) - 1] = '\0';
    if (!sta_has_disabled) {
        /* No "disabled" option = enabled; if STA section exists with mode=sta, treat as enabled */
        strcpy(disabled1, (strcmp(mode1, "sta") == 0) ? "0" : "1");
    }

    if (strcmp(disabled0, "1") == 0 &&
        strcmp(disabled1, "1") == 0) {
        /* Both interfaces disabled */
        current_mode = 0;
    } else if (strcmp(disabled0, "1") == 0 &&
               strcmp(disabled1, "0") == 0 &&
               strcmp(mode1, "sta") == 0) {
        /* STA only */
        current_mode = 1;
    } else if (strcmp(disabled0, "0") == 0 &&
               strcmp(mode0, "ap") == 0 &&
               strcmp(disabled1, "1") == 0) {
        /* AP only */
        current_mode = 2;
    } else if (strcmp(disabled0, "0") == 0 &&
               strcmp(mode0, "ap") == 0 &&
               strcmp(disabled1, "0") == 0 &&
               strcmp(mode1, "sta") == 0) {
        /* AP + STA */
        current_mode = 3;
    } else {
        /* Fallback: prefer STA if it looks active, else AP, else 0 */
        if (strcmp(mode1, "sta") == 0 && strcmp(disabled1, "0") == 0) {
            current_mode = (strcmp(disabled0, "0") == 0 && strcmp(mode0, "ap") == 0) ? 3 : 1;
        } else if (strcmp(mode0, "ap") == 0 && strcmp(disabled0, "0") == 0) {
            current_mode = 2;
        } else {
            current_mode = 0;
        }
    }

    send_response(uart, "+WIFIMODE:%d", current_mode);
    send_response(uart, "OK");
}

/****************************************************************
* cmd_wifiap_clients
*
* Handles: AT+WIFIAP?
*
* Enumerates all stations currently associated with the AP
* interface (phy0-ap0) using `iw dev phy0-ap0 station dump`.
* For each MAC, looks up the DHCP-assigned IP in /tmp/dhcp.leases.
* Reports "0.0.0.0" if no lease is found.
*
* Parameters:
*   uart  - UART instance for sending the response
*
* Response:
*   +WIFIAP:CLIENT,<MAC>,<IP>   (one line per connected client)
*   OK
*   (Just OK if no clients are connected or iw fails)
****************************************************************/
static void cmd_wifiap_clients(uart_inst_t *uart) {
    FILE *fp = NULL;
    FILE *leases = NULL;
    char line[256] = {0};
    char mac[18] = {0};
    char ip[16] = "0.0.0.0";
    char lease_line[256] = {0};
    int client_count = 0;

    fp = popen("iw dev phy0-ap0 station dump | grep Station | awk '{print $2}'", "r");
    if (!fp) {
        send_response(uart, "OK");
        return;
    }

    while (fgets(line, sizeof(line), fp)) {
        line[strcspn(line, "\n")] = 0;

        if (strlen(line) > 0) {
            strncpy(mac, line, sizeof(mac) - 1);
            mac[sizeof(mac) - 1] = '\0';
            strcpy(ip, "0.0.0.0");
            leases = fopen("/tmp/dhcp.leases", "r");
            if (leases) {
                while (fgets(lease_line, sizeof(lease_line), leases)) {
                    if (lease_line_get_ip(lease_line, mac, ip, sizeof(ip)))
                        break;
                }
                fclose(leases);
                leases = NULL;
            }

            send_response(uart, "+WIFIAP:CLIENT,%s,%s", mac, ip);
            client_count++;
        }
    }
    pclose(fp);

    send_response(uart, "OK");
}

/****************************************************************
* cmd_wifista_cfg
*
* Handles: AT+WIFISTACFG=<SSID>,<PASSWORD>,<SECURITY>
*
* Stores STA credentials in UCI section wifinet1 (network wwan).
* Creates wifinet1 and network.wwan if they do not exist.
* Does NOT trigger a wifi restart — call AT+WIFISTA=1 or
* AT+WIFIMODE=1/3 to apply.
*
* Parameters:
*   uart    - UART instance for sending the response
*   params  - Pointer to the substring after "AT+WIFISTACFG="
*             Expected format: "SSID,PASSWORD,SECURITY"
*
* Validation:
*   SSID     : 1–32 printable ASCII characters
*   PASSWORD : 8–63 printable ASCII characters (WPA/WPA2 only)
*   SECURITY : OPEN | WPA | WPA2 | WPA_WPA2
*
* Response:
*   OK on success; ERROR:1 on invalid params; ERROR:2 on UCI failure.
****************************************************************/
static void cmd_wifista_cfg(uart_inst_t *uart, const char *params) {
    char ssid[33] = "";
    char password[64] = "";
    char security[16] = "";
    const char *sta_section = STA_IFACE_NAME;
    char tmp[8] = "";

    int parsed = sscanf(params, "%32[^,],%63[^,],%15s",
                        ssid, password, security);
    if (parsed != 3) {
        send_response(uart, "ERROR:1");
        return;
    }

    if (strlen(ssid) < 1 || strlen(ssid) > 32 || !validate_wifi_string(ssid)) {
        send_response(uart, ERR_INVALID_PARAM);
        return;
    }

    const char *encryption = NULL;
    if (strcmp(security, "OPEN") == 0) {
        encryption = "none";
    } else if (strcmp(security, "WPA") == 0) {
        encryption = "psk";
        if (strlen(password) < 8 || strlen(password) > 63 || !validate_wifi_string(password)) {
            send_response(uart, ERR_INVALID_PARAM);
            return;
        }
    } else if (strcmp(security, "WPA2") == 0) {
        encryption = "psk2";
        if (strlen(password) < 8 || strlen(password) > 63 || !validate_wifi_string(password)) {
            send_response(uart, ERR_INVALID_PARAM);
            return;
        }
    } else if (strcmp(security, "WPA_WPA2") == 0) {
        encryption = "psk-mixed";
        if (strlen(password) < 8 || strlen(password) > 63 || !validate_wifi_string(password)) {
            send_response(uart, ERR_INVALID_PARAM);
            return;
        }
    } else {
        send_response(uart, ERR_INVALID_PARAM);
        return;
    }

    /* Ensure wifinet1 (and network.wwan) exist so we always write STA to wifinet1 */
    if (uci_get_string("wireless", STA_IFACE_NAME, "device", tmp, sizeof(tmp)) < 0) {
        system("uci set network.wwan=interface 2>/dev/null");
        system("uci set network.wwan.proto='dhcp' 2>/dev/null");
        system("uci commit network 2>/dev/null");
        system("uci set wireless." STA_IFACE_NAME "=wifi-iface");
        system("uci set wireless." STA_IFACE_NAME ".device='radio0'");
        system("uci set wireless." STA_IFACE_NAME ".mode='sta'");
        system("uci set wireless." STA_IFACE_NAME ".network='wwan'");
        system("uci commit wireless");
    }
    sta_section = STA_IFACE_NAME;

    if (uci_set_string("wireless", sta_section, "mode", "sta") < 0) {
        send_response(uart, ERR_CMD_FAILED);
        return;
    }
    if (uci_set_string("wireless", sta_section, "ssid", ssid) < 0) {
        send_response(uart, ERR_CMD_FAILED);
        return;
    }
    if (uci_set_string("wireless", sta_section, "encryption", encryption) < 0) {
        send_response(uart, ERR_CMD_FAILED);
        return;
    }
    if (strcmp(security, "OPEN") != 0) {
        if (uci_set_string("wireless", sta_section, "key", password) < 0) {
            send_response(uart, ERR_CMD_FAILED);
            return;
        }
    }

    system("uci commit wireless");
    send_response(uart, "OK");
}

/****************************************************************
* cmd_wifistacfg_query
*
* Handles: AT+WIFISTACFG?
*
* Reads the stored STA credentials from UCI.  Prefers wifinet1;
* falls back to @wifi-iface[1] then @wifi-iface[0].
*
* Parameters:
*   uart  - UART instance for sending the response
*
* Response:
*   +WIFISTACFG:SSID=<ssid>,SEC=<OPEN|WPA|WPA2|WPA_WPA2>,PASSWORD=<password>
*   OK
*   (PASSWORD is empty for OPEN networks)
****************************************************************/
static void cmd_wifistacfg_query(uart_inst_t *uart) {
    char ssid[33] = "";
    char encryption[16] = "";
    char password[64] = "";
    const char *sta_section = STA_IFACE_NAME;
    char tmp[8] = "";

    /* Prefer wifinet1, else @wifi-iface[1], else @wifi-iface[0] */
    if (uci_get_string("wireless", sta_section, "device", tmp, sizeof(tmp)) < 0) {
        sta_section = "@wifi-iface[1]";
        if (uci_get_string("wireless", sta_section, "mode", tmp, sizeof(tmp)) < 0) {
            sta_section = "@wifi-iface[0]";
        }
    }

    if (uci_get_string("wireless", sta_section, "ssid", ssid, sizeof(ssid)) < 0) {
        strcpy(ssid, "");
    }
    if (uci_get_string("wireless", sta_section, "encryption", encryption, sizeof(encryption)) < 0) {
        strcpy(encryption, "none");
    }
    if (uci_get_string("wireless", sta_section, "key", password, sizeof(password)) < 0) {
        strcpy(password, "");
    }

    const char *sec_str = "OPEN";
    if (strcmp(encryption, "psk") == 0) sec_str = "WPA";
    else if (strcmp(encryption, "psk2") == 0) sec_str = "WPA2";
    else if (strcmp(encryption, "psk-mixed") == 0) sec_str = "WPA_WPA2";

    send_response(uart, "+WIFISTACFG:SSID=%s,SEC=%s,PASSWORD=%s", ssid, sec_str, password);
    send_response(uart, "OK");
}

/****************************************************************
* cmd_wifista_set
*
* Handles: AT+WIFISTA=<1|0>
*
* Enables (1) or disables (0) the STA interface by setting the
* UCI "disabled" flag on wifinet1 (or @wifi-iface[1] as fallback),
* committing the change, and running `wifi` to apply it.
*
* Parameters:
*   uart   - UART instance for sending the response
*   param  - Pointer to the digit string after "AT+WIFISTA="
*            Must be exactly "0" or "1".
*
* Response:
*   OK on success; ERROR:1 if param is not 0 or 1; ERROR:2 on failure.
****************************************************************/
static void cmd_wifista_set(uart_inst_t *uart, const char *param) {
    int enable = -1;
    char cmd[320];
    char tmp[8] = "";
    const char *sta = STA_IFACE_NAME;

    if (sscanf(param, "%d", &enable) != 1 || (enable != 0 && enable != 1)) {
        send_response(uart, ERR_INVALID_PARAM);
        return;
    }

    if (uci_get_string("wireless", sta, "device", tmp, sizeof(tmp)) < 0) {
        sta = "@wifi-iface[1]";
    }
    if (enable == 1) {
        snprintf(cmd, sizeof(cmd),
                 "uci set wireless.%s.disabled='0'; "
                 "uci commit wireless; wifi >/dev/null 2>&1", sta);
    } else { /* enable == 0 */
        snprintf(cmd, sizeof(cmd),
                 "uci set wireless.%s.disabled='1'; "
                 "uci commit wireless; wifi >/dev/null 2>&1", sta);
    }

    int ret = system(cmd);
    if (ret == 0) {
        send_response(uart, "OK");
    } else {
        send_response(uart, ERR_CMD_FAILED);
    }
}

/****************************************************************
* get_sta_connected_iface
*
* Discovers the active STA wireless interface by iterating all
* interfaces reported by `iw dev` and checking each one (except
* the AP interface phy0-ap0) with `iw dev <iface> link`.
*
* The first interface that reports "Connected" (and not
* "Not connected") is returned.
*
* Parameters:
*   sta_iface_out  - Buffer to receive the interface name (e.g. "wlan0")
*   len            - Size of sta_iface_out; must be >= 2
*
* Returns:
*   1 if a connected STA interface was found (sta_iface_out filled),
*   0 if no connected STA interface exists or len < 2.
****************************************************************/
static int get_sta_connected_iface(char *sta_iface_out, size_t len) {
    FILE *fp = NULL;
    FILE *link = NULL;
    char line[256] = {0};
    char iface[32] = {0};
    int connected = 0;

    if (!sta_iface_out || len < 2) return 0;
    sta_iface_out[0] = '\0';

    fp = popen("iw dev 2>/dev/null | awk '/Interface/ {print $2}'", "r");
    if (!fp) return 0;
    while (fgets(line, sizeof(line), fp)) {
        line[strcspn(line, "\n")] = '\0';
        if (sscanf(line, "%31s", iface) != 1) continue;
        if (strcmp(iface, "phy0-ap0") == 0) continue; /* skip AP interface */
        snprintf(line, sizeof(line), "iw dev %s link 2>/dev/null", iface);
        link = popen(line, "r");
        if (!link) continue;
        connected = 0;
        while (fgets(line, sizeof(line), link)) {
            if (strstr(line, "Not connected")) { connected = 0; break; }
            if (strstr(line, "Connected")) connected = 1;
        }
        drain_popen(link);
        pclose(link);
        link = NULL;
        if (connected) {
            strncpy(sta_iface_out, iface, len - 1);
            sta_iface_out[len - 1] = '\0';
            drain_popen(fp);
            pclose(fp);
            return 1;
        }
    }
    drain_popen(fp);
    pclose(fp);
    return 0;
}

/****************************************************************
* cmd_wifista_query
*
* Handles: AT+WIFISTA?
*
* Reports the current STA connection state by:
*   1. Finding the active STA interface via get_sta_connected_iface().
*   2. Reading its IPv4 address via `ip -4 addr show`.
*   3. Reading signal strength (RSSI) via `iw dev <iface> link`.
*   4. Reading the encryption type from UCI.
*
* Parameters:
*   uart  - UART instance for sending the response
*
* Response (connected):
*   +WIFISTA:CONNECTED,IP=<ip>,RSSI=<dBm>,SEC=<OPEN|WPA|WPA2|WPA_WPA2>
*   OK
* Response (not connected):
*   +WIFISTA:DISCONNECTED,REASON=NOT_CONNECTED
*   OK
****************************************************************/
static void cmd_wifista_query(uart_inst_t *uart) {
    FILE *fp = NULL;
    char line[256] = {0};
    char sta_iface[32] = {0};
    char ip[32] = "0.0.0.0";
    char rssi[16] = "0";
    char encryption[16] = "";
    const char *sec_str = "OPEN";

    if (!get_sta_connected_iface(sta_iface, sizeof(sta_iface))) {
        send_response(uart, "+WIFISTA:DISCONNECTED,REASON=NOT_CONNECTED");
        send_response(uart, "OK");
        return;
    }

    snprintf(line, sizeof(line), "ip -4 addr show %s 2>/dev/null | awk '/inet /{print $2}' | cut -d/ -f1", sta_iface);
    fp = popen(line, "r");
    if (fp) {
        if (fgets(line, sizeof(line), fp)) {
            line[strcspn(line, "\n")] = 0;
            if (strlen(line) > 0) {
                strncpy(ip, line, sizeof(ip) - 1);
                ip[sizeof(ip) - 1] = '\0';
            }
        }
        drain_popen(fp);
        pclose(fp);
        fp = NULL;
    }

    strcpy(rssi, "0");
    snprintf(line, sizeof(line), "iw dev %s link 2>/dev/null | awk '/signal:/ {print $2}'", sta_iface);
    fp = popen(line, "r");
    if (fp) {
        if (fgets(line, sizeof(line), fp)) {
            line[strcspn(line, "\n")] = 0;
            if (strlen(line) > 0) {
                strncpy(rssi, line, sizeof(rssi) - 1);
                rssi[sizeof(rssi) - 1] = '\0';
            }
        }
        drain_popen(fp);
        pclose(fp);
        fp = NULL;
    }

    if (uci_get_string("wireless", STA_IFACE_NAME, "encryption", encryption, sizeof(encryption)) != 0) {
        uci_get_string("wireless", "@wifi-iface[1]", "encryption", encryption, sizeof(encryption));
    }
    if (encryption[0]) {
        if (strcmp(encryption, "psk") == 0) sec_str = "WPA";
        else if (strcmp(encryption, "psk2") == 0) sec_str = "WPA2";
        else if (strcmp(encryption, "psk-mixed") == 0) sec_str = "WPA_WPA2";
    }

    send_response(uart, "+WIFISTA:CONNECTED,IP=%s,RSSI=%s,SEC=%s",
                  ip, rssi, sec_str);
    send_response(uart, "OK");
}

/*============================================================================
 * WIFI ASYNCHRONOUS EVENTS (APJOIN, APLEAVE, STACONN, STADISCONN)
 *============================================================================*/
#define WIFI_EVENT_AP_CLIENTS_MAX  32

static struct {
    char ap_macs[WIFI_EVENT_AP_CLIENTS_MAX][18];
    char ap_ips[WIFI_EVENT_AP_CLIENTS_MAX][16];
    int  ap_count;
    int  sta_connected;
    char sta_ip[32];
    char sta_sec[16];
} wifi_event_prev;

/****************************************************************
* wifi_event_get_ap_clients
*
* Builds the current list of MAC addresses associated with the AP
* (phy0-ap0) and their DHCP-assigned IPs from /tmp/dhcp.leases.
*
* Parameters:
*   macs   - Output 2-D array; each entry is a 17-char MAC string
*            (e.g. "aa:bb:cc:dd:ee:ff") + null terminator
*   ips    - Output 2-D array; each entry is up to 15-char IPv4
*            string + null terminator; "0.0.0.0" if no lease found
*   count  - Output: number of entries filled in macs[] and ips[]
*
* Notes:
*   Silently returns count=0 if `iw dev phy0-ap0 station dump` fails.
*   Caller must size macs and ips to at least WIFI_EVENT_AP_CLIENTS_MAX.
****************************************************************/
static void wifi_event_get_ap_clients(char macs[][18], char ips[][16], int *count) {
    FILE *fp = NULL;
    FILE *leases = NULL;
    char line[256] = {0};
    char lease_line[256] = {0};

    *count = 0;

    fp = popen("iw dev phy0-ap0 station dump 2>/dev/null | grep Station | awk '{print $2}'", "r");
    if (!fp) return;

    while (*count < WIFI_EVENT_AP_CLIENTS_MAX && fgets(line, sizeof(line), fp)) {
        line[strcspn(line, "\n")] = '\0';
        if (strlen(line) < 17) continue;
        strncpy(macs[*count], line, 17);
        macs[*count][17] = '\0';
        strcpy(ips[*count], "0.0.0.0");
        leases = fopen("/tmp/dhcp.leases", "r");
        if (leases) {
            while (fgets(lease_line, sizeof(lease_line), leases)) {
                if (lease_line_get_ip(lease_line, macs[*count], ips[*count], 16))
                    break;
            }
            fclose(leases);
            leases = NULL;
        }
        (*count)++;
    }
    drain_popen(fp);
    pclose(fp);
}

/****************************************************************
* wifi_event_get_sta_state
*
* Determines whether the STA is currently connected and, if so,
* retrieves its IP address and security type.
*
* Parameters:
*   ip       - Output buffer for IPv4 address string (or empty "")
*   ip_len   - Size of ip buffer
*   sec      - Output buffer for security string (OPEN/WPA/WPA2/WPA_WPA2)
*   sec_len  - Size of sec buffer
*
* Returns:
*   1 if STA is connected (ip and sec filled),
*   0 if no connected STA interface is found.
****************************************************************/
static int wifi_event_get_sta_state(char *ip, size_t ip_len, char *sec, size_t sec_len) {
    FILE *fp = NULL;
    char line[256] = {0};
    char sta_iface[32] = {0};

    if (ip_len) ip[0] = '\0';
    if (sec_len) sec[0] = '\0';

    if (!get_sta_connected_iface(sta_iface, sizeof(sta_iface)))
        return 0;

    snprintf(line, sizeof(line), "ip -4 addr show %s 2>/dev/null | awk '/inet /{print $2}' | cut -d/ -f1", sta_iface);
    fp = popen(line, "r");
    if (fp) {
        if (fgets(line, sizeof(line), fp)) {
            line[strcspn(line, "\n")] = '\0';
            strncpy(ip, line, ip_len - 1);
            ip[ip_len - 1] = '\0';
        }
        drain_popen(fp);
        pclose(fp);
    }

    if (uci_get_string("wireless", STA_IFACE_NAME, "encryption", sec, sec_len) != 0) {
        uci_get_string("wireless", "@wifi-iface[1]", "encryption", sec, sec_len);
    }
    if (sec[0]) {
        if (strcmp(sec, "psk") == 0) strncpy(sec, "WPA", sec_len - 1);
        else if (strcmp(sec, "psk2") == 0) strncpy(sec, "WPA2", sec_len - 1);
        else if (strcmp(sec, "psk-mixed") == 0) strncpy(sec, "WPA_WPA2", sec_len - 1);
    } else {
        strncpy(sec, "OPEN", sec_len - 1);
    }
    sec[sec_len - 1] = '\0';
    return 1;
}

/****************************************************************
* wifi_event_mac_in_list
*
* Case-insensitive search for a MAC address string in a 2-D array.
*
* Parameters:
*   mac    - MAC string to search for (e.g. "AA:BB:CC:DD:EE:FF")
*   macs   - Array of MAC strings to search
*   count  - Number of valid entries in macs[]
*
* Returns:
*   1 if mac is found (case-insensitive), 0 otherwise.
****************************************************************/
static int wifi_event_mac_in_list(const char *mac, char macs[][18], int count) {
    int i = 0;
    for (i = 0; i < count; i++) {
        if (strcasecmp(mac, macs[i]) == 0) return 1;
    }
    return 0;
}

/****************************************************************
* ap_client_lookup_ip
*
* Searches /tmp/dhcp.leases for a lease matching the given MAC
* and copies its IP address to ip_out.
*
* Parameters:
*   mac     - MAC address string to look up (case-insensitive)
*   ip_out  - Output buffer to receive the IP string
*   ip_len  - Size of ip_out; must be >= 8
*
* Returns:
*   1 if a matching lease was found and IP was copied to ip_out,
*   0 if not found, file missing, or ip_len < 8.
*
* Notes:
*   Used by the non-blocking APJOIN pending retry logic to poll
*   for a DHCP lease on each wifi_events_poll() tick.
****************************************************************/
static int ap_client_lookup_ip(const char *mac, char *ip_out, size_t ip_len) {
    FILE *fp = NULL;
    char line[256] = {0};
    if (!ip_out || ip_len < 8) return 0;
    ip_out[0] = '\0';
    fp = fopen("/tmp/dhcp.leases", "r");
    if (!fp) return 0;
    while (fgets(line, sizeof(line), fp)) {
        if (lease_line_get_ip(line, mac, ip_out, ip_len)) {
            fclose(fp);
            return 1;
        }
    }
    fclose(fp);
    return 0;
}

/*
 * apjoin_pending: non-blocking DHCP retry table.
 * When a client joins but has no IP yet, we record its MAC here and retry
 * the lease lookup on subsequent poll ticks (every ~1 s from the select loop)
 * rather than blocking with sleep().  After retries_left reaches 0 the event
 * is fired with whatever IP (or "0.0.0.0") was found.
 */
#define APJOIN_PENDING_MAX  8
static struct {
    char mac[18];
    int  retries_left;  /* decremented each poll tick; fire when 0 */
} apjoin_pending[APJOIN_PENDING_MAX];
static int apjoin_pending_count = 0;

/****************************************************************
* wifi_events_poll
*
* Called once per main-loop tick.  Compares the current WiFi state
* against the previous snapshot (wifi_event_prev) and emits
* unsolicited events for any changes:
*
*   +WIFI:APJOIN,<MAC>,<IP>       — new client associated with AP
*   +WIFI:APLEAVE,<MAC>           — client left AP
*   +WIFI:STACONN,<IP>,<SEC>      — STA connected and has IP
*   +WIFI:STADISCONN,NOT_CONNECTED — STA lost connection
*
* APJOIN DHCP retry (non-blocking):
*   When a client joins but has no IP yet, its MAC is added to the
*   apjoin_pending[] table.  On each subsequent tick the lease file
*   is re-checked.  The event fires as soon as an IP appears or
*   after 3 retries (whichever comes first), keeping the event
*   loop unblocked.
*
* Parameters:
*   uart  - Open UART instance; events written directly via send_event()
*
* Notes:
*   Not re-entrant; designed for single-threaded use.
*   State is maintained in the static wifi_event_prev struct and
*   the apjoin_pending[] table.
****************************************************************/
void wifi_events_poll(uart_inst_t *uart) {
    char cur_macs[WIFI_EVENT_AP_CLIENTS_MAX][18];
    char cur_ips[WIFI_EVENT_AP_CLIENTS_MAX][16];
    int cur_ap = 0;
    int cur_sta = 0;
    int i = 0;
    int new_pending_count = 0;
    char cur_sta_ip[32] = "";
    char cur_sta_sec[16] = "OPEN";

    wifi_event_get_ap_clients(cur_macs, cur_ips, &cur_ap);
    cur_sta = wifi_event_get_sta_state(cur_sta_ip, sizeof(cur_sta_ip), cur_sta_sec, sizeof(cur_sta_sec));

    /* --- Process pending APJOIN retries (non-blocking DHCP wait) --- */
    new_pending_count = 0;
    for (i = 0; i < apjoin_pending_count; i++) {
        char ip[16] = "0.0.0.0";
        int got_ip = ap_client_lookup_ip(apjoin_pending[i].mac, ip, sizeof(ip));
        if (got_ip || apjoin_pending[i].retries_left <= 0) {
            send_event(uart, "+WIFI:APJOIN,%s,%s", apjoin_pending[i].mac,
                       (got_ip && ip[0]) ? ip : "0.0.0.0");
        } else {
            apjoin_pending[i].retries_left--;
            apjoin_pending[new_pending_count++] = apjoin_pending[i];
        }
    }
    apjoin_pending_count = new_pending_count;

    /* APJOIN: in current, not in previous */
    for (i = 0; i < cur_ap; i++) {
        if (!wifi_event_mac_in_list(cur_macs[i], wifi_event_prev.ap_macs, wifi_event_prev.ap_count)) {
            if (cur_ips[i][0] && strcmp(cur_ips[i], "0.0.0.0") != 0) {
                /* IP already available - fire immediately */
                send_event(uart, "+WIFI:APJOIN,%s,%s", cur_macs[i], cur_ips[i]);
            } else if (apjoin_pending_count < APJOIN_PENDING_MAX) {
                /* Defer: retry on subsequent poll ticks */
                strncpy(apjoin_pending[apjoin_pending_count].mac, cur_macs[i], 17);
                apjoin_pending[apjoin_pending_count].mac[17] = '\0';
                apjoin_pending[apjoin_pending_count].retries_left = 3;
                apjoin_pending_count++;
            } else {
                /* Pending table full - fire now with no IP */
                send_event(uart, "+WIFI:APJOIN,%s,0.0.0.0", cur_macs[i]);
            }
        }
    }
    /* APLEAVE: in previous, not in current */
    for (i = 0; i < wifi_event_prev.ap_count; i++) {
        if (!wifi_event_mac_in_list(wifi_event_prev.ap_macs[i], cur_macs, cur_ap)) {
            send_event(uart, "+WIFI:APLEAVE,%s", wifi_event_prev.ap_macs[i]);
        }
    }

    /* STACONN: just connected (got IP) */
    if (cur_sta && (!wifi_event_prev.sta_connected || strcmp(wifi_event_prev.sta_ip, cur_sta_ip) != 0)) {
        send_event(uart, "+WIFI:STACONN,%s,%s", cur_sta_ip, cur_sta_sec);
    }
    /* STADISCONN: just disconnected */
    if (wifi_event_prev.sta_connected && !cur_sta) {
        send_event(uart, "+WIFI:STADISCONN,NOT_CONNECTED");
    }

    /* update previous state */
    wifi_event_prev.ap_count = cur_ap;
    for (i = 0; i < cur_ap; i++) {
        strncpy(wifi_event_prev.ap_macs[i], cur_macs[i], 17);
        wifi_event_prev.ap_macs[i][17] = '\0';
        strncpy(wifi_event_prev.ap_ips[i], cur_ips[i], 15);
        wifi_event_prev.ap_ips[i][15] = '\0';
    }
    wifi_event_prev.sta_connected = cur_sta;
    strncpy(wifi_event_prev.sta_ip, cur_sta_ip, sizeof(wifi_event_prev.sta_ip) - 1);
    wifi_event_prev.sta_ip[sizeof(wifi_event_prev.sta_ip) - 1] = '\0';
    strncpy(wifi_event_prev.sta_sec, cur_sta_sec, sizeof(wifi_event_prev.sta_sec) - 1);
    wifi_event_prev.sta_sec[sizeof(wifi_event_prev.sta_sec) - 1] = '\0';
}

/*============================================================================
 * ETHERNET ASYNC EVENTS (spec 302-330): +ETH:UP, +ETH:DOWN, +ETH:CLIENT, +ETH:CLIENT_LEAVE
 * MT7628 single LAN port = PORT 1
 *============================================================================*/
#define ETH_PORT                1
#define ETH_EVENT_CLIENTS_MAX   32

static struct {
    int  carrier;
    char client_macs[ETH_EVENT_CLIENTS_MAX][18];
    char client_ips[ETH_EVENT_CLIENTS_MAX][16];
    int  client_count;
} eth_event_prev;

/****************************************************************
* eth_get_reachable_macs
*
* Fills reachable_macs with the MAC addresses currently present in
* the Linux ARP neighbour table with state REACHABLE or STALE on
* br-lan and eth0.  Used to cross-reference /tmp/dhcp.leases so
* only actively-reachable clients are reported.
*
* Parameters:
*   reachable_macs  - Output 2-D array; caller must size to
*                     ETH_EVENT_CLIENTS_MAX entries of 18 bytes
*   count           - Output: number of entries filled
****************************************************************/
static void eth_get_reachable_macs(char reachable_macs[][18], int *count) {
    FILE *fp = NULL;
    char line[256] = {0};
    const char *devs[] = { "br-lan", "eth0", NULL };
    int d = 0;

    *count = 0;
    for (d = 0; devs[d] && *count < ETH_EVENT_CLIENTS_MAX; d++) {
        snprintf(line, sizeof(line), "ip neigh show dev %s 2>/dev/null | awk '$NF==\"REACHABLE\" || $NF==\"STALE\" {print $(NF-1)}'", devs[d]);
        fp = popen(line, "r");
        if (!fp) continue;
        while (*count < ETH_EVENT_CLIENTS_MAX && fgets(line, sizeof(line), fp)) {
            line[strcspn(line, "\n")] = '\0';
            if (strlen(line) >= 17) {
                strncpy(reachable_macs[*count], line, 17);
                reachable_macs[*count][17] = '\0';
                (*count)++;
            }
        }
        drain_popen(fp);
        pclose(fp);
        fp = NULL;
    }
}

/****************************************************************
* eth_get_lan_ip
*
* Reads the first IPv4 address assigned to the LAN interface.
* Tries br-lan first (the default OpenWrt bridge), then falls back
* to eth0.  Writes an empty string to ip if no address is found.
*
* Parameters:
*   ip      - Output buffer for the IPv4 address string
*   ip_len  - Size of ip buffer
****************************************************************/
static void eth_get_lan_ip(char *ip, size_t ip_len) {
    FILE *fp=NULL;
    char line[256]={0};
    const char *ifaces[] = { "br-lan", "eth0", NULL };
    int i=0;

    if (ip_len) ip[0] = '\0';
    for (i = 0; ifaces[i]; i++) {
        snprintf(line, sizeof(line), "ip -4 addr show %s 2>/dev/null | awk '/inet /{print $2}' | cut -d/ -f1", ifaces[i]);
        fp = popen(line, "r");
        if (fp) {
            if (fgets(line, sizeof(line), fp) && strlen(line) > 0) {
                line[strcspn(line, "\n")] = '\0';
                if (strlen(line) > 0) {
                    strncpy(ip, line, ip_len - 1);
                    ip[ip_len - 1] = '\0';
                    drain_popen(fp);
                    pclose(fp);
                    return;
                }
            }
            drain_popen(fp);
            pclose(fp);
        }
    }
}

/****************************************************************
* eth_event_get_state
*
* Reads the complete current Ethernet state in one call:
*   - carrier: whether /sys/class/net/eth0/carrier reads 1
*   - ip:      LAN IPv4 address (br-lan or eth0)
*   - macs/ips: list of DHCP clients that also appear as
*               REACHABLE/STALE in the ARP neighbour table
*
* Parameters:
*   carrier       - Output: 1 if link is up, 0 if down
*   ip            - Output: LAN IP string (empty if none)
*   ip_len        - Size of ip buffer
*   macs          - Output: client MAC array
*   ips           - Output: client IP array (parallel to macs)
*   client_count  - Output: number of valid entries in macs/ips
*
* Notes:
*   Returns immediately with carrier=0 if link is down; macs and
*   ips are not filled in that case.
****************************************************************/
static void eth_event_get_state(int *carrier, char *ip, size_t ip_len,
                                char macs[][18], char ips[][16], int *client_count) {
    FILE *fp = NULL;
    char line[256] = {0};
    char mac[32] = {0};
    char cip[32] = {0};
    char name[64] = {0};
    char reachable_macs[ETH_EVENT_CLIENTS_MAX][18];
    int reachable_count = 0;
    int i = 0;
    unsigned long ts = 0;

    *carrier = 0;
    if (ip_len) ip[0] = '\0';
    *client_count = 0;

    fp = fopen("/sys/class/net/eth0/carrier", "r");
    if (fp) {
        if (fgets(line, sizeof(line), fp)) *carrier = atoi(line);
        fclose(fp);
        fp = NULL;
    }

    if (!*carrier) return;

    eth_get_lan_ip(ip, ip_len);

    eth_get_reachable_macs(reachable_macs, &reachable_count);

    fp = fopen("/tmp/dhcp.leases", "r");
    if (!fp) return;
    while (fgets(line, sizeof(line), fp) && *client_count < ETH_EVENT_CLIENTS_MAX) {
        if (sscanf(line, "%lu %31s %31s %63s", &ts, mac, cip, name) == 4) {
            for (i = 0; i < reachable_count; i++) {
                if (strcasecmp(mac, reachable_macs[i]) == 0) {
                    strncpy(macs[*client_count], mac, 17);
                    macs[*client_count][17] = '\0';
                    strncpy(ips[*client_count], cip, 15);
                    ips[*client_count][15] = '\0';
                    (*client_count)++;
                    break;
                }
            }
        }
    }
    fclose(fp);
    fp = NULL;
}

/****************************************************************
* eth_event_mac_in_list
*
* Case-insensitive search for a MAC address in a 2-D array.
* Ethernet counterpart of wifi_event_mac_in_list.
*
* Parameters:
*   mac    - MAC string to search for
*   macs   - Array of MAC strings to search
*   count  - Number of valid entries in macs[]
*
* Returns:
*   1 if found, 0 otherwise.
****************************************************************/
static int eth_event_mac_in_list(const char *mac, char macs[][18], int count) {
    int i;
    for (i = 0; i < count; i++)
        if (strcasecmp(mac, macs[i]) == 0) return 1;
    return 0;
}

/*
 * eth_up_pending: non-blocking DHCP wait for +ETH:UP event.
 * When the Ethernet carrier comes up but no IP is assigned yet we defer
 * the +ETH:UP event and retry on subsequent poll ticks instead of sleeping.
 */
static struct {
    int active;         /* 1 = waiting for IP before firing +ETH:UP */
    int retries_left;   /* fire (with 0.0.0.0 if needed) when this hits 0 */
} eth_up_pending = {0, 0};

/****************************************************************
* eth_events_poll
*
* Called once per main-loop tick.  Compares the current Ethernet
* state against the previous snapshot (eth_event_prev) and emits
* unsolicited events for any changes:
*
*   +ETH:UP,<PORT>,IP=<ip>        — carrier came up
*   +ETH:DOWN,<PORT>              — carrier went down
*   +ETH:CLIENT,<PORT>,<MAC>,<IP> — new LAN client with DHCP lease
*   +ETH:CLIENT_LEAVE,<PORT>,<MAC>— LAN client left (ARP gone)
*
* ETH UP DHCP retry (non-blocking):
*   When the carrier comes up but no IP is assigned yet, the
*   eth_up_pending state defers the +ETH:UP event for up to 5
*   poll ticks (~5 s).  The event fires as soon as an IP appears
*   or retries are exhausted, without blocking the event loop.
*
* Parameters:
*   uart  - Open UART instance; events written via send_event()
*
* Notes:
*   Not re-entrant; designed for single-threaded use.
*   State is maintained in eth_event_prev and eth_up_pending.
****************************************************************/
void eth_events_poll(uart_inst_t *uart) {
    int carrier = 0;
    int i = 0;
    int cur_count = 0;
    char ip[32] = "";
    char cur_macs[ETH_EVENT_CLIENTS_MAX][18];
    char cur_ips[ETH_EVENT_CLIENTS_MAX][16];

    eth_event_get_state(&carrier, ip, sizeof(ip), cur_macs, cur_ips, &cur_count);

    /* --- Resolve pending ETH UP (waiting for DHCP) --- */
    if (eth_up_pending.active) {
        if (!carrier) {
            /* Link dropped before we got an IP - cancel and fall through to DOWN */
            eth_up_pending.active = 0;
        } else if ((ip[0] && strcmp(ip, "0.0.0.0") != 0) || eth_up_pending.retries_left <= 0) {
            send_event(uart, "+ETH:UP,%d,IP=%s", ETH_PORT, (ip[0] && strcmp(ip, "0.0.0.0") != 0) ? ip : "0.0.0.0");
            eth_up_pending.active = 0;
        } else {
            eth_up_pending.retries_left--;
        }
    } else if (carrier && !eth_event_prev.carrier) {
        /* Link just came up */
        if (ip[0] && strcmp(ip, "0.0.0.0") != 0) {
            send_event(uart, "+ETH:UP,%d,IP=%s", ETH_PORT, ip);
        } else {
            /* No IP yet - defer for up to 5 poll ticks (~5 s) */
            eth_up_pending.active = 1;
            eth_up_pending.retries_left = 5;
        }
    } else if (!carrier && eth_event_prev.carrier) {
        eth_up_pending.active = 0;
        send_event(uart, "+ETH:DOWN,%d", ETH_PORT);
    }

    if (carrier) {
        for (i = 0; i < cur_count; i++) {
            if (!eth_event_mac_in_list(cur_macs[i], eth_event_prev.client_macs, eth_event_prev.client_count)) {
                send_event(uart, "+ETH:CLIENT,%d,%s,%s", ETH_PORT, cur_macs[i], cur_ips[i]);
            }
        }
        for (i = 0; i < eth_event_prev.client_count; i++) {
            if (!eth_event_mac_in_list(eth_event_prev.client_macs[i], cur_macs, cur_count)) {
                send_event(uart, "+ETH:CLIENT_LEAVE,%d,%s", ETH_PORT, eth_event_prev.client_macs[i]);
            }
        }
    }

    eth_event_prev.carrier = carrier;
    eth_event_prev.client_count = cur_count;
    for (i = 0; i < cur_count; i++) {
        strncpy(eth_event_prev.client_macs[i], cur_macs[i], 17);
        eth_event_prev.client_macs[i][17] = '\0';
        strncpy(eth_event_prev.client_ips[i], cur_ips[i], 15);
        eth_event_prev.client_ips[i][15] = '\0';
    }
}

/****************************************************************
* cmd_eth_query
*
* Handles: AT+ETH?
*
* Reports current Ethernet link state, the LAN IP address, and
* the list of active DHCP clients that are also present in the
* ARP neighbour table.
*
* Response when DOWN:
*   +ETH:DOWN
*   OK
*
* Response when UP:
*   +ETH:UP,IP=<ip>,CLIENTS=<n>
*   +ETH:CLIENT,1,<MAC>,<IP>    (one line per active client)
*   OK
*
* PORT is always 1 (MT7628N has one physical LAN port).
*
* Parameters:
*   uart  - UART instance for sending the response
****************************************************************/
typedef struct {
    char mac[32];
    char ip[32];
} eth_client_t;

static void cmd_eth_query(uart_inst_t *uart) {
    FILE *fp = NULL;
    char line[256] = {0};
    char ip[32] = "0.0.0.0";
    char mac[32] = {0};
    char cip[32] = {0};
    char name[64] = {0};
    char reachable_macs[ETH_EVENT_CLIENTS_MAX][18];
    unsigned long ts = 0;
    int carrier = 0;
    int client_count = 0;
    int reachable_count = 0;
    int i = 0;
    eth_client_t clients[64];

    memset(clients, 0, sizeof(clients));

    fp = fopen("/sys/class/net/eth0/carrier", "r");
    if (fp) {
        if (fgets(line, sizeof(line), fp)) {
            carrier = atoi(line);
        }
        fclose(fp);
        fp = NULL;
    }

    if (!carrier) {
        send_response(uart, "+ETH:DOWN");
        send_response(uart, "OK");
        return;
    }

    eth_get_lan_ip(ip, sizeof(ip));
    eth_get_reachable_macs(reachable_macs, &reachable_count);

    fp = fopen("/tmp/dhcp.leases", "r");
    if (fp) {
        while (fgets(line, sizeof(line), fp) && client_count < 64) {
            ts = 0;
            mac[0] = cip[0] = name[0] = '\0';
            if (sscanf(line, "%lu %31s %31s %63s",
                       &ts, mac, cip, name) == 4) {
                for (i = 0; i < reachable_count; i++) {
                    if (strcasecmp(mac, reachable_macs[i]) == 0) {
                        strncpy(clients[client_count].mac, mac,
                                sizeof(clients[client_count].mac) - 1);
                        clients[client_count].mac[sizeof(clients[client_count].mac) - 1] = '\0';
                        strncpy(clients[client_count].ip, cip,
                                sizeof(clients[client_count].ip) - 1);
                        clients[client_count].ip[sizeof(clients[client_count].ip) - 1] = '\0';
                        client_count++;
                        break;
                    }
                }
            }
        }
        fclose(fp);
        fp = NULL;
    }

    send_response(uart, "+ETH:UP,IP=%s,CLIENTS=%d", ip, client_count);

    for (i = 0; i < client_count; i++) {
        send_response(uart, "+ETH:CLIENT,1,%s,%s",
                      clients[i].mac, clients[i].ip);
    }

    send_response(uart, "OK");
}

/****************************************************************
* cmd_reset
*
* Handles: AT+RST
*
* Sends OK immediately (before the reboot), then issues
* `reboot &` asynchronously so the UART response is transmitted
* before the system goes down.  After reboot the daemon
* re-initialises and sends +SYS:BOOT,READY.
*
* Parameters:
*   uart  - UART instance for sending the response
****************************************************************/
static void cmd_reset(uart_inst_t *uart) {
    send_response(uart, "OK");
    system("reboot &");
}

/****************************************************************
* cmd_factory
*
* Handles: AT+FACTORY
*
* Sends OK immediately, then runs `firstboot -y && reboot &`
* asynchronously.  `firstboot` wipes all overlay changes (UCI
* config, installed packages, etc.) restoring factory defaults.
* After reboot, +SYS:BOOT,READY is sent again.
*
* Parameters:
*   uart  - UART instance for sending the response
*
* Warning:
*   This is destructive and irreversible — all user configuration
*   is erased.
****************************************************************/
static void cmd_factory(uart_inst_t *uart) {
    send_response(uart, "OK");
    system("firstboot -y && reboot &");
}

/****************************************************************
* cmd_save
*
* Handles: AT+SAVE
*
* Commits all pending UCI changes to non-volatile storage by
* running `uci commit`.  Under normal operation each UCI write
* (AT+WIFIAPCFG, AT+WIFISTACFG, etc.) already commits its own
* package, so this command is provided as an explicit flush
* for any changes left uncommitted.
*
* Parameters:
*   uart  - UART instance for sending the response
*
* Response:
*   OK on success; ERROR:2 if `uci commit` exits non-zero.
****************************************************************/
static void cmd_save(uart_inst_t *uart) {
    int ret = system("uci commit");
    if (ret == 0) {
        send_response(uart, "OK");
    } else {
        send_response(uart, ERR_CMD_FAILED);
    }
}

/*============================================================================
 * COMMAND DISPATCH
 *============================================================================*/
/****************************************************************
* command_callback
*
* UART line callback registered by atcmd_init().  Invoked by
* uart_process_events() for every complete AT command line
* received from the STM32 master.
*
* Dispatches by exact string match (for query/action commands) or
* strncmp prefix match (for parameterised set commands).
* Unrecognised commands reply with ERROR:6 (NOT_SUPPORTED).
* Empty lines are silently ignored.
*
* Parameters:
*   uart       - UART instance (forwarded to send_response())
*   line       - Null-terminated AT command line (no \r\n)
*   user_data  - Unused; reserved for future use
****************************************************************/
static void command_callback(uart_inst_t *uart, const char *line, void *user_data) {
    (void)user_data;

    if (strlen(line) == 0) return;

    if (strcmp(line, "AT") == 0) {
        send_response(uart, "OK");
    }
    else if (strcmp(line, "AT+VER?") == 0) {
        send_response(uart, "+VER:%s", AT_VERSION);
        send_response(uart, "OK");
    }
    else if (strcmp(line, "AT+RST") == 0) {
        cmd_reset(uart);
    }
    else if (strcmp(line, "AT+FACTORY") == 0) {
        cmd_factory(uart);
    }
    else if (strncmp(line, "AT+WIFIMODE=", 12) == 0) {
        cmd_wifimode_set(uart, line + 12);
    }
    else if (strcmp(line, "AT+WIFIMODE?") == 0) {
        cmd_wifimode_query(uart);
    }
    else if (strncmp(line, "AT+WIFIAPCFG=", 13) == 0) {
        cmd_wifiapcfg_set(uart, line + 13);
    }
    else if (strcmp(line, "AT+WIFIAPCFG?") == 0) {
        cmd_wifiapcfg_query(uart);
    }
    else if (strcmp(line, "AT+WIFIAP?") == 0) {
        cmd_wifiap_clients(uart);
    }
    else if (strncmp(line, "AT+WIFISTACFG=", 14) == 0) {
        cmd_wifista_cfg(uart, line + 14);
    }
    else if (strcmp(line, "AT+WIFISTACFG?") == 0) {
        cmd_wifistacfg_query(uart);
    }
    else if (strncmp(line, "AT+WIFISTA=", 11) == 0) {
        cmd_wifista_set(uart, line + 11);
    }
    else if (strcmp(line, "AT+WIFISTA?") == 0) {
        cmd_wifista_query(uart);
    }
    else if (strcmp(line, "AT+ETH?") == 0) {
        cmd_eth_query(uart);
    }
    else if (strcmp(line, "AT+SAVE") == 0) {
        cmd_save(uart);
    }
    else {
        send_response(uart, ERR_NOT_SUPPORTED);
    }
}

/****************************************************************
* atcmd_init
*
* Initialises the AT command subsystem.
*
* Registers command_callback on the UART instance so every
* received line is dispatched to the AT command handlers.
* Sends the +SYS:BOOT,READY event to notify the STM32 master
* that the daemon is up and ready to accept commands.
*
* Must be called once after uart_open() and before the main
* select() loop starts.
*
* Parameters:
*   uart  - Open and configured UART instance
****************************************************************/
void atcmd_init(uart_inst_t *uart) {
    uart_register_callback(uart, "", command_callback, NULL);
    send_response(uart, "+SYS:BOOT,READY");
}

