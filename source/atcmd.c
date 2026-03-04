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
#include <uci.h>

#include "uart.h"
#include "atcmd.h"

/*============================================================================
 * RESPONSE/EVENT HELPERS
 *============================================================================*/
/****************************************************************
* send_response
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

/** Drain popen stream before pclose to avoid "Broken pipe" from child. */
static void drain_popen(FILE *fp) {
    char buf[256];
    if (fp) while (fgets(buf, sizeof(buf), fp)) {}
}

/** Parse dnsmasq lease line "timestamp mac ip hostname client_id"; return 1 if mac matches (case-insensitive) and copy ip. */
static int lease_line_get_ip(const char *lease_line, const char *mac, char *ip_out, size_t ip_len) {
    unsigned int ts;
    char mac_buf[18], ip_buf[16];
    if (sscanf(lease_line, "%u %17s %15s", &ts, mac_buf, ip_buf) < 3)
        return 0;
    if (strcasecmp(mac_buf, mac) != 0)
        return 0;
    strncpy(ip_out, ip_buf, ip_len - 1);
    ip_out[ip_len - 1] = '\0';
    return 1;
}

/****************************************************************
* send_event - unsolicited event to STM32 (no OK line)
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

/**
 * AT+WIFIAPCFG=<SSID>,<PASSWORD>,<SECURITY>
 */
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

    if (strlen(ssid) < 1 || strlen(ssid) > 32) {
        send_response(uart, "ERROR:1");
        return;
    }

    const char *encryption = NULL;
    if (strcmp(security, "OPEN") == 0) {
        encryption = "none";
    } else if (strcmp(security, "WPA") == 0) {
        encryption = "psk";
        if (strlen(password) < 8 || strlen(password) > 63) {
            send_response(uart, "ERROR:1");
            return;
        }
    } else if (strcmp(security, "WPA2") == 0) {
        encryption = "psk2";
        if (strlen(password) < 8 || strlen(password) > 63) {
            send_response(uart, "ERROR:1");
            return;
        }
    } else if (strcmp(security, "WPA_WPA2") == 0) {
        encryption = "psk-mixed";
        if (strlen(password) < 8 || strlen(password) > 63) {
            send_response(uart, "ERROR:1");
            return;
        }
    } else {
        send_response(uart, "ERROR:1");
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
        send_response(uart, "ERROR");
        return;
    }
    if (uci_set_string("wireless", AP_IFACE_NAME, "network", "lan") < 0) {
        send_response(uart, "ERROR");
        return;
    }
    if (uci_set_string("wireless", AP_IFACE_NAME, "ssid", ssid) < 0) {
        send_response(uart, "ERROR");
        return;
    }
    if (uci_set_string("wireless", AP_IFACE_NAME, "encryption", encryption) < 0) {
        send_response(uart, "ERROR");
        return;
    }
    if (strcmp(security, "OPEN") != 0) {
        if (uci_set_string("wireless", AP_IFACE_NAME, "key", password) < 0) {
            send_response(uart, "ERROR");
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
****************************************************************/
/**
 * AT+WIFIAPCFG? - Query AP configuration
 * Response: +WIFIAPCFG:SSID=<ssid>,SEC=<sec>,CLIENTS=<n>
 */
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
****************************************************************/
/**
 * AT+WIFIMODE=<mode>
 * mode: 0=OFF, 1=STA, 2=AP, 3=AP+STA
 */
static void cmd_wifimode_set(uart_inst_t *uart, const char *param) {
    int mode = atoi(param);
    char command[512] = {0};
    char buffer[256] = {0};
    char tmp[8] = {0};
    FILE *fp = NULL;
    int ret = 0;

    if (mode < 0 || mode > 3) {
        send_response(uart, "ERROR: Invalid mode");
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
        send_response(uart, "ERROR: Failed to execute command");
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
        send_response(uart, "ERROR");
    }
}

/**
 * AT+WIFIMODE? - Query current WiFi mode
 * Response: +WIFIMODE:<0|1|2|3>
 */
/****************************************************************
* cmd_wifimode_query
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

/**
 * AT+WIFIAP? - Get detailed list of connected clients
 * Response: +WIFIAP:CLIENT,<MAC>,<IP> for each client, then OK
 */
/****************************************************************
* cmd_wifiap_clients
****************************************************************/
static void cmd_wifiap_clients(uart_inst_t *uart) {
    FILE *fp;
    char line[256];
    int client_count = 0;

    fp = popen("iw dev phy0-ap0 station dump | grep Station | awk '{print $2}'", "r");
    if (!fp) {
        send_response(uart, "OK");
        return;
    }

    while (fgets(line, sizeof(line), fp)) {
        line[strcspn(line, "\n")] = 0;

        if (strlen(line) > 0) {
            char mac[18];
            strncpy(mac, line, sizeof(mac) - 1);
            mac[sizeof(mac) - 1] = '\0';

            char ip[16] = "0.0.0.0";
            FILE *leases = fopen("/tmp/dhcp.leases", "r");
            if (leases) {
                char lease_line[256];
                while (fgets(lease_line, sizeof(lease_line), leases)) {
                    if (lease_line_get_ip(lease_line, mac, ip, sizeof(ip)))
                        break;
                }
                fclose(leases);
            }

            send_response(uart, "+WIFIAP:CLIENT,%s,%s", mac, ip);
            client_count++;
        }
    }
    pclose(fp);

    send_response(uart, "OK");
}

/**
 * AT+WIFISTACFG=<SSID>,<PASSWORD>,<SECURITY>
 * SSID      : 1 to 32 characters
 * PASSWORD  : 8 to 63 characters (ignored if OPEN)
 * SECURITY  : OPEN | WPA | WPA2 | WPA_WPA2
 */
/****************************************************************
* cmd_wifista_cfg
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

    if (strlen(ssid) < 1 || strlen(ssid) > 32) {
        send_response(uart, "ERROR:1");
        return;
    }

    const char *encryption = NULL;
    if (strcmp(security, "OPEN") == 0) {
        encryption = "none";
    } else if (strcmp(security, "WPA") == 0) {
        encryption = "psk";
        if (strlen(password) < 8 || strlen(password) > 63) {
            send_response(uart, "ERROR:1");
            return;
        }
    } else if (strcmp(security, "WPA2") == 0) {
        encryption = "psk2";
        if (strlen(password) < 8 || strlen(password) > 63) {
            send_response(uart, "ERROR:1");
            return;
        }
    } else if (strcmp(security, "WPA_WPA2") == 0) {
        encryption = "psk-mixed";
        if (strlen(password) < 8 || strlen(password) > 63) {
            send_response(uart, "ERROR:1");
            return;
        }
    } else {
        send_response(uart, "ERROR:1");
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
        send_response(uart, "ERROR");
        return;
    }
    if (uci_set_string("wireless", sta_section, "ssid", ssid) < 0) {
        send_response(uart, "ERROR");
        return;
    }
    if (uci_set_string("wireless", sta_section, "encryption", encryption) < 0) {
        send_response(uart, "ERROR");
        return;
    }
    if (strcmp(security, "OPEN") != 0) {
        if (uci_set_string("wireless", sta_section, "key", password) < 0) {
            send_response(uart, "ERROR");
            return;
        }
    }

    system("uci commit wireless");
    send_response(uart, "OK");
}

/**
 * AT+WIFISTACFG? - Query STA configuration
 * Response: +WIFISTACFG:SSID=<ssid>,SEC=<OPEN|WPA|WPA2|WPA_WPA2>,PASSWORD=<password>
 */
/****************************************************************
* cmd_wifistacfg_query
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

/**
 * AT+WIFISTA=1   Connect STA
 * AT+WIFISTA=0   Disconnect STA
 */
/****************************************************************
* cmd_wifista_set
****************************************************************/
static void cmd_wifista_set(uart_inst_t *uart, const char *param) {
    int enable = atoi(param);
    char cmd[320];
    char tmp[8] = "";
    const char *sta = STA_IFACE_NAME;

    if (uci_get_string("wireless", sta, "device", tmp, sizeof(tmp)) < 0) {
        sta = "@wifi-iface[1]";
    }
    if (enable == 1) {
        snprintf(cmd, sizeof(cmd),
                 "uci set wireless.%s.disabled='0'; "
                 "uci commit wireless; wifi >/dev/null 2>&1", sta);
    } else if (enable == 0) {
        snprintf(cmd, sizeof(cmd),
                 "uci set wireless.%s.disabled='1'; "
                 "uci commit wireless; wifi >/dev/null 2>&1", sta);
    } else {
        send_response(uart, "ERROR:1");
        return;
    }

    int ret = system(cmd);
    if (ret == 0) {
        send_response(uart, "OK");
    } else {
        send_response(uart, "ERROR");
    }
}

/**
 * Discover STA interface: first interface (other than AP iface) that shows "Connected" in iw link.
 * Writes name to sta_iface_out (max len bytes) and returns 1 if found, else 0.
 */
static int get_sta_connected_iface(char *sta_iface_out, size_t len) {
    FILE *fp;
    char line[256];
    char iface[32];

    if (!sta_iface_out || len < 2) return 0;
    sta_iface_out[0] = '\0';

    fp = popen("iw dev 2>/dev/null | awk '/Interface/ {print $2}'", "r");
    if (!fp) return 0;
    while (fgets(line, sizeof(line), fp)) {
        line[strcspn(line, "\n")] = '\0';
        if (sscanf(line, "%31s", iface) != 1) continue;
        if (strcmp(iface, "phy0-ap0") == 0) continue; /* skip AP interface */
        snprintf(line, sizeof(line), "iw dev %s link 2>/dev/null", iface);
        FILE *link = popen(line, "r");
        if (!link) continue;
        int connected = 0;
        while (fgets(line, sizeof(line), link)) {
            if (strstr(line, "Not connected")) { connected = 0; break; }
            if (strstr(line, "Connected")) connected = 1;
        }
        drain_popen(link);
        pclose(link);
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

/**
 * AT+WIFISTA? - Query STA connection status
 */
/****************************************************************
* cmd_wifista_query
****************************************************************/
static void cmd_wifista_query(uart_inst_t *uart) {
    FILE *fp;
    char line[256];
    char sta_iface[32];

    if (!get_sta_connected_iface(sta_iface, sizeof(sta_iface))) {
        send_response(uart, "+WIFISTA:DISCONNECTED,REASON=NOT_CONNECTED");
        send_response(uart, "OK");
        return;
    }

    char ip[32] = "0.0.0.0";
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
    }

    char rssi[16] = "0";
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
    }

    char encryption[16] = "";
    const char *sec_str = "OPEN";
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

static void wifi_event_get_ap_clients(char macs[][18], char ips[][16], int *count) {
    FILE *fp;
    char line[256];
    *count = 0;

    fp = popen("iw dev phy0-ap0 station dump 2>/dev/null | grep Station | awk '{print $2}'", "r");
    if (!fp) return;

    while (*count < WIFI_EVENT_AP_CLIENTS_MAX && fgets(line, sizeof(line), fp)) {
        line[strcspn(line, "\n")] = '\0';
        if (strlen(line) < 17) continue;
        strncpy(macs[*count], line, 17);
        macs[*count][17] = '\0';
        strcpy(ips[*count], "0.0.0.0");
        FILE *leases = fopen("/tmp/dhcp.leases", "r");
        if (leases) {
            char lease_line[256];
            while (fgets(lease_line, sizeof(lease_line), leases)) {
                if (lease_line_get_ip(lease_line, macs[*count], ips[*count], 16))
                    break;
            }
            fclose(leases);
        }
        (*count)++;
    }
    drain_popen(fp);
    pclose(fp);
}

static int wifi_event_get_sta_state(char *ip, size_t ip_len, char *sec, size_t sec_len) {
    FILE *fp;
    char line[256];
    char sta_iface[32];

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

static int wifi_event_mac_in_list(const char *mac, char macs[][18], int count) {
    for (int i = 0; i < count; i++) {
        if (strcasecmp(mac, macs[i]) == 0) return 1;
    }
    return 0;
}

/** Look up one MAC in /tmp/dhcp.leases; copy IP to ip_out if found. Returns 1 if IP set. */
static int ap_client_lookup_ip(const char *mac, char *ip_out, size_t ip_len) {
    FILE *fp;
    char line[256];
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

/****************************************************************
* wifi_events_poll - call from main loop; emits +WIFI:APJOIN/APLEAVE/STACONN/STADISCONN
****************************************************************/
void wifi_events_poll(uart_inst_t *uart) {
    char cur_macs[WIFI_EVENT_AP_CLIENTS_MAX][18];
    char cur_ips[WIFI_EVENT_AP_CLIENTS_MAX][16];
    int cur_ap = 0, i;
    int cur_sta = 0;
    char cur_sta_ip[32] = "";
    char cur_sta_sec[16] = "OPEN";

    wifi_event_get_ap_clients(cur_macs, cur_ips, &cur_ap);
    cur_sta = wifi_event_get_sta_state(cur_sta_ip, sizeof(cur_sta_ip), cur_sta_sec, sizeof(cur_sta_sec));

    /* APJOIN: in current, not in previous; if IP still 0.0.0.0, give DHCP a moment and re-lookup */
    for (i = 0; i < cur_ap; i++) {
        if (!wifi_event_mac_in_list(cur_macs[i], wifi_event_prev.ap_macs, wifi_event_prev.ap_count)) {
            if (!cur_ips[i][0] || strcmp(cur_ips[i], "0.0.0.0") == 0) {
                int retries = 3;
                while (retries-- > 0) {
                    sleep(1);
                    if (ap_client_lookup_ip(cur_macs[i], cur_ips[i], 16)) break;
                }
            }
            send_event(uart, "+WIFI:APJOIN,%s,%s", cur_macs[i], cur_ips[i][0] ? cur_ips[i] : "0.0.0.0");
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

/** Get LAN IPv4: on OpenWrt the address is usually on br-lan (bridge), not eth0. Try br-lan then eth0. */
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

static void eth_event_get_state(int *carrier, char *ip, size_t ip_len,
                                char macs[][18], char ips[][16], int *client_count) {
    FILE *fp=NULL;
    char line[256]={0};
    char mac[32]={0};
    char cip[32]={0};
    char name[64]={0};
    unsigned long ts=0;

    *carrier = 0;
    if (ip_len) ip[0] = '\0';
    *client_count = 0;

    fp = fopen("/sys/class/net/eth0/carrier", "r");
    if (fp) {
        if (fgets(line, sizeof(line), fp)) *carrier = atoi(line);
        fclose(fp);
    }

    if (!*carrier) return;

    eth_get_lan_ip(ip, ip_len);

    fp = fopen("/tmp/dhcp.leases", "r");
    if (!fp) return;
    while (fgets(line, sizeof(line), fp) && *client_count < ETH_EVENT_CLIENTS_MAX) {
        if (sscanf(line, "%lu %31s %31s %63s", &ts, mac, cip, name) == 4) {
            strncpy(macs[*client_count], mac, 17);
            macs[*client_count][17] = '\0';
            strncpy(ips[*client_count], cip, 15);
            ips[*client_count][15] = '\0';
            (*client_count)++;
        }
    }
    fclose(fp);
}

static int eth_event_mac_in_list(const char *mac, char macs[][18], int count) {
    int i;
    for (i = 0; i < count; i++)
        if (strcasecmp(mac, macs[i]) == 0) return 1;
    return 0;
}

/****************************************************************
* eth_events_poll - emits +ETH:UP, +ETH:DOWN, +ETH:CLIENT, +ETH:CLIENT_LEAVE
****************************************************************/
void eth_events_poll(uart_inst_t *uart) {
    int carrier = 0, i;
    char ip[32] = "";
    char cur_macs[ETH_EVENT_CLIENTS_MAX][18];
    char cur_ips[ETH_EVENT_CLIENTS_MAX][16];
    int cur_count = 0;

    eth_event_get_state(&carrier, ip, sizeof(ip), cur_macs, cur_ips, &cur_count);

    if (carrier && !eth_event_prev.carrier) {
        /* Link just came up: if no IP yet, give DHCP time then re-read (e.g. on boot) */
        if (!ip[0] || strcmp(ip, "0.0.0.0") == 0) {
            int retries = 5;   /* 5 x 1s = up to 5s for DHCP */
            while (retries-- > 0) {
                sleep(1);
                eth_event_get_state(&carrier, ip, sizeof(ip), cur_macs, cur_ips, &cur_count);
                if (ip[0] && strcmp(ip, "0.0.0.0") != 0) break;
            }
        }
        send_event(uart, "+ETH:UP,%d,IP=%s", ETH_PORT, ip[0] ? ip : "0.0.0.0");
    } else if (!carrier && eth_event_prev.carrier) {
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

/**
 * AT+ETH? - Query Ethernet link status, IP, client count and list (spec 274-300)
 * Response when DOWN: +ETH:DOWN then OK
 * Response when UP:   +ETH:UP,IP=<ip>,CLIENTS=<n> then +ETH:CLIENT,<PORT>,<MAC>,<IP> per client then OK
 * PORT = physical Ethernet port number (1-based); MT7628 single LAN = port 1
 */
/****************************************************************
* cmd_eth_query
****************************************************************/
static void cmd_eth_query(uart_inst_t *uart) {
    FILE *fp=NULL;
    char line[256]={0};

    int carrier = 0;
    fp = fopen("/sys/class/net/eth0/carrier", "r");
    if (fp) {
        if (fgets(line, sizeof(line), fp)) {
            carrier = atoi(line);
        }
        fclose(fp);
    }

    if (!carrier) {
        send_response(uart, "+ETH:DOWN");
        send_response(uart, "OK");
        return;
    }

    char ip[32] = "0.0.0.0";
    eth_get_lan_ip(ip, sizeof(ip));

    typedef struct {
        char mac[32];
        char ip[32];
    } eth_client_t;

    eth_client_t clients[64];
    int client_count = 0;

    fp = fopen("/tmp/dhcp.leases", "r");
    if (fp) {
        while (fgets(line, sizeof(line), fp) && client_count < 64) {
            unsigned long ts;
            char mac[32], cip[32], name[64];
            if (sscanf(line, "%lu %31s %31s %63s",
                       &ts, mac, cip, name) == 4) {
                strncpy(clients[client_count].mac, mac,
                        sizeof(clients[client_count].mac) - 1);
                clients[client_count].mac[sizeof(clients[client_count].mac) - 1] = '\0';
                strncpy(clients[client_count].ip, cip,
                        sizeof(clients[client_count].ip) - 1);
                clients[client_count].ip[sizeof(clients[client_count].ip) - 1] = '\0';
                client_count++;
            }
        }
        fclose(fp);
    }

    send_response(uart, "+ETH:UP,IP=%s,CLIENTS=%d", ip, client_count);

    for (int i = 0; i < client_count; i++) {
        send_response(uart, "+ETH:CLIENT,1,%s,%s",
                      clients[i].mac, clients[i].ip);
    }

    send_response(uart, "OK");
}

/**
 * AT+RST - Software reset (Linux reboot)
 */
/****************************************************************
* cmd_reset
****************************************************************/
static void cmd_reset(uart_inst_t *uart) {
    send_response(uart, "OK");
    system("reboot &");
}

/**
 * AT+FACTORY - Factory reset and reboot
 */
/****************************************************************
* cmd_factory
****************************************************************/
static void cmd_factory(uart_inst_t *uart) {
    send_response(uart, "OK");
    system("firstboot -y && reboot &");
}

/**
 * AT+SAVE - Save configuration to non-volatile (spec 333-341). Response: OK or ERROR.
 */
/****************************************************************
* cmd_save
****************************************************************/
static void cmd_save(uart_inst_t *uart) {
    int ret = system("uci commit");
    if (ret == 0) {
        send_response(uart, "OK");
    } else {
        send_response(uart, "ERROR");
    }
}

/*============================================================================
 * COMMAND DISPATCH
 *============================================================================*/
/****************************************************************
* command_callback
****************************************************************/
static void command_callback(uart_inst_t *uart, const char *line, void *user_data) {
    (void)user_data;

    if (strlen(line) == 0) return;

    if (strcmp(line, "AT") == 0) {
        send_response(uart, "OK");
    }
    else if (strcmp(line, "AT+VER?") == 0) {
        send_response(uart, "+VER:MT7628N-AT-1.2.0");
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
        send_response(uart, "ERROR");
    }
}

/****************************************************************
* atcmd_init
****************************************************************/
void atcmd_init(uart_inst_t *uart) {
    uart_register_callback(uart, "", command_callback, NULL);
    send_response(uart, "+SYS:BOOT,READY");
}

