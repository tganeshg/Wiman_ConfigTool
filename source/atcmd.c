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
    if (fp && fgets(buf, sizeof(buf), fp)) {
        clients = atoi(buf);
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

    /* Ensure wifinet0 exists when switching to AP (mode 2 or 3) */
    if ((mode == 2 || mode == 3) &&
        uci_get_string("wireless", AP_IFACE_NAME, "device", tmp, sizeof(tmp)) < 0) {
        system("uci set wireless." AP_IFACE_NAME "=wifi-iface");
        system("uci set wireless." AP_IFACE_NAME ".device='radio0'");
        system("uci set wireless." AP_IFACE_NAME ".network='lan'");
        system("uci set wireless." AP_IFACE_NAME ".mode='ap'");
        system("uci set wireless." AP_IFACE_NAME ".disabled='0'");
        system("uci commit wireless");
    }

    fp = popen("uci get wireless." AP_IFACE_NAME ".disabled 2>/dev/null", "r");
    if (fp && fgets(buffer, sizeof(buffer), fp)) {
        buffer[strcspn(buffer, "\n")] = '\0';
        pclose(fp);
    }

    fp = popen("uci get wireless." AP_IFACE_NAME ".mode 2>/dev/null", "r");
    if (fp && fgets(buffer, sizeof(buffer), fp)) {
        buffer[strcspn(buffer, "\n")] = 0;
        pclose(fp);
    }

    if (mode == 2) { /* AP only */
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
            system("uci set wireless." STA_IFACE_NAME "=wifi-iface");
        }
        system("uci set wireless." STA_IFACE_NAME ".device='radio0'");
        system("uci set wireless." STA_IFACE_NAME ".mode='sta'");
        system("uci set wireless." STA_IFACE_NAME ".network='wwan'");
        /* Copy STA credentials from @wifi-iface[1] to wifinet1 if present */
        if (uci_get_string("wireless", "@wifi-iface[1]", "ssid", buffer, sizeof(buffer)) == 0 && buffer[0]) {
            uci_set_string("wireless", STA_IFACE_NAME, "ssid", buffer);
        }
        if (uci_get_string("wireless", "@wifi-iface[1]", "encryption", buffer, sizeof(buffer)) == 0 && buffer[0]) {
            uci_set_string("wireless", STA_IFACE_NAME, "encryption", buffer);
        }
        if (uci_get_string("wireless", "@wifi-iface[1]", "key", buffer, sizeof(buffer)) == 0 && buffer[0]) {
            uci_set_string("wireless", STA_IFACE_NAME, "key", buffer);
        }
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
            system("uci set wireless." STA_IFACE_NAME ".device='radio0'");
            system("uci set wireless." STA_IFACE_NAME ".mode='sta'");
            system("uci set wireless." STA_IFACE_NAME ".network='wwan'");
            if (uci_get_string("wireless", "@wifi-iface[1]", "ssid", buffer, sizeof(buffer)) == 0 && buffer[0]) {
                uci_set_string("wireless", STA_IFACE_NAME, "ssid", buffer);
            }
            if (uci_get_string("wireless", "@wifi-iface[1]", "encryption", buffer, sizeof(buffer)) == 0 && buffer[0]) {
                uci_set_string("wireless", STA_IFACE_NAME, "encryption", buffer);
            }
            if (uci_get_string("wireless", "@wifi-iface[1]", "key", buffer, sizeof(buffer)) == 0 && buffer[0]) {
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
    if (fp && fgets(buffer, sizeof(buffer), fp)) {
        buffer[strcspn(buffer, "\n")] = 0;
        pclose(fp);
    }

    fp = popen("uci get wireless." AP_IFACE_NAME ".mode 2>/dev/null", "r");
    if (fp && fgets(buffer, sizeof(buffer), fp)) {
        buffer[strcspn(buffer, "\n")] = 0;
        pclose(fp);
    }

    fp = popen("ps | grep -E 'hostapd|wpa_supplicant' | grep -v grep", "r");
    while (fp && fgets(buffer, sizeof(buffer), fp)) {
        buffer[strcspn(buffer, "\n")] = 0;
    }
    if (fp) pclose(fp);

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
                    if (strstr(lease_line, mac)) {
                        sscanf(lease_line, "%*s %*s %15s", ip);
                        break;
                    }
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
 * AT+WIFISTA? - Query STA connection status
 */
/****************************************************************
* cmd_wifista_query
****************************************************************/
static void cmd_wifista_query(uart_inst_t *uart) {
    FILE *fp;
    char line[256];
    int connected = 0;

    fp = popen("iw dev wlan0 link 2>/dev/null", "r");
    if (fp) {
        while (fgets(line, sizeof(line), fp)) {
            if (strstr(line, "Not connected")) {
                connected = 0;
                break;
            }
            if (strstr(line, "Connected")) {
                connected = 1;
            }
        }
        pclose(fp);
    }

    if (!connected) {
        send_response(uart, "+WIFISTA:DISCONNECTED,REASON=NOT_CONNECTED");
        send_response(uart, "OK");
        return;
    }

    char ip[32] = "0.0.0.0";
    fp = popen("ip -4 addr show wlan0 | awk '/inet /{print $2}' | cut -d/ -f1", "r");
    if (fp && fgets(line, sizeof(line), fp)) {
        line[strcspn(line, "\n")] = 0;
        if (strlen(line) > 0) {
            strncpy(ip, line, sizeof(ip) - 1);
            ip[sizeof(ip) - 1] = '\0';
        }
        pclose(fp);
    }

    char rssi[16] = "0";
    fp = popen("iw dev wlan0 link | awk '/signal:/ {print $2}'", "r");
    if (fp && fgets(line, sizeof(line), fp)) {
        line[strcspn(line, "\n")] = 0;
        if (strlen(line) > 0) {
            strncpy(rssi, line, sizeof(rssi) - 1);
            rssi[sizeof(rssi) - 1] = '\0';
        }
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

/**
 * AT+ETH?
 */
/****************************************************************
* cmd_eth_query
****************************************************************/
static void cmd_eth_query(uart_inst_t *uart) {
    FILE *fp;
    char line[256];

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
    fp = popen("ip -4 addr show eth0 | awk '/inet /{print $2}' | cut -d/ -f1", "r");
    if (fp && fgets(line, sizeof(line), fp)) {
        line[strcspn(line, "\n")] = 0;
        if (strlen(line) > 0) {
            strncpy(ip, line, sizeof(ip) - 1);
            ip[sizeof(ip) - 1] = '\0';
        }
        pclose(fp);
    }

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
 * AT+SAVE - Save configuration
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

