/*****************************************************************
*
*   AT Command Daemon for MT7628N (OpenWrt) using libuci
*   Version 2.0
*   Implements protocol: STM32 <-> MT7628N UART AT commands
*
*****************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <time.h>
#include <uci.h>                // libuci

/*============================================================================
 * MACROS
 *============================================================================*/
#define UART_DEVICE     "ttyS1"
#define UART_BAUD       115200
#define UART_MODE       "8N1"
#define RX_BUFFER_SIZE  2048
#define CMD_BUFFER_SIZE 256
#define MAX_CALLBACKS   16
#define TIMEOUT_MS      3000

/*============================================================================
 * UART INSTANCE (reused from previous version)
 *============================================================================*/
typedef struct uart_inst uart_inst_t;
typedef void (*uart_callback_t)(uart_inst_t *uart, const char *line, void *user_data);

struct uart_inst {
    int         fd;
    char        device[64];
    speed_t     baud_rate;
    tcflag_t    data_bits;
    tcflag_t    parity;
    tcflag_t    stop_bits;
    struct      termios tios;
    int         timeout_ms;

    unsigned char *rx_buffer;
    size_t      rx_buffer_size;
    size_t      rx_data_len;

    char        line_buffer[256];
    size_t      line_len;

    struct {
        char        pattern[64];
        uart_callback_t callback;
        void       *user_data;
    } callbacks[MAX_CALLBACKS];
    int         callback_count;
};

/* UART API (implemented below) */
uart_inst_t* uart_init(size_t rx_buffer_size);
int uart_configure(uart_inst_t *uart, const char *device, int baudrate, const char *mode);
int uart_open(uart_inst_t *uart);
ssize_t uart_write(uart_inst_t *uart, const void *data, size_t len);
int uart_register_callback(uart_inst_t *uart, const char *pattern, uart_callback_t cb, void *user_data);
void uart_process_events(uart_inst_t *uart);
void uart_destroy(uart_inst_t *uart);

/*============================================================================
 * UART IMPLEMENTATION
 *============================================================================*/
uart_inst_t* uart_init(size_t rx_buffer_size) {
    uart_inst_t *uart = calloc(1, sizeof(uart_inst_t));
    if (!uart) return NULL;
    uart->fd = -1;
    uart->timeout_ms = 100;
    uart->rx_buffer = malloc(rx_buffer_size);
    if (!uart->rx_buffer) { free(uart); return NULL; }
    uart->rx_buffer_size = rx_buffer_size;
    uart->callback_count = 0;
    return uart;
}

static speed_t uart_baud_to_const(int baudrate) {
    switch(baudrate) {
        case 115200: return B115200;
        default: return B0;
    }
}

static int uart_parse_mode(const char *mode, tcflag_t *data_bits, tcflag_t *parity, tcflag_t *stop_bits, tcflag_t *iflags) {
    if (!mode || strlen(mode) != 3) return -1;
    switch(mode[0]) {
        case '8': *data_bits = CS8; break;
        default: return -1;
    }
    switch(mode[1]) {
        case 'N': case 'n': *parity = 0; *iflags = IGNPAR; break;
        default: return -1;
    }
    switch(mode[2]) {
        case '1': *stop_bits = 0; break;
        default: return -1;
    }
    return 0;
}

int uart_configure(uart_inst_t *uart, const char *device, int baudrate, const char *mode) {
    if (!uart || !device || !mode) return -1;
    snprintf(uart->device, sizeof(uart->device), "/dev/%s", device);
    uart->baud_rate = uart_baud_to_const(baudrate);
    if (uart->baud_rate == 0) return -1;
    return uart_parse_mode(mode, &uart->data_bits, &uart->parity, &uart->stop_bits, &uart->iflags);
}

int uart_open(uart_inst_t *uart) {
    if (!uart || uart->fd >= 0) return -1;
    uart->fd = open(uart->device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart->fd < 0) return -1;
    memset(&uart->tios, 0, sizeof(uart->tios));
    tcgetattr(uart->fd, &uart->tios);
    cfmakeraw(&uart->tios);
    uart->tios.c_cflag = uart->data_bits | uart->parity | uart->stop_bits | CLOCAL | CREAD | HUPCL;
    uart->tios.c_iflag = uart->iflags;
    uart->tios.c_cc[VMIN] = 0;
    uart->tios.c_cc[VTIME] = uart->timeout_ms / 100;
    cfsetispeed(&uart->tios, uart->baud_rate);
    cfsetospeed(&uart->tios, uart->baud_rate);
    tcflush(uart->fd, TCIFLUSH);
    if (tcsetattr(uart->fd, TCSANOW, &uart->tios) < 0) {
        close(uart->fd);
        uart->fd = -1;
        return -1;
    }
    return 0;
}

ssize_t uart_write(uart_inst_t *uart, const void *data, size_t len) {
    if (!uart || uart->fd < 0) return -1;
    return write(uart->fd, data, len);
}

int uart_register_callback(uart_inst_t *uart, const char *pattern, uart_callback_t cb, void *user_data) {
    if (!uart || !pattern || !cb || uart->callback_count >= MAX_CALLBACKS) return -1;
    strncpy(uart->callbacks[uart->callback_count].pattern, pattern, sizeof(uart->callbacks[0].pattern)-1);
    uart->callbacks[uart->callback_count].callback = cb;
    uart->callbacks[uart->callback_count].user_data = user_data;
    uart->callback_count++;
    return 0;
}

void uart_process_events(uart_inst_t *uart) {
    if (!uart || uart->fd < 0) return;
    ssize_t n = read(uart->fd, uart->rx_buffer + uart->rx_data_len,
                     uart->rx_buffer_size - uart->rx_data_len);
    if (n > 0) {
        uart->rx_data_len += n;
        char *p = (char*)uart->rx_buffer;
        char *end = p + uart->rx_data_len;
        while (p < end) {
            char *newline = memchr(p, '\n', end - p);
            if (!newline) break;
            size_t line_len = newline - p;
            if (line_len > 0 && p[line_len-1] == '\r') line_len--;
            if (line_len < sizeof(uart->line_buffer)) {
                memcpy(uart->line_buffer, p, line_len);
                uart->line_buffer[line_len] = '\0';
                for (int i = 0; i < uart->callback_count; i++) {
                    if (strstr(uart->line_buffer, uart->callbacks[i].pattern)) {
                        uart->callbacks[i].callback(uart, uart->line_buffer,
                                                    uart->callbacks[i].user_data);
                    }
                }
            }
            p = newline + 1;
        }
        if (p < end) {
            memmove(uart->rx_buffer, p, end - p);
            uart->rx_data_len = end - p;
        } else {
            uart->rx_data_len = 0;
        }
    }
}

void uart_destroy(uart_inst_t *uart) {
    if (!uart) return;
    if (uart->fd >= 0) close(uart->fd);
    if (uart->rx_buffer) free(uart->rx_buffer);
    free(uart);
}

/*============================================================================
 * RESPONSE/EVENT HELPERS
 *============================================================================*/
static void send_response(uart_inst_t *uart, const char *fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    uart_write(uart, buf, strlen(buf));
    uart_write(uart, "\r\n", 2);
}

static void send_event(uart_inst_t *uart, const char *event) {
    uart_write(uart, event, strlen(event));
    uart_write(uart, "\r\n", 2);
}

/*============================================================================
 * UCI HELPERS
 *============================================================================*/
static int uci_get_string(const char *pkg, const char *section, const char *option, char *out, size_t out_len) {
    struct uci_context *ctx = uci_alloc_context();
    struct uci_ptr ptr;
    char path[256];
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

static int uci_set_string(const char *pkg, const char *section, const char *option, const char *value) {
    struct uci_context *ctx = uci_alloc_context();
    struct uci_ptr ptr;
    char path[256];
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
 * GLOBAL STATE (only for dynamic values)
 *============================================================================*/
static uart_inst_t *g_uart = NULL;
static volatile int g_running = 1;

/*============================================================================
 * COMMAND HANDLERS (using libuci for configuration)
 *============================================================================*/
static void cmd_at(uart_inst_t *uart) {
    send_response(uart, "OK");
}

static void cmd_version(uart_inst_t *uart) {
    send_response(uart, "+VER:MT7628N-AT-1.2.0");
    send_response(uart, "OK");
}

static void cmd_reset(uart_inst_t *uart) {
    send_response(uart, "OK");
    system("reboot");
}

static void cmd_factory(uart_inst_t *uart) {
    send_response(uart, "OK");
    system("firstboot && reboot");
}

static void cmd_wifimode_set(uart_inst_t *uart, int mode) {
    const char *mode_str;
    switch (mode) {
        case 0: mode_str = "off"; break;
        case 1: mode_str = "sta"; break;
        case 2: mode_str = "ap"; break;
        case 3: mode_str = "apsta"; break;
        default:
            send_response(uart, "ERROR:1");
            return;
    }
    if (uci_set_string("wireless", "@wifi-iface[0]", "mode", mode_str) < 0) {
        send_response(uart, "ERROR");
        return;
    }
    system("wifi");  // restart wireless
    send_response(uart, "OK");
}

static void cmd_wifimode_get(uart_inst_t *uart) {
    char mode_str[16];
    if (uci_get_string("wireless", "@wifi-iface[0]", "mode", mode_str, sizeof(mode_str)) < 0) {
        send_response(uart, "ERROR");
        return;
    }
    int mode = 3; // default apsta
    if (strcmp(mode_str, "off") == 0) mode = 0;
    else if (strcmp(mode_str, "sta") == 0) mode = 1;
    else if (strcmp(mode_str, "ap") == 0) mode = 2;
    else if (strcmp(mode_str, "apsta") == 0) mode = 3;
    send_response(uart, "+WIFIMODE:%d", mode);
    send_response(uart, "OK");
}

static void cmd_wifiapcfg_set(uart_inst_t *uart, const char *ssid, const char *pwd, const char *sec) {
    // Validate security and map to UCI encryption
    const char *enc;
    if (strcmp(sec, "OPEN") == 0) enc = "none";
    else if (strcmp(sec, "WPA") == 0) enc = "psk";
    else if (strcmp(sec, "WPA2") == 0) enc = "psk2";
    else if (strcmp(sec, "WPA_WPA2") == 0) enc = "psk-mixed";
    else {
        send_response(uart, "ERROR:1");
        return;
    }
    if (uci_set_string("wireless", "@wifi-iface[0]", "ssid", ssid) < 0 ||
        uci_set_string("wireless", "@wifi-iface[0]", "key", pwd) < 0 ||
        uci_set_string("wireless", "@wifi-iface[0]", "encryption", enc) < 0) {
        send_response(uart, "ERROR");
        return;
    }
    system("wifi");
    send_response(uart, "OK");
}

static void cmd_wifiapcfg_get(uart_inst_t *uart) {
    char ssid[33], enc[16];
    if (uci_get_string("wireless", "@wifi-iface[0]", "ssid", ssid, sizeof(ssid)) < 0 ||
        uci_get_string("wireless", "@wifi-iface[0]", "encryption", enc, sizeof(enc)) < 0) {
        send_response(uart, "ERROR");
        return;
    }
    // Map encryption back to protocol security string
    const char *sec = "OPEN";
    if (strcmp(enc, "psk") == 0) sec = "WPA";
    else if (strcmp(enc, "psk2") == 0) sec = "WPA2";
    else if (strcmp(enc, "psk-mixed") == 0) sec = "WPA_WPA2";

    // Get client count (via iw)
    FILE *fp = popen("iw dev wlan0 station dump | grep Station | wc -l", "r");
    char buf[16];
    int clients = 0;
    if (fp && fgets(buf, sizeof(buf), fp)) clients = atoi(buf);
    if (fp) pclose(fp);

    send_response(uart, "+WIFIAPCFG:SSID=%s,SEC=%s,CLIENTS=%d", ssid, sec, clients);
    send_response(uart, "OK");
}

static void cmd_wifiap(uart_inst_t *uart) {
    FILE *fp = popen("iw dev wlan0 station dump | grep Station | awk '{print $2}'", "r");
    char mac[18];
    while (fgets(mac, sizeof(mac), fp)) {
        mac[strcspn(mac, "\n")] = 0;
        // Get IP from DHCP leases
        char ip[16] = "0.0.0.0";
        FILE *leases = fopen("/tmp/dhcp.leases", "r");
        if (leases) {
            char line[256];
            while (fgets(line, sizeof(line), leases)) {
                if (strstr(line, mac)) {
                    sscanf(line, "%*s %*s %s", ip);
                    break;
                }
            }
            fclose(leases);
        }
        send_response(uart, "+WIFIAP:CLIENT,%s,%s", mac, ip);
    }
    pclose(fp);
    send_response(uart, "OK");
}

static void cmd_wifista_connect(uart_inst_t *uart, int enable) {
    if (enable) {
        system("wpa_cli -i wlan1 reconfigure");
    } else {
        system("wpa_cli -i wlan1 disconnect");
    }
    send_response(uart, "OK");
}

static void cmd_wifista_get(uart_inst_t *uart) {
    // Check if connected
    FILE *fp = popen("iw dev wlan1 link | grep 'Not connected'", "r");
    char buf[256];
    int connected = 1;
    if (fp && fgets(buf, sizeof(buf), fp)) connected = 0;
    if (fp) pclose(fp);

    if (connected) {
        char ip[16] = "";
        fp = popen("ifconfig wlan1 | grep 'inet addr' | awk '{print $2}' | cut -d: -f2", "r");
        if (fp && fgets(ip, sizeof(ip), fp)) ip[strcspn(ip, "\n")] = 0;
        if (fp) pclose(fp);

        int rssi = 0;
        fp = popen("iw dev wlan1 link | grep signal | awk '{print $2}'", "r");
        if (fp && fgets(buf, sizeof(buf), fp)) rssi = atoi(buf);
        if (fp) pclose(fp);

        char sec[16] = "";
        uci_get_string("wireless", "@wifi-iface[1]", "encryption", sec, sizeof(sec));
        const char *sec_str = "OPEN";
        if (strcmp(sec, "psk") == 0) sec_str = "WPA";
        else if (strcmp(sec, "psk2") == 0) sec_str = "WPA2";
        else if (strcmp(sec, "psk-mixed") == 0) sec_str = "WPA_WPA2";

        send_response(uart, "+WIFISTA:CONNECTED,IP=%s,RSSI=%d,SEC=%s", ip, rssi, sec_str);
    } else {
        send_response(uart, "+WIFISTA:DISCONNECTED,REASON=unknown");
    }
    send_response(uart, "OK");
}

static void cmd_eth_get(uart_inst_t *uart) {
    // Check link status
    FILE *fp = popen("cat /sys/class/net/eth0/carrier", "r");
    char buf[16];
    int link = 0;
    if (fp && fgets(buf, sizeof(buf), fp)) link = atoi(buf);
    if (fp) pclose(fp);

    if (link) {
        char ip[16] = "";
        fp = popen("ip -4 addr show eth0 | grep inet | awk '{print $2}' | cut -d/ -f1", "r");
        if (fp && fgets(ip, sizeof(ip), fp)) ip[strcspn(ip, "\n")] = 0;
        if (fp) pclose(fp);

        // Get client count (via DHCP leases)
        int clients = 0;
        fp = popen("grep -c 'eth0' /tmp/dhcp.leases 2>/dev/null || echo 0", "r");
        if (fp && fgets(buf, sizeof(buf), fp)) clients = atoi(buf);
        if (fp) pclose(fp);

        send_response(uart, "+ETH:UP,IP=%s,CLIENTS=%d", ip, clients);
        // Detailed client list could be added similarly
    } else {
        send_response(uart, "+ETH:DOWN");
    }
    send_response(uart, "OK");
}

static void cmd_save(uart_inst_t *uart) {
    system("uci commit");
    send_response(uart, "OK");
}

/*============================================================================
 * COMMAND PARSER
 *============================================================================*/
static void command_callback(uart_inst_t *uart, const char *line, void *user_data) {
    (void)user_data;
    if (strlen(line) == 0) return;

    if (strcmp(line, "AT") == 0) {
        cmd_at(uart);
    }
    else if (strcmp(line, "AT+VER?") == 0) {
        cmd_version(uart);
    }
    else if (strcmp(line, "AT+RST") == 0) {
        cmd_reset(uart);
    }
    else if (strcmp(line, "AT+FACTORY") == 0) {
        cmd_factory(uart);
    }
    else if (strncmp(line, "AT+WIFIMODE=", 13) == 0) {
        int mode = atoi(line + 13);
        cmd_wifimode_set(uart, mode);
    }
    else if (strcmp(line, "AT+WIFIMODE?") == 0) {
        cmd_wifimode_get(uart);
    }
    else if (strncmp(line, "AT+WIFIAPCFG=", 14) == 0) {
        // Format: AT+WIFIAPCFG=<SSID>,<PASSWORD>,<SECURITY>
        char ssid[64] = "", pwd[64] = "", sec[16] = "";
        if (sscanf(line + 14, "%63[^,],%63[^,],%15s", ssid, pwd, sec) == 3) {
            cmd_wifiapcfg_set(uart, ssid, pwd, sec);
        } else {
            send_response(uart, "ERROR:1");
        }
    }
    else if (strcmp(line, "AT+WIFIAPCFG?") == 0) {
        cmd_wifiapcfg_get(uart);
    }
    else if (strcmp(line, "AT+WIFIAP?") == 0) {
        cmd_wifiap(uart);
    }
    else if (strncmp(line, "AT+WIFISTA=", 12) == 0) {
        int enable = atoi(line + 12);
        cmd_wifista_connect(uart, enable);
    }
    else if (strcmp(line, "AT+WIFISTA?") == 0) {
        cmd_wifista_get(uart);
    }
    else if (strcmp(line, "AT+ETH?") == 0) {
        cmd_eth_get(uart);
    }
    else if (strcmp(line, "AT+SAVE") == 0) {
        cmd_save(uart);
    }
    else {
        send_response(uart, "ERROR");
    }
}

/*============================================================================
 * ASYNC EVENT MONITOR (polling – can be extended with netlink/ubus)
 *============================================================================*/
static void check_network_events(void) {
    static int last_eth_link = -1;
    static char last_eth_ip[16] = "";
    static int last_sta_conn = -1;

    // Ethernet link and IP
    FILE *fp = popen("cat /sys/class/net/eth0/carrier", "r");
    char buf[16];
    int link = 0;
    if (fp && fgets(buf, sizeof(buf), fp)) link = atoi(buf);
    if (fp) pclose(fp);

    char ip[16] = "";
    if (link) {
        fp = popen("ip -4 addr show eth0 | grep inet | awk '{print $2}' | cut -d/ -f1", "r");
        if (fp && fgets(ip, sizeof(ip), fp)) ip[strcspn(ip, "\n")] = 0;
        if (fp) pclose(fp);
    }

    if (link != last_eth_link) {
        if (link) {
            send_event(g_uart, "+ETH:UP,1");  // port 1 assumed
        } else {
            send_event(g_uart, "+ETH:DOWN,1");
        }
        last_eth_link = link;
    } else if (link && strcmp(ip, last_eth_ip) != 0) {
        // IP changed
        char event[64];
        snprintf(event, sizeof(event), "+ETH:UP,1,IP=%s", ip);
        send_event(g_uart, event);
    }
    strcpy(last_eth_ip, ip);

    // STA connection status
    fp = popen("iw dev wlan1 link | grep 'Not connected'", "r");
    int conn = 1;
    if (fp && fgets(buf, sizeof(buf), fp)) conn = 0;
    if (fp) pclose(fp);

    if (conn != last_sta_conn) {
        if (conn) {
            // Get IP for event
            fp = popen("ifconfig wlan1 | grep 'inet addr' | awk '{print $2}' | cut -d: -f2", "r");
            char sta_ip[16] = "";
            if (fp && fgets(sta_ip, sizeof(sta_ip), fp)) sta_ip[strcspn(sta_ip, "\n")] = 0;
            if (fp) pclose(fp);
            char event[64];
            snprintf(event, sizeof(event), "+WIFI:STACONN,IP=%s,SEC=WPA2", sta_ip); // security from UCI
            send_event(g_uart, event);
        } else {
            send_event(g_uart, "+WIFI:STADISCONN,link lost");
        }
        last_sta_conn = conn;
    }
}

/*============================================================================
 * MAIN LOOP
 *============================================================================*/
static void sigint_handler(int sig) {
    (void)sig;
    g_running = 0;
}

int main(void) {
    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

    g_uart = uart_init(RX_BUFFER_SIZE);
    if (!g_uart) {
        fprintf(stderr, "Failed to init UART\n");
        return 1;
    }

    if (uart_configure(g_uart, UART_DEVICE, UART_BAUD, UART_MODE) < 0) {
        fprintf(stderr, "Failed to configure UART\n");
        uart_destroy(g_uart);
        return 1;
    }

    if (uart_open(g_uart) < 0) {
        fprintf(stderr, "Failed to open UART %s\n", UART_DEVICE);
        uart_destroy(g_uart);
        return 1;
    }

    uart_register_callback(g_uart, "", command_callback, NULL); // catch all lines

    send_event(g_uart, "+SYS:BOOT,READY");

    fd_set read_fds;
    int max_fd = g_uart->fd;
    time_t last_check = 0;

    while (g_running) {
        FD_ZERO(&read_fds);
        FD_SET(g_uart->fd, &read_fds);
        struct timeval tv = {1, 0}; // 1 sec timeout

        int ret = select(max_fd + 1, &read_fds, NULL, NULL, &tv);
        if (ret < 0 && errno != EINTR) break;

        if (ret > 0 && FD_ISSET(g_uart->fd, &read_fds)) {
            uart_process_events(g_uart);
        }

        time_t now = time(NULL);
        if (now - last_check >= 2) { // check every 2 seconds
            check_network_events();
            last_check = now;
        }
    }

    uart_destroy(g_uart);
    return 0;
}