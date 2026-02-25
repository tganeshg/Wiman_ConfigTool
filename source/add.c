/*****************************************************************
*
*   AT Command Daemon for MT7628N (OpenWrt) using libuci
*   Version 2.0
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
#include <stdarg.h>      // For va_start, va_end
#include <stddef.h>       // For size_t
#include <uci.h>          // libuci

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
 * TYPES
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
    tcflag_t    iflags;          // ← ADD THIS - WAS MISSING
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

/*============================================================================
 * FORWARD DECLARATIONS
 *============================================================================*/
static speed_t uart_baud_to_const(int baudrate);
static int uart_parse_mode(const char *mode, tcflag_t *data_bits, tcflag_t *parity, 
                           tcflag_t *stop_bits, tcflag_t *iflags);
static void send_response(uart_inst_t *uart, const char *fmt, ...);
static void command_callback(uart_inst_t *uart, const char *line, void *user_data);

/*============================================================================
 * UART API FUNCTIONS
 *============================================================================*/
uart_inst_t* uart_init(size_t rx_buffer_size) {
    uart_inst_t *uart = calloc(1, sizeof(uart_inst_t));
    if (!uart) return NULL;
    uart->fd = -1;
    uart->timeout_ms = 100;
    uart->rx_buffer = malloc(rx_buffer_size);
    if (!uart->rx_buffer) { 
        free(uart); 
        return NULL; 
    }
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

static int uart_parse_mode(const char *mode, tcflag_t *data_bits, tcflag_t *parity, 
                           tcflag_t *stop_bits, tcflag_t *iflags) {
    if (!mode || strlen(mode) != 3) return -1;
    
    // Data bits
    switch(mode[0]) {
        case '8': *data_bits = CS8; break;
        case '7': *data_bits = CS7; break;
        case '6': *data_bits = CS6; break;
        case '5': *data_bits = CS5; break;
        default: return -1;
    }
    
    // Parity
    switch(mode[1]) {
        case 'N': case 'n':
            *parity = 0;
            *iflags = IGNPAR;
            break;
        case 'E': case 'e':
            *parity = PARENB;
            *iflags = INPCK;
            break;
        case 'O': case 'o':
            *parity = PARENB | PARODD;
            *iflags = INPCK;
            break;
        default:
            return -1;
    }
    
    // Stop bits
    switch(mode[2]) {
        case '1': *stop_bits = 0; break;
        case '2': *stop_bits = CSTOPB; break;
        default: return -1;
    }
    
    return 0;
}

int uart_configure(uart_inst_t *uart, const char *device, int baudrate, const char *mode) {
    if (!uart || !device || !mode) return -1;
    
    snprintf(uart->device, sizeof(uart->device), "/dev/%s", device);
    
    uart->baud_rate = uart_baud_to_const(baudrate);
    if (uart->baud_rate == 0) return -1;
    
    int ret = uart_parse_mode(mode, &uart->data_bits, &uart->parity, 
                               &uart->stop_bits, &uart->iflags);
    return ret;  // Return the result
}

int uart_open(uart_inst_t *uart) {
    if (!uart || uart->fd >= 0) return -1;
    
    uart->fd = open(uart->device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart->fd < 0) return -1;
    
    memset(&uart->tios, 0, sizeof(uart->tios));
    tcgetattr(uart->fd, &uart->tios);
    
    cfmakeraw(&uart->tios);
    
    uart->tios.c_cflag = uart->data_bits | uart->parity | uart->stop_bits | 
                         CLOCAL | CREAD | HUPCL;
    uart->tios.c_iflag = uart->iflags;  // Now this works
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

int uart_register_callback(uart_inst_t *uart, const char *pattern, 
                           uart_callback_t cb, void *user_data) {
    if (!uart || !pattern || !cb || uart->callback_count >= MAX_CALLBACKS) return -1;
    
    strncpy(uart->callbacks[uart->callback_count].pattern, pattern, 
            sizeof(uart->callbacks[0].pattern)-1);
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
            // Look for ANY line ending (\n or \r)
            char *newline = memchr(p, '\n', end - p);
            if (!newline) {
                // Try looking for \r as alternative line ending
                newline = memchr(p, '\r', end - p);
            }
            
            if (!newline) break;
            
            // Process the line
            size_t line_len = newline - p;
            
            // Handle CRLF (\r\n) case - if we found \r, check if next is \n
            if (*newline == '\r' && (newline + 1) < end && *(newline + 1) == '\n') {
                // This is \r\n, we'll process both together
                line_len = (newline + 1) - p;
            }
            
            // Copy line to buffer (safely)
            if (line_len < sizeof(uart->line_buffer)) {
                memcpy(uart->line_buffer, p, line_len);
                uart->line_buffer[line_len] = '\0';
                
                // Call all registered callbacks
                for (int i = 0; i < uart->callback_count; i++) {
                    uart->callbacks[i].callback(uart, uart->line_buffer,
                                                uart->callbacks[i].user_data);
                }
            }
            
            // Move past this line (handle both \r and \n)
            p = newline + 1;
            if (*(newline) == '\r' && p < end && *p == '\n') {
                p++; // Skip the \n after \r
            }
        }
        
        // Keep any remaining partial data
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
    char buf[256]={0};
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    uart_write(uart, buf, strlen(buf));
    uart_write(uart, "\r\n", 2);
}

/*============================================================================
 * UCI HELPERS (add these if not already present)
 *============================================================================*/
static int uci_get_string(const char *pkg, const char *section, const char *option, 
                          char *out, size_t out_len) {
    struct uci_context *ctx = uci_alloc_context();
    struct uci_ptr ptr;
    char path[256]={0};
    
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

static int uci_set_string(const char *pkg, const char *section, const char *option, 
                          const char *value) {
    struct uci_context *ctx = uci_alloc_context();
    struct uci_ptr ptr;
    char path[256]={0};
    
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
 * AP CONFIGURATION COMMANDS
 *============================================================================*/

/**
 * AT+WIFIAPCFG=<SSID>,<PASSWORD>,<SECURITY>
 * Removed STATE parameter - now only configures AP
 */
static void cmd_wifiapcfg_set(uart_inst_t *uart, const char *params) {
    char ssid[33] = "";
    char password[64] = "";
    char security[16] = "";
    
    // Parse parameters: SSID,PASSWORD,SECURITY
    int parsed = sscanf(params, "%32[^,],%63[^,],%15s", 
                        ssid, password, security);
    
    if (parsed != 3) {
        send_response(uart, "ERROR:1");  // INVALID_PARAM
        return;
    }
    
    // Validate SSID length
    if (strlen(ssid) < 1 || strlen(ssid) > 32) {
        send_response(uart, "ERROR:1");
        return;
    }
    
    // Validate security and map to UCI encryption
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
    
    // Set SSID
    if (uci_set_string("wireless", "@wifi-iface[0]", "ssid", ssid) < 0) {
        send_response(uart, "ERROR");
        return;
    }
    
    // Set encryption
    if (uci_set_string("wireless", "@wifi-iface[0]", "encryption", encryption) < 0) {
        send_response(uart, "ERROR");
        return;
    }
    
    // Set password (key) if not OPEN
    if (strcmp(security, "OPEN") != 0) {
        if (uci_set_string("wireless", "@wifi-iface[0]", "key", password) < 0) {
            send_response(uart, "ERROR");
            return;
        }
    }
    
    // Commit changes
    system("uci commit wireless");
    system("wifi");
    
    send_response(uart, "OK");
}

/**
 * AT+WIFIAPCFG? - Query AP configuration
 * Response: +WIFIAPCFG:SSID=<ssid>,SEC=<sec>,CLIENTS=<n>
 * Note: MODE field removed as requested
 */
static void cmd_wifiapcfg_query(uart_inst_t *uart) {
    char ssid[33] = "";
    char encryption[16] = "";
    int clients = 0;
    
    // Get SSID
    if (uci_get_string("wireless", "@wifi-iface[0]", "ssid", ssid, sizeof(ssid)) < 0) {
        strcpy(ssid, "OpenWrt");
    }
    
    // Get encryption
    if (uci_get_string("wireless", "@wifi-iface[0]", "encryption", encryption, sizeof(encryption)) < 0) {
        strcpy(encryption, "none");
    }
    
    // Map to security string
    const char *sec_str = "OPEN";
    if (strcmp(encryption, "psk") == 0) sec_str = "WPA";
    else if (strcmp(encryption, "psk2") == 0) sec_str = "WPA2";
    else if (strcmp(encryption, "psk-mixed") == 0) sec_str = "WPA_WPA2";
    
    // Get client count
    FILE *fp = popen("iw dev wlan0 station dump | grep -c 'Station' 2>/dev/null || echo 0", "r");
    char buf[16];
    if (fp && fgets(buf, sizeof(buf), fp)) {
        clients = atoi(buf);
        pclose(fp);
    }
    
    // Send response - MODE field removed as requested
    send_response(uart, "+WIFIAPCFG:SSID=%s,SEC=%s,CLIENTS=%d", 
                  ssid, sec_str, clients);
    send_response(uart, "OK");
}

/**
 * AT+WIFIMODE=<mode>
 * mode: 0=OFF, 1=STA, 2=AP, 3=AP+STA
 */
static void cmd_wifimode_set(uart_inst_t *uart, const char *param) {
    int mode = atoi(param);
    char command[512] = "";
    char buffer[256];
    FILE *fp;
    int ret;
    
    send_response(uart, "DEBUG: Starting WiFi mode set");
    
    // Validate mode
    if (mode < 0 || mode > 3) {
        send_response(uart, "ERROR: Invalid mode");
        return;
    }
    
    send_response(uart, "DEBUG: Mode=%d", mode);
    
    // Check current state
    fp = popen("uci get wireless.@wifi-iface[0].disabled 2>/dev/null || echo 'not set'", "r");
    if (fp && fgets(buffer, sizeof(buffer), fp)) {
        buffer[strcspn(buffer, "\n")] = 0;
        send_response(uart, "DEBUG: Current disabled='%s'", buffer);
        pclose(fp);
    }
    
    fp = popen("uci get wireless.@wifi-iface[0].mode 2>/dev/null || echo 'not set'", "r");
    if (fp && fgets(buffer, sizeof(buffer), fp)) {
        buffer[strcspn(buffer, "\n")] = 0;
        send_response(uart, "DEBUG: Current mode='%s'", buffer);
        pclose(fp);
    }
    
    // Build command with error output captured - FIXED SYNTAX
    if (mode == 2) { // AP only
        snprintf(command, sizeof(command),
            "(echo 'Step 1: Setting mode=ap'; "
            "uci set wireless.@wifi-iface[0].mode='ap' 2>&1; "
            "echo 'Step 2: Setting disabled=0'; "
            "uci set wireless.@wifi-iface[0].disabled='0' 2>&1; "
            "echo 'Step 3: Committing'; "
            "uci commit wireless 2>&1; "
            "echo 'Step 4: Restarting WiFi'; "
            "wifi 2>&1; "
            "echo 'Exit code: $?') 2>&1");
    } else if (mode == 0) { // OFF
        snprintf(command, sizeof(command),
            "(echo 'Step 1: Setting disabled=1'; "
            "uci set wireless.@wifi-iface[0].disabled='1' 2>&1; "
            "echo 'Step 2: Committing'; "
            "uci commit wireless 2>&1; "
            "echo 'Step 3: Restarting WiFi'; "
            "wifi 2>&1; "
            "echo 'Exit code: $?') 2>&1");
    } else {
        send_response(uart, "ERROR: Mode %d not supported", mode);
        return;
    }
    
    send_response(uart, "DEBUG: Executing command...");
    send_response(uart, "DEBUG: Command: %s", command);
    
    // Execute and capture all output
    fp = popen(command, "r");
    if (!fp) {
        send_response(uart, "ERROR: Failed to execute command");
        return;
    }
    
    while (fgets(buffer, sizeof(buffer), fp)) {
        buffer[strcspn(buffer, "\n")] = 0;
        send_response(uart, "CMD: %s", buffer);
    }
    
    ret = pclose(fp);
    send_response(uart, "DEBUG: Command exit status: %d", ret);
    
    // Verify final state
    fp = popen("uci get wireless.@wifi-iface[0].disabled 2>/dev/null", "r");
    if (fp && fgets(buffer, sizeof(buffer), fp)) {
        buffer[strcspn(buffer, "\n")] = 0;
        send_response(uart, "DEBUG: Final disabled='%s'", buffer);
        pclose(fp);
    }
    
    fp = popen("uci get wireless.@wifi-iface[0].mode 2>/dev/null", "r");
    if (fp && fgets(buffer, sizeof(buffer), fp)) {
        buffer[strcspn(buffer, "\n")] = 0;
        send_response(uart, "DEBUG: Final mode='%s'", buffer);
        pclose(fp);
    }
    
    // Check if WiFi processes are running
    fp = popen("ps | grep -E 'hostapd|wpa_supplicant' | grep -v grep", "r");
    send_response(uart, "DEBUG: WiFi processes:");
    while (fgets(buffer, sizeof(buffer), fp)) {
        buffer[strcspn(buffer, "\n")] = 0;
        send_response(uart, "  %s", buffer);
    }
    pclose(fp);
    
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
static void cmd_wifimode_query(uart_inst_t *uart) {
    char mode[8] = "";
    char disabled[8] = "";
    int current_mode = 0;
    
    // Check if interface is disabled
    uci_get_string("wireless", "@wifi-iface[0]", "disabled", disabled, sizeof(disabled));
    
    if (strcmp(disabled, "1") == 0) {
        current_mode = 0;  // OFF
    } else {
        // Get mode
        if (uci_get_string("wireless", "@wifi-iface[0]", "mode", mode, sizeof(mode)) == 0) {
            if (strcmp(mode, "ap") == 0) {
                current_mode = 2;  // AP only
            } else if (strcmp(mode, "sta") == 0) {
                current_mode = 1;  // STA only
            } else {
                current_mode = 2;  // Default to AP
            }
        } else {
            current_mode = 2;  // Default to AP
        }
    }
    
    send_response(uart, "+WIFIMODE:%d", current_mode);
    send_response(uart, "OK");
}

static void cmd_debug(uart_inst_t *uart) {
    char buffer[256];
    FILE *fp;
    
    send_response(uart, "=== WIRELESS DEBUG ===");
    
    // Show full wireless config
    fp = popen("uci show wireless", "r");
    while (fgets(buffer, sizeof(buffer), fp)) {
        buffer[strcspn(buffer, "\n")] = 0;
        send_response(uart, "%s", buffer);
    }
    pclose(fp);
    
    // Show interface status
    fp = popen("iw dev", "r");
    send_response(uart, "=== IW DEV ===");
    while (fgets(buffer, sizeof(buffer), fp)) {
        buffer[strcspn(buffer, "\n")] = 0;
        send_response(uart, "%s", buffer);
    }
    pclose(fp);
    
    // Check if hostapd is running
    fp = popen("ps | grep hostapd | grep -v grep", "r");
    send_response(uart, "=== HOSTAPD ===");
    while (fgets(buffer, sizeof(buffer), fp)) {
        buffer[strcspn(buffer, "\n")] = 0;
        send_response(uart, "%s", buffer);
    }
    pclose(fp);
    
    send_response(uart, "=== END ===");
    send_response(uart, "OK");
}

/**
 * AT+WIFIAP? - Get detailed list of connected clients
 * Response: +WIFIAP:CLIENT,<MAC>,<IP> for each client, then OK
 */
static void cmd_wifiap_clients(uart_inst_t *uart) {
    FILE *fp;
    char line[256];
    int client_count = 0;
    
    // Get list of connected stations from iw
    fp = popen("iw dev wlan0 station dump | grep Station | awk '{print $2}'", "r");
    if (!fp) {
        send_response(uart, "OK");  // Just OK if command fails
        return;
    }
    
    while (fgets(line, sizeof(line), fp)) {
        // Remove newline
        line[strcspn(line, "\n")] = 0;
        
        if (strlen(line) > 0) {
            char mac[18];
            strncpy(mac, line, sizeof(mac) - 1);
            mac[sizeof(mac) - 1] = '\0';
            
            // Try to get IP from DHCP leases
            char ip[16] = "0.0.0.0";
            FILE *leases = fopen("/tmp/dhcp.leases", "r");
            if (leases) {
                char lease_line[256];
                while (fgets(lease_line, sizeof(lease_line), leases)) {
                    if (strstr(lease_line, mac)) {
                        // Format: timestamp mac ip name *
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
    
#ifdef DEBUG
    printf("Reported %d connected clients\n", client_count);
#endif
}

/*============================================================================
 * STA / ETH / RESET / SAVE COMMANDS
 *  (added on top of basic and WiFi-AP/MODE commands)
 *============================================================================*/

/**
 * AT+WIFISTACFG=<SSID>,<PASSWORD>,<SECURITY>
 * SSID      : 1 to 32 characters
 * PASSWORD  : 8 to 63 characters (ignored if OPEN)
 * SECURITY  : OPEN | WPA | WPA2 | WPA_WPA2
 *
 * This configures the STA interface (assumed @wifi-iface[1]).
 */
static void cmd_wifista_cfg(uart_inst_t *uart, const char *params) {
    char ssid[33] = "";
    char password[64] = "";
    char security[16] = "";

    int parsed = sscanf(params, "%32[^,],%63[^,],%15s",
                        ssid, password, security);
    if (parsed != 3) {
        send_response(uart, "ERROR:1");  // INVALID_PARAM
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

    /* Configure STA iface: assume @wifi-iface[1] is STA */
    if (uci_set_string("wireless", "@wifi-iface[1]", "mode", "sta") < 0) {
        send_response(uart, "ERROR");
        return;
    }
    if (uci_set_string("wireless", "@wifi-iface[1]", "ssid", ssid) < 0) {
        send_response(uart, "ERROR");
        return;
    }
    if (uci_set_string("wireless", "@wifi-iface[1]", "encryption", encryption) < 0) {
        send_response(uart, "ERROR");
        return;
    }
    if (strcmp(security, "OPEN") != 0) {
        if (uci_set_string("wireless", "@wifi-iface[1]", "key", password) < 0) {
            send_response(uart, "ERROR");
            return;
        }
    }

    system("uci commit wireless");
    send_response(uart, "OK");
}

/**
 * AT+WIFISTA=1   Connect STA
 * AT+WIFISTA=0   Disconnect STA
 */
static void cmd_wifista_set(uart_inst_t *uart, const char *param) {
    int enable = atoi(param);
    char cmd[256];

    if (enable == 1) {
        snprintf(cmd, sizeof(cmd),
                 "uci set wireless.@wifi-iface[1].disabled='0'; "
                 "uci commit wireless; wifi");
    } else if (enable == 0) {
        snprintf(cmd, sizeof(cmd),
                 "uci set wireless.@wifi-iface[1].disabled='1'; "
                 "uci commit wireless; wifi");
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
 *
 * Connected:
 *  +WIFISTA:CONNECTED,IP=<ip>,RSSI=<rssi>,SEC=<security>
 *  OK
 *
 * Disconnected:
 *  +WIFISTA:DISCONNECTED,REASON=<reason>
 *  OK
 */
static void cmd_wifista_query(uart_inst_t *uart) {
    FILE *fp;
    char line[256];
    int connected = 0;

    /* Determine connection state using iw */
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

    /* Get IP address */
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

    /* Get RSSI */
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

    /* Get security */
    char encryption[16] = "";
    const char *sec_str = "OPEN";
    if (uci_get_string("wireless", "@wifi-iface[1]", "encryption",
                       encryption, sizeof(encryption)) == 0) {
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
 *
 * When UP:
 *  +ETH:UP,IP=<ip>,CLIENTS=<n>
 *  +ETH:CLIENT,<PORT>,<MAC>,<IP>
 *  ...
 *  OK
 *
 * When DOWN:
 *  +ETH:DOWN
 *  OK
 */
static void cmd_eth_query(uart_inst_t *uart) {
    FILE *fp;
    char line[256];

    /* Check link state via carrier */
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

    /* Get IP of eth0 */
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

    /* Collect DHCP clients from /tmp/dhcp.leases */
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
        /* We only have one physical port here, use PORT=1 */
        send_response(uart, "+ETH:CLIENT,1,%s,%s",
                      clients[i].mac, clients[i].ip);
    }

    send_response(uart, "OK");
}

/**
 * AT+RST - Software reset (Linux reboot)
 */
static void cmd_reset(uart_inst_t *uart) {
    send_response(uart, "OK");
    /* Reboot asynchronously so response can be transmitted */
    system("reboot &");
}

/**
 * AT+FACTORY - Factory reset and reboot
 */
static void cmd_factory(uart_inst_t *uart) {
    send_response(uart, "OK");
    /* Reset configuration to defaults and reboot */
    system("firstboot -y && reboot &");
}

/**
 * AT+SAVE - Save configuration
 */
static void cmd_save(uart_inst_t *uart) {
    int ret = system("uci commit");
    if (ret == 0) {
        send_response(uart, "OK");
    } else {
        send_response(uart, "ERROR");
    }
}

/*============================================================================
 * COMMAND HANDLER (simplified for testing)
 *============================================================================*/
static void command_callback(uart_inst_t *uart, const char *line, void *user_data) {
    (void)user_data;
    
    if (strlen(line) == 0) return;
    
    // Basic commands
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
    // WiFi Mode commands
    else if (strncmp(line, "AT+WIFIMODE=", 13) == 0) {
        cmd_wifimode_set(uart, line + 13);
    }
    else if (strcmp(line, "AT+WIFIMODE?") == 0) {
        cmd_wifimode_query(uart);
    }
    // AP Configuration commands
    else if (strncmp(line, "AT+WIFIAPCFG=", 14) == 0) {
        cmd_wifiapcfg_set(uart, line + 14);
    }
    else if (strcmp(line, "AT+WIFIAPCFG?") == 0) {
        cmd_wifiapcfg_query(uart);
    }
    // AP Client list
    else if (strcmp(line, "AT+WIFIAP?") == 0) {
        cmd_wifiap_clients(uart);
    }
    // STA configuration and control
    else if (strncmp(line, "AT+WIFISTACFG=", 14) == 0) {
        cmd_wifista_cfg(uart, line + 14);
    }
    else if (strncmp(line, "AT+WIFISTA=", 11) == 0) {
        cmd_wifista_set(uart, line + 11);
    }
    else if (strcmp(line, "AT+WIFISTA?") == 0) {
        cmd_wifista_query(uart);
    }
    // Ethernet status
    else if (strcmp(line, "AT+ETH?") == 0) {
        cmd_eth_query(uart);
    }
    // Save configuration
    else if (strcmp(line, "AT+SAVE") == 0) {
        cmd_save(uart);
    }
    // Add to command_callback:
    else if (strcmp(line, "AT+DEBUG") == 0) {
        cmd_debug(uart);
    }
    else {
        send_response(uart, "ERROR");
    }
}

/*============================================================================
 * MAIN FUNCTION
 *============================================================================*/
static volatile int g_running = 1;

static void sigint_handler(int sig) {
    (void)sig;
    g_running = 0;
}

int main(void) {
    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);
    
    uart_inst_t *uart = uart_init(RX_BUFFER_SIZE);
    if (!uart) {
        fprintf(stderr, "Failed to init UART\n");
        return 1;
    }
    
    if (uart_configure(uart, UART_DEVICE, UART_BAUD, UART_MODE) < 0) {
        fprintf(stderr, "Failed to configure UART\n");
        uart_destroy(uart);
        return 1;
    }
    
    if (uart_open(uart) < 0) {
        fprintf(stderr, "Failed to open UART %s\n", UART_DEVICE);
        uart_destroy(uart);
        return 1;
    }
    
    uart_register_callback(uart, "", command_callback, NULL);
    
    send_response(uart, "+SYS:BOOT,READY");
    
    fd_set read_fds;
    int max_fd = uart->fd;
    
    while (g_running) {
        FD_ZERO(&read_fds);
        FD_SET(uart->fd, &read_fds);
        
        struct timeval tv = {1, 0};
        
        int ret = select(max_fd + 1, &read_fds, NULL, NULL, &tv);
        if (ret < 0 && errno != EINTR) break;
        
        if (ret > 0 && FD_ISSET(uart->fd, &read_fds)) {
            uart_process_events(uart);
        }
    }
    
    uart_destroy(uart);
    return 0;
}