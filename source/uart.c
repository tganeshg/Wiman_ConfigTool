/*****************************************************************
*
*	Copyright (c) 2026
*	All rights reserved.
*
*	Project 		:
*	Last Updated on	:
*	Author			: Ganesh
*
*	Revision History
****************************************************************
*	Date			Version		Name		Description
****************************************************************
*	07/02/2026		1.0			Ganesh		Initial Development
*
*****************************************************************/
#include "general.h"

/*============================================================================
 * MACROS
 *============================================================================*/
#define MAX_RETRY_CNT       3
#define READ_TIMEOUT_MS     100
#define DEFAULT_READ_WAIT   100000  // 100ms in microseconds

/*============================================================================
 * STATIC FUNCTIONS
 *============================================================================*/

/**
 * Convert integer baud rate to termios constant
 */
static speed_t uart_baud_to_const(int baudrate)
{
    switch(baudrate)
    {
        case 0:         return B0;
        case 50:        return B50;
        case 75:        return B75;
        case 110:       return B110;
        case 134:       return B134;
        case 150:       return B150;
        case 200:       return B200;
        case 300:       return B300;
        case 600:       return B600;
        case 1200:      return B1200;
        case 1800:      return B1800;
        case 2400:      return B2400;
        case 4800:      return B4800;
        case 9600:      return B9600;
        case 19200:     return B19200;
        case 38400:     return B38400;
        case 57600:     return B57600;
        case 115200:    return B115200;
        case 230400:    return B230400;
        case 460800:    return B460800;
        case 500000:    return B500000;
        case 576000:    return B576000;
        case 921600:    return B921600;
        case 1000000:   return B1000000;
        case 1152000:   return B1152000;
        case 1500000:   return B1500000;
        case 2000000:   return B2000000;
        case 2500000:   return B2500000;
        case 3000000:   return B3000000;
        case 3500000:   return B3500000;
        case 4000000:   return B4000000;
        default:        return RET_OK;
    }
}

/**
 * Parse mode string (e.g., "8N1", "7E2") into termios flags
 */
static int uart_parse_mode(const char *mode, tcflag_t *data_bits, 
                           tcflag_t *parity, tcflag_t *stop_bits, 
                           tcflag_t *iflags)
{
    if(!mode || strlen(mode) != 3)
        return RET_FAILURE;

    // Data bits
    switch(mode[0])
    {
        case '5': *data_bits = CS5; break;
        case '6': *data_bits = CS6; break;
        case '7': *data_bits = CS7; break;
        case '8': *data_bits = CS8; break;
        default: return RET_FAILURE;
    }

    // Parity
    switch(mode[1])
    {
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
            return RET_FAILURE;
    }

    // Stop bits
    switch(mode[2])
    {
        case '1': *stop_bits = 0; break;
        case '2': *stop_bits = CSTOPB; break;
        default: return RET_FAILURE;
    }

    return RET_OK;
}

/*============================================================================
 * PUBLIC FUNCTIONS
 *============================================================================*/

/**
 * Initialize UART instance with default values
 */
UART_INST* uart_init(size_t rx_buffer_size)
{
    UART_INST *uart = calloc(1, sizeof(UART_INST));
    if(!uart)
        return NULL;

    uart->fd = -1;
    uart->timeout_ms = READ_TIMEOUT_MS;
    uart->read_retry = MAX_RETRY_CNT;
    
    // Allocate receive buffer
    uart->rx_buffer = malloc(rx_buffer_size);
    if(!uart->rx_buffer)
    {
        free(uart);
        return NULL;
    }
    uart->rx_buffer_size = rx_buffer_size;
    
    return uart;
}

/**
 * Configure UART parameters
 */
int uart_configure(UART_INST *uart, const char *device, int baudrate, const char *mode)
{
    int ret;

    if(!uart || !device || !mode)
        return RET_FAILURE;

    // Store device path
    snprintf(uart->device, sizeof(uart->device), "/dev/%s", device);

    // Set baud rate
    uart->baud_rate = uart_baud_to_const(baudrate);
    if(uart->baud_rate == 0)
        return RET_FAILURE;

    // Parse mode
    ret = uart_parse_mode(mode, &uart->data_bits, &uart->parity, 
                          &uart->stop_bits, &uart->iflags);
    if(ret < 0)
        return RET_FAILURE;

    return RET_OK;
}

/**
 * Open UART device with configured parameters
 */
int uart_open(UART_INST *uart)
{
    int status=0;

    if(!uart || uart->fd >= 0)
        return RET_FAILURE;

    // Open device
    uart->fd = open(uart->device, O_RDWR | O_NOCTTY | O_NDELAY);
    if(uart->fd < 0)
        return RET_FAILURE;

    // Get current settings
    memset(&uart->tios, 0, sizeof(uart->tios));
    tcgetattr(uart->fd, &uart->tios);

    // Configure raw mode
    cfmakeraw(&uart->tios);

    // Set custom flags
    uart->tios.c_cflag = uart->data_bits | uart->parity | uart->stop_bits | 
                         CLOCAL | CREAD | HUPCL;
    uart->tios.c_iflag = uart->iflags;
    uart->tios.c_cc[VMIN] = 0;      // Non-blocking
    uart->tios.c_cc[VTIME] = uart->timeout_ms / 100; // Convert to deciseconds

    // Set baud rate
    cfsetispeed(&uart->tios, uart->baud_rate);
    cfsetospeed(&uart->tios, uart->baud_rate);

    // Apply settings
    tcflush(uart->fd, TCIFLUSH);
    if(tcsetattr(uart->fd, TCSANOW, &uart->tios) < 0) {
        close(uart->fd);
        uart->fd = -1;
        return RET_FAILURE;
    }

    // Set RTS/DTR (optional)
    if(ioctl(uart->fd, TIOCMGET, &status) == 0) {
        status |= (TIOCM_DTR | TIOCM_RTS);
        ioctl(uart->fd, TIOCMSET, &status);
    }

    return RET_OK;
}

/**
 * Write data to UART
 */
ssize_t uart_write(UART_INST *uart, const void *data, size_t len)
{
    ssize_t written=0;

    if(!uart || uart->fd < 0 || !data)
        return RET_FAILURE;

    written = write(uart->fd, data, len);

    if(written < 0)
        return RET_FAILURE;

    // Optional: Wait for data to be transmitted
    tcdrain(uart->fd);

    return written;
}

/**
 * Read data from UART with timeout
 */
ssize_t uart_read(UART_INST *uart, void *buffer, size_t buffer_len)
{
    fd_set read_fds;
    struct timeval tv;
    ssize_t total = 0;
    ssize_t n=0;
    int ret=0;

    if(!uart || uart->fd < 0 || !buffer)
        return RET_FAILURE;

    FD_ZERO(&read_fds);
    FD_SET(uart->fd, &read_fds);

    tv.tv_sec = uart->timeout_ms / 1000;
    tv.tv_usec = (uart->timeout_ms % 1000) * 1000;

    ret = select(uart->fd + 1, &read_fds, NULL, NULL, &tv);

    if(ret < 0)
        return RET_FAILURE;
    if(ret == 0)
        return RET_OK;  // Timeout

    // Data available, read it
    n = read(uart->fd, buffer, buffer_len);
    if(n < 0)
        return RET_FAILURE;

    return n;
}

/**
 * Read all available data into internal buffer
 */
ssize_t uart_read_all(UART_INST *uart)
{
    int bytes_avail;
    ssize_t total = 0;
    ssize_t n;
    
    if(!uart || uart->fd < 0)
        return RET_FAILURE;
    
    // Reset buffer
    uart->rx_data_len = 0;
    
    // Check how many bytes are available
    if(ioctl(uart->fd, FIONREAD, &bytes_avail) < 0)
        return RET_FAILURE;
    
    if(bytes_avail == 0)
        return RET_OK;
    
    // Limit to buffer size
    if(bytes_avail > uart->rx_buffer_size)
        bytes_avail = uart->rx_buffer_size;
    
    // Read all available data
    while(total < bytes_avail) {
        n = read(uart->fd, uart->rx_buffer + total, bytes_avail - total);
        if(n < 0) {
            if(errno == EAGAIN || errno == EWOULDBLOCK)
                break;
            return RET_FAILURE;
        }
        if(n == 0)
            break;
        
        total += n;
    }
    
    uart->rx_data_len = total;
    return total;
}

/**
 * Get pointer to received data
 */
unsigned char* uart_get_data(UART_INST *uart, size_t *len)
{
    if(!uart)
        return NULL;
    
    if(len)
        *len = uart->rx_data_len;
    
    return uart->rx_buffer;
}

/**
 * Flush UART buffers
 */
int uart_flush(UART_INST *uart)
{
    if(!uart || uart->fd < 0)
        return RET_FAILURE;
    
    return tcflush(uart->fd, TCIOFLUSH);
}

/**
 * Set UART line status (RTS/DTR)
 */
int uart_set_line(UART_INST *uart, int dtr, int rts)
{
    int status;
    
    if(!uart || uart->fd < 0)
        return RET_FAILURE;
    
    if(ioctl(uart->fd, TIOCMGET, &status) < 0)
        return RET_FAILURE;
    
    if(dtr >= 0) {
        if(dtr)
            status |= TIOCM_DTR;
        else
            status &= ~TIOCM_DTR;
    }
    
    if(rts >= 0) {
        if(rts)
            status |= TIOCM_RTS;
        else
            status &= ~TIOCM_RTS;
    }
    
    return ioctl(uart->fd, TIOCMSET, &status);
}

/**
 * Close UART
 */
int uart_close(UART_INST *uart)
{
    if(!uart || uart->fd < 0)
        return RET_FAILURE;
    
    tcflush(uart->fd, TCIOFLUSH);
    close(uart->fd);
    uart->fd = -1;
    
    return RET_OK;
}

/**
 * Destroy UART instance and free resources
 */
void uart_destroy(UART_INST *uart)
{
    if(!uart)
        return;
    
    if(uart->fd >= 0)
        uart_close(uart);
    
    if(uart->rx_buffer)
        free(uart->rx_buffer);
    
    free(uart);
}

/*============================================================================
 * EXAMPLE USAGE
 *============================================================================*/

#ifdef UART_TEST_MAIN

int main(void)
{
    UART_INST *uart;
    ssize_t len;
    unsigned char *data;
    
    // Initialize UART with 1KB buffer
    uart = uart_init(1024);
    if(!uart) {
        printf("Failed to initialize UART\n");
        return RET_FAILURE;
    }
    
    // Configure UART: ttyS1, 115200, 8N1
    if(uart_configure(uart, "ttyS1", 115200, "8N1") < 0) {
        printf("Failed to configure UART\n");
        uart_destroy(uart);
        return RET_FAILURE;
    }
    
    // Open UART
    if(uart_open(uart) < 0) {
        printf("Failed to open UART\n");
        uart_destroy(uart);
        return RET_FAILURE;
    }
    
    // Send AT command
    const char *cmd = "AT\r\n";
    len = uart_write(uart, cmd, strlen(cmd));
    printf("Sent %zd bytes\n", len);
    
    // Read response
    len = uart_read_all(uart);
    if(len > 0) {
        data = uart_get_data(uart, NULL);
        printf("Received %zd bytes: %s\n", len, data);
    }
    
    // Clean up
    uart_close(uart);
    uart_destroy(uart);
    
    return RET_OK;
}

// Use ttyS1 for GPS
UART_INST *gps = uart_init(512);
uart_configure(gps, "ttyS1", 9600, "8N1");
uart_open(gps);

// Use ttyS2 for modem
UART_INST *modem = uart_init(1024);
uart_configure(modem, "ttyS2", 115200, "8N1");
uart_open(modem);

// Read from GPS
uart_read_all(gps);
process_gps_data(uart_get_data(gps, NULL));

// Send to modem
uart_write(modem, "AT\r\n", 4);

// Clean up
uart_destroy(gps);
uart_destroy(modem);

#endif
/* EOF */