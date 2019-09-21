#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ipc.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm-generic/termbits.h>
#include <errno.h>
#define RCINPUT_MEASURE_INTERVAL_US 4700
/* define range mapping here, -+100% -> 1000..2000 */
#define SBUS_RANGE_MIN 200.0f
#define SBUS_RANGE_MAX 1800.0f
#define SBUS_TARGET_MIN 1000.0f
#define SBUS_TARGET_MAX 2000.0f
/* pre-calculate the floating point stuff as far as possible at compile time */
#define SBUS_SCALE_FACTOR ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN))
#define SBUS_SCALE_OFFSET (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f))

int      device_fd;                /** serial port device to read SBUS; */
int      channels_data[16]; /** 16 channels support; */
uint8_t  buffer[25];
char     device[30] = "/dev/ttyUSB0";
uint8_t  sbusData[25];
uint8_t  sbusData_int[25] = { 0x0f, 0x01, 0x04, 0x20, 0x00, 0xff, 0x07, 0x40, 0x00, 0x02, 0x10, 
           0x80, 0x2c, 0x64, 0x21, 0x0b, 0x59, 0x08, 0x40, 0x00, 0x02, 0x10, 0x80, 0x00, 0x00 };

int init()
{
    /* open the serial port */
    device_fd = open(device, O_RDWR | O_NONBLOCK | O_CLOEXEC);

    if (-1 == device_fd) {
        printf("Open SBUS input %s failed, status %d \n", device, (int) device_fd);
        fflush(stdout);
        return -1;
    }

    struct termios2 tio;
    if (0 != ioctl(device_fd, TCGETS2, &tio)) {
        close(device_fd);
        device_fd = -1;
        return -1;
    }

    /* Setting serial port,8E2, non-blocking.100Kbps */
    tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tio.c_iflag |= (INPCK | IGNPAR);
    tio.c_oflag &= ~OPOST;
    tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tio.c_cflag &= ~(CSIZE | CRTSCTS | PARODD | CBAUD);
    /* use BOTHER to specify speed directly in c_[io]speed member */
    tio.c_cflag |= (CS8 | CSTOPB | CLOCAL | PARENB | BOTHER | CREAD);
    tio.c_ispeed = 100000;
    tio.c_ospeed = 100000;
    tio.c_cc[VMIN] = 25;
    tio.c_cc[VTIME] = 0;

    if (0 != ioctl(device_fd, TCSETS2, &tio)) {
        close(device_fd);
        device_fd = -1;
        return -1;
    }

    printf("Open SBUS input %s worked, status %d \n", device, (int) device_fd);
    return 0;
}

void run(void)
{
    int nread;
    int count_bad = 0;
    int wait = 0;
    while (1) {
        nread = read(device_fd, &sbusData, sizeof(sbusData));
        //good packet
        if (25 == nread){
            if (0x0f == sbusData[0] && 0x00 == sbusData[24]) { break; } 
            else { ++count_bad; break; }
        }
        ++wait;
        if(wait > 100){
            printf("no good packet in 500ms port closing\n");
            close(device_fd);
            exit(1);
        }
        usleep(5000);
    }

    /* parse sbus data to pwm */
    channels_data[0]  = (uint16_t)(((sbusData[1]       | sbusData[2]  << 8) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
    channels_data[1]  = (uint16_t)(((sbusData[2]  >> 3 | sbusData[3]  << 5) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
    channels_data[2]  = (uint16_t)(((sbusData[3]  >> 6 | sbusData[4]  << 2 | sbusData[5] << 10) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
    channels_data[3]  = (uint16_t)(((sbusData[5]  >> 1 | sbusData[6]  << 7) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
    channels_data[4]  = (uint16_t)(((sbusData[6]  >> 4 | sbusData[7]  << 4) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
    channels_data[5]  = (uint16_t)(((sbusData[7]  >> 7 | sbusData[8]  << 1 | sbusData[9] << 9) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET; 
    channels_data[6]  = (uint16_t)(((sbusData[9]  >> 2 | sbusData[10] << 6) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
    channels_data[7]  = (uint16_t)(((sbusData[10] >> 5 | sbusData[11] << 3) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET; 
    // & the other 8 + 2 channels if you need them
    channels_data[8]  = (uint16_t)(((sbusData[12]      | sbusData[13] << 8) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
    channels_data[9]  = (uint16_t)(((sbusData[13] >> 3 | sbusData[14] << 5) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
    channels_data[10] = (uint16_t)(((sbusData[14] >> 6 | sbusData[15] << 2 | sbusData[16] << 10) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
    channels_data[11] = (uint16_t)(((sbusData[16] >> 1 | sbusData[17] << 7) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
    channels_data[12] = (uint16_t)(((sbusData[17] >> 4 | sbusData[18] << 4) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
    channels_data[13] = (uint16_t)(((sbusData[18] >> 7 | sbusData[19] << 1 | sbusData[20] << 9) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
    channels_data[14] = (uint16_t)(((sbusData[20] >> 2 | sbusData[21] << 6) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
    channels_data[15] = (uint16_t)(((sbusData[21] >> 5 | sbusData[22] << 3) & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;

    printf("%3dms(wait) %3d(bad)       ch1 =%5d   ch2 =%5d   ch3 =%5d   ch4 =%5d   ch5 =%5d   ch6 =%5d   ch7 =%5d   ch8 =%5d\n", 5*wait, count_bad, 
         channels_data[0], channels_data[1], channels_data[2], channels_data[3], channels_data[4], channels_data[5], channels_data[6], channels_data[7] );     
}

int main(int argc, char **argv)
{
    init();
    
    write(device_fd, &sbusData_int, sizeof(sbusData_int));
    run();

    write(device_fd, &sbusData_int, sizeof(sbusData_int));
    usleep(5000);
    run();

    run();

    return 0;
}
