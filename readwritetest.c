#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <time.h>

// 手动定义缺失的宏（确保兼容性）
#ifndef TIOCSRS485
#define TIOCSRS485      0x542F
#endif

#ifndef SER_RS485_ENABLED
#define SER_RS485_ENABLED      (1 << 0)
#define SER_RS485_RTS_ON_SEND  (1 << 1)
#define SER_RS485_RTS_AFTER_SEND (1 << 2)
#endif

#ifndef SER_RS422_ENABLED
#define SER_RS422_ENABLED      (1 << 5)
#endif

#ifndef SER_RS232_ENABLED
#define SER_RS232_ENABLED      (1 << 6)
#endif

// 要发送的固定数据（十六进制格式）
const unsigned char send_data[] = {0x01, 0x03, 0x05, 0xE4, 0x00, 0x02, 0x85, 0xB8};
const int send_data_len = sizeof(send_data);

// 接口类型定义
typedef enum {
    CM_232 = 0,
    CM_485,
    CM_422
} InterfaceType;

// 设置串口模式
static int SetPortIntertype(char* devname, InterfaceType type) {
    int fd = -1;
    struct serial_rs485 rs485conf = {0};

    if ((fd = open(devname, O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0) {
        perror("open");
        return -1;
    }

    switch (type) {
        case CM_485:
            rs485conf.flags |= SER_RS485_ENABLED;
            rs485conf.flags |= SER_RS485_RTS_ON_SEND;
            rs485conf.flags &= ~SER_RS485_RTS_AFTER_SEND;
            rs485conf.delay_rts_before_send = 1;
            rs485conf.delay_rts_after_send = 0;
            break;
        case CM_422:
            rs485conf.flags |= SER_RS422_ENABLED;
            break;
        default: // CM_232
            rs485conf.flags |= SER_RS232_ENABLED;
    }

    if (ioctl(fd, TIOCSRS485, &rs485conf) < 0) {
        perror("ioctl TIOCSRS485");
        close(fd);
        return -1;
    }

    close(fd);
    return 0;
}

// 设置串口参数
int SetComCfg(int fd, char *configs) {
    struct termios options;
    bzero(&options, sizeof(options));

    // 解析配置字符串（格式："波特率,数据位,停止位,校验位,流控"）
    int baud = 9600, databits = 8, stopbits = 1, ctsrts = 0;
    char parity = 'N';
    sscanf(configs, "%d,%d,%d,%c,%d", &baud, &databits, &stopbits, &parity, &ctsrts);

    // 设置波特率
    speed_t speed;
    switch (baud) {
        case 9600:   speed = B9600; break;
        case 115200: speed = B115200; break;
        default:
            fprintf(stderr, "Unsupported baud rate\n");
            return -1;
    }
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    // 设置数据位
    options.c_cflag &= ~CSIZE;
    switch (databits) {
        case 5: options.c_cflag |= CS5; break;
        case 6: options.c_cflag |= CS6; break;
        case 7: options.c_cflag |= CS7; break;
        case 8: options.c_cflag |= CS8; break;
        default:
            fprintf(stderr, "Unsupported data bits\n");
            return -1;
    }

    // 设置停止位
    if (stopbits == 2) options.c_cflag |= CSTOPB;

    // 设置校验位
    switch (parity) {
        case 'n':
        case 'N':
            options.c_iflag = IGNPAR;
            break;
        case 'e':
        case 'E':
            options.c_cflag |= PARENB;
            break;
        case 'o':
        case 'O':
            options.c_cflag |= (PARENB | PARODD);
            break;
        default:
            fprintf(stderr, "Unsupported parity\n");
            return -1;
    }

    // 设置流控
    if (ctsrts) options.c_cflag |= CRTSCTS;

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_lflag = 0;
    options.c_oflag = 0;
    options.c_cc[VMIN] = 1;
    options.c_cc[VTIME] = 0;

    tcflush(fd, TCIOFLUSH);
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        perror("tcsetattr");
        return -1;
    }

    return 0;
}

int main() {
    char *dev = "/dev/ttyS1";
    char *config = "9600,8,1,N,0"; // RS485模式配置
    int fd, ret;
    int i; // C89兼容的循环变量声明

    // 设置串口模式为RS485
    ret = SetPortIntertype(dev, CM_485);
    if (ret != 0) {
        fprintf(stderr, "Failed to set port type to RS485\n");
        return -1;
    }

    // 打开串口
    fd = open(dev, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror("open");
        return -1;
    }

    // 配置串口参数
    ret = SetComCfg(fd, config);
    if (ret != 0) {
        fprintf(stderr, "Failed to set serial port configuration\n");
        close(fd);
        return -1;
    }

    printf("Serial port %s opened in RS485 mode, config: %s\n", dev, config);

    fd_set read_fds;
    struct timeval timeout;
    unsigned char recv_buf[256];
    unsigned int recv_len;
    time_t last_write_time = 0;

    while (1) {
        FD_ZERO(&read_fds);
        FD_SET(fd, &read_fds);
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        ret = select(fd + 1, &read_fds, NULL, NULL, &timeout);
        if (ret < 0) {
            perror("select");
            break;
        }

        // 处理接收数据
        if (ret > 0 && FD_ISSET(fd, &read_fds)) {
            recv_len = read(fd, recv_buf, sizeof(recv_buf));
            if (recv_len > 0) {
                printf("Received %d bytes: ", recv_len);
                for (i = 0; i < recv_len; i++) {
                    printf("%02X ", recv_buf[i]);
                }
                printf("\n");
            }
        }

        // 每2秒发送一次数据
        time_t now = time(NULL);
        if (now - last_write_time >= 2) {
            ret = write(fd, send_data, send_data_len);
            if (ret == send_data_len) {
                printf("Sent %d bytes: ", ret);
                for (i = 0; i < send_data_len; i++) {
                    printf("%02X ", send_data[i]);
                }
                printf("\n");
            } else {
                perror("write");
            }
            last_write_time = now;
        }
    }

    close(fd);
    return 0;
}
