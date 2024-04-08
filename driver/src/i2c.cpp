#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>

#include "i2c.h"

static inline __s32 i2c_smbus_access(int file, char read_write, __u8 command,
                                     int size, union i2c_smbus_data *data)
{
    struct i2c_smbus_ioctl_data args;

    args.read_write = read_write;
    args.command = command;
    args.size = size;
    args.data = data;
    return ioctl(file, I2C_SMBUS, &args);
}

static inline __s32 i2c_smbus_read_byte_data(int file, __u8 command)
{
    union i2c_smbus_data data;
    if (i2c_smbus_access(file, I2C_SMBUS_READ, command,
                         I2C_SMBUS_BYTE_DATA, &data))
        return -1;
    else
        return 0x0FF & data.byte;
}

static inline __s32 i2c_smbus_write_byte_data(int file, __u8 command, __u8 value)
{
    union i2c_smbus_data data;
    data.byte = value;
    if (i2c_smbus_access(file, I2C_SMBUS_WRITE, command,
                         I2C_SMBUS_BYTE_DATA, &data))
        return -1;
    else
        return 0;
}

static inline __s32 i2c_smbus_read_word_data(int file, __u8 command)
{
    union i2c_smbus_data data;
    if (i2c_smbus_access(file, I2C_SMBUS_READ, command,
                         I2C_SMBUS_WORD_DATA, &data))
        return -1;
    else
        return 0x0FFFF & data.word;
}

static inline __s32 i2c_smbus_write_word_data(int file, __u8 command, __u16 value)
{
    union i2c_smbus_data data;
    data.word = value;
    if (i2c_smbus_access(file, I2C_SMBUS_WRITE, command,
                         I2C_SMBUS_WORD_DATA, &data))
        return -1;
    else
        return 0;
}

int i2c_open(const char *path, uint16_t addr)
{
    int fd;
    int rc;

    fd = open(path, O_RDWR);
    if (fd < 0)
    {
        printf("open %s failed.\n", path);
        rc = -1;
        return rc;
    }

    rc = ioctl(fd, I2C_SLAVE, addr);
    if (rc < 0)
    {
        rc = -2;
        close(fd);
        return rc;
    }

	// 尝试与设备通信
    int res = i2c_smbus_write_byte_data(fd, 0,0);
   	if (res < 0) {
        printf("Device not found at address 0x%X\n", addr);
		return -3;
    } else {
        printf("Device found at address 0x%X\n", addr);
    }

    return fd;

    
}

int i2c_read(int fd, uint8_t reg, uint8_t *value)
{
    int rc;
    rc = i2c_smbus_read_byte_data(fd, reg);
    if (rc < 0)
    {
        printf("i2c_read failed.\n");
        return -1;
    }
    *value = rc & 0xFF;
    return 0;
}

int i2c_write(int fd, uint8_t reg, uint8_t value)
{
    int rc;
    rc = i2c_smbus_write_byte_data(fd, reg, value);
    if (rc < 0)
    {
        printf("i2c_write failed.\n");
        return -1;
    }
    return 0;
}

void i2c_close(int fd)
{
    close(fd);
}

int i2c_read_word(int fd, uint8_t reg, uint16_t *value)
{
    int rc;
    rc = i2c_smbus_read_word_data(fd, reg);
    if (rc < 0)
    {
        printf("i2c_read failed.\n");
        return -1;
    }
    *value = rc & 0xFFFF;
    return 0;
}

int i2c_write_word(int fd, uint8_t reg, uint16_t value)
{
    int rc;
    rc = i2c_smbus_write_word_data(fd, reg, value);
    if (rc < 0)
    {
        printf("i2c_write failed.\n");
        return -1;
    }
    return 0;
}