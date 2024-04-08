#ifndef SRC_I2C_H_
#define SRC_I2C_H_

#include <stdint.h>

int i2c_open(const char *path, uint16_t addr);
int i2c_read(int fd, uint8_t reg, uint8_t *value);
int i2c_write(int fd, uint8_t reg, uint8_t value);
int i2c_read_word(int fd, uint8_t reg, uint16_t *value);
int i2c_write_word(int fd, uint8_t reg, uint16_t value);
void i2c_close(int fd);

#endif /* SRC_I2C_H_ */