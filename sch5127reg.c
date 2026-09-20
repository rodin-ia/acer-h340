#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>

#define HW_INDEX_PORT 0x0870
#define HW_DATA_PORT  0x0871

static int read_reg(int fd, uint8_t reg, uint8_t *value)
{
    ssize_t n;

    /*
     * SCH5127 uses an indexed ISA register interface:
     *   0x0870 = register index
     *   0x0871 = register data
     */
    n = pwrite(fd, &reg, 1, HW_INDEX_PORT);
    if (n != 1)
        return -1;

    n = pread(fd, value, 1, HW_DATA_PORT);
    if (n != 1)
        return -1;

    return 0;
}

static int sign_extend_12(uint16_t value)
{
    value &= 0x0fff;

    if (value & 0x0800)
        return (int)value - 0x1000;

    return (int)value;
}

static int temp_mc_from_raw12(uint16_t raw12)
{
    int value = sign_extend_12(raw12);

    return (value * 1000) / 16;
}

static void print_temperature(const char *name,
                              uint8_t msb,
                              unsigned lsb)
{
    uint16_t raw12 = ((uint16_t)msb << 4) | (lsb & 0x0f);
    int mc = temp_mc_from_raw12(raw12);
    int integer = mc / 1000;
    int fraction = abs(mc % 1000);

    printf("%-10s raw=0x%03X  %d.%03d C\n",
           name,
           raw12,
           integer,
           fraction);
}

static unsigned pwm_min_percent(uint8_t value)
{
    static const unsigned percent[8] = {
        0, 20, 25, 30, 35, 40, 45, 50
    };

    return percent[value & 0x07];
}

static const char *pwm_mode(uint8_t value)
{
    static const char *modes[8] = {
        "zone1 auto",
        "zone2 auto",
        "zone3 auto",
        "full on",
        "disabled",
        "hottest zone2/3 auto",
        "hottest zone1/2/3 auto",
        "manual"
    };

    return modes[(value >> 5) & 0x07];
}

static const char *pwm_freq_from_code(unsigned code)
{
    static const char *freq[16] = {
        "11 Hz",
        "15 Hz",
        "22 Hz",
        "29 Hz",
        "35 Hz",
        "44 Hz",
        "59 Hz",
        "88 Hz",
        "15 kHz",
        "20 kHz",
        "30 kHz",
        "25 kHz",
        "reserved",
        "reserved",
        "reserved",
        "reserved"
    };

    return freq[code & 0x0f];
}

static unsigned temp_range_mc_from_code(unsigned code)
{
    static const unsigned range_mc[16] = {
         2000,
         2500,
         3333,
         4000,
         5000,
         6666,
         8000,
        10000,
        13333,
        16000,
        20000,
        26666,
        32000,
        40000,
        53333,
        80000
    };

    return range_mc[code & 0x0f];
}

static void print_reg(uint8_t reg, uint8_t value)
{
    const char *name = "";

    switch (reg) {
    case 0x25:
        name = "Remote diode 1 temperature MSB";
        break;
    case 0x26:
        name = "Internal diode temperature MSB";
        break;
    case 0x27:
        name = "Remote diode 2 temperature MSB";
        break;

    case 0x28:
        name = "Fan1 tach LSB";
        break;
    case 0x29:
        name = "Fan1 tach MSB";
        break;
    case 0x2a:
        name = "Fan2 tach LSB";
        break;
    case 0x2b:
        name = "Fan2 tach MSB";
        break;
    case 0x2c:
        name = "Fan3 tach LSB";
        break;
    case 0x2d:
        name = "Fan3 tach MSB";
        break;

    case 0x30:
        name = "PWM1 current duty";
        break;
    case 0x31:
        name = "PWM2 current duty";
        break;
    case 0x32:
        name = "PWM3 current duty";
        break;

    case 0x3d:
        name = "Device ID";
        break;
    case 0x3e:
        name = "Company ID";
        break;
    case 0x3f:
        name = "Revision";
        break;
    case 0x40:
        name = "Configuration";
        break;

    case 0x5c:
        name = "PWM1 configuration";
        break;
    case 0x5d:
        name = "PWM2 configuration";
        break;
    case 0x5e:
        name = "PWM3 configuration";
        break;

    case 0x5f:
        name = "Zone1 range / PWM1 frequency";
        break;
    case 0x60:
        name = "PWM2 frequency";
        break;
    case 0x61:
        name = "Zone3 range / PWM3 frequency";
        break;

    case 0x62:
        name = "PWM1 ramp rate / OFF1";
        break;
    case 0x63:
        name = "PWM2/PWM3 ramp rate / OFF2/OFF3";
        break;

    case 0x64:
        name = "PWM1 minimum duty";
        break;
    case 0x65:
        name = "PWM2 minimum duty";
        break;
    case 0x66:
        name = "PWM3 minimum duty";
        break;

    case 0x67:
        name = "Zone1 low temperature";
        break;
    case 0x68:
        name = "Reserved / zone3 low temperature";
        break;
    case 0x69:
        name = "Zone2 low temperature";
        break;

    case 0x6a:
        name = "Zone1 absolute temperature";
        break;
    case 0x6b:
        name = "Reserved / zone3 absolute temperature";
        break;
    case 0x6c:
        name = "Zone2 absolute temperature";
        break;

    case 0x6d:
        name = "MCHP test register";
        break;
    case 0x6e:
        name = "MCHP test register";
        break;

    case 0x7c:
        name = "Special function";
        break;
    case 0x7d:
        name = "MUX control";
        break;
    case 0x7e:
        name = "Interrupt enable 1";
        break;
    case 0x7f:
        name = "Configuration";
        break;
    case 0x80:
        name = "Interrupt enable 2";
        break;
    case 0x81:
        name = "Tach/PWM association";
        break;

    case 0x84:
        name = "A/D LSB: IN5/IN6";
        break;
    case 0x85:
        name = "A/D LSB: TEMP3/TEMP1";
        break;
    case 0x86:
        name = "A/D LSB: IN4/TEMP2";
        break;
    case 0x87:
        name = "A/D LSB: IN3/IN0";
        break;
    case 0x88:
        name = "A/D LSB: IN2/IN1";
        break;
    case 0x89:
        name = "A/D LSB: reserved/IN7";
        break;

    case 0x90:
        name = "Fan1 options";
        break;
    case 0x91:
        name = "Fan2 options";
        break;
    case 0x92:
        name = "Fan3 options";
        break;

    case 0x94:
        name = "PWM1 options";
        break;
    case 0x95:
        name = "PWM2 options";
        break;
    case 0x96:
        name = "PWM3 options";
        break;
    }

    printf("0x%02X  0x%02X  %3u  %-40s",
           reg,
           value,
           value,
           name);

    switch (reg) {
    case 0x30:
    case 0x31:
    case 0x32:
        printf(" duty=%u.%u%%",
               (value * 100) / 255,
               ((value * 1000) / 255) % 10);
        break;

    case 0x5c:
    case 0x5d:
    case 0x5e:
        printf(" mode=%s",
               pwm_mode(value));
        break;

    case 0x5f:
    case 0x60:
    case 0x61: {
        unsigned range_code = (value >> 4) & 0x0f;
        unsigned range = temp_range_mc_from_code(range_code);

        printf(" range=%u.%03u C freq=%s",
               range / 1000,
               range % 1000,
               pwm_freq_from_code(value & 0x0f));
        break;
    }

    case 0x62:
        printf(" RR1E=%u rate_code=%u OFF1=%u",
               (value >> 3) & 1,
               value & 0x07,
               (value >> 5) & 1);
        break;

    case 0x63:
        printf(" RR2E=%u rate2=%u RR3E=%u rate3=%u",
               (value >> 7) & 1,
               (value >> 4) & 7,
               (value >> 3) & 1,
               value & 7);
        break;

    case 0x64:
    case 0x65:
    case 0x66:
        printf(" code=%u minimum=%u%%",
               value & 7,
               pwm_min_percent(value));
        break;

    case 0x67:
    case 0x68:
    case 0x69:
    case 0x6a:
    case 0x6b:
    case 0x6c:
        printf(" signed=%d C",
               (int8_t)value);
        break;

    case 0x7c:
        printf(" MONMD=%u INT_EN=%u",
               (value >> 1) & 1,
               (value >> 2) & 1);
        break;

    case 0x7d:
        printf(" zone1_sel=%u zone2_sel=%u adjust=%s",
               (value >> 2) & 3,
               (value >> 1) & 1,
               (value & 0x10) ? "-128" : "-64");
        break;

    case 0x7f:
        printf(" INIT=%u SUREN=%u TRDY=%u",
               (value >> 7) & 1,
               (value >> 4) & 1,
               (value >> 3) & 1);
        break;

    case 0x81:
        printf(" tach1=%u tach2=%u tach3=%u",
               (value & 3) + 1,
               ((value >> 2) & 3) + 1,
               ((value >> 4) & 3) + 1);
        break;

    case 0x85:
        printf(" TEMP3_LSB=0x%X TEMP1_LSB=0x%X",
               (value >> 4) & 0x0f,
               value & 0x0f);
        break;

    case 0x86:
        printf(" IN4_LSB=0x%X TEMP2_LSB=0x%X",
               (value >> 4) & 0x0f,
               value & 0x0f);
        break;

    default:
        break;
    }

    putchar('\n');
}

static int parse_reg(const char *s, unsigned *out)
{
    char *end;
    unsigned long value;

    errno = 0;

    value = strtoul(s, &end, 0);

    if (errno != 0 || *end != '\0' || value > 0xff)
        return -1;

    *out = (unsigned)value;

    return 0;
}

static int read_temperatures(int fd,
                             uint8_t *temp1_msb,
                             uint8_t *temp2_msb,
                             uint8_t *temp3_msb,
                             uint8_t *lsb85,
                             uint8_t *lsb86)
{
    /*
     * The SCH5127 latches the temperature LSBs when the MSB
     * registers are read. Therefore read all MSBs first.
     */
    if (read_reg(fd, 0x25, temp1_msb) < 0)
        return -1;

    if (read_reg(fd, 0x26, temp2_msb) < 0)
        return -1;

    if (read_reg(fd, 0x27, temp3_msb) < 0)
        return -1;

    if (read_reg(fd, 0x85, lsb85) < 0)
        return -1;

    if (read_reg(fd, 0x86, lsb86) < 0)
        return -1;

    return 0;
}

static void print_temperatures(int fd)
{
    uint8_t temp1_msb;
    uint8_t temp2_msb;
    uint8_t temp3_msb;
    uint8_t lsb85;
    uint8_t lsb86;

    printf("\n=== TEMPERATURES ===\n");

    if (read_temperatures(fd,
                          &temp1_msb,
                          &temp2_msb,
                          &temp3_msb,
                          &lsb85,
                          &lsb86) < 0) {
        fprintf(stderr,
                "temperature read failed: %s\n",
                strerror(errno));
        return;
    }

    print_temperature("temp1 RD1",
                      temp1_msb,
                      lsb85 & 0x0f);

    print_temperature("temp2 AMBIENT",
                      temp2_msb,
                      lsb86 & 0x0f);

    print_temperature("temp3 RD2",
                      temp3_msb,
                      (lsb85 >> 4) & 0x0f);
}

static int read_fan_tach(int fd, uint8_t reg, unsigned *rpm)
{
    uint8_t lsb;
    uint8_t msb;
    uint16_t raw;

    /*
     * Fan tach registers must be read LSB first.
     */
    if (read_reg(fd, reg, &lsb) < 0)
        return -1;

    if (read_reg(fd, reg + 1, &msb) < 0)
        return -1;

    raw = ((uint16_t)msb << 8) | lsb;

    if (raw == 0 || raw == 0xffff) {
        *rpm = 0;
        return 0;
    }

    /*
     * SCH5127 tachometer clock:
     * 90 kHz, value = periods per revolution.
     *
     * RPM = 90000 * 60 / raw.
     */
    *rpm = (90000U * 60U) / raw;

    return 0;
}

static void print_fans(int fd)
{
    unsigned fan1;
    unsigned fan2;
    unsigned fan3;

    printf("\n=== FAN TACHOMETERS ===\n");

    if (read_fan_tach(fd, 0x28, &fan1) < 0 ||
        read_fan_tach(fd, 0x2a, &fan2) < 0 ||
        read_fan_tach(fd, 0x2c, &fan3) < 0) {
        fprintf(stderr,
                "fan tach read failed: %s\n",
                strerror(errno));
        return;
    }

    printf("fan1: %u RPM\n", fan1);
    printf("fan2: %u RPM\n", fan2);
    printf("fan3: %u RPM\n", fan3);
}

static int read_fan_curve(int fd,
                          int *current_temp_mc,
                          unsigned *current_pwm,
                          unsigned *min_percent,
                          int *point1_c,
                          int *point2_c,
                          unsigned *range_c,
                          const char **mode,
                          const char **freq,
                          unsigned *ramp_enable,
                          unsigned *ramp_code,
                          unsigned *off1)
{
    uint8_t temp1_msb;
    uint8_t temp2_msb;
    uint8_t temp3_msb;
    uint8_t lsb85;
    uint8_t lsb86;

    uint8_t pwm1;
    uint8_t cfg1;
    uint8_t min1;
    uint8_t freq1;
    uint8_t ramp1;

    unsigned range_mc;
    uint16_t temp_raw;

    if (read_temperatures(fd,
                          &temp1_msb,
                          &temp2_msb,
                          &temp3_msb,
                          &lsb85,
                          &lsb86) < 0)
        return -1;

    temp_raw = ((uint16_t)temp1_msb << 4) |
               (lsb85 & 0x0f);

    *current_temp_mc = temp_mc_from_raw12(temp_raw);

    if (read_reg(fd, 0x30, &pwm1) < 0 ||
        read_reg(fd, 0x5c, &cfg1) < 0 ||
        read_reg(fd, 0x64, &min1) < 0 ||
        read_reg(fd, 0x5f, &freq1) < 0 ||
        read_reg(fd, 0x62, &ramp1) < 0)
        return -1;

    range_mc = temp_range_mc_from_code((freq1 >> 4) & 0x0f);

    *current_pwm = pwm1;
    *min_percent = pwm_min_percent(min1);
    *point1_c = (int8_t)0x00;

    /*
     * Zone1 low temperature comes from 67h.
     */
    {
        uint8_t point1;
        uint8_t point2;

        if (read_reg(fd, 0x67, &point1) < 0 ||
            read_reg(fd, 0x6a, &point2) < 0)
            return -1;

        *point1_c = (int8_t)point1;
        *point2_c = *point1_c + (int)(range_mc / 1000);

        /*
         * point2 is the programmed absolute limit as well on H340.
         * Keep the actual programmed 6Ah value visible if different.
         */
        if ((int8_t)point2 != *point2_c)
            *point2_c = (int8_t)point2;
    }

    *range_c = range_mc / 1000;
    *mode = pwm_mode(cfg1);
    *freq = pwm_freq_from_code(freq1 & 0x0f);
    *ramp_enable = (ramp1 >> 3) & 1;
    *ramp_code = ramp1 & 7;
    *off1 = (ramp1 >> 5) & 1;

    return 0;
}

static void print_pwm_summary(int fd)
{
    int temp_mc;
    unsigned current_pwm;
    unsigned min_percent;
    int point1_c;
    int point2_c;
    unsigned range_c;
    const char *mode;
    const char *freq;
    unsigned ramp_enable;
    unsigned ramp_code;
    unsigned off1;

    printf("\n=== PWM1 FAN CURVE ===\n");

    if (read_fan_curve(fd,
                       &temp_mc,
                       &current_pwm,
                       &min_percent,
                       &point1_c,
                       &point2_c,
                       &range_c,
                       &mode,
                       &freq,
                       &ramp_enable,
                       &ramp_code,
                       &off1) < 0) {
        fprintf(stderr,
                "PWM1 read failed: %s\n",
                strerror(errno));
        return;
    }

    printf("MODE:     %s\n", mode);
    printf("FREQ:     %s\n", freq);
    printf("LOW:      %d C -> %u%% (%u/255)\n",
           point1_c,
           min_percent,
           (min_percent * 255U + 50U) / 100U);
    printf("HIGH:     %d C -> 100%% (255/255)\n",
           point1_c + (int)range_c);

    if (point2_c != point1_c + (int)range_c) {
        printf("ABSOLUTE: %d C\n",
               point2_c);
    } else {
        printf("ABSOLUTE: %d C\n",
               point2_c);
    }

    printf("HYST:     n/a (SCH5127)\n");

    printf("RAMP:     %s",
           ramp_enable ? "enabled" : "disabled");

    if (ramp_enable)
        printf(" code=%u", ramp_code);

    putchar('\n');

    printf("BELOW LOW: minimum/off hardware mode, OFF1=%u\n",
           off1);

    printf("NOW:      %.3f C -> PWM=%u/%u (%.1f%%)\n",
           temp_mc / 1000.0,
           current_pwm,
           255U,
           current_pwm * 100.0 / 255.0);
}

static const uint8_t interesting[] = {
    0x25, 0x26, 0x27,

    0x28, 0x29,
    0x2a, 0x2b,
    0x2c, 0x2d,

    0x30, 0x31, 0x32,

    0x3d, 0x3e, 0x3f, 0x40,

    0x5c, 0x5d, 0x5e,
    0x5f, 0x60, 0x61,
    0x62, 0x63,
    0x64, 0x65, 0x66,

    0x67, 0x68, 0x69,
    0x6a, 0x6b, 0x6c,

    0x6d, 0x6e,

    0x7c, 0x7d, 0x7e, 0x7f,
    0x80, 0x81,

    0x84, 0x85, 0x86, 0x87, 0x88, 0x89,

    0x90, 0x91, 0x92,
    0x94, 0x95, 0x96
};

static void usage(const char *prog)
{
    fprintf(stderr,
            "Usage:\n"
            "  %s             Dump fan-control registers and summary\n"
            "  %s REG         Read one register\n"
            "  %s FROM TO     Read register range\n"
            "\n"
            "REG accepts decimal or hexadecimal (0x..).\n",
            prog,
            prog,
            prog);
}

int main(int argc, char **argv)
{
    int fd;

    fd = open("/dev/port", O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        fprintf(stderr,
                "open /dev/port failed: %s\n",
                strerror(errno));
        return 1;
    }

    if (argc == 1) {
        size_t i;

        printf("SCH5127 hardware monitor @ I/O 0x%04X\n",
               HW_INDEX_PORT);

        printf("\n=== REGISTERS ===\n");
        printf("REG   HEX   DEC  NAME\n");
        printf("----  ----  ---  ----------------------------------------\n");

        for (i = 0; i < sizeof(interesting); i++) {
            uint8_t value;

            if (read_reg(fd, interesting[i], &value) < 0) {
                fprintf(stderr,
                        "read 0x%02X failed: %s\n",
                        interesting[i],
                        strerror(errno));
                close(fd);
                return 1;
            }

            print_reg(interesting[i], value);
        }

        print_temperatures(fd);
        print_fans(fd);
        print_pwm_summary(fd);

    } else {
        unsigned from;
        unsigned to;

        if (argc == 2 &&
            !strcasecmp(argv[1], "all")) {

            from = 0x00;
            to = 0xff;

        } else if (parse_reg(argv[1], &from) < 0) {

            usage(argv[0]);
            close(fd);
            return 2;

        } else {

            to = from;

            if (argc == 3) {
                if (parse_reg(argv[2], &to) < 0 ||
                    to < from) {
                    usage(argv[0]);
                    close(fd);
                    return 2;
                }
            } else if (argc != 2) {
                usage(argv[0]);
                close(fd);
                return 2;
            }
        }

        printf("SCH5127 registers 0x%02X-0x%02X\n",
               from,
               to);

        for (unsigned reg = from; reg <= to; reg++) {
            uint8_t value;

            if (read_reg(fd, (uint8_t)reg, &value) < 0) {
                fprintf(stderr,
                        "read 0x%02X failed: %s\n",
                        reg,
                        strerror(errno));
                close(fd);
                return 1;
            }

            print_reg((uint8_t)reg, value);

            if (reg == 0xff)
                break;
        }
    }

    close(fd);
    return 0;
}
