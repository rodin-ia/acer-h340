/*
 * h340led - Acer Aspire easyStore H340 front panel LED controller
 *
 * CLI and daemon are the same binary:
 *
 *   h340led hdd1 blue
 *   h340led hdd2 purple
 *   h340led hdd3 blink-red 0.5
 *   h340led hdd4 blink-purple 2
 *   h340led info blue
 *   h340led info blink-purple 1.0
 *   h340led all off
 *   h340led status
 *   h340led daemon
 *
 * The CLI modifies a state file.
 * Only the daemon accesses /dev/port.
 *
 * H340 hardware:
 *
 * SCH5127 @ 0x0800
 *
 *   GP1 = 0x084b
 *   GP5 = 0x084f
 *
 * HDD1:
 *   blue = GP5 bit 6
 *   red  = GP5 bit 7
 *
 * HDD2:
 *   blue = GP5 bit 2
 *   red  = GP5 bit 3
 *
 * HDD3:
 *   blue = GP5 bit 0
 *   red  = GP5 bit 1
 *
 * HDD4:
 *   blue = GP1 bit 4
 *   red  = GP1 bit 1
 *
 * ICH7 GPIO:
 *
 *   GPIO base = 0x1180
 *   GP_LVL    = 0x118c
 *   GPO_BLINK = 0x1198
 *
 * INFO:
 *   blue = GPIO20, active low
 *   red  = GPIO24, active low
 *
 * IMPORTANT:
 *
 * GPIO20 and GPIO24 also have hardware blink capability through
 * ICH7 GPO_BLINK. Those bits must be cleared when software-controlled
 * blinking is used, otherwise the hardware blink continues to toggle
 * the LEDs regardless of GP_LVL.
 *
 * Network LED is intentionally not implemented yet because its
 * physical GPIO has not been identified.
 */

#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#define DEVPORT             "/dev/port"

#define STATE_DIR           "/var/run/h340led"
#define STATE_FILE          STATE_DIR "/state"
#define STATE_TMP           STATE_DIR "/state.tmp"
#define LOCK_FILE           STATE_DIR "/lock"
#define DAEMON_LOCK_FILE    STATE_DIR "/daemon.lock"

#define SCH_GP1             0x084b
#define SCH_GP5             0x084f

#define ICH_GP_LVL          0x118c
#define ICH_GPO_BLINK       0x1198

#define INFO_BLUE_GPIO      20
#define INFO_RED_GPIO       24

#define LED_COUNT           5

#define LED_HDD1            0
#define LED_HDD2            1
#define LED_HDD3            2
#define LED_HDD4            3
#define LED_INFO            4

#define STATE_VERSION       1

#define DEFAULT_PERIOD      1.0
#define MIN_PERIOD          0.05
#define MAX_PERIOD          3600.0

#define LOOP_MS             25
#define STATE_POLL_MS       50

enum color {
    COLOR_OFF = 0,
    COLOR_BLUE,
    COLOR_RED,
    COLOR_PURPLE
};

enum mode {
    MODE_SOLID = 0,
    MODE_BLINK
};

struct led_state {
    enum mode mode;
    enum color color;
    double period;
};

struct controller_state {
    int version;
    struct led_state led[LED_COUNT];
};

static int port_fd = -1;
static int lock_fd = -1;
static int daemon_lock_fd = -1;

static volatile sig_atomic_t running = 1;

/* --------------------------------------------------------------------- */

static void die(const char *msg)
{
    perror(msg);
    exit(EXIT_FAILURE);
}

static void die_msg(const char *msg)
{
    fprintf(stderr, "%s\n", msg);
    exit(EXIT_FAILURE);
}

/* --------------------------------------------------------------------- */

static void ensure_state_dir(void)
{
    struct stat st;

    if (stat(STATE_DIR, &st) == 0) {
        if (!S_ISDIR(st.st_mode))
            die_msg(STATE_DIR " exists but is not a directory");

        return;
    }

    if (errno != ENOENT)
        die("stat");

    if (mkdir(STATE_DIR, 0755) < 0)
        die("mkdir");
}

/* --------------------------------------------------------------------- */

static void lock_state(void)
{
    ensure_state_dir();

    lock_fd = open(LOCK_FILE,
                   O_RDWR | O_CREAT | O_CLOEXEC,
                   0644);

    if (lock_fd < 0)
        die("open state lock");

    if (flock(lock_fd, LOCK_EX) < 0)
        die("flock state");
}

static void unlock_state(void)
{
    if (lock_fd >= 0) {
        flock(lock_fd, LOCK_UN);
        close(lock_fd);
        lock_fd = -1;
    }
}

/* --------------------------------------------------------------------- */

static void lock_daemon(void)
{
    ensure_state_dir();

    daemon_lock_fd = open(DAEMON_LOCK_FILE,
                          O_RDWR | O_CREAT | O_CLOEXEC,
                          0644);

    if (daemon_lock_fd < 0)
        die("open daemon lock");

    if (flock(daemon_lock_fd, LOCK_EX | LOCK_NB) < 0) {
        if (errno == EWOULDBLOCK || errno == EAGAIN)
            die_msg("h340led daemon is already running");

        die("flock daemon");
    }
}

static void unlock_daemon(void)
{
    if (daemon_lock_fd >= 0) {
        flock(daemon_lock_fd, LOCK_UN);
        close(daemon_lock_fd);
        daemon_lock_fd = -1;
    }
}

/* --------------------------------------------------------------------- */

static void state_defaults(struct controller_state *s)
{
    int i;

    memset(s, 0, sizeof(*s));

    s->version = STATE_VERSION;

    for (i = 0; i < LED_COUNT; i++) {
        s->led[i].mode = MODE_SOLID;
        s->led[i].color = COLOR_OFF;
        s->led[i].period = DEFAULT_PERIOD;
    }
}

/* --------------------------------------------------------------------- */

static const char *led_name(int index)
{
    switch (index) {
    case LED_HDD1:
        return "hdd1";

    case LED_HDD2:
        return "hdd2";

    case LED_HDD3:
        return "hdd3";

    case LED_HDD4:
        return "hdd4";

    case LED_INFO:
        return "info";
    }

    return "unknown";
}

static int led_index(const char *name)
{
    if (!strcasecmp(name, "hdd1"))
        return LED_HDD1;

    if (!strcasecmp(name, "hdd2"))
        return LED_HDD2;

    if (!strcasecmp(name, "hdd3"))
        return LED_HDD3;

    if (!strcasecmp(name, "hdd4"))
        return LED_HDD4;

    if (!strcasecmp(name, "info"))
        return LED_INFO;

    return -1;
}

/* --------------------------------------------------------------------- */

static const char *color_to_string(enum color c)
{
    switch (c) {
    case COLOR_OFF:
        return "off";

    case COLOR_BLUE:
        return "blue";

    case COLOR_RED:
        return "red";

    case COLOR_PURPLE:
        return "purple";
    }

    return "off";
}

static const char *mode_to_string(enum mode m)
{
    return m == MODE_BLINK ? "blink" : "solid";
}

/* --------------------------------------------------------------------- */

static enum color parse_color(const char *s)
{
    if (!strcasecmp(s, "off"))
        return COLOR_OFF;

    if (!strcasecmp(s, "blue"))
        return COLOR_BLUE;

    if (!strcasecmp(s, "red"))
        return COLOR_RED;

    if (!strcasecmp(s, "purple"))
        return COLOR_PURPLE;

    die_msg("Invalid color");

    return COLOR_OFF;
}

/* --------------------------------------------------------------------- */

static int parse_period(const char *s, double *result)
{
    char *end;
    double value;

    errno = 0;
    end = NULL;

    value = strtod(s, &end);

    if (errno != 0 || end == s || *end != '\0')
        return -1;

    if (!isfinite(value))
        return -1;

    if (value < MIN_PERIOD || value > MAX_PERIOD)
        return -1;

    *result = value;

    return 0;
}

/* --------------------------------------------------------------------- */

static void state_save(const struct controller_state *s)
{
    FILE *f;
    int i;

    ensure_state_dir();

    f = fopen(STATE_TMP, "w");

    if (!f)
        die("fopen state");

    fprintf(f, "version %d\n", STATE_VERSION);

    for (i = 0; i < LED_COUNT; i++) {
        fprintf(f,
                "%s %s %s %.6f\n",
                led_name(i),
                mode_to_string(s->led[i].mode),
                color_to_string(s->led[i].color),
                s->led[i].period);
    }

    if (fflush(f) != 0) {
        fclose(f);
        die("fflush state");
    }

    if (fsync(fileno(f)) != 0) {
        fclose(f);
        die("fsync state");
    }

    if (fclose(f) != 0)
        die("fclose state");

    if (rename(STATE_TMP, STATE_FILE) < 0)
        die("rename state");
}

/* --------------------------------------------------------------------- */

static int state_load(struct controller_state *s)
{
    FILE *f;
    char line[256];

    state_defaults(s);

    f = fopen(STATE_FILE, "r");

    if (!f)
        return -1;

    while (fgets(line, sizeof(line), f)) {
        char name[32];
        char mode[32];
        char color[32];
        double period;
        int index;

        if (!strncmp(line, "version ", 8))
            continue;

        if (sscanf(line,
                   "%31s %31s %31s %lf",
                   name,
                   mode,
                   color,
                   &period) != 4)
            continue;

        index = led_index(name);

        if (index < 0)
            continue;

        if (!strcasecmp(mode, "blink"))
            s->led[index].mode = MODE_BLINK;
        else
            s->led[index].mode = MODE_SOLID;

        s->led[index].color = parse_color(color);

        if (period >= MIN_PERIOD &&
            period <= MAX_PERIOD &&
            isfinite(period)) {

            s->led[index].period = period;
        }
    }

    fclose(f);

    return 0;
}

/* --------------------------------------------------------------------- */

static void open_port(void)
{
    port_fd = open(DEVPORT, O_RDWR | O_CLOEXEC);

    if (port_fd < 0)
        die(DEVPORT);
}

/* --------------------------------------------------------------------- */

static uint8_t port_read8(off_t port)
{
    uint8_t value;

    if (pread(port_fd, &value, 1, port) != 1)
        die("pread8");

    return value;
}

static void port_write8(off_t port, uint8_t value)
{
    if (pwrite(port_fd, &value, 1, port) != 1)
        die("pwrite8");
}

static uint32_t port_read32(off_t port)
{
    uint32_t value;

    if (pread(port_fd, &value, 4, port) != 4)
        die("pread32");

    return value;
}

static void port_write32(off_t port, uint32_t value)
{
    if (pwrite(port_fd, &value, 4, port) != 4)
        die("pwrite32");
}

/* --------------------------------------------------------------------- */

/*
 * Read-modify-write an 8-bit register.
 *
 * Only bits in mask are changed.
 */
static void update_reg8(off_t port, uint8_t mask, uint8_t value)
{
    uint8_t old_value;
    uint8_t new_value;

    old_value = port_read8(port);

    new_value = (old_value & (uint8_t)~mask) |
                (value & mask);

    if (new_value != old_value)
        port_write8(port, new_value);
}

/* --------------------------------------------------------------------- */

/*
 * Read-modify-write a 32-bit ICH register.
 *
 * Only one bit is modified.
 */
static void update_reg32_bit(off_t port, int bit, int value)
{
    uint32_t old_value;
    uint32_t new_value;
    uint32_t mask;

    mask = (uint32_t)1U << bit;

    old_value = port_read32(port);

    if (value)
        new_value = old_value | mask;
    else
        new_value = old_value & ~mask;

    if (new_value != old_value)
        port_write32(port, new_value);
}

/* --------------------------------------------------------------------- */

static void update_gpio32(int bit, int level)
{
    update_reg32_bit(ICH_GP_LVL, bit, level);
}

/* --------------------------------------------------------------------- */

/*
 * Disable hardware blinking for an ICH GPIO.
 *
 * The H340 INFO LED uses GPIO20 and GPIO24. If the corresponding
 * GPO_BLINK bit remains set, GP_LVL cannot produce a steady LED.
 */
static void disable_gpio_hardware_blink(int bit)
{
    update_reg32_bit(ICH_GPO_BLINK, bit, 0);
}

/* --------------------------------------------------------------------- */

/*
 * Return the combined red+blue bit mask for an HDD.
 */
static uint8_t hdd_mask(int hdd)
{
    switch (hdd) {
    case 1:
        return 0xc0;

    case 2:
        return 0x0c;

    case 3:
        return 0x03;

    case 4:
        return 0x12;
    }

    return 0;
}

/* --------------------------------------------------------------------- */

/*
 * Return the bits needed to display a color on an HDD.
 */
static uint8_t hdd_color_bits(int hdd, enum color color)
{
    uint8_t blue;
    uint8_t red;

    switch (hdd) {
    case 1:
        blue = 0x40;
        red  = 0x80;
        break;

    case 2:
        blue = 0x04;
        red  = 0x08;
        break;

    case 3:
        blue = 0x01;
        red  = 0x02;
        break;

    case 4:
        blue = 0x10;
        red  = 0x02;
        break;

    default:
        return 0;
    }

    switch (color) {
    case COLOR_OFF:
        return 0;

    case COLOR_BLUE:
        return blue;

    case COLOR_RED:
        return red;

    case COLOR_PURPLE:
        return blue | red;
    }

    return 0;
}

/* --------------------------------------------------------------------- */

static void hardware_set_hdd(int hdd, enum color color)
{
    uint8_t mask;
    uint8_t value;

    mask = hdd_mask(hdd);

    if (!mask)
        die_msg("Invalid HDD number");

    value = hdd_color_bits(hdd, color);

    /*
     * HDD1-3 share SCH5127 GP5.
     *
     * HDD4 uses SCH5127 GP1.
     *
     * Read-modify-write is mandatory because these registers contain
     * other unrelated outputs.
     */
    if (hdd <= 3)
        update_reg8(SCH_GP5, mask, value);
    else
        update_reg8(SCH_GP1, mask, value);
}

/* --------------------------------------------------------------------- */

/*
 * ICH7 INFO LED outputs are active-low.
 *
 * GPIO20 = INFO blue
 * GPIO24 = INFO red
 *
 * Hardware blink is explicitly disabled first.
 */
static void hardware_set_info(enum color color)
{
    int blue;
    int red;

    /*
     * Disable the ICH hardware blink generator for both INFO
     * channels. Software blinking is handled by the daemon.
     */
    disable_gpio_hardware_blink(INFO_BLUE_GPIO);
    disable_gpio_hardware_blink(INFO_RED_GPIO);

    blue = color == COLOR_BLUE ||
           color == COLOR_PURPLE;

    red = color == COLOR_RED ||
          color == COLOR_PURPLE;

    /*
     * Active low:
     *
     * LED on  -> GPIO 0
     * LED off -> GPIO 1
     */
    update_gpio32(INFO_BLUE_GPIO, !blue);
    update_gpio32(INFO_RED_GPIO, !red);
}

/* --------------------------------------------------------------------- */

static void hardware_set(int index, enum color color)
{
    if (index >= LED_HDD1 &&
        index <= LED_HDD4) {

        hardware_set_hdd(index + 1, color);
        return;
    }

    if (index == LED_INFO) {
        hardware_set_info(color);
        return;
    }

    die_msg("Invalid LED index");
}

/* --------------------------------------------------------------------- */

static void all_off(void)
{
    hardware_set_hdd(1, COLOR_OFF);
    hardware_set_hdd(2, COLOR_OFF);
    hardware_set_hdd(3, COLOR_OFF);
    hardware_set_hdd(4, COLOR_OFF);
    hardware_set_info(COLOR_OFF);
}

/* --------------------------------------------------------------------- */

static double monotonic_seconds(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) < 0)
        die("clock_gettime");

    return (double)ts.tv_sec +
           (double)ts.tv_nsec / 1000000000.0;
}

/* --------------------------------------------------------------------- */

static void sleep_ms(long ms)
{
    struct timespec ts;

    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000L;

    while (nanosleep(&ts, &ts) < 0) {
        if (errno != EINTR)
            break;

        if (!running)
            break;
    }
}

/* --------------------------------------------------------------------- */

static int blink_is_on(double now, double period)
{
    double phase;

    phase = fmod(now, period);

    return phase < period / 2.0;
}

/* --------------------------------------------------------------------- */

/*
 * Apply current state to hardware.
 *
 * output_cache contains the color which is currently physically
 * applied to each LED. This avoids unnecessary /dev/port accesses.
 */
static void apply_state(const struct controller_state *s,
                        double now,
                        enum color *output_cache,
                        int *cache_valid)
{
    int i;

    for (i = 0; i < LED_COUNT; i++) {
        enum color output_color;

        output_color = s->led[i].color;

        if (s->led[i].mode == MODE_BLINK &&
            !blink_is_on(now, s->led[i].period)) {

            output_color = COLOR_OFF;
        }

        if (!*cache_valid ||
            output_cache[i] != output_color) {

            hardware_set(i, output_color);
            output_cache[i] = output_color;
        }
    }

    *cache_valid = 1;
}

/* --------------------------------------------------------------------- */

static void daemon_signal(int sig)
{
    (void)sig;

    running = 0;
}

/* --------------------------------------------------------------------- */

static void daemon_run(void)
{
    struct controller_state state;
    enum color output_cache[LED_COUNT];

    int have_state = 0;
    int cache_valid = 0;

    double next_reload = 0.0;

    memset(&state, 0, sizeof(state));
    memset(output_cache, 0, sizeof(output_cache));

    signal(SIGTERM, daemon_signal);
    signal(SIGINT, daemon_signal);

    /*
     * Only one daemon may control the hardware.
     */
    lock_daemon();

    open_port();

    /*
     * Start from a known state.
     *
     * This also disables hardware blinking for INFO GPIO20/24.
     */
    all_off();

    while (running) {
        double now;

        now = monotonic_seconds();

        /*
         * Reload state every 50 ms.
         */
        if (now >= next_reload) {
            struct controller_state new_state;

            lock_state();

            if (state_load(&new_state) == 0) {
                if (!have_state ||
                    memcmp(&state,
                           &new_state,
                           sizeof(state)) != 0) {

                    /*
                     * Force the new configuration to hardware.
                     */
                    cache_valid = 0;
                }

                state = new_state;
                have_state = 1;
            }

            unlock_state();

            next_reload = now +
                          (double)STATE_POLL_MS / 1000.0;
        }

        if (have_state) {
            apply_state(&state,
                        now,
                        output_cache,
                        &cache_valid);
        }

        sleep_ms(LOOP_MS);
    }

    /*
     * Never leave LEDs on after daemon termination.
     *
     * This also disables INFO hardware blink.
     */
    all_off();

    close(port_fd);
    port_fd = -1;

    unlock_daemon();
}

/* --------------------------------------------------------------------- */

static void print_status(void)
{
    struct controller_state s;
    int i;

    lock_state();

    if (state_load(&s) < 0)
        state_defaults(&s);

    unlock_state();

    for (i = 0; i < LED_COUNT; i++) {
        printf("%-5s %-6s %-7s %.3fs\n",
               led_name(i),
               mode_to_string(s.led[i].mode),
               color_to_string(s.led[i].color),
               s.led[i].period);
    }
}

/* --------------------------------------------------------------------- */

/*
 * Handle:
 *
 *   h340led hdd1 blue
 *   h340led hdd1 red
 *   h340led hdd1 purple
 *   h340led hdd1 off
 *
 * and:
 *
 *   h340led hdd1 blink-blue 0.5
 *
 * main() passes:
 *
 *   argc - 1
 *   argv + 1
 *
 * Therefore:
 *
 *   argv[0] = "hdd1"
 *   argv[1] = "blue"
 *   argv[2] = "0.5"
 */
static void command_led(int argc, char **argv)
{
    struct controller_state s;
    int index;

    if (argc < 2)
        die_msg("Missing LED/color");

    index = led_index(argv[0]);

    if (index < 0) {
        fprintf(stderr, "Unknown LED: %s\n", argv[0]);
        exit(EXIT_FAILURE);
    }

    lock_state();

    if (state_load(&s) < 0)
        state_defaults(&s);

    /*
     * Blink mode.
     */
    if (!strncasecmp(argv[1], "blink-", 6)) {
        double period;

        if (argc != 3) {
            unlock_state();
            die_msg("Blink requires PERIOD");
        }

        if (parse_period(argv[2], &period) < 0) {
            unlock_state();
            die_msg("Invalid blink period");
        }

        s.led[index].mode = MODE_BLINK;
        s.led[index].color =
            parse_color(argv[1] + 6);
        s.led[index].period = period;
    } else {
        /*
         * Solid mode.
         */
        if (argc != 2) {
            unlock_state();
            die_msg("Invalid argument count");
        }

        s.led[index].mode = MODE_SOLID;
        s.led[index].color = parse_color(argv[1]);

        /*
         * Keep the configured period. It has no effect while solid.
         */
    }

    state_save(&s);

    unlock_state();
}

/* --------------------------------------------------------------------- */

static void command_all_off(void)
{
    struct controller_state s;

    state_defaults(&s);

    lock_state();

    state_save(&s);

    unlock_state();
}

/* --------------------------------------------------------------------- */

static void usage(const char *prog)
{
    printf(
        "Acer Aspire easyStore H340 LED controller\n"
        "\n"
        "Usage:\n"
        "  %s hdd1|hdd2|hdd3|hdd4 COLOR\n"
        "  %s hdd1|hdd2|hdd3|hdd4 blink-COLOR PERIOD\n"
        "  %s info COLOR\n"
        "  %s info blink-COLOR PERIOD\n"
        "  %s all off\n"
        "  %s status\n"
        "  %s daemon\n"
        "\n"
        "COLOR:\n"
        "  off\n"
        "  blue\n"
        "  red\n"
        "  purple\n"
        "\n"
        "PERIOD:\n"
        "  0.05 .. 3600 seconds\n"
        "\n"
        "Examples:\n"
        "  %s hdd1 blue\n"
        "  %s hdd2 red\n"
        "  %s hdd3 purple\n"
        "  %s hdd4 blink-blue 0.5\n"
        "  %s info purple\n"
        "  %s info blink-red 2.0\n"
        "  %s all off\n"
        "\n"
        "State:\n"
        "  %s\n",
        prog,
        prog,
        prog,
        prog,
        prog,
        prog,
        prog,
        prog,
        prog,
        prog,
        prog,
        prog,
        prog,
        prog,
        STATE_FILE);
}

/* --------------------------------------------------------------------- */

int main(int argc, char **argv)
{
    if (argc < 2) {
        usage(argv[0]);
        return EXIT_FAILURE;
    }

    /*
     * Daemon mode.
     */
    if (!strcasecmp(argv[1], "daemon")) {
        if (argc != 2) {
            usage(argv[0]);
            return EXIT_FAILURE;
        }

        daemon_run();

        return EXIT_SUCCESS;
    }

    /*
     * Status.
     */
    if (!strcasecmp(argv[1], "status")) {
        if (argc != 2) {
            usage(argv[0]);
            return EXIT_FAILURE;
        }

        print_status();

        return EXIT_SUCCESS;
    }

    /*
     * All LEDs off.
     */
    if (!strcasecmp(argv[1], "all")) {
        if (argc != 3 ||
            strcasecmp(argv[2], "off") != 0) {

            usage(argv[0]);
            return EXIT_FAILURE;
        }

        command_all_off();

        return EXIT_SUCCESS;
    }

    /*
     * LED command.
     *
     * Pass argv+1 so command_led() receives:
     *
     *   argv[0] = LED name
     *   argv[1] = color / blink-color
     *   argv[2] = period
     */
    if (argc >= 3) {
        command_led(argc - 1, argv + 1);

        return EXIT_SUCCESS;
    }

    usage(argv[0]);

    return EXIT_FAILURE;
}
