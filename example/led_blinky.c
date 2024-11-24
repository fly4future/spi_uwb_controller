#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <string.h> // Added header

#define LED "/sys/class/leds/uvled/"

int keep_running = 1;

void int_handler(int dummy) {
    keep_running = 0;
}

void write_to_file(const char *path, const char *value) {
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("open");
        exit(EXIT_FAILURE);
    }
    if (write(fd, value, strlen(value)) < 0) {
        perror("write");
        close(fd);
        exit(EXIT_FAILURE);
    }
    close(fd);
}

int main() {
    signal(SIGINT, int_handler);

    write_to_file(LED "trigger", "none\n");

    while (keep_running) {
        write_to_file(LED "brightness", "1\n");
        sleep(1);
        write_to_file(LED "brightness", "0\n");
        sleep(1);
    }

    write_to_file(LED "brightness", "0\n");
    write_to_file(LED "trigger", "heartbeat\n");

    return 0;
}
