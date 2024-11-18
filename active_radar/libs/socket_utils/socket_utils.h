#ifndef SOCKET_UTILS_H
#define SOCKET_UTILS_H

#include <sys/socket.h>
#include <time.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_PACKET_LEN 127

// Function declarations
ssize_t sendto_delayed(int __fd, const void *__buf, size_t __n, int __flags,
                       const struct sockaddr *__addr, socklen_t __addr_len,
                       uint64_t txtime);

ssize_t sendto_ts(int __fd, const void *__buf, size_t __n, int __flags,
                  const struct sockaddr *__addr, socklen_t __addr_len,
                  uint64_t *txtime);

ssize_t recv_ts(int __fd, void *__buf, size_t __n, int __flags,
                struct sockaddr *__addr, socklen_t __addr_len,
                uint64_t *rxtime);

#ifdef __cplusplus
}
#endif

#endif // SOCKET_UTILS_H
