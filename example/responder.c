// SPDX-License-Identifier: ISC

/* IEEE 802.15.4 socket example */
/* gcc ranging_responder.c -o ranging_responder */

#include <asm-generic/socket.h>
#include <linux/net_tstamp.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include <errno.h>

#include <linux/if.h>
#include <linux/sockios.h>
#include <sys/ioctl.h>
#include <sys/select.h> // select()

#define IEEE802154_ADDR_LEN 8
#define MAX_PACKET_LEN 127
#define EXTENDED 0

#define DWT_TIME_UNITS ((long double)15.65e-12) //!< = 15.65e-12 s
#define SPEED_OF_LIGHT ((long double)299702547.0)
#define UUS_TO_DWT_TIME 64267

enum {
  IEEE802154_ADDR_NONE = 0x0,
  IEEE802154_ADDR_SHORT = 0x2,
  IEEE802154_ADDR_LONG = 0x3,
};

struct ieee802154_addr_sa {
  int addr_type;
  uint16_t pan_id;
  union {
    uint8_t hwaddr[IEEE802154_ADDR_LEN];
    uint16_t short_addr;
  };
};

#define ENCODED_RANGING_PKT_LENGTH 17
#define RANGING_MSG_TYPE 0x10

struct ranging_pkt_t {
  uint8_t packet_number;
  uint32_t RoundA;
  uint32_t DelayA;
  uint32_t RoundB;
  uint32_t DelayB;
  float power;
};

struct sockaddr_ieee802154 {
  sa_family_t family;
  struct ieee802154_addr_sa addr;
};

/**
 * @brief Calculate TOD from Ra, Da, Rb, Db
 *
 * @param data pointer to ranging packet
 * @return float calculated time of flight
 */
long double ToF_DS(const struct ranging_pkt_t *data) {
  long double Ra, Rb, Da, Db;

  Ra = (long double)data->RoundA;
  Da = (long double)data->DelayA;
  Rb = (long double)data->RoundB;
  Db = (long double)data->DelayB;

  long double tof =
      (long double)DWT_TIME_UNITS * (Ra * Rb - Da * Db) / (Ra + Da + Rb + Db);

  return tof;
}

ssize_t sendto_delayed(int __fd, const void *__buf, size_t __n, int __flags,
                       const struct sockaddr *__addr, socklen_t __addr_len,
                       uint64_t txtime) {
  struct msghdr msg;
  struct iovec iov;
  char cmsg_buf[CMSG_SPACE(sizeof(u_int64_t))];

  memset(&msg, 0, sizeof(msg));
  memset(&iov, 0, sizeof(iov));
  memset(cmsg_buf, 0, sizeof(cmsg_buf));

  msg.msg_name = (void *)__addr;
  msg.msg_namelen = __addr_len;

  iov.iov_base = (void *)__buf;
  iov.iov_len = __n;

  msg.msg_iov = &iov;
  msg.msg_iovlen = 1;

  msg.msg_control = cmsg_buf;
  msg.msg_controllen = sizeof(cmsg_buf);

  struct cmsghdr *cmsg = CMSG_FIRSTHDR(&msg);

  cmsg->cmsg_level = SOL_SOCKET;
  cmsg->cmsg_type = SO_TXTIME;
  cmsg->cmsg_len = CMSG_LEN(sizeof(uint64_t));

  memcpy(CMSG_DATA(cmsg), &txtime, sizeof(uint64_t));

  return sendmsg(__fd, &msg, __flags);
}

ssize_t recv_ts(int __fd, void *__buf, size_t __n, int __flags,
                struct sockaddr *__addr, socklen_t __addr_len,
                uint64_t *rxtime) {
  struct msghdr msg;
  struct iovec iov;
  char cmsg_buf[CMSG_SPACE(3 * sizeof(struct timespec))];

  memset(&msg, 0, sizeof(msg));
  memset(&iov, 0, sizeof(iov));
  memset(cmsg_buf, 0, sizeof(cmsg_buf));

  iov.iov_base = __buf;
  iov.iov_len = __n;

  msg.msg_name = __addr;
  msg.msg_namelen = __addr_len;

  msg.msg_iov = &iov;
  msg.msg_iovlen = 1;

  msg.msg_control = cmsg_buf;
  msg.msg_controllen = sizeof(cmsg_buf);

  ssize_t ret = recvmsg(__fd, &msg, __flags | SO_TIMESTAMPING);

  if (ret < 0 || rxtime == NULL)
    return ret;

  struct cmsghdr *cmsg;

  for (cmsg = CMSG_FIRSTHDR(&msg); cmsg != NULL;
       cmsg = CMSG_NXTHDR(&msg, cmsg)) {
    if (cmsg->cmsg_level == SOL_SOCKET &&
        cmsg->cmsg_type == SO_TIMESTAMPING_NEW) {
      struct timespec *timestamp = (struct timespec *)CMSG_DATA(cmsg);

      *rxtime = (timestamp[2].tv_sec * 1000000000L) + timestamp[2].tv_nsec;
    }
  }

  return ret;
}

/**
 * @brief Decode ranging buffer
 *
 * @param ranging_pkt pointer to ranging paket
 * @param buffer_rx pointer to received buffer
 * @return int returns length of message
 */
int decode_ranging_pkt(struct ranging_pkt_t *ranging_pkt,
                       const uint8_t *buffer_rx) {
  memcpy(&ranging_pkt->packet_number, buffer_rx, sizeof(uint8_t));
  buffer_rx += sizeof(uint8_t);
  memcpy(&ranging_pkt->RoundB, buffer_rx, sizeof(uint32_t));
  buffer_rx += sizeof(uint32_t);
  memcpy(&ranging_pkt->DelayB, buffer_rx, sizeof(uint32_t));
  buffer_rx += sizeof(uint32_t);
  memcpy(&ranging_pkt->DelayA, buffer_rx, sizeof(uint32_t));
  buffer_rx += sizeof(uint32_t);
  memcpy(&ranging_pkt->power, buffer_rx, sizeof(float));
  buffer_rx += sizeof(uint32_t);

  return ENCODED_RANGING_PKT_LENGTH;
}

/**
 * @brief
 *
 * @param ranging_pkt pointer to ranging paket
 * @param buffer_tx pointer to transmit buffer
 * @return int returns length of message
 */
int encode_ranging_pkt(const struct ranging_pkt_t *ranging_pkt,
                       uint8_t *buffer_tx) {
  memcpy(buffer_tx, &ranging_pkt->packet_number, sizeof(uint8_t));
  buffer_tx += sizeof(uint8_t);
  memcpy(buffer_tx, &ranging_pkt->RoundA, sizeof(uint32_t));
  buffer_tx += sizeof(uint32_t);
  memcpy(buffer_tx, &ranging_pkt->DelayA, sizeof(uint32_t));
  buffer_tx += sizeof(uint32_t);
  memcpy(buffer_tx, &ranging_pkt->DelayB, sizeof(uint32_t));
  buffer_tx += sizeof(uint32_t);
  memcpy(buffer_tx, &ranging_pkt->power, sizeof(float));
  buffer_tx += sizeof(uint32_t);

  return ENCODED_RANGING_PKT_LENGTH;
}

int main(int argc, char *argv[]) {
  int ret, sd;
  struct sockaddr_ieee802154 src, dst;

  /* Create IEEE 802.15.4 address family socket for the SOCK_DGRAM type */
  sd = socket(PF_IEEE802154, SOCK_DGRAM, 0);
  if (sd < 0) {
    perror("socket");
    return 1;
  }

  memset(&src, 0, sizeof(src));
  src.family = AF_IEEE802154;
  src.addr.pan_id = 0xabcd;

  src.addr.addr_type = IEEE802154_ADDR_SHORT;
  src.addr.short_addr = 0x5678;

  memset(&dst, 0, sizeof(dst));
  dst.family = AF_IEEE802154;
  dst.addr.pan_id = 0xabcd;

  dst.addr.addr_type = IEEE802154_ADDR_SHORT;
  dst.addr.short_addr = 0x1234;

  /* Bind socket on this side */
  ret = bind(sd, (struct sockaddr *)&src, sizeof(src));
  if (ret) {
    perror("bind");
    close(sd);
    return 1;
  }

  const int timestamping_flags =
      SOF_TIMESTAMPING_RX_HARDWARE | SOF_TIMESTAMPING_RX_SOFTWARE |
      SOF_TIMESTAMPING_TX_HARDWARE | SOF_TIMESTAMPING_TX_SOFTWARE |
      SOF_TIMESTAMPING_RAW_HARDWARE;
  if (setsockopt(sd, SOL_SOCKET, SO_TIMESTAMPING_NEW, &timestamping_flags,
                 sizeof(timestamping_flags))) {
    perror("setsockopt SO_TIMESTAMPING is not supported by your Linux kernel");
    return 1;
  }

  int off = 0;

  if ((ret = setsockopt(sd, PF_IEEE802154, 0, &off, sizeof(off)))) {
    perror("setsockopt txtime");
    return 1;
  }

  int on = 1;

  if ((ret =
           setsockopt(sd, SOL_SOCKET, SO_SELECT_ERR_QUEUE, &on, sizeof(on)))) {
    perror("setsockopt err queue");
    return 1;
  }

  struct timeval timeout;
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;

  if ((ret = setsockopt(sd, SOL_SOCKET, SO_RCVTIMEO, &timeout,
                        sizeof(timeout)))) {
    perror("setsockopt rcvtimeout");
    return 1;
  }

  struct ranging_pkt_t ranging_pkt;

  uint8_t tx_buf[ENCODED_RANGING_PKT_LENGTH + 1];
  tx_buf[0] = RANGING_MSG_TYPE;

  uint8_t rx_buf[MAX_PACKET_LEN + 1];

  uint64_t rx_timestamp = 0;
  uint64_t tx_timestamp = 0;

  while (1) {
    rx_timestamp = 0;
    ret = recv_ts(sd, rx_buf, sizeof(rx_buf), 0, (struct sockaddr *)&dst,
                  (socklen_t)sizeof(dst), &rx_timestamp);

    if (ret < 0) {
      if (errno == EAGAIN) {
        printf("EAGAIN\n");
        continue;
      }
      perror("recv_ts");
      close(sd);
      return 1;
    }

    if (rx_buf[0] != RANGING_MSG_TYPE) {
      continue;
    }

    memset(&ranging_pkt, 0, sizeof(ranging_pkt));

    decode_ranging_pkt(&ranging_pkt, &rx_buf[1]);

    if(ranging_pkt.packet_number > 1)
      ranging_pkt.RoundA = rx_timestamp - tx_timestamp;

    memset(rx_buf, 0, sizeof(rx_buf));

    ranging_pkt.packet_number++;

    tx_timestamp = rx_timestamp + (12000*UUS_TO_DWT_TIME);
    tx_timestamp &= 0xfffffffe00;
    ranging_pkt.DelayA = tx_timestamp - rx_timestamp;

    encode_ranging_pkt(&ranging_pkt, &tx_buf[1]);

    ret = sendto_delayed(sd, tx_buf, ENCODED_RANGING_PKT_LENGTH + 1, 0,
                         (struct sockaddr *)&dst, sizeof(dst), tx_timestamp >> 8);

    if (ret < 0) {
      perror("sendto");
      close(sd);
      return 1;
    }
  }

  shutdown(sd, SHUT_RDWR);
  close(sd);
  return 0;
}