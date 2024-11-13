# Double-sided Two way ranging example

This is very simple example of double-sided two way ranging implementation with DW3000 Linux driver. The example follows ranging schema called double-sided two way ranging that is ideal for scenario with different clock speeds.

The initiator expects PHY with MAC address `0x1234` and responder expects PHY with MAC address `0x5678`, though this can be modified endlessly to your liking.

## Files Overview

### responder.c

The `responder.c` file implements the responder side of the double-sided two-way ranging protocol. It is responsible for receiving ranging requests from the initiator, processing them, and sending back the appropriate responses. Key functionalities include:

- **Timestamping**: The responder uses hardware and software timestamping to accurately measure the time of flight (ToF) of the ranging packets. This is crucial for calculating the distance between the initiator and the responder.
- **Delayed Transmission**: The responder schedules the transmission of response packets with a precise delay to ensure accurate ToF measurements. This is achieved using the `sendto_delayed` function, which sends packets at a specified future time.

### initiator.c

The `initiator.c` file implements the initiator side of the double-sided two-way ranging protocol. It initiates the ranging process by sending ranging requests to the responder and processes the responses to calculate the distance. Key functionalities include:

- **Timestamping**: Similar to the responder, the initiator uses hardware and software timestamping to measure the ToF of the ranging packets. This helps in accurately determining the distance.
- **Delayed Transmission**: The initiator also schedules the transmission of packets with precise delays using the `sendto_delayed` function. This ensures that the ranging process is synchronized and the ToF measurements are accurate.

## Timestamping and Delayed Transmission

Both `responder.c` and `initiator.c` rely heavily on timestamping and delayed transmission to achieve accurate distance measurements. Here’s a deeper look into these concepts:

### Timestamping

Timestamping is used to record the exact time when a packet is sent or received. This is done using hardware and software mechanisms provided by the Linux kernel. The `recv_ts` and `sendto_ts` functions are used to receive and send packets with timestamps. These timestamps are then used to calculate the ToF, which is essential for determining the distance.

### Delayed Transmission

Delayed transmission is used to schedule the sending of packets at a precise future time. This is crucial for ensuring that the ranging process is synchronized and the ToF measurements are accurate. The `sendto_delayed` function is used to send packets with a specified delay. This function uses control messages to set the transmission time, ensuring that the packets are sent exactly when needed.

By combining timestamping and delayed transmission, the double-sided two-way ranging protocol can achieve highly accurate distance measurements, even in scenarios with different clock speeds.

## Usage

To compile and run the examples, use the following commands:

```sh
gcc responder.c -o responder
gcc initiator.c -o initiator
```

Run the responder on one device:

```sh
./responder
```

And the initiator on another device:

```sh
./initiator
```

Make sure both devices are configured with the correct MAC addresses as mentioned above.

For more details on the double-sided two-way ranging protocol, refer to the [UWB thesis](https://vitpetrik.github.io/UWB-thesis/chapters/3_uwb.html#two-way-ranging).
