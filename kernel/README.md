# Adding HW TX timestamp for IEEE802154

Since linux kernel does not fully support transmit timestaping for IEEE 802.15.4 a small patch need to be made to use DW3000 driver for ranging applications.

To apply the patch please follow loosely these steps, you might be able to make it more streamlined as i'm linux n00b 🤯.

1. Clone the Raspberry PI kernel source from https://github.com/raspberrypi/linux.
2. Apple the patch in this folder to file `net/ieee802154/socket.c`.
3. Compile the kernel as whole or just the one kernel module using  `make M=net/ieee802154`.
4. Install the modified kernel or just the module and reload.

Example application should work now, if everything went smoothly.