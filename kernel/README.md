# Adding HW TX timestamp for IEEE802154

Since linux kernel does not fully support transmit timestaping for IEEE 802.15.4 a small patch need to be made to use DW3000 driver for ranging applications.

To apply the patch please follow loosely these steps, you might be able to make it more streamlined as i'm linux n00b 🤯.

1. Clone the Raspberry PI kernel source from https://github.com/raspberrypi/linux.
2. Apple the patch in this folder to file `net/ieee802154/socket.c`.
3. Compile the kernel as whole or just the one kernel module using  `make M=net/ieee802154`.
4. Install the modified kernel or just the module and reload.

Example application should work now, if everything went smoothly.

## Easier approach without needing kernel source

You don't actually need the whole kernel source, header files are enough. It should be enough to run 
```
make -C /lib/modules/$(uname -r)/build M=$(pwd)/ieee802154 modules
```
Then the file **ieee802154_socket.ko** is what's most important for you. Just run 

``` bash
sudo rmmod ieee802154_socket 
sudo insmod ieee802154/ieee802154_socket.ko
```

To insert the modified version one time. You can also replace the original *.ko.xz file in `\lib\modules`

```bash
xz -f ieee802154/ieee802154_socket.ko
sudo cp ieee802154/ieee802154_socket.ko.xz /lib/modules/$(uname -r)/kernel/net/ieee802154/ieee802154_socket.ko.xz
```