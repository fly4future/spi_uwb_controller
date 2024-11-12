# DW3000 Linux kernel driver

This is implementation of driver for Qorvo DW3000 with support for HW timestamping, thus enabling usage in ranging applications.

To build the kernel just simply call `make` and it should work. To make the driver work you need to install desired overlay and enabling the driver in `/boot/firmware/config.txt`

For example:

```
    dtoverlay=spi1-1cs,cs0_spidev=off
    dtoverlay=dw3000,interrupt=26
```

Please note that spidev needs to be disabled, otherwise in will collide with DW3000 driver. Spidev is implementation for SPI usage in userspace, we don't need that for the driver.

## Loading necessary modules

The driver need a few modules to be loaded, namely *mac802154 ieee802154_socket and regmap-spi*. The modules can be manually loaded every time at boot using `sudo modprobe mac802154 ieee802154_socket and regmap-spi`, or you can load them automatically by adding them to */etc/modules-load.d/modules.conf* each in separate line like this:

```
mac802154
ieee802154_socket
regmap-spi
```

## Loading and using the DW3000

After you compiled the driver using `make` the driver can be loaded as out-of-tree module. using

`sudo insmod ./dw3000-drv.ko`

This however will not set **PAN_ID and MAC** address. For this reason recommended way of loading the driver is 

```
sudo insmod ./dw3000-drv.ko
sudo ip link set wpan0 down
sudo iwpan dev wpan0 set pan_id "0xabcd"                        
sudo iwpan dev wpan0 set short_addr "0x1234"
sudo ip link set wpan0 up
```
Feel free to change the pan_id and short_addr as you wish.