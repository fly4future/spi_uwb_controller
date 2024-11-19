#!/bin/sh
sudo rmmod dw3000_drv
sudo insmod ./dw3000-drv.ko
#wait for the module to load
sleep 1
sudo ip link set wpan0 down
sudo ip link set wpan1 down
sudo iwpan dev wpan0 set pan_id "0xabcd"
sudo iwpan dev wpan1 set pan_id "0xabcd"
sudo iwpan dev wpan0 set short_addr "0x1234"
sudo iwpan dev wpan1 set short_addr "0x5678"
sudo ip link set wpan0 up
sudo ip link set wpan1 up
