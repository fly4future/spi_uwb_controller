#!/bin/sh
if [ -z "$1" ]; then
	ID=1234
else
	ID=$1
fi

cd "$(dirname "$0")"
sudo rmmod dw3000_drv
sudo insmod ./dw3000-drv.ko
if [ $? -ne 0 ]; then
	return 1
fi
#wait for the module to load
sleep 1
sudo ip link set wpan0 down &&
sudo iwpan dev wpan0 set pan_id "0x${ID}" && 
sudo iwpan dev wpan0 set short_addr "0x0000" &&
sudo ip link set wpan0 up &&
return 0

return 1

#sudo ip link set wpan1 down
#sudo iwpan dev wpan1 set pan_id "0xabcd"
#sudo iwpan dev wpan1 set short_addr "0x5678"
#sudo ip link set wpan1 up
