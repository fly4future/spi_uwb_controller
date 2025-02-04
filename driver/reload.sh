#!/bin/sh
if [ -z "$1" ]; then
	input=-1
	ID=1234
else
	input=${1}
	ID=`printf '%x\n' ${1}`
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
sudo iwpan dev wpan0 set pan_id "0xabcd" && 
sudo iwpan dev wpan0 set short_addr "0x${ID}" &&
sudo ip link set wpan0 up &&

echo $input
return 0


#sudo ip link set wpan1 down
#sudo iwpan dev wpan1 set pan_id "0xabcd"
#sudo iwpan dev wpan1 set short_addr "0x5678"
#sudo ip link set wpan1 up
