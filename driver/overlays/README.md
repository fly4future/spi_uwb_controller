Use command to compile and copy overlay into linux firmware
```
dtc -@ -I dts -O dtb -o <OUTPUT>.dtbo <SOURCE>.dts 
sudo cp <OUTPUT>.dtbo /boot/firmware/overlays 
```


Use the overlay by enabling it in **config.txt** by adding line 

```
dtoverlay=uwb_hat,speed=10000000,port1=disabled,port2=disabled,port3=disabled,led_trigger=heartbeat
```

### Overlay Details

**Name:** `uwb_hat`  
**Info:** Overlay for 4 port UWB+UVLED HAT for F4F modules  
**Load:** `dtoverlay=uwb_hat,<param>[=<val>]`

**Params:**
- `port<1~4>`: Set to `disabled` if you want to explicitly disable UWB on port<1~4>. Default is `okay`.
- `speed`: Control SPI clock speed in Hz. Default value is 32000000 Hz (Max SPI speed for DW3000).
- `led_trigger`: UV LED driver can be set to generate heartbeat or act as activity LED. Use for fun, no real use IMHO. Default is `none`
