# Double-sided Two way ranging example

This is very simple example of double-sided two way ranging implementation with DW3000 Linux driver. The example follows ranging schema called double-sided two way ranging that is ideal for scenario with different clock speeds. The schema is further described in https://vitpetrik.github.io/UWB-thesis/chapters/3_uwb.html#two-way-ranging.

The initiator expects PHY with MAC address `0x1234` and responder expects PHY with MAC address `0x5678`, though this can be modified endlessly to your liking.