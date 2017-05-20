sk-at91sam9g45-xc6slx
=====================
Directory arm-fpga_iface contains dummy project files named blinky.
It implements 33 bit counter and simple read/write interface between arm and fpga, so fpga can store word and return it.
The leds outputs are on X13:
  - X13 39 is set to 1 just to make sure fpga is programmed;
  - X13 37 is 29 bit in counter starting from 0
  - X13 35 is 30 bit in counter starting from 0
  - X13 33 is 31 bit in counter starting from 0
  - X13 31 is 32 bit in counter starting from 0
There is a reset pin from arm to fpga: connect 36 pin of X2 with pin 3 of X13, otherwise no leds blinking occurs.
Do not forget to set Drive Done Pin High in startup options.
Example output:
# ./fpga_loader ./blinky_iface.bit 
pio mapped
pins preset done
prog pulse done, wait for init
input file size 464288
buffer allocated
file read and closed
start main loop
loaded 453 kB
main loop finished
FPGA started
start sequence completed
buffer freed
# head -c 2 /dev/fpga 
[   12.600000] Reading first short: c49a3000  defa
# echo "AA" > /dev/fpga 
[   16.930000] Writing first short: 4141
# head -c 2 /dev/fpga 
[   18.290000] Reading first short: c49a7000  4141
# echo "AB" > /dev/fpga 
[   22.570000] Writing first short: 4241
# head -c 2 /dev/fpga 
[   25.380000] Reading first short: c49ab000  4241
# [   35.120000] random: crng init done
# head -c 2 /dev/fpga 
[   57.000000] Reading first short: c49ad000  0Example output:
