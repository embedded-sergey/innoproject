# Open-source control panel for greenhouses and vertical farms
The system is designed for small-scale indoor farming based on open electronics platforms and free open-source tools. The idea behind is to reduce costs of a control panel (automation box) used in agriculture start-ups or research laboratories.

The control panel consists of three core elements:
- [Conrollino Maxi](https://www.controllino.com/product/controllino-maxi/) (as PLC) to control light, AC/DC pumps, valves & actuators and to stream data from wired sensors to Raspberry Pi;
- [Raspberry Pi 4B](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/) or [Zero 2W](https://www.raspberrypi.com/products/raspberry-pi-zero-2-w/) to log data from both wired & wireless sensors to an SD card and data stream to a cload/display;
- Free IoT/cloud service for data storing and visualisation (*not decided yet*).

The other automation box components will be mounted on the DIN rail: i.e. 220 VAC to 12/24 VDC transformer, circuit breakers, terminal blocks etc. However, LED screen and sensor meters will be installed on the breadboard located next to Controllino Maxi. All the components will be installed into 300x400 mm [control cabinet](https://www.amazon.de/ELEKTRO-PLAST-Control-Distribution-Industrial-Surface-Mounted/dp/B00R3HS41U/ref=rvi_7/261-8255680-4129054?pd_rd_w=nTgOY&pf_rd_p=22019d9a-e205-410a-b337-2be913e3a486&pf_rd_r=WC1628P0KMTCZAG0ZTGB&pd_rd_r=ef9b4e1b-41d8-4ea9-bd4d-cda1da2f85f8&pd_rd_wg=aH9x4&pd_rd_i=B00R3HS41U&psc=1).

## Installation intructions:
...
