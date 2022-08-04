import time
import os
from datetime import datetime

from ruuvitag_sensor.ruuvitag import RuuviTag
#sss
# Change here your own device's mac-address
mac = 'F8:0C:D7:84:57:F6'
mac2 = 'FA:67:DF:BE:4C:F9'

print('Starting')
file=open("/home/pi/Desktop/ruuvi_data.txt","a")
file.write("Sensors:" + "\t" + "\t" + "RuviTag (F6)" + "\t" +"\t" + "\t" +"\t" + 
"RuviPro (F9)" "\n" + "Date & Time" + 
"\t" + "\t" + "Humidity" + "\t" + 
"Temperature" + "\t" + "\t" + "Humidity" + "\t" + 
"Temperature" + "\n")

sensor = RuuviTag(mac)
sensor2 = RuuviTag(mac2)

while True:

    data = sensor.update()
    data2 = sensor2.update()

    line_sen = str.format('Sensor - {0}', mac)
    line_tem = str.format('Temperature: {0} C', data['temperature'])
    line_hum = str.format('Humidity:    {0}', data['humidity'])
    line_pre = str.format('Pressure:    {0}', data['pressure'])

    line_sen2 = str.format('Sensor - {0}', mac2)
    line_tem2 = str.format('Temperature: {0} C', data2['temperature'])
    line_hum2 = str.format('Humidity:    {0}', data2['humidity'])
    line_pre2 = str.format('Pressure:    {0}', data2['pressure'])

    # Clear screen and print sensor data
    os.system('clear')
    print('Press Ctrl+C to quit.\n\r\n\r')
    print(str(datetime.now().strftime("%d-%m-%Y %H:%M:%S")))
    print(line_sen)
    print(line_tem)
    print(line_hum)
    print(line_pre)
    print('\n')
    
    print(str(datetime.now().strftime("%d-%m-%Y %H:%M:%S")))
    print(line_sen2)
    print(line_tem2)
    print(line_hum2)
    print(line_pre2)
    print('\n\r\n\r.......')

    # Wait for 2 seconds and start over again
    try:
        time.sleep(2)
        file.write(str(datetime.now().strftime("%d-%m-%Y %H:%M:%S")) 
        +"\t" +  str(data["humidity"]) + "\t" + "\t" + str(data["temperature"]) 
        + "\t" + "\t" + "\t" + str(data2["humidity"]) + "\t" + "\t" + 
        str(data2["temperature"]) + "\n")

    except KeyboardInterrupt:
        # When Ctrl+C is pressed execution of the while loop is stopped
        print('Exit')
        break