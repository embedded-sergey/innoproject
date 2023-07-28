# Testbench prototyping system based on Controllino & Raspberry Pi

The aim of this innovation project is to design, assemble and test a universal IoT control system based on Controllino PLC and Raspberry Pi which can be used as a backbone for practical applications in agriculture, animal husbandry, or research facilities. The innocation project was a part of studies at Metropolia University of Applied Science implemented in UrbanFarmLab and AIoT innovation hubs.

<img width="900" alt="Screenshot 2023-07-28 at 16 56 25" src="https://github.com/embedded-sergey/innovation-project/assets/76912739/5897c01e-b310-42c7-8b1f-5bca0ff71727">

Various types of sensors and probes were integrated into the test system to demonstrate its versatility for the end user (see the table below). Some of the sensors can be used only in water, while others were designed for air measurements. Protocol communication of Controllino with wired probes also varied from simple end-to-end configuration (digital, analog, UART) to bus systems (1-Wire and Modbus RTU). Both low-budget sensors (e.g., HC-SR04 & DS18B20) and industrial probes (i.e., Atlas Scientific ORP & Vaisala GMP252) have been presented.

<img width="600" alt="image" src="https://github.com/embedded-sergey/innovation-project/assets/76912739/9f81e211-acd6-48a9-8547-ac476f72afd0">

The output devices add functionality to the system and ensure that it works properly: a water pump, a fan, an RGB LED strip, and a buzzer. Raspberry Pi itself is responsible for the wireless part of the system: it consists out of several RuuviTags, which are communicating to the RaspberryPi via BLE. It also has an LCD display connected to it and served as a server for the whole setup. A web application for displaying all the measured data was created and made ready to work with all the other components. The system workflow was organised and demonstrated by the following state machine. 

<img width="600" alt="Screenshot 2023-07-28 at 16 57 22" src="https://github.com/embedded-sergey/innovation-project/assets/76912739/7999456c-210e-46e0-9295-ad422da693af">

Note, this repository does not include any code for WebUI/server running on Raspberry Pi. Please see this fork if you are interested in its actual implementation: <https://github.com/embedded-sergey/innoproject-rasppi>. Alternatively, we could also recommend to use IoT services ThingSpeak or NodeRed, because they require minimum coding skills and are really easy to maintain and upgrade.

The full project report can be found here: <https://github.com/embedded-sergey/innoproject/blob/main/REPORT.pdf>
