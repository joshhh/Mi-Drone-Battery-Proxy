# Mi-Drone-Battery-Proxy
Software implementation of Xiaomi FiMi Mi Drone locked battery hack via Arduino

This application allows to use Arduino like a I2C Proxy between Xiaomi Mi Drone and locked battery.

It's strongly recommended to change bad(undervoltaged) bank in locked battery to good bank from another locked battery before using I2C proxy for unlock battery!

The main functionality is changing response from locked battery to drone for 2 commands:

// 0xFF 0x03 0x4A 0x00 0x02 - undervoltage times - always send zero value - 0x0 0x0 0x0 0x0
// 0x07 - battery status - always send good status - 0x04 0x0 0x0B 0x0B

Also this app allows to send commands from Serial to battery for analyzing purpose. It's better to set DEBUG_MODE=0 when analyzing is not needed

Connection:
I2C Bus from drone should be connected to the hardware I2C port, bacause only hardware I2C supports slave mode.
For the Arduino Pro Mini it's A4(SDA) and A5(SCL) pins.
I2C Bus from battery could be connected to any PWM pins. These pins should be defined as I2C_BATTERY_SDA and I2C_BATTERY_SCL. Using pin 5 (SDA) and 6 (SCL) in this example.
