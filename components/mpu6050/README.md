#### **MPU6050** 
datasheet link:
* [MPU6050 datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
* [MPU6000 and MPU6050 register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)

**Base library**
MPU6050 Driver Component from Espressif
ESP Register Component link: https://components.espressif.com/components/espressif/mpu6050/versions/1.2.1/readme
GitHub Repository: https://github.com/espressif/esp-bsp/tree/604890dd0abbf11f5d77461bc864dcfb153b0b45/components/mpu6050

**Why this**
Same library as made from Espressif but with upgrade i2c (EOL) to i2c_master and new fuction ``` mpu6050_calibrate ```

**I2C Operating Frequency:**
- All registers, Fast-mode 400 kHz
- All registers, Standard-mode 100 kHz

**INTERNAL CLOCK SOURCE:**
- Gyroscope Sample Rate, Fast DLPFCFG=0
- Gyroscope Sample Rate, Slow DLPFCFG=1,2,3,4,5, or 6
