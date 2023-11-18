# Setting up Romi 32U4 Board  

This is done only once by using a Linux machine. You need to connect your Linux laptop/PC to Romi 32U4 board via a USB cable. We use Arduino IDE. 

1. Check [romi user guide](https://www.pololu.com/docs/0J69/5) on how to install Arduino IDE and connect romi board. 
2. Load the RomiRPiSlaveDemo into the romi board via Arduino IDE:

Open File->Examples->PololuRpiSlave->RomiRPiSlaveDemo
Click the upload button (somewhere top left)

If you get access errors when you attempt to upload the program, check this page: https://support.arduino.cc/hc/en-us/articles/360016495679-Fix-port-access-on-Linux

# Preparing Raspi and Romi 32U4 Board Communications 
## Do the Following On Raspi
```
sudo apt update 
sudo apt upgrade -y
sudo apt install -y i2c-tools python3-pip
sudo apt-get install python3 python3-flask python3-smbus
```
### clone the repository
```
wget https://github.com/pololu/pololu-rpi-slave-arduino-library/archive/<version>.tar.gz
tar -xzf 2.0.0.tar.gz
mv pololu-rpi-slave-arduino-library-master pololu-rpi-slave-arduino-library-2.0.0
```
### setup I2C connection 
adapted from: https://lexruee.ch/setting-i2c-permissions-for-non-root-users.html
```
sudo groupadd i2c
sudo chown :i2c /dev/i2c-1
sudo chmod g+rw /dev/i2c-1
sudo usermod -aG i2c <username>
```
To make the changes permanent
```
sudo -s
```
```
echo 'KERNEL=="i2c-[0-9]*", GROUP="i2c"' >> /etc/udev/rules.d/10-local_i2c_group.rules
```
## Do the following on Romi 32U4 Board
Nothing needs to be done additionally after the first step

# Checking i2c Connection between Raspi and Romi 32U4 Board
### On raspi
```
i2cdetect -y 1 
```
make sure address 20 (0x14) is available

Run
```
run python3 pololu-rpi-slave-arduino-library/pi/blink.py
```

### On Romi 32U4 Board
Check if LED is blinking (should be Green LED)

## New updates:
It seems new rpi firmware prohibit non-root user from changing GPIO pin setting(edge detection , I2C , etc...) check 
https://www.pololu.com/blog/663/building-a-raspberry-pi-robot-with-the-romi-chassis

for the section regarding adding user to dialout group to solve it. 
