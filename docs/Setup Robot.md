## Programming the Romi 32U4
1. Install the [Arduino IDE](https://www.arduino.cc/en/software)
2. Optionally install [Windows Drivers](https://www.pololu.com/docs/0J69/5.1)
3. Add IDE support for the [Pololu Device](https://www.pololu.com/docs/0J69/5.2) <br>
    i. Optionally, test with a provided example
4. Load the **RomiRPiSlaveDemo** into the board 

*Troubleshooting documentation can be found [here](https://support.arduino.cc/hc/en-us/articles/360016495679-Fix-port-access-on-Linux).*
***TODO, Refine Romi programming instructions***


## Setting up the SD-Card
1. Install the [Raspberry Pi Imager](https://www.raspberrypi.com/software/)
2. Set target platform to **Raspberry Pi 4**
3. Select OS as **General Purpose > Ubuntu Server 22.04.4 LTS (64-Bit)**
4. Select SD Card as target device (at least 32 GB)
5. Enter advanced customisation <br>
    i. Configure username + password <br>
        User:       *ubuntu* <br>
        Password:   *95000sriaman* <br>
    ii. Configure wireless connection <br>
        SSID:       *lightrobot* <br>
        Password:   *lightrobot2023* <br>
6. Format SD card with customised OS


## Create a wireless network
- Setup a local network, with a Wireless Access Point. <br>
    SSID:       *lightrobot* <br>
    Password:   *lightrobot2023* <br>
- On Windows 11, you can create a [Mobile Hotspot](https://techcommunity.microsoft.com/t5/windows-11/how-to-set-up-a-mobile-hotspot-in-windows-11/m-p/2764785).


## Raspberry Pi basic setup
- On Windows, [WinSCP](https://winscp.net/eng/download.php) (includes PuTTY) can be used.
- Alternatively, [PuTTY](https://www.putty.org/) can be used as a standalone.

1. Inspect the devices connected to the *lightrobot* network and identify the IP address of the EBug.
2. SSH into the EBug using the identified IP address.
3. Upgrade existing packages
    ```sh
    sudo apt update
    sudo apt upgrade
    ```
4. Ensure time is synced to correct timezone
    ```sh 
    sudo timedatectl set-timezone Australia/Melbourne
    sudo timedatectl set-ntp true
    ```
5. Update Raspberry Pi firmware
    ```sh
    sudo apt install linux-modules-extra-raspi
    sudo rpi-eeprom-update
    sudo reboot
    ```
6. Install Git and [Docker](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
    ```sh
    sudo apt install git
    
    # Follow official Docker install here:
    # https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository
    ```
7. Add *ubuntu* user to Docker user group
    ```sh
    sudo usermod -aG docker ${USER}
    su - ${USER}
    ```

## Raspberry Pi I2C setup
Pololu Tutorial [here](https://www.pololu.com/blog/663/building-a-raspberry-pi-robot-with-the-romi-chassis).
1. Install dependencies
    ```sh
    sudo apt install i2c-tools raspi-config
    ```
2. Enable I2C interface
    ```sh
    sudo raspi-config

    # > 3 Interface Options
    #   > I5 I2C
    #     > Enable
    
    # Inspect I2C status
    sudo i2cdetect -y 1
    ```
3. Increase I2C communication rate
    ```sh
    sudo nano /boot/config.txt

    # At the end of the config file, add:
    # dtparam=i2c_arm_baudrate=400000
    ```
7. Add *ubuntu* user to I2C user group
    ```sh
    sudo usermod -aG i2c,dialout ${USER}
    su - ${USER}
    ```
8. Validate i2c-1 device
    ```sh
    ls -la /dev/i2c-1

    # This should show the i2c device being owned by the root user, and dialout grouo
    ```

## Identify USB Camera Devices
1. Install Video-4-Linux Utils, and identify devices
    ```sh
    sudo apt install v4l-utils
    v4l2-ctl --list-devices
    ```
2. Typically, the four cameras will be mounted to `/dev/video0`, `/dev/video2`, `/dev/video4`, and `/dev/video6`.

## Running the containers
Follow the instructions [here](/docs/Deploy%20Containers.md) to build and deploy the relevant container(s).