# JILA-RPi-Laser-Adjuster
Python script that allows a Raspberry Pi to connect to a Newport motor controller and Allied Vision camera, and adjust piezo mirror mount according in laser position in the frame.

## Instructions for Raspberry Pi mirror adjustment device!

**Required equipment:**
- Newport Motor Controller 8742
- Newport Motorized Mirror Mount 8821
- Mako Camera U 130 B or other Vimba X compatible camera
- Raspberry Pi 4 Model B (I used the 4GB RAM version)
- NEW or formatted MicroSD Card and SD adapter (I used a 64GB card)


### Setting Up Raspberry Pi:

#### Installing RaspberryPi OS:

To install RPI OS, download the raspberry pi imager onto a pc or laptop. Insert the SD card, and within the imager, select “Raspberry Pi 4” for Raspberry Pi Device, select “Raspberry Pi OS (64-bit)” for Operating System and select your SD card for storage device. Once the OS has installed, eject the card from your device, and insert into the bottom of the Pi.

#### Initial Set Up and Connecting:
	
Once your Pi has its SD card, power it via USB-C, this is the only way it can be powered. From there, connect the Pi to a monitor, via a micro-HDMI cord, a mouse, via USB, and a keyboard, via USB. Set an easy to remember password for logging into the Pi, and choose your default browser as Chromium. Connect the Pi to the JILAn wifi, and then register the Pi to the JILA computing department. To this, go to the JILA website, and click the tools icon, and access JILA Virtual with your JILA login. From there, go to the Tools, Resources & Quicklinks section, and select “See More Forms”. From there, find the “Computer/Device Registration” form, and register the Pi. If you need help with this, find someone from the computing department on floor 2, and they can help you quickly register the Pi! 


#### Checking for updates:

Once your Pi is set up, it is important to check for updates. This process can be simply done by opening the terminal entering these commands:

```
sudo apt update 
sudo apt upgrade
```

**These commands can be copy and pasted into the terminal by copying from this doc and pasting into the terminal by right clicking and clicking paste. The usual CTRL-C and CTRL-V will not work in the terminal! **

Once you have verified that your system is up to date, check your python version by entering into the terminal

```
python -V
```

If your version is not recent, you can install a newer version with this command:

```
sudo apt install python3.12
```

Once you have run these commands, reboot your Pi, by clicking the Raspberry icon, Log out, and reboot.

#### Creating a virtual environment:

To run our program, we will need to create a virtual environment. First, create a folder on your desktop to contain the environment. To do this, navigate to the terminal and use these commands:

```
cd Desktop
python -m venv /environmentName
```

Once you have executed this, when you navigate to the environment folder on your desktop, it should now contain a folder of the same name, which contains folders such as bin, include, lib, etc. 

Now that you have created your virtual environment, you must activate it. You must do this every time you need to run a program in the environment. To do this, enter the following commands in the terminal when you are within the Desktop folder that we entered with cd:

```
cd environmentName
. environmentName/bin/activate
```

Once you have created your environment, you need to install the required python packages with the following commands. If you select all the commands and paste them together and enter, they will all run at the same time.

```
pip install -r requirements.txt
```

#### Installing Vimba X and Vimba API:

The first step of installing the Vimba API for python is to download Vimba X for Linux ARM64. Navigate to this link, https://www.alliedvision.com/en/products/software/vimba-x-sdk/, on the Pi using Chromium. Scroll down to the downloads section and download the file called “VimbaX_Setup-2023-4-Linux_ARM64.tar.gz”. Once this is downloaded, drag to file to your desktop, and place it within the folder we created. Once inside, place it within the virtual environment folder. The file should be on the same level as the bin, lib, include, and other folders that were created with the virtual environment. After placing the file on the correct level, open a new terminal and run the following command:

```
tar xvf /PATH/TO/VimbaX_Setup-2023-4-Linux_ARM64.tar.gz
````

**Paths to folders and files can be found in the top of the file viewer window**

Once you have successfully unpacked the file, use cd and the activation command to enter your virtual environment. When your environment is activated, use this command to install the Vimba API for python:

```
python -m pip install /path/to/environment/folder/VimbaX_2023-4/api/python/vmbpy-1.0.4-py3-none-any.whl
```

**NOTE, if your Vimba X installation is not recent to May, 2024, then this path and file name/version may not be the same, and you can find the correct path and name by navigating to Vimba_X->api->python**


#### Installing Newport Motor USB Drivers:

Currently, the given USB driver software that comes with the motor controller is only made for windows, as it is in the form of a .exe file. To bypass this, there are a few steps that will be needed to emulate windows on the Pi, but be warned, these take quite a while to download! The first step is to install Wine, by following this YouTube video. The only notable difference from the video is that you must install the 64 bit versions of everything, instead of the 86x, ie, Box64, etc. Now, there is an additional add-on needed to run the .exe file from Newport, called .NET. To install this, simply run the following command outside in a new terminal outside of your virtual environment:

```
winetricks dotnetsp1
```

Once .NET is installed, we can finally install the USB drivers for the Newport Motor Controller. To do so, first plug the USB drive from Newport into the Pi. It should appear on the Desktop. Next, open the applications menu (the raspberry menu in the top left), and go to System Tools, and click Wine Program Manager. Next, click the Install button and navigate to the USB drive from Newport, called “picoMotorUSB”. Click “Setup.exe” and then click the Open button. Then click 64-bit mode on 64-bit operating system and OK. This should successfully install the drivers on the Pi, but if it fails, try again. I personally had to try it twice.
