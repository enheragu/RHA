

GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

printGreen ()
{
	echo -e "${GREEN}$1${NC}"
}

printRed () 
{
        echo -e "${RED}$1${NC}"
}

printBlue () 
{
        echo -e "${BLUE}$1${NC}"
}



#sudo apt-get install software-properties-common
#printGreen "- Innstalling sublime text editor"
#sudo add-apt-repository ppa_webupd8team/sublime-text-3
#sudo apt-get update
#sudo apt-get install sublime-text-installer -y

printGreen "- Installing terminator"
sudo apt-get install terminator -y

printGreen "- Installing cmake"
sudo apt-get install cmake -y

printGreen "- Cloning RHA project from GITHUB repository"
git clone https://github.com/enheragu/RHA.git

printGreen "- Cloning Piduino from GITHUB repository"
git clone https://github.com/enheragu/RasPiArduino

cd ${HOME}/rha_project/RasPiArduino/tools/
make clean
make all

printGreen "- Creating simbolic link to piduino"
cd ${HOME}/rha_project/RHA/SW/RHAspberry_lib
ln -s $HOME/rha_project/RasPiArduino/cores/piduino
#ln -s $HOME/rha_project/RasPiArduino/variants/bplus/pins_arduino.h
ln -s $HOME/rha_project/RasPiArduino/libraries/Wire
ln -s $HOME/rha_project/RasPiArduino/tools/build/libPiDuino.so


########## RASPBERRY PI CONFIG ##########

#  cat > /boot/cmdline.txt <<EOL
#  dwc_otg.lpm_enable=0 console=tty1 root=/dev/mmcblk0p2 rootfstype=ext4 elevator=deadline fsck.repair=yes rootwait
#  EOL

#  systemctl disable serial-getty@ttyAMA0
#  default system pasword: raspberry

## To get Raspberry IP:
# hostname -I
# 192.168.1.40
