#!/bin/bash

# camera transport layer setup
wget https://downloads.alliedvision.com/Vimba64_v6.0_Linux.tgz -O /tmp/Vimba64_v6.0_Linux.tgz
mkdir -p /tmp/Vimba64_v6.0_Linux
tar -xzf /tmp/Vimba64_v6.0_Linux.tgz -C /tmp/Vimba64_v6.0_Linux/
sudo mkdir -p /opt/VimbaGigETL
sudo mv /tmp/Vimba64_v6.0_Linux/Vimba_6_0/VimbaGigETL/CTI /opt/VimbaGigETL
sudo mv /opt/VimbaGigETL/CTI/test/x86_64bit /opt/VimbaGigETL/CTI
sudo rm -rf /opt/VimbaGigETL/CTI/test
printf "#!/bin/sh\n\n\
#Do not edit this file manually because it may be overwritten automatically.\n\
export GENICAM_GENTL64_PATH=%s\n" \
    /opt/VimbaGigETL/CTI/x86_64bit | sudo tee /etc/profile.d/VimbaGigETL_64bit.sh
sudo chmod +x /etc/profile.d/VimbaGigETL_64bit.sh

# TODO: vectornav serial port driver fetch, compile and install
