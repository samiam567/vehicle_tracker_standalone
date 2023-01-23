#!/bin/bash

# TODO: network setup
# TODO: canbus setup

# being able to receive large packets
# insert the following lines to /etc/sysctl.conf if they do not exist
# # for AVT Vimba cameras
sudo sed -i "/^#[[:blank:]]\+for[[:blank:]]\+AVT[[:blank:]]\+Vimba[[:blank:]]\+cameras/h;
                \${x;/^\$/{s//# for AVT Vimba cameras/;H};x}" /etc/sysctl.conf
# net.core.rmem_max=26214400
sudo sed -i "/^net.core.rmem_max/{h;s/rmem_max.*/rmem_max=26214400/};
                \${x;/^\$/{s//net.core.rmem_max=26214400/;H};x}" /etc/sysctl.conf
# net.core.rmem_default=26214400
sudo sed -i "/^net.core.rmem_default/{h;s/rmem_default.*/rmem_default=26214400/};
                \${x;/^\$/{s//net.core.rmem_default=26214400/;H};x}" /etc/sysctl.conf
# # improve network throughput
sudo sed -i "/^#[[:blank:]]\+improve[[:blank:]]\+network[[:blank:]]\+throughput/h;
                \${x;/^\$/{s//# improve network throughput/;H};x}" /etc/sysctl.conf
# net.core.default_qdisc=fq_codel
sudo sed -i "/^net.core.default_qdisc/{h;s/default_qdisc.*/default_qdisc=fq_codel/};
                \${x;/^\$/{s//net.core.default_qdisc=fq_codel/;H};x}" /etc/sysctl.conf
# net.ipv4.tcp_window_scaling=1
sudo sed -i "/^net.ipv4.tcp_window_scaling/{h;s/tcp_window_scaling.*/tcp_window_scaling=1/};
                \${x;/^\$/{s//net.ipv4.tcp_window_scaling=1/;H};x}" /etc/sysctl.conf
# net.ipv4.tcp_congestion_control=bbr
sudo sed -i "/^net.ipv4.tcp_congestion_control/{h;s/tcp_congestion_control.*/tcp_congestion_control=bbr/};
                \${x;/^\$/{s//net.ipv4.tcp_congestion_control=bbr/;H};x}" /etc/sysctl.conf

# being able to access serial port devices
sudo usermod -aG dialout $USER
