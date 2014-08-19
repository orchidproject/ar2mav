## AR2MAV 

AR2MAV - A python based module for interfacing a Parrot's ARDrone 2.0 trough the full capabilities of MAVLink. The module sanitises the manual commands and RC channels of ARDrone 2.0 to hide them and present the drone fully as a MAVLink UAV. Also this allows multiple ARDrones to be connected to a network for Swarm control.
The module was developed as part of the MOSAIC project.

* ArDrone official website - http://ardrone2.parrot.com/
* MAVlink official website - http://qgroundcontrol.org/mavlink/start
* MOSAIC Project official website - http://mosaicproject.info/research.php
The module sanitises the manual commands and RC channels of ARDrone 2.0 to hide them and present the drone fully as a MAVLink UAV. 

### Installation ###
1. Install git 1.8.3+ [Easy way to do this](http://linuxg.net/how-to-install-git-1-8-4-on-ubuntu-14-0413-1013-0412-1012-04-linux-mint-16151413-pear-os-87-and-elementary-os-0-2/)
2. Clone repository
3. git submodule init
4. git submodule update --init --remote
5. Modify ar2mav/mavlink/message_definitions/v1.0/common.xml lines 1175,1176,1777 have double '<' on the closing </field> tag, should have only one.
6. chmod 777 ar2mav/mavlink/pymavlink/setup.py
7. cd ar2mav/mavlink/pymavlink 
8. python setup.py install
9. Installation is complete with a stable version of MAVLink
