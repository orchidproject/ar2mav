## AR2MAV 

AR2MAV - A python based module for interfacing a Parrot's ARDrone 2.0 trough the full capabilities of MAVLink. The module sanitises the manual commands and RC channels of ARDrone 2.0 to hide them and present the drone fully as a MAVLink UAV. Also this allows multiple ARDrones to be connected to a network for Swarm control.
The module was developed as part of the MOSAIC project.

* ArDrone official website - http://ardrone2.parrot.com/
* MAVlink official website - http://qgroundcontrol.org/mavlink/start
* MOSAIC Project official website - http://mosaicproject.info/research.php

### Installation ###
1. Install git 1.8.3+ [Easy way to do this](http://linuxg.net/how-to-install-git-1-8-4-on-ubuntu-14-0413-1013-0412-1012-04-linux-mint-16151413-pear-os-87-and-elementary-os-0-2/)
2. Clone repository and change directory to the repo (cd ar2mav)
3. git submodule init
4. git submodule update --init --remote
5. Modify mavlink/message_definitions/v1.0/common.xml lines 1175,1176,1777 have double '<' on the closing </field> tag, should have only one.
6. chmod 777 mavlink/pymavlink/setup.py
7. cd ar2mav/mavlink/pymavlink 
8. python setup.py install
9. Installation is complete with a stable version of MAVLink

### Multiple Drones 

For enabling the connection of multiple drones several steps are required:

1. Open autoconnect.sh and adjust the options for your spesific infrastructure network
2. Connect with your computer the network emitted by the drone
2. telnet 192.168.1.1
3. cd home/default (on the drone)
4. mkdir wifi (on the drone)
5. cd wifi (on the drone)
6. nc -l -p 1234 > autoconnect.sh (on the drone)
7. Open a new terminal and change directory to the main repository folder
8. nc -w 3 192.168.1.1 1234 < autoconnect.sh (on your computer)
9. chmod 777 autoconnect.sh (on the drone)

This should have your drone set up for connection. If you want a one time connection just do`./autoconnect.sh &`, if you want this script to be run on boot up of the drone you have to edit the file /bin/wifi_setup.sh on the drone and add the following line at the bottom:

`/home/default/wifi/autoconnect.sh &`

Contributors
============
Alexander Botev

Luke Teacy

License
=======
Copyright (c) 2014, University of Southampton
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the {organization} nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
