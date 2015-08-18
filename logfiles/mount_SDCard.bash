#!/bin/bash
sudo mount -t vfat /dev/sdc1 ./SDCard -o rw,uid=1000,gid=1000,umask=022
