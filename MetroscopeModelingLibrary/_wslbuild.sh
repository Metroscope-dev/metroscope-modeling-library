#!/bin/bash

# must use bash for array support

# wrapper for dsbuild.sh

which dos2unix
if [ ! $? = 0 ]
then
    echo "You need to install dos2unix in your WSL, e.g. using \"sudo apt install dos2unix\"" > dslog.txt
    exit
fi

# fetch environment variables
dos2unix ./wslenv.sh
if [ ! $? = 0 ]
then
    echo "Cannot convert environment script to Linux, assuming dos2unix is correctly installed then:">dslog.txt
    echo "Seems we cannot change file permission in WSL; due to missing mount option" >> dslog.txt
    echo "Create/edit /etc/wsl.conf and add the following two lines and reboot computer" >> dslog.txt
    echo "[automount]" >> dslog.txt
    echo "options = \"metadata\"" >> dslog.txt
    exit 1
fi
. ./wslenv.sh

# convert to wsl path
DYMOLA_WIN=$DYMOLA
export DYMOLA=$(wslpath "$DYMOLA");

if [ ! -d "$DYMOLA" ]
then
    # check if mounted drive or UNC path and provide proper error messages
    echo $DYMOLA_WIN|grep ^[[:alpha:]]:
    if [ $? -eq 0 ]
    then
        # mounted drive
        DRIVE=$(echo $DYMOLA_WIN | sed -E 's/^([[:alpha:]]):.*/\1/')
        DRIVE_LOWER=$(echo $DRIVE | tr '[:upper:]' '[:lower:]')
        echo "Dymola seems to be run from a Windows mounted location ($DRIVE:)" >> dslog.txt
        echo "If so, you need to mount the drive also from WSL, using e.g. the non-persistent variant mkdir + mount:" >> dslog.txt
        echo "mkdir /mnt/$DRIVE_LOWER" >> dslog.txt
        echo "sudo mount -t drvfs $DRIVE: /mnt/$DRIVE_LOWER" >> dslog.txt
    else
        # check if UNC path
        echo DYMOLA_WIN=$DYMOLA_WIN >> dslog.txt
        echo DYMOLA=$DYMOLA >> dslog.txt
        echo $DYMOLA_WIN|grep ^//
        if [ $? -eq 0 ]
        then
            echo "WSL cannot handle when Dymola started from UNC path: $DYMOLA_WIN." >> dslog.txt
            echo "You need to mount the path on a drive and then restart Dymola using that drive." >> dslog.txt
        else
		    echo "Could not get WSL path from: $DYMOLA_WIN" >> dslog.txt
        fi
    fi
    exit 1
fi

IFS_SAVED=$IFS
IFS=';'

# prepare paths to suit gcc preprocessor
declare -a IOPTS
for p in $DYMOLAINC
do
  p=$(wslpath "$p");
  IOPTS=$IOPTS" -I '$p'"
done

declare -a LOPTS
for p in $DYMOLALIB
do
  p=$(wslpath "$p");
  LOPTS=$LOPTS" -L '$p'"
done

IFS=$IFS_SAVED

# invoke dsbuild.sh
"$DYMOLA/bin/dsbuild.sh" "$IOPTS" "$LOPTS" "$@"
