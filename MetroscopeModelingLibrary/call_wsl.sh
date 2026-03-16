#!/bin/sh

# Wrapper for WSL executables such as dymosim etc

# fetch environment variables to get DYMOLA etc
case "$1" in
	*dymosim)
		# ordinary dsmodel
		. ./wslenv.sh
		;;
	*)
		# function
		wslenv_func=wslenv_$1.sh
		if [ ! -f $wslenv_func ]
		then
		  # new function translation, save environment for subsequent runs
		  cp wslenv.sh $wslenv_func
		fi
		. ./$wslenv_func
		;;
esac

# make sure there is Linux style line endings
#if [ -f dsin.txt ]
#then
	dos2unix *dsin.txt
#fi

prog=$1
shift

export LD_LIBRARY_PATH="$(wslpath "$DYMOLA")/bin/lib64"

IFS_SAVED=$IFS
IFS=';'
for p in $DYMOLALIB
do
  p=$(wslpath "$p");
  LD_LIBRARY_PATH=$LD_LIBRARY_PATH":$p"
done
IFS=$IFS_SAVED

"./$prog" "$@"
