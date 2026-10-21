#!/bin/bash

# must use bash for array support

# wrapper for dsbuildfmu.sh

# fetch environment variables
dos2unix ./wslenv.sh
. ./wslenv.sh

# convert to wsl path
export DYMOLA=$(wslpath "$DYMOLA");

IFS_SAVED=$IFS
IFS=';'

# prepare paths to suit gcc preprocessor
declare -a IOPTS
for p in $DYMOLAINC
do
  p=$(wslpath "$p");
  IOPTS=$IOPTS" -I '$p'"
done

declare -a SOPTS
for p in $DYMOLASRC
do
  p=$(wslpath "$p");
  SOPTS=$SOPTS" -S '$p'"
done

declare -a LOPTS
for p in $DYMOLALIB
do
  p=$(wslpath "$p");
  LOPTS=$LOPTS" -L '$p'"
done

IFS=$IFS_SAVED

# invoke dsbuildfmu.sh
"$DYMOLA/bin/dsbuildfmu.sh" "$IOPTS" "$SOPTS" "$LOPTS" "$@"
