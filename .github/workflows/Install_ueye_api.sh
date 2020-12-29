#!/bin/bash

SW_VERSION=4.94

echo "Installing wget..." 
DEBIAN_FRONTEND=noninteractive apt-get -qq update
DEBIAN_FRONTEND=noninteractive apt-get -qq --assume-yes install wget

echo "Downloading and extracting ids ${SW_VERSION}..."
wget -q --tries=2 https://de.ids-imaging.com/files/downloads/ids-software-suite/software/linux-desktop/ids-software-suite-linux-${SW_VERSION}-64.tgz -P /tmp
mkdir /tmp/ids-software-suite-linux
tar -xzf /tmp/ids-software-suite-linux-${SW_VERSION}-64.tgz -C /tmp/ids-software-suite-linux
/tmp/ids-software-suite-linux/ueye_*_amd64.run --auto


if [ -z "$(ldconfig -p | grep ueye_api)" ]
then
      echo "ERROR: ueye_api not found after installation"
      exit 1
else
      echo "Library ueye_api has been installed successfully"
fi
