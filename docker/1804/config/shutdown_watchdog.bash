#!/bin/bash

date
sleep 60

while [ ! -f ~/log/shutdownrequest ] && [ ! -f  ~/log/rebootrequest ]; do

  sleep 5

  if [ -f  ~/log/dockerrestart ]; then
    rm -f ~/log/dockerrestart
    echo "docker restart..."
    docker restart rchomeedu
  fi

done

echo "Closing docker..."
docker stop rchomeedu
docker stop nginx

if [ -f  ~/log/shutdownrequest ]; then
  rm -f ~/log/shutdownrequest
  echo "Shutdown..."
  sudo halt
fi

if [ -f  ~/log/rebootrequest ]; then
  rm -f ~/log/rebootrequest
  echo "Reboot..."
  sudo reboot
fi
