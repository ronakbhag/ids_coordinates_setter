#!/bin/sh
echo "Hello world, i'm supposed to be a camera script"
echo "Will wait 10 seconds"

x=1
while [ $x -le 20 ]
do
  echo "Welcome $x times"
  x=$(( $x + 1 ))
  sleep 1
done

echo "Done!"
