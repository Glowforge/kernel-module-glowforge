#!/bin/sh

LEDDIR="/sys/class/leds/$1"
TRIGGER="$LEDDIR/trigger"
TARGET="$LEDDIR/target"
SPEED="$LEDDIR/speed"
PULSEON="$LEDDIR/pulse_on"
PULSEOFF="$LEDDIR/pulse_off"

if [ ! "$TRIGGER" ]; then
  echo "usage: $0 ledname"
  exit 1
fi

echo "Brightness:  <1> through <-> keys"
echo "Speed:       <Q> through <[> keys"
echo "Pulse speed: <A> through <'> keys"

echo "smooth" > $TRIGGER

setpulse () {
  # "tee" returns an I/O error... why doesn't that work?
  echo "$1" > "$PULSEON"
  echo "$1" > "$PULSEOFF"
}

while IFS= read -r -n1 ch
do
  case $ch in
    1)  echo 0    > $TARGET ;;
    2)  echo 26   > $TARGET ;;
    3)  echo 51   > $TARGET ;;
    4)  echo 77   > $TARGET ;;
    5)  echo 102  > $TARGET ;;
    6)  echo 128  > $TARGET ;;
    7)  echo 153  > $TARGET ;;
    8)  echo 179  > $TARGET ;;
    9)  echo 204  > $TARGET ;;
    0)  echo 230  > $TARGET ;;
    -)  echo 255  > $TARGET ;;
    q)  echo 1    > $SPEED ;;
    w)  echo 4    > $SPEED ;;
    e)  echo 8    > $SPEED ;;
    r)  echo 16   > $SPEED ;;
    t)  echo 32   > $SPEED ;;
    y)  echo 53   > $SPEED ;;
    u)  echo 75   > $SPEED ;;
    i)  echo 96   > $SPEED ;;
    o)  echo 117  > $SPEED ;;
    p)  echo 138  > $SPEED ;;
    [)  echo 160  > $SPEED ;;
    a)  setpulse 0    ;;
    s)  setpulse 250  ;;
    d)  setpulse 500  ;;
    f)  setpulse 750  ;;
    g)  setpulse 1000 ;;
    h)  setpulse 1500 ;;
    j)  setpulse 2000 ;;
    k)  setpulse 3000 ;;
    l)  setpulse 4000 ;;
    \;) setpulse 5000 ;;
    \') setpulse 6000 ;;
  esac
done