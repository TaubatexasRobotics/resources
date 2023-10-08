#!/bin/bash -e

cd "$(dirname $0)"

# Keep this list alphabetically sorted
BASE_TESTS="
autonomous-navX-neo-with-spark
digitalinput
drive/arcade-drive
drive/cheezy-drive
drive/combined-drive
drive/drivetrain-navX
drive/tank-drive
encoder
max-neo-encoder
navX
neo-motor
photonvision
pneumatics
shuffleboard
smartdashboard/combobox
smartdashboard
uart-usb-arduino
"

IGNORED_TESTS="
2023-base-code/version1
2023-base-code/version2
button-binding
autonomous-forward-two-meters/version1
autonomous-forward-two-meters/version2
autonomous-forward-two-meters/version3
autonomous-forward-two-meters/version4
autonomous-forward-two-meters/version5
pid
networktables
"

ALL_TESTS="${BASE_TESTS}"
EVERY_TESTS="${ALL_TESTS} ${IGNORED_TESTS}"
TESTS="${ALL_TESTS}"

TMPD=$(mktemp -d)
trap 'rm -rf "$TMPD"' EXIT

# Ensure that when new samples are added, they are added to the list of things
# to test. Otherwise, exit.
for i in ${EVERY_TESTS}; do
  echo ./$i/robot.py
done | sort > $TMPD/a

find . -name robot.py | sort > $TMPD/b

#if ! diff -u $TMPD/a $TMPD/b; then
#
#  if [ -z "$FORCE_ANYWAYS" ]; then
#    echo "ERROR: Not every robot.py file is in the list of tests!"
#    exit 1
#  fi
#fi

for t in ${TESTS}; do
  pushd $t > /dev/null
  pwd
  if ! python3 robot.py test --builtin "${@:2}"; then
    EC=$?
    echo "Test in $(pwd) failed"
    exit 1
  fi
  popd > /dev/null
done

echo "All tests successful!"
