#!/bin/bash

# Usage: sudo ./map_franka_usb.sh franka_left_gripper
# Argument 1: mapping name
MAP_NAME=$1

if [ -z "$MAP_NAME" ]; then
    echo "Usage: $0 <mapping_name>"
    exit 1
fi

RULE_FILE="/etc/udev/rules.d/99-franka.rules"

# 1. Check current number of /dev/ttyUSB devices
USB_DEVICES=($(ls /dev/ttyUSB* 2>/dev/null))
NUM_USB=${#USB_DEVICES[@]}

if [ "$NUM_USB" -eq 0 ]; then
    echo "No /dev/ttyUSB* devices detected."
    exit 1
elif [ "$NUM_USB" -gt 1 ]; then
    echo "Only one USB device should be connected for mapping. Currently found $NUM_USB devices."
    exit 1
fi

DEVICE=${USB_DEVICES[0]}

# 2. Get idVendor, idProduct, serial of the device
IDVENDOR=$(udevadm info -a -n $DEVICE | grep 'ATTRS{idVendor}' | head -n1 | awk -F'"' '{print $2}')
IDPRODUCT=$(udevadm info -a -n $DEVICE | grep 'ATTRS{idProduct}' | head -n1 | awk -F'"' '{print $2}')
SERIAL=$(udevadm info -a -n $DEVICE | grep 'ATTRS{serial}' | head -n1 | awk -F'"' '{print $2}')

if [ -z "$SERIAL" ]; then
    echo "Failed to get device serial number."
    exit 1
fi

# 3. Check if rule file exists
if [ ! -f "$RULE_FILE" ]; then
    sudo touch "$RULE_FILE"
fi

# 4. Check if mapping name already exists
EXISTING_LINE=$(grep "SYMLINK+=\"$MAP_NAME\"" $RULE_FILE)

if [ -n "$EXISTING_LINE" ]; then
    EXISTING_SERIAL=$(echo $EXISTING_LINE | grep -o 'ATTRS{serial}=="[^"]*"' | awk -F'"' '{print $2}')
    if [ "$EXISTING_SERIAL" == "$SERIAL" ]; then
        echo "Rule already exists with the same serial number. No changes made."
    else
        echo "Rule exists but with a different serial number. Replacing with new one."
        sudo sed -i "/SYMLINK+=\"$MAP_NAME\"/d" $RULE_FILE
        echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$IDVENDOR\", ATTRS{idProduct}==\"$IDPRODUCT\", ATTRS{serial}==\"$SERIAL\", SYMLINK+=\"$MAP_NAME\"" | sudo tee -a $RULE_FILE
    fi
else
    echo "Adding new rule..."
    echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$IDVENDOR\", ATTRS{idProduct}==\"$IDPRODUCT\", ATTRS{serial}==\"$SERIAL\", SYMLINK+=\"$MAP_NAME\"" | sudo tee -a $RULE_FILE
fi

# 5. Reload udev rules
sudo udevadm control --reload
sudo udevadm trigger
sleep 2

# 6. Verify mapping
if [ -e "/dev/$MAP_NAME" ]; then
    LINK_TARGET=$(readlink -f "/dev/$MAP_NAME")
    echo "Mapping successful: /dev/$MAP_NAME -> $LINK_TARGET"
    SERIAL_CHECK=$(udevadm info -a -n "$LINK_TARGET" | grep 'ATTRS{serial}' | head -n1 | awk -F'"' '{print $2}')
    echo "   Serial number: $SERIAL_CHECK"
else
    echo "Mapping failed: /dev/$MAP_NAME does not exist."
fi
