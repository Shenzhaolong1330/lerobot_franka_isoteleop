#!/usr/bin/env bash
set -euo pipefail

MAP_NAME="${1:-franka_pgi_gripper}"
RULE_FILE="/etc/udev/rules.d/99-franka-pgi-gripper.rules"

mapfile -t USB_DEVICES < <(compgen -G "/dev/ttyUSB*" || true)
if [[ "${#USB_DEVICES[@]}" -ne 1 ]]; then
    echo "Please connect exactly one /dev/ttyUSB* device; found ${#USB_DEVICES[@]}."
    exit 1
fi

DEVICE="${USB_DEVICES[0]}"
echo "Detected PGI serial device: $DEVICE"

ID_VENDOR=""
ID_PRODUCT=""
SERIAL=""
while IFS='=' read -r key value; do
    case "$key" in
        ID_VENDOR_ID) ID_VENDOR="$value" ;;
        ID_MODEL_ID) ID_PRODUCT="$value" ;;
        ID_SERIAL_SHORT) SERIAL="$value" ;;
    esac
done < <(udevadm info -q property -n "$DEVICE")

if [[ -z "$ID_VENDOR" || -z "$ID_PRODUCT" || -z "$SERIAL" ]]; then
    echo "Could not read USB vendor, product, or serial from $DEVICE."
    exit 1
fi

echo "USB identity: vendor=$ID_VENDOR product=$ID_PRODUCT serial=$SERIAL"
echo "Writing udev rule (sudo permission is required)..."
RULE="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$ID_VENDOR\", ATTRS{idProduct}==\"$ID_PRODUCT\", ATTRS{serial}==\"$SERIAL\", GROUP=\"dialout\", MODE=\"0660\", SYMLINK+=\"$MAP_NAME\""
sudo touch "$RULE_FILE"
sudo sed -i "/SYMLINK+=\"$MAP_NAME\"/d" "$RULE_FILE"
printf '%s\n' "$RULE" | sudo tee -a "$RULE_FILE" >/dev/null
echo "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo udevadm settle

echo "Mapped $DEVICE as /dev/$MAP_NAME. Replug the gripper if the link is not visible yet."
