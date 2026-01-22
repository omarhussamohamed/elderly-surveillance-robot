#!/bin/bash
# Fix GPIO permissions for Jetson Nano
# Run this script once, then reboot

echo "Fixing GPIO permissions for user: $USER"

# Add user to gpio group
sudo usermod -a -G gpio $USER

# Create udev rule for GPIO access
sudo bash -c 'cat > /etc/udev/rules.d/99-gpio.rules << EOF
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", GROUP="gpio", MODE="0660"
SUBSYSTEM=="gpio", ACTION=="add", PROGRAM="/bin/sh -c \"chgrp -R gpio /sys%p && chmod -R g+w /sys%p\""
EOF'

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

echo ""
echo "=========================================="
echo "GPIO permissions fixed!"
echo "User '$USER' added to 'gpio' group"
echo ""
echo "IMPORTANT: You must REBOOT for changes to take effect:"
echo "  sudo reboot"
echo "=========================================="
