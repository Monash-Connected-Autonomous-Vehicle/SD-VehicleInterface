sudo modprobe vcan && sudo ip link add can0 type vcan && sudo ip link set can0 up

# Check if the previous command was successful
if [ $? -eq 0 ]
then
    echo "CAN bus setup successful"
else
    echo "Error. CAN bus setup failed. Disconnect and try again"
fi
