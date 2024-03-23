# can_simulator

This tool is created as a way to add a virtual CAN Bus for testing and validation purposes

# Prerequisite

This library requires that you have set up a virtual can interface:
### Start interface
1. `sudo apt-get install can-utils`
2. `sudo modprobe vcan`
3. `sudo ip link add dev vcan0 type vcan`
4. `sudo ip link set up vcan0`

### Shut down interface
This command disables the interface, but it does not remove the configuration. You can bring it back up later if needed.

```shell
sudo ip link set down vcan0
```

If you decide that you no longer need the virtual CAN interface at all, you can remove it completely with the following command. Once removed, the interface configuration is lost, and you would need to recreate it if you need it again in the future.
```shell
sudo ip link delete vcan0
```

