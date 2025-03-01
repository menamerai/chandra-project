# Development Guide

This project has a lot of moving parts. A lot of it are very weird parts. In the name of reproducability, I will write down the specifics of how I install things, and what I do differently, how I troubleshooted things. Hopefully, this will make things easier.

## Turtlebot3 Setup

The project's main focus is on modularity, so we should be able to work on the turtlebot3 as well as on the GhostRobotics V60. Moreover, we are more likely to be able to use the turtlebot3 for demo purposes, and less on the V60. For this reason, we can discuss installation for the turtlebot3.

### System Requirements

- OS: Ubuntu 22.04 ideally, otherwise WSL2 running Ubuntu 22.04 should work with extra configuration steps.
- Storage: Enough for ROS 2 Humble.
- Hardware: turtlebot3 burger.

### Installation Guide

Follow the guide in the official link:

https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup

Make sure to use the Humble tab to install ROS 2 Humble and do thing in that environment.

### WSL2 Troubleshooting

The main problem that I ran into with WSL2 is that WSL2's networking implementation is incompatible with what ROS 2 multicast traffic. WSL1 uses the network stack from Windows directly, but WSL2 operate in a virtualized network that doesn't always handle multicast packets across network boundaries. To test if you have this issue, on the PC, run

```
ros2 multicast receive
```

and on the turtlebot's built-in Raspberry Pi, run

```
ros2 multicast send
```

You should receive a message titled "Hello world" on the WSL2 machine. For safety, you can also try passing messages in the other direction.

If doing this doesn't work, we can switch WSL2 into the mirrored networking mode, which requires Windows 11 version 22H2 or higher. Open File Explorer and type `%USERPROFILE%` on the navigation bar. Then, create a file called `.wslconfig` in that folder and edit the file with:

```
[wsl2]
networkingMode=mirrored
```

Then, restart WSL2 by running on PowerShell as admin

```
wsl --shutdown
```

Then reopen your WSL2 machine.

You should try the message passing test again. In my exeprience, this is enough to allow traffic to pass one way, but a firewall would still block multicast traffic from the Pi. To disable this, run PowerShell as admin and execute the command

```
Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
```

Then, open the desired WSL2 instance and run

```
sudo ufw allow in proto udp to 224.0.0.0/4
sudo ufw allow in proto udp from 224.0.0.0/4
```

and you might also need to allow specific IP subnet traffic with

```
sudo ufw allow from 192.168.0.0/24
```

(replace with your actual subnet if it is different). If none of this is working, you can also consider downgrading to WSL1, but I have not tested this

```
wsl --set-version Ubuntu 1
```