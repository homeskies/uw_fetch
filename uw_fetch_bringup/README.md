# uw_fetch_bringup

Our fork of [fetch_bringup](https://github.com/fetchrobotics/fetch_robots/tree/melodic-devel/fetch_bringup). Holds the main launch files for software that runs underneath all tasks.

### Additional Configuration

We've also tracked additional system configuration for the robot in the `config` folder.

To enable internet sharing with the external computer, make sure `iptables-persistent` and `netfilter-persistent` are installed and enabled.

Make sure you've enabled ip forwarding in `/etc/sysctl.conf`. Uncomment this in that file:

   # net.ipv4.ip_forward=1

Then place the `rules.v4` in `/etc/iptables` on the robot.
