#!/usr/bin/env python

from power_switching_tools_ros.power_switching_driver_base import PowerSwitchingDriverBase
import os
import rospy
import subprocess


class UsbPppsHubDriver(PowerSwitchingDriverBase):

    def __init__(self):
        try:
            hub_loc = rospy.get_param('~hub_location')
            hub_port = rospy.get_param('~hub_port')
        except KeyError as e:
            rospy.logfatal(
                '[{}] Please set hub_location and hub_port'.format(
                    rospy.get_name()))
            rospy.signal_shutdown('Could not get required params')
            return
        if isinstance(hub_loc, list):
            self.hub_locs = [str(x) for x in hub_loc]
        else:
            self.hub_locs = [str(hub_loc)]
        if isinstance(hub_port, list):
            self.hub_ports = [str(x) for x in hub_port]
        else:
            self.hub_ports = [str(hub_port)]
        if len(self.hub_locs) != len(self.hub_ports):
            rospy.logfatal(
                '[{}] hub_location and hub_port '
                'should be the same length'.format(
                    rospy.get_name()))
            rospy.signal_shutdown('Param shapes are strange')
            return
        uhubctl_exec_str = os.path.expanduser(
            rospy.get_param('~uhubctl_executable', 'uhubctl'))
        self.uhubctl_exec = uhubctl_exec_str.split(' ')

        # Check if provided parameters work
        for hub_loc, hub_port in zip(self.hub_locs, self.hub_ports):
            try:
                cmd = self.uhubctl_exec[:]
                cmd += ['-l', hub_loc,
                        '-p', hub_port]
                res = subprocess.run(cmd, capture_output=True, text=True)
            except FileNotFoundError as e:
                rospy.logfatal(
                    '[{}] {} does not exist. Install that executable '
                    '(e.g., "sudo apt install uhubctl")'.format(
                        rospy.get_name(), cmd[0]))
                rospy.signal_shutdown('uhubctl is not installed')
                return
            if ((res.returncode == 0
                 and 'Port {}:'.format(hub_port) not in res.stdout)
                or ('Bad port spec' in res.stderr)):
                rospy.logfatal(
                    '[{}] hub_port {} is invalid'.format(
                        rospy.get_name(), hub_port))
                rospy.signal_shutdown('hub_port is invalid')
                return
            if res.returncode != 0:
                if ('No compatible smart hubs '
                    'detected at location {}'.format(
                        hub_loc) in res.stderr):
                    rospy.logfatal(
                        '[{}] hub_location {} is invalid'.format(
                            rospy.get_name(), hub_loc))
                    rospy.signal_shutdown('hub_location is invalid')
                    return
                else:
                    rospy.logfatal(
                        '[{}] uhubctl returned some error'.format(
                            rospy.get_name()))
                    rospy.logfatal(
                        '[{}] Error message returned from uhubctl: {}'.format(
                            rospy.get_name(), res.stderr))
                    rospy.signal_shutdown('uhubctl returned some error')
                    return

        super(UsbPppsHubDriver, self).__init__()

    def _pwr(self, is_on):
        for hub_loc, hub_port in zip(self.hub_locs, self.hub_ports):
            cmd = self.uhubctl_exec[:]
            cmd += ['-a', str(int(is_on)),
                    '-l', hub_loc,
                    '-p', hub_port]
            res = subprocess.run(cmd, capture_output=True, text=True)
            if res.returncode != 0:
                rospy.logerr(
                    '[{}] uhubctl returned some error. '
                    'Check if hub is connected'.format(
                        rospy.get_name()))
                rospy.logerr(
                    '[{}] Error message returned from uhubctl: {}'.format(
                        rospy.get_name(), res.stderr))
                return False
        rospy.loginfo(
            '[{}] Changed power state to {}'.format(
                rospy.get_name(), is_on))
        return True


if __name__ == '__main__':
    rospy.init_node('usb_ppps_hub_driver')
    app = UsbPppsHubDriver()
    rospy.spin()
