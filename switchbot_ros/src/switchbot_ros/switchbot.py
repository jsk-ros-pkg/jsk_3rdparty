from __future__ import print_function

import json
import os.path
import requests


class SwitchBotAPIClient(object):
    """
    For Using SwitchBot via official API.
    Please see https://github.com/OpenWonderLabs/SwitchBotAPI for details.
    """
    def __init__(self, token):
        self._host_domain = "https://api.switch-bot.com/v1.0/"
        self.token = token
        self.device_list = None
        self.infrared_remote_list = None
        self.scene_list = None
        self.device_name_id = {}
        self.scene_name_id = {}
        self.update_device_list()
        self.update_scene_list()


    def request(self, method='GET', devices_or_scenes='devices', service_id='', service='', json_body=None):
        """
        Execute HTTP request
        """
        if devices_or_scenes not in ['devices', 'scenes']:
            raise ValueError('Please set devices_or_scenes variable devices or scenes')

        url = os.path.join(self._host_domain, devices_or_scenes, service_id, service)

        if method == 'GET':
            response = requests.get(
                url,
                headers={'Authorization': self.token}
            )
        elif method == 'POST':
            response = requests.post(
                url,
                json_body,
                headers={
                    'Content-Type': 'application/json; charset=utf8',
                    'Authorization': self.token
            })
        else:
            raise ValueError('Got unexpected http request method. Please use GET or POST.')

        response_json = response.json()

        # Catch the HTTP 4XX, 5XX error
        try:
            response.raise_for_status()
        except requests.exceptions.RequestException as e:
            if response.status_code == 422:
                raise InvalidRequestError()
            elif response.status_code == 429:
                raise ExceededRequestError()
            else:
                raise e
        else:
            if response_json['statusCode'] == 100:
                return response_json
            elif response_json['statusCode'] == 151:
                raise DeviceTypeError()
            elif response_json['statusCode'] == 152:
                raise DeviceNotFoundError()
            elif response_json['statusCode'] == 160:
                raise CommandNotSupportedError()
            elif response_json['statusCode'] == 161:
                raise DeviceOfflineError()
            elif response_json['statusCode'] == 171:
                raise HubDeviceOfflineError()
            elif response_json['statusCode'] == 190:
                raise DeviceInternalError()
            else:
                raise ValueError("Got unknown status code : " + str(response_json['statusCode']))


    def update_device_list(self):
        """
        Update the information of deviceIds, deviceNames, deviceBots.
        """
        res = self.request()
        self.device_list = res['body']['deviceList']
        self.infrared_remote_list = res['body']['infraredRemoteList']
        for device in self.device_list:
            self.device_name_id[device['deviceName']] = device['deviceId']
        for infrated_remote in self.infrared_remote_list:
            self.device_name_id[device['deviceName']] = device['deviceId']

        return self.device_list, self.infrared_remote_list


    def update_scene_list(self):
        """
        Update the information of sceneIDs, sceneNames.
        """
        self.scene_list = self.request(devices_or_scenes='scenes')['body']
        for scene in self.scene_list:
            self.scene_name_id[scene['sceneName']] = device['sceneId']

        return self.scene_list


    def device_status(self, device_id=None,  device_name=None):
        """
        Get the device status.
        """
        if device_id:
            pass
        elif device_name:
            device_id = self.device_name_id[device_name]
        else:
            raise ValueError("Please set device_id or device_name.")

        return self.request(service_id=device_id, service='status')['body']


    def control_device(self, command, parameter='default', command_type='command', device_id=None, device_name=None):
        """
        Send Command to the device. Please see https://github.com/OpenWonderLabs/SwitchBotAPI#send-device-control-commands for command options.
        """
        json_body = json.dumps({
            "command": command,
            "parameter": parameter,
            "commandType": command_type
        })
        if device_id:
            pass
        elif device_name:
            device_id = self.device_name_id[device_name]
        else:
            raise ValueError("Please set device_id or device_name.")
        
        return self.request(method='POST', service_id=device_id, service='commands', json_body=json_body)['message']


    def execute_scene(self, scene_id=None, scene_name=None):
        """
        Execute your registered scene.
        """
        if scene_id:
            pass
        elif scene_name:
            scene_id = self.scene_name_id[scene_name]
        else:
            raise ValueError("Please set scene_id or scene_name.")

        return self.request(method='POST', devices_or_scenes='scenes', service_id=scene_id, service='execute')['message']


# Error classes
class DeviceError(Exception):
    def __init__(self):
        pass

class DeviceTypeError(DeviceError):
    def __str__(self):
        return("Device type is not correct")

class DeviceNotFoundError(DeviceError):
    def __str__(self):
        return("The device is not found")

class CommandNotSupportedError(DeviceError):
    def __str__(self):
        return("Your command is not supported")

class DeviceOfflineError(DeviceError):
    def __str__(self):
        return("The device is offline now")

class HubDeviceOfflineError(DeviceError):
    def __str__(self):
        return("The hub device is offline now")

class DeviceInternalError(DeviceError):
    def __str__(self):
        return("Device internal error due to device states not synchronized with server. Or command format is invalid")


class SwitchBotAPIError(Exception):
    def __init__(self):
        pass

class InvalidRequestError(SwitchBotAPIError):
    def __str__(self):
        return("The client has issued an invalid request. This is commonly used to specify validation errors in a request payload.")

class ExceededRequestError(SwitchBotAPIError):
    def __str__(self):
        return("The client has exceeded the number of requests allowed for a given time window.")
