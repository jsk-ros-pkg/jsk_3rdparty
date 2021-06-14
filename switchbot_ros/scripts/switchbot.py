#!/usr/bin/env python
from __future__ import print_function
import requests, json
from os import path


class SwitchBotAPIClient:
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
        self.deviceNameId = {}
        self.sceneNameId = {}
        self.update_device_list()
        self.update_scene_list()


    def request(self, method='GET', devices_or_scenes='devices', Id='', service='', json_body=None):
        """
        Execute HTTP request
        """
        if not (devices_or_scenes == 'devices' or devices_or_scenes == 'scenes'):
            raise RuntimeError('Please set devices_or_scenes variable devices or scenes')

        url = path.join(self._host_domain, devices_or_scenes, Id, service)

        if method == 'GET':
            response = requests.get(
                url,
                headers={'Authorization': self.token}
            )
        elif method == 'POST':
            response = requests.post(
                url,
                json_body,
                headers={'Content-Type': 'application/json; charset=utf8',
                         'Authorization': self.token
            })
        else:
            raise RuntimeError('Got unexpected http request method. Please use GET or POST.')

        response_json = response.json()

        # Catch the HTTP 4XX, 5XX error
        try:
            response.raise_for_status()
        except requests.exceptions.RequestException as e:
            print(e)
            if response.status_code == 422:
                print("The client has issued an invalid request. This is commonly used to specify validation errors in a request payload.")
            elif response.status_code == 429:
                print("The client has exceeded the number of requests allowed for a given time window.")
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
                raise RuntimeError("Got unknown status code : " + str(response_json['statusCode']))


    def update_device_list(self):
        """
        Update the information of deviceIds, deviceNames, deviceBots.
        """
        res = self.request()
        self.device_list = res['body']['deviceList']
        self.infrared_remote_list = res['body']['infraredRemoteList']
        for device in self.device_list:
            self.deviceNameId[device['deviceName']] = device['deviceId']
        for infrated_remote in self.infrared_remote_list:
            self.deviceNameId[device['deviceName']] = device['deviceId']

        return self.device_list, self.infrared_remote_list


    def update_scene_list(self):
        """
        Update the information of sceneIDs, sceneNames.
        """
        self.scene_list = self.request(devices_or_scenes='scenes')['body']
        for scene in self.scene_list:
            self.sceneNameId[scene['sceneName']] = device['sceneId']

        return self.scene_list


    def device_status(self, deviceId=None,  deviceName=None):
        """
        Get the device status.
        """
        if deviceId:
            pass
        elif deviceName:
            deviceId = self.deviceNameId[deviceName]
        else:
            raise RuntimeError("Please set deviceId or deviceName.")

        return self.request(Id=deviceId, service='status')['body']


    def control_device(self, command, parameter='default', commandType='command', deviceId=None, deviceName=None):
        """
        Send Command to the device. Please see https://github.com/OpenWonderLabs/SwitchBotAPI#send-device-control-commands for command options.
        """
        json_body = json.dumps({
            "command": command,
            "parameter": parameter,
            "commandType": commandType
        })
        if deviceId:
            pass
        elif deviceName:
            deviceId = self.deviceNameId[deviceName]
        else:
            raise RuntimeError("Please set deviceId or deviceName.")
        
        return self.request(method='POST', Id=deviceId, service='commands', json_body=json_body)


    def execute_scene(self, sceneId=None, sceneName=None):
        """
        Execute your registered scene.
        """
        if sceneId:
            pass
        elif sceneName:
            sceneId = self.sceneNameId[sceneName]
        else:
            raise RuntimeError("Please set sceneId or sceneName.")

        return self.request(method='POST', devices_or_scenes='scenes', Id=sceneId, service='execute')


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
