import logging
import json
import time
from tuya_connector import (
    TuyaOpenAPI,
    TuyaOpenPulsar,
    TuyaCloudPulsarTopic,
    TUYA_LOGGER,
)
import sys


ACCESS_ID = "ughek85kyqmnjct3hjdq"
ACCESS_KEY = "76076a73bb6440d0a86eff928ee07140"
API_ENDPOINT = "https://openapi.tuyaus.com"
MQ_ENDPOINT = "wss://mqe.tuyaus.com:8285/"

# Enable debug log
TUYA_LOGGER.setLevel(logging.DEBUG)

# Init openapi and connect
openapi = TuyaOpenAPI(API_ENDPOINT, ACCESS_ID, ACCESS_KEY)
openapi.connect()

DEVICE_ID1 ="ebc54ae3e4b1fdce28hlbl"
DEVICE_ID2 = "eb44f111d14b200002uuq5"
DEVICE_ID1_G2 = "eb9142885efba136c6uguz"
DEVICE_ID2_G2 = "ebe8a17c76714f8a86n7es"


DEVICE_ID_BLE1 = "ebcf28d925izlrwk"
DEVICE_ID_BLE2 = "eb3781ev6mzwy7kk"
DEVICE_ID_BLE3 = "ebad32at69mfppin"
DEVICE_ID_BLE4 = "ebd7d88v6oevejmj"
DEVICE_ID_BLE5 = "ebba07t6f3auaiol"

DEVICE_ID_ZB = "eb6ba731d6c3c5c48cyq3m"

commands = {'commands': [{'code': 'switch_1', 'value': True}]}
commands1 = {'commands': [{'code': 'switch', 'value': True}]}
commands2 = {'commands': [{'code': 'switch', 'value': False}]}
                    # Your code to send the command to the device
# time_first = time.time()
# print(time_first)

def select_fingerbot(name_fingerbot):
    if name_fingerbot =="Zigbee_1":
        openapi.post('/v1.0/iot-03/devices/{}/commands'.format(DEVICE_ID1), commands)
    elif name_fingerbot =="Zigbee_2":
        openapi.post('/v1.0/iot-03/devices/{}/commands'.format(DEVICE_ID2), commands)
    elif name_fingerbot =="BLE_1T":
        openapi.post('/v1.0/iot-03/devices/{}/commands'.format(DEVICE_ID_BLE1), commands1)
    elif name_fingerbot =="BLE_1F":
        openapi.post('/v1.0/iot-03/devices/{}/commands'.format(DEVICE_ID_BLE1), commands2)
    elif name_fingerbot =="BLE_2":
        openapi.post('/v1.0/iot-03/devices/{}/commands'.format(DEVICE_ID_BLE2), commands1)
    elif name_fingerbot =="BLE_3":
        openapi.post('/v1.0/iot-03/devices/{}/commands'.format(DEVICE_ID_BLE3), commands1)
    elif name_fingerbot =="BLE_4":
        openapi.post('/v1.0/iot-03/devices/{}/commands'.format(DEVICE_ID_BLE4), commands1)
    elif name_fingerbot =="BLE_5":
        openapi.post('/v1.0/iot-03/devices/{}/commands'.format(DEVICE_ID_BLE5), commands1)



