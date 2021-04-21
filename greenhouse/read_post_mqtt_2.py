#!/usr/bin/env python3
import sys
from datetime import datetime
import bluetooth._bluetooth as bluez

#LoRa Imports
import RPi.GPIO as GPIO
import serial
import time


#Bluetooth Setup
from bluetooth_utils import (toggle_device, enable_le_scan,
                             parse_le_advertising_events,
                             disable_le_scan, raw_packet_to_str)

#LoRa Setup
M0 = 22
M1 = 27

CFG_REG = [b'\xC2\x00\x09\xFF\xFF\x00\x62\x00\x17\x03\x00\x00',
                   b'\xC2\x00\x09\x00\x00\x00\x62\x00\x17\x03\x00\x00']
RET_REG = [b'\xC1\x00\x09\xFF\xFF\x00\x62\x00\x17\x03\x00\x00',
                   b'\xC1\x00\x09\x00\x00\x00\x62\x00\x17\x03\x00\x00']
r_buff = ""
delay_temp = 1

time.sleep(0.001)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(M0,GPIO.OUT)
GPIO.setup(M1,GPIO.OUT)

GPIO.output(M0,GPIO.LOW)
GPIO.output(M1,GPIO.HIGH)
time.sleep(1)

ser = serial.Serial("/dev/ttyS0",9600)
ser.flushInput()

if ser.isOpen():
    ser.write(CFG_REG[0])
    print("It's Setting BROADCAST and MONITOR mode")

# Hard code devices to MQTT topic names
deviceName = {
    "A4:C1:38:AE:64:5F":"office",
    "A4:C1:38:D5:4C:EB":"outside",
    "A4:C1:38:D8:C0:00":"attic"
}


# Use 0 for hci0
dev_id = 0
toggle_device(dev_id, True)

try:
    sock = bluez.hci_open_dev(dev_id)
except:
    print("Cannot open bluetooth device %i" % dev_id)
    raise

# Set filter to "True" to see only one packet per device
enable_le_scan(sock, filter_duplicates=False)

try:
    def le_advertise_packet_handler(mac, adv_type, data, rssi):
        data_str = raw_packet_to_str(data)
        # Check for ATC preamble
        if data_str[6:10] == '1a18':
            temp_raw = int(data_str[22:26], 16)
            if temp_raw>1000:
                temp_raw = temp_raw-65535
            temp = round((temp_raw / 10)*1.8+32,1)
            hum = int(data_str[26:28], 16)
            batt = int(data_str[28:30], 16)
            temperature_payload='MQTT:home/'+deviceName[mac]+'/temperature:'+str(temp)
            humidity_payload=   'MQTT:home/'+deviceName[mac]+'/humidity:'+str(hum)
            battery_payload=    'MQTT:home/'+deviceName[mac]+'/battery:'+str(batt)

            if ser.inWaiting() > 0 :
                time.sleep(0.1)
                r_buff = ser.read(ser.inWaiting())
                if r_buff == RET_REG[0] :
                    print("BROADCAST and MONITOR mode was actived")
                    GPIO.output(M1,GPIO.LOW)
                    time.sleep(0.01)
                    r_buff = ""

            ser.write(temperature_payload.encode('ascii'))
            time.sleep(1)
#            ser.write(('|').encode('ascii'))
            ser.write(humidity_payload.encode('ascii'))
            time.sleep(1)
#            ser.write(('|').encode('ascii'))
            ser.write(battery_payload.encode('ascii'))
            time.sleep(1)

            print(temperature_payload)
            print(humidity_payload)
            print(battery_payload)


#            mqtt_msg=[{'topic':"home/"+deviceName[mac]+"/temperature",'payload':temp},
#                      {'topic':"home/"+deviceName[mac]+"/humidity",'payload':hum},
#                      {'topic':"home/"+deviceName[mac]+"/battery",'payload':batt}]
#            publish.multiple(mqtt_msg, hostname="iotserver.local", auth={'username':"iotuser",'password':"iotuser"})

    # Called on new LE packet
    parse_le_advertising_events(sock,
                                handler=le_advertise_packet_handler,
                                debug=False)
    print("what the fuck is shit shit")

# Scan until Ctrl-C
except KeyboardInterrupt:
    disable_le_scan(sock)
    if ser.isOpen():
        ser.close()
    GPIO.cleanup
