#!/usr/bin/python
# Write a clock on the 20 x4 line LCD Display.
# Subscribe to MQTT feed and display that as well.
# pete@hoffswell.com
# January, 2017
import smbus
import time
import socket, struct, fcntl
import os
import sys
import CHIP_IO.GPIO as GPIO
import paho.mqtt.client as mqtt
import json
import time

t0 = t1 = rssi = last = gwid = dev_id = "?"

# MQTT parameters
# Set APPLICATION, KEY, and DEVICE according to your TTN info
username='APPLICATION'
password='KEY'
broker='us-west.thethings.network'
topic='+/devices/DEVICE/up'

# MQTT Connect
def on_connect(client, userdata, flags,rc):
    print("Connected with result code "+str(rc))
    client.subscribe(topic)

# MQTT Message
def on_message(client, userdata, msg):
    # parse out mqtt JSON payload for the stuff you want.
    global t0 
    global t1
    global rssi
    global gwid 
    global dev_id
    global last
    last = time.strftime('%l:%M%p')
    print(msg.topic+" "+str(msg.payload))
    parsed_json = json.loads(str(msg.payload))
#   print(parsed_json)
    t0 = str(parsed_json["payload_fields"]["t0"])
    t1 = str(parsed_json["payload_fields"]["t1"])
    rssi = str(parsed_json["metadata"]["gateways"][0]["rssi"])
    gwid = str(parsed_json["metadata"]["gateways"][0]["gtw_id"])
    dev_id = str(parsed_json["dev_id"])

    # which device is this?
    if dev_id == 'mdot-c8db' : 
       dev_id = 'Prototype'
    elif dev_id == 'mdot-c8dc' :
       dev_id = 'Brick'
    elif dev_id == 'mdot-d83b' :
       dev_id = 'Bench'

    t0 = t0.replace("{u'value': ", "",1)
    t0 = t0.replace("}", "",1)
    print(t0)
    t1 = t1.replace("{u'value': ", "",1)
    t1 = t1.replace("}", "",1)
    print(t1)
    print("RSSI: "+rssi)
    print("gwid: "+gwid)
    print("dev_id: "+dev_id)

# Define some device parameters
I2C_ADDR  = 0x27 # I2C device address
LCD_WIDTH = 20   # Maximum characters per line

# Define some device constants
LCD_CHR = 1 # Mode - Sending data
LCD_CMD = 0 # Mode - Sending command

LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line
LCD_LINE_3 = 0x94 # LCD RAM address for the 3rd line
LCD_LINE_4 = 0xD4 # LCD RAM address for the 4th line

LCD_BACKLIGHT  = 0x08  # On
#LCD_BACKLIGHT = 0x00  # Off

ENABLE = 0b00000100 # Enable bit

# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005

# pubnub setup

#Set up IP Address
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sockfd = sock.fileno()
SIOCGIFADDR = 0x8915

def get_ip(iface = 'wlan0'):
    ifreq = struct.pack('16sH14s', iface, socket.AF_INET, '\x00'*14)
    try:
        res = fcntl.ioctl(sockfd, SIOCGIFADDR, ifreq)
    except:
        return None
    ip = struct.unpack('16sH2x4s8x', res)[2]
    return socket.inet_ntoa(ip)

#Open I2C interface
bus = smbus.SMBus(1) 

def lcd_init():
  # Initialise display
  lcd_byte(0x33,LCD_CMD) # 110011 Initialise
  lcd_byte(0x32,LCD_CMD) # 110010 Initialise
  lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
  lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off 
  lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
  lcd_byte(0x01,LCD_CMD) # 000001 Clear display
  time.sleep(E_DELAY)

def lcd_byte(bits, mode):
  # Send byte to data pins
  # bits = the data
  # mode = 1 for data
  #        0 for command

  bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
  bits_low = mode | ((bits<<4) & 0xF0) | LCD_BACKLIGHT

  # High bits
  bus.write_byte(I2C_ADDR, bits_high)
  lcd_toggle_enable(bits_high)

  # Low bits
  bus.write_byte(I2C_ADDR, bits_low)
  lcd_toggle_enable(bits_low)

def lcd_toggle_enable(bits):
  # Toggle enable
  time.sleep(E_DELAY)
  bus.write_byte(I2C_ADDR, (bits | ENABLE))
  time.sleep(E_PULSE)
  bus.write_byte(I2C_ADDR,(bits & ~ENABLE))
  time.sleep(E_DELAY)

def lcd_string(message,line):
  # Send string to display

  message = message.ljust(LCD_WIDTH," ")

  lcd_byte(line, LCD_CMD)

  for i in range(LCD_WIDTH):
    lcd_byte(ord(message[i]),LCD_CHR)

def main():
  # Main program block

  # Initialise display
  lcd_init()

  # start up mqtt
  client = mqtt.Client()
  client.on_connect = on_connect
  client.on_message = on_message
  client.username_pw_set(username, password)
  client.connect(broker, 1883, 60)
  client.loop_start()

  while True:

    # Display Text
    lcd_string(time.strftime('  %l:%M:%S%p %Z  '),LCD_LINE_1)
#    lcd_string(time.strftime('    %b %d,%Y '),LCD_LINE_2)
    lcd_string("T0:" + t0 + "   T1:" + t1 + "F",LCD_LINE_2)
    lcd_string("From:" + dev_id ,LCD_LINE_3)
    lcd_string("RSSI:" + rssi + " at" + last ,LCD_LINE_4)
    time.sleep(.5)

# Send some more text
#    lcd_string("CHIP IP ADDRESS",LCD_LINE_1)
#    lcd_string(get_ip('wlan0'),LCD_LINE_3)
#    time.sleep(3)



if __name__ == '__main__':

  try:
    main()
  except KeyboardInterrupt:
    pass
  finally:
    lcd_byte(0x01, LCD_CMD)

