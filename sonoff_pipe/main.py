import time
import utime
from umqttsimple import MQTTClient
import machine
import ubinascii
import micropython
import network
import esp
import uos
from machine import UART

esp.osdebug(None)
import gc
gc.collect()

RESET_TIMEOUT = 6.28 * 3600 * 1000
HEARTBEAT_TIMEOUT = 4 * 60 * 1000

running = True

uart = UART(0,115200)

ssid = 'J-C'
password = 'VictorHanny862'
mqtt_server = 'jcerasmus.ddns.net'
#EXAMPLE IP ADDRESS
#mqtt_server = '192.168.1.144'
mac_string = ubinascii.hexlify(network.WLAN().config('mac'),':').decode()
client_id = ubinascii.hexlify(machine.unique_id())
topic_sub = b'/down/esp/' + mac_string
topic_pub = b'/up/esp/' + mac_string

#give time for UART to settle after boot
time.sleep(2)

print("My MAC is: " + mac_string)

client = None

station = network.WLAN(network.STA_IF)

station.active(True)
station.connect(ssid, password)

while station.isconnected() == False:
  print("We are waiting for WIFI connection")
  time.sleep(1)
  pass

print('Connection successful')
print(station.ifconfig())

led = machine.Pin(13, machine.Pin.OUT)
led.on()
relay = machine.Pin(12, machine.Pin.OUT)
relay.off()

# Complete project details at https://RandomNerdTutorials.com

def sub_cb(topic, msg):
  global timeout
  global heartbeat
  print((topic, msg))
  timeout = utime.ticks_ms() + RESET_TIMEOUT
  heartbeat = utime.ticks_ms() + HEARTBEAT_TIMEOUT
  if topic == topic_sub:
    if msg == b'set':
        print('Set relay received')
        relay.on()
        return
    
    if msg == b'reset':
        print('Reset relay received')
        relay.off()
        return

    if msg == b'exit':
        global running
        print("We will reset to terminal")
        running = False
        return

    uart.write(msg)
    uart.write("~\n\r")

def connect_and_subscribe():
  global client_id, mqtt_server, topic_sub, topic_pub
  client = MQTTClient(client_id, mqtt_server, 1883, 'janus', 'Janus506')
  client.set_last_will(topic_pub, '{\"msg\":\"offline\"}')
  client.set_callback(sub_cb)
  client.connect()
  client.subscribe(topic_sub)
  print('Connected to %s MQTT broker, subscribed to %s topic' % (mqtt_server, topic_sub))
  return client

def restart_and_reconnect():
  print('Failed to connect to MQTT broker. Reconnecting...')
  time.sleep(10)
  machine.reset()

try:
  client = connect_and_subscribe()
  led.off()
  client.publish(topic_pub, '{\"msg\":\"online\"}')
except OSError as e:
  print("Exception in connect")
  restart_and_reconnect()

print ("MQTT client connected, listening for messages")

# Disable terminal so we can read input from serial
uos.dupterm(None, 1)

rx_buffer = None

def handleSerialLine(inputLine):
  global running
  if inputLine == "exit":     
    uart.write("stopping: \n\r")
    uos.dupterm(uart, 1)
    running = False
    return
  
  uart.write(inputLine)
  uart.write("\n\rKO\n\r")

def publishSerialData(inputLine):
  global client
  global heartbeat
  heartbeat = utime.ticks_ms() + HEARTBEAT_TIMEOUT
  client.publish(topic_pub, inputLine)
  uart.write("OK\n\r")
  

# Input frames are framed with 0x7E
def waitSerialFrame():
  global rx_buffer
  if uart.any() > 0:
    rx_bytes = uart.read(64)  
    for b in rx_bytes:
      if b == 0x7E: # 126 is a MQTT message
        if rx_bytes is not None and len(rx_bytes) > 0:
          publishSerialData(str(rx_buffer))
          rx_buffer = None
      elif (b == 0x0D) or (b == 0x0A):
        if rx_bytes is not None and len(rx_bytes) > 0:
          handleSerialLine(str(rx_buffer))
          rx_buffer = None
      else:
        if rx_buffer:
          rx_buffer += chr(b)
        else:
          rx_buffer = chr(b)

  if rx_buffer is not None and len(rx_buffer) > 64:
    rx_buffer = None
          

def handleMQTTclient():
  global client
  global running
  try:
    reply = client.check_msg()
    if reply is not None:
      client.publish(topic_pub, "{\"msg\":\"OK\"}")
      print("Reply:" + reply)
    
    
  except OSError as e:
    uos.dupterm(uart, 1)
    print("Exception in main loop ", e)
    running = False

def toggleLED():
  global led
  value = led.value()
  if value == 1:
    led.value(0)
  else:
    led.value(1)

#Reset every 2*Pi hours
timeout = utime.ticks_ms() + RESET_TIMEOUT
heartbeat = utime.ticks_ms() + HEARTBEAT_TIMEOUT

try:
  while running:
    # uart.write("running\n\r")
    handleMQTTclient()
    waitSerialFrame()
    toggleLED()

    # Flash LED faster while relay is set
    if relay.value() == 1:      
      time.sleep(0.2)
    else:
      time.sleep(0.5)
      
    if utime.ticks_ms() > heartbeat:
      heartbeat = utime.ticks_ms() + HEARTBEAT_TIMEOUT
      print("Keep alive heartbeat")
      client.publish(topic_pub, "{\"msg\":\"heartbeat\"}")
      
    if utime.ticks_ms() > timeout:
      break;

except:
  print("Something Fucky, hard reset");
  machine.reset()

uos.dupterm(uart, 1)
if running:
  print("Performing a sanity reset")
  machine.reset()

client.publish(topic_pub, '{\"msg\":\"offline\"}')
client.disconnect()
station.disconnect()
led.value(1)

print("DONE")
