'''from machine import Pin, I2C
import utime as time
from dht import DHT11, InvalidChecksum

while True:
    time.sleep(1)
    pin = Pin(28, Pin.OUT, Pin.PULL_DOWN)
    sensor = DHT11(pin)
    t  = (sensor.temperature)
    h = (sensor.humidity)
    print("Temperature: {}".format(sensor.temperature))
    print("Humidity: {}".format(sensor.humidity))'''
import uos
from machine import I2C, Pin
from i2c_lcd import I2cLcd 
from utime import sleep
import utime
#from utime import ticks_diff, ticks_us
from dht import DHT11, InvalidChecksum
DEFAULT_I2C_ADDR = 0x27                               # LCD 1602 I2C address
pin = Pin(28, Pin.OUT, Pin.PULL_DOWN)
analog_value = machine.ADC(26)
dht11 = DHT11(pin)
myHOST = 'api.thingspeak.com'
myPORT = '80'
myAPI = 'XYAQE37CLRVDKP6I'

print()
print("Machine: \t" + uos.uname()[4])
print("MicroPython: \t" + uos.uname()[3])
uart0 = machine.UART(0, baudrate=115200)
print(uart0)
def setup():
    global lcd 
    i2c = I2C(0,sda=Pin(4),scl=Pin(5),freq=400000)
    lcd = I2cLcd(i2c, DEFAULT_I2C_ADDR, 2, 16)      # Initialize(device address, cursor settings)
 
def Rx_ESP_Data():
    recv=bytes()
    while uart0.any()>0:
        recv+=uart0.read(1)
    res=recv.decode('utf-8')
    return res
def Connect_WiFi(cmd, uart=uart0, timeout=3000):
    print("CMD: " + cmd)
    uart.write(cmd)
    sleep(7.0)
    Wait_ESP_Rsp(uart, timeout)
    print()

def Send_AT_Cmd(cmd, uart=uart0, timeout=3000):
    print("CMD: " + cmd)
    uart.write(cmd)
    Wait_ESP_Rsp(uart, timeout)
    print()
    
def Wait_ESP_Rsp(uart=uart0, timeout=3000):
    prvMills = utime.ticks_ms()
    resp = b""
    while (utime.ticks_ms()-prvMills)<timeout:
        if uart.any():
            resp = b"".join([resp, uart.read(1)])
    print("resp:")
    try:
        print(resp.decode())
    except UnicodeError:
        print(resp)
def WiFi_Init():
    Send_AT_Cmd('AT\r\n')          #Test AT startup
    Send_AT_Cmd('AT+GMR\r\n')      #Check version information
    Send_AT_Cmd('AT+CIPSERVER=0\r\n')      #Check version information
    Send_AT_Cmd('AT+RST\r\n')      #Check version information
    Send_AT_Cmd('AT+RESTORE\r\n')  #Restore Factory Default Settings
    Send_AT_Cmd('AT+CWMODE?\r\n')  #Query the Wi-Fi mode
    Send_AT_Cmd('AT+CWMODE=1\r\n') #Set the Wi-Fi mode = Station mode
    Send_AT_Cmd('AT+CWMODE?\r\n')  #Query the Wi-Fi mode again
    Connect_WiFi('AT+CWJAP="HAVASYA","Havasya__9"\r\n', timeout=5000) #Connect to AP
    Send_AT_Cmd('AT+CIFSR\r\n',timeout=5000)    #Obtain the Local IP Address
    Send_AT_Cmd('AT+CIPMUX=1\r\n')    #Obtain the Local IP Address
    sleep(1.0)
    print ('Starting connection to ESP8266...')

def loop():
    try:
        while True:
            t=str(dht11.temperature)
            h=str(dht11.humidity)
            reading = str(analog_value.read_u16()/1000)
            print("ADC: ",reading)
            sleep(1.2)
            lcd.putstr("Temp    Hum   CO")
            lcd.move_to(0,1)
            lcd.putstr("{}".format(dht11.temperature))
            lcd.move_to(7,1)
            lcd.putstr("{}".format(dht11.humidity))
            lcd.move_to(12,1)
            lcd.putstr("{}".format(reading))
            print(t)
            print(h)
            sleep(1)
            print ('!About to send data to thingspeak')
            sendData = 'GET /update?api_key='+ myAPI +'&field1='+t +'&field2='+h + '&field3='+reading
            Send_AT_Cmd('AT+CIPSTART=0,\"TCP\",\"'+ myHOST +'\",'+ myPORT+'\r\n')
            utime.sleep(1.0)
            Send_AT_Cmd('AT+CIPSEND=0,' +str(len(sendData)+4) +'\r\n')
            utime.sleep(1.0)
            Send_AT_Cmd(sendData +'\r\n')
            utime.sleep(4.0)
            Send_AT_Cmd('AT+CIPCLOSE=0'+'\r\n') # once file sent, close connection
            utime.sleep(4.0)
            print ('Data send to thing speak')
            lcd.clear()
    except InvalidChecksum:
        print("Checksum from the sensor was invalid")
        
setup()
lcd.putstr("Connecting WiFi")
WiFi_Init()
sleep(1)
lcd.clear()
lcd.putstr("WiFi Connected")
sleep(1)
lcd.clear()
lcd.putstr("IoT Basede Air")
lcd.move_to(0,1)
lcd.putstr("Polution Mon sys")
sleep(1)
lcd.clear()
if __name__ == '__main__':
   
    
    loop()