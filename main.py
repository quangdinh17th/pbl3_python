import network
import machine
from time import sleep
import dht 
import time
import urequests
import machine
import bmp180
from time import sleep
from machine import Pin, SoftI2C
from lcd_api import LcdApi
from i2c_lcd import I2cLcd
from time import sleep
import utime
import umail
from machine import I2C, Pin
from umqtt.simple import MQTTClient
sender_email = 'kaztji28@gmail.com'
sender_name = 'ESP32' #sender name
sender_app_password = 'qxbdgyoohpkcmqsh'
recipient_email ='vanquang17th@gmail.com'
email_subject ='Warning: Water level rising rapidly'
I2C_ADDR = 0x27
totalRows = 4
totalColumns = 20
i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq=10000)     #initializing the I2C method for ESP32
lcd = I2cLcd(i2c, I2C_ADDR, totalRows, totalColumns)
ssid = "Gia Bao"
password = "29122003"
server = "broker.emqx.io"          
topic = "pbl3"        
client_id = "mqttx_9c8af0b3"
def connect_wifi():
    wifi = network.WLAN(network.STA_IF)
    wifi.active(True)
    wifi.connect(ssid, password)
    while not wifi.isconnected():pass
    print("Connected to Wi-Fi")
def connect_mqtt():
    global client
    try:
        client = MQTTClient(client_id, server)
        client.connect()
        print("Connected to MQTT broker")
        client.set_callback(callback)
        client.subscribe(topic)
    except Exception as e:
        print("Error connecting to MQTT broker:", e)

def publish_data(temperature, humidity, pressure, distance_change, water_level_percent):
    client.publish("PBL3/temp", str(temperature))
    client.publish("PBL3/humidity", str(humidity))
    client.publish("PBL3/pressure", str(pressure))
    client.publish("PBL3/distance_change", str(distance_change))
    client.publish("PBL3/water_level", str(water_level_percent))
        
def callback(topic, msg):
    print("-------New message from broker-----")
    print("Topic:", topic)
    print("Message:", msg.decode('utf-8'))
# Function to connect BMP180 sensor
def connect_bmp_sensor():
    i2c = machine.SoftI2C(scl=machine.Pin(22), sda=machine.Pin(21))
    bmp = bmp180.BMP180(i2c)
    return bmp
# Initialize BMP180 sensor
bmp_sensor = connect_bmp_sensor()
def read_bmp_sensor(bmp_sensor):
    try:
        pressure = bmp_sensor.pressure
        altitude = bmp_sensor.altitude
        print("Pressure: %3.1f hPa" %(pressure/100))
        print("Altitude: %3.1f meters" %(abs(altitude)))
        #url = "http://api.thingspeak.com/update?api_key=6KGNFPZI9CD5WO37&field3={:.2f}".format(pressure)
        #response = urequests.get(url)
        #response.close()
        return pressure 
    except:
        print("Failed to connect to BMP180 sensor")
# DHT22 sensor configuration
dht_sensor = dht.DHT22(machine.Pin(14))
def DHT22():
        try:
            dht_sensor.measure()
            temperature = dht_sensor.temperature()
            humidity = dht_sensor.humidity()
            print('Temperature: %3.1f C' %temperature)
            print('Humidity: %3.1f %%' %humidity)
            #url = "http://api.thingspeak.com/update?api_key=6KGNFPZI9CD5WO37&field1={:.2f}&field2={:.2f}".format(temperature, humidity)
            #response = urequests.get(url)
            #response.close()
            return temperature, humidity
        except:
            print('Failed to read sensor.')

connect_wifi()
connect_mqtt()
# DS1307 I2C address
DS1307_I2C_ADDRESS = 0x68
# Initialize i2c
i2c = I2C(scl=Pin(22), sda=Pin(21), freq=100000)
# Function to read the time from DS1307
def Real_time_ds1307():
    data = bytearray(7)
    i2c.readfrom_mem_into(DS1307_I2C_ADDRESS, 0, data)
    second = _bcd_to_decimal(data[0])
    minute = _bcd_to_decimal(data[1])
    hour = _bcd_to_decimal(data[2])
    date = _bcd_to_decimal(data[4])
    month = _bcd_to_decimal(data[5])
    year = _bcd_to_decimal(data[6]) + 2000

    return date, month, year, hour, minute, second

# Function to convert BCD to decimal
def _bcd_to_decimal(bcd):
    return (bcd & 0x0F) + ((bcd >> 4) & 0x0F) * 10

def RTC_sensor():
    date, month, year, hour, minute, second= Real_time_ds1307()
    print("{}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(date, month, year, hour, minute, second))
    lcd.move_to(0,0)
    lcd.putstr("{}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(date, month, year, hour, minute, second))

def zambretti_algorithm():
    now = utime.localtime()
    curr_p = (bmp_sensor.pressure) / 100
    prev_p = curr_p
    h = abs(bmp_sensor.altitude)
    T = dht_sensor.temperature()
    p = curr_p * pow((1 - ((0.0065*h)/(T+(0.0065*h)+273.15))), -5.257)
    P = round(p)
    print(f"\nP : {p}")
    # Calculate Z - Forecast number
    # Pressure trend
    if curr_p < prev_p:  # falling
        z = 127 - 0.12 * P
        if 6 <= now[1] < 9:  # pressure falling & summer(1/6 -> 31/8)
            z -= 1
    elif curr_p == prev_p:  # pressure steady
        z = 144 - 0.13 * P
    elif curr_p > prev_p:  # pressure rising
        z = 185 - 0.16 * P
        if 12 <= now[1] < 3:  # pressure rising & winter (1/12 -> hết tháng 2)
            z += 1
    z = round(z)
    print("\nZ:", z)
    # Explain Zambretti values
    case = {
        1: "Thoi tiet tot",
        2: "Thoi tiet on dinh",
        3: "Thoi tiet tot",
        4: "Thoi tiet gan mua",
        5: "Mua rao",
        6: "Thoi tiet bat on, mua",
        7: "Mua, luc sau te hon",
        8: "Co luc mua, te hon",
        9: "Rat bat on, mua",
        10: "Thoi tiet tot",
        11: "Thoi tiet on dinh",
        12: "Co the mua rao",
        13: "Co the mua rao",
        14: "Mua rao, troi sang",
        15: "Thinh thoang mua",
        16: "Bat on, co luc mua",
        17: "Mua thuong xuyen",
        18: "Rat bat on, mua",
        19: "Bao, mua nhieu",
        20: "Thoi tiet tot",
        21: "Thoi tiet on dinh",
        22: "Tro nen tot hon",
        23: "Troi dang cai thien",
        24: "Co the mua rao som",
        25: "Co mua som",
        26: "Thoi tiet thay doi",
        27: "Kha bat on",
        28: "Dang cai thien",
        29: "Chua on time ngan",
        30: "Co luc tot hon",
        31: "Bao,co the cai thien",
        32: "Bao, mua nhieu"
    }

    result = case.get(z, "Invalid Forecast")
    lcd.move_to(0, 1)
    lcd.putstr(result)
    print("\n", result)
    prev_p = curr_p
    print("\nPrev_P :", prev_p, "hPa")
# Function to measure distance
TRIG_PIN = 27
ECHO_PIN = 26
Rain_sensor_pin = 4
empty_tank_distance = 160
full_tank_distance = 30
is_raining = False
duration = 0
distance = 0
water_level_percent = 0
last_distance_check_time = 0
initial_distance = 0
def measure_distance():
    global is_raining, duration, distance
    trig = Pin(TRIG_PIN, Pin.OUT)
    echo = Pin(ECHO_PIN, Pin.IN)
    trig.value(0)
    utime.sleep_us(2)
    trig.value(1)
    utime.sleep_us(10)
    trig.value(0)
    is_raining = Pin(Rain_sensor_pin, Pin.IN).value()
    duration = machine.time_pulse_us(echo, 1)
    distance = (duration / 2) * 0.343  # mm
def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

distance_change = 0
water_level_percent = 0
def warning_rain():
    global initial_distance
    global last_distance_check_time
    global distance_change
    global water_level_percent
    last_distance_check_time = time.ticks_ms()
    if not is_raining :
        if initial_distance == 0:
            initial_distance = distance
            last_distance_check_time = time.ticks_ms()

        distance_change = abs(initial_distance - distance)
        print(" Đang mưa")
        print("Mực nước dâng :", distance_change, "mm")
        #url = "http://api.thingspeak.com/update?api_key=6KGNFPZI9CD5WO37&field5={:.2f}".format(distance_change)
        #response = urequests.get(url)
        #response.close()
        lcd.move_to(0, 2)
        lcd.putstr("It's raining")
        lcd.move_to(0, 3)
        lcd.putstr("Rainwater rises:%1.0fmm " %round(distance_change))
        if abs(time.ticks_diff(time.ticks_ms(), last_distance_check_time)) <= 5000:
            if distance_change >= 30:
                print("Water level has risen by more than 30mm.")
                lcd.move_to(0, 3)
                lcd.putstr("Red warning         ")
                # Send the email
                smtp = umail.SMTP('smtp.gmail.com', 465, ssl=True) # Gmail's SSL port
                smtp.login(sender_email, sender_app_password)
                smtp.to(recipient_email)
                smtp.write("From:" + sender_name + "<"+ sender_email+">\n")
                smtp.write("Subject:" + email_subject + "\n")
                smtp.write("Water level has risen by more than 30mm.")
                smtp.send()
                smtp.quit()
            elif distance_change >= 20:
                print("Water level has risen by more than 20mm.")
                lcd.putstr(0, 3)
                lcd.putstr("Warning             ")
                smtp = umail.SMTP('smtp.gmail.com', 465, ssl=True) # Gmail's SSL port
                smtp.login(sender_email, sender_app_password)
                smtp.to(recipient_email)
                smtp.write("From:" + sender_name + "<"+ sender_email+">\n")
                smtp.write("Subject:" + email_subject + "\n")
                smtp.write("Water level has risen by more than 20mm.")
                smtp.send()
                smtp.quit()
    else:
        initial_distance = 0
        if distance > full_tank_distance and distance < empty_tank_distance:
            water_level_percent = map_value(distance, empty_tank_distance, full_tank_distance, 0, 100)
            #url = "http://api.thingspeak.com/update?api_key=6KGNFPZI9CD5WO37&field4={:.2f}".format(water_level_percent)
            #response = urequests.get(url)
            #response.close()
            lcd.move_to(0, 2)
            lcd.putstr("Percent: %1.0f%%" %water_level_percent)
            print("Percent: ", water_level_percent,"%" )
            lcd.move_to(0,3)
            lcd.putstr("                    ")
    return water_level_percent, distance_change

while True:
    RTC_sensor()
    zambretti_algorithm()
    DHT22()
    read_bmp_sensor(bmp_sensor)
    measure_distance()
    warning_rain()
    #MQTT
    client.check_msg()
    temperature, humidity = DHT22()
    pressure = read_bmp_sensor(bmp_sensor)
    distance_change, water_level_percent = warning_rain()
    publish_data(temperature, humidity, pressure, distance_change, water_level_percent)
