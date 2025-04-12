import websockets
import asyncio
import requests
import time
import json
import math
import threading
#from lib import LCD_2inch
import RPi.GPIO as GPIO
from luma.core.interface.serial import spi
from luma.core.render import canvas
from luma.lcd.device import st7789
from PIL import Image, ImageDraw, ImageFont
from datetime import datetime
from gpiozero import PWMLED
from collections import deque


global backlight
serial = spi(port = 0,device = 0,gpio_DC = 22,gpio_RST = 27)
device = st7789(serial, rotate = 2, gpio_LIGHT = 18, active_low=False,width=320, height=240)
device.backlight(True)  # Turn on the backlight

# Set up the PWM LED on GPIO 18
backlight = PWMLED(18)
# Turn on the backlight at full brightness
backlight.value = 1 # 100% duty cycle



from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306

serial_64 = i2c(port=1, address=0x3D)
device_64 = ssd1306(serial_64, width=128, height=64)

serial_32 = i2c(port=1, address=0x3C)
device_32 = ssd1306(serial_32, width=128, height=64)

global gps_update_success
gps_update_success = False  # This flag will be True if GPS update was successful, False otherwise
global receiver_lat, receiver_lon, receiver_altitude, receiver_climb
global GPSSatellitesSeen, GPSSatellitesTracked
GPSSatellitesSeen = 0
GPSSatellitesTracked = 0

receiver_lat = None
receiver_lon = None
receiver_altitude = 0
receiver_climb = 0 
receiver_fix = None
receiver_quality = None
receiver_time = None

global closest_traffic

global own_ship_speed
own_ship_speed = 0
global own_ship_alt
own_ship_alt = 0
global own_ship_lat
own_ship_lat =0
global own_ship_lon
own_ship_lon = 0
global own_ship_track
own_ship_track = 0
global own_ship
own_ship = False
global own_ship_last_update_time
own_ship_last_update_time = time.time()  # Initialize Timestamp for last update
global own_ship_squawk
own_ship_squawk = 0
global own_ship_climb
own_ship_climb = 0
global own_ship_hex
own_ship_hex = 0
global own_ship_selected
own_ship_selected = False
global awaiting_response
awaiting_response = False
not_own_ship = set()
global uSat
uSat = 0
global filter_mode
filter_mode = False
global orientation
orientation = False


global trackup
trackup = False

# Initialize global variable for radar range
global radar_range_nm
radar_range_nm = 5

global display_mode
display_mode = 2

global max_ac
max_ac = 5

global BaroPressureAltitude

STRATUX_WS_URL = "ws://localhost/situation"  # Stratux WebSocket URL
UPDATE_RATE = 0.05  # Target 20 Hz refresh rate (every 50ms)
SMOOTHING_WINDOW = 5  # Number of frames to smooth

# Global variables for smoothing
pitch_buffer = deque(maxlen=SMOOTHING_WINDOW)
roll_buffer = deque(maxlen=SMOOTHING_WINDOW)
slip_buffer = deque(maxlen=SMOOTHING_WINDOW)
attitude_data = {"pitch": 0, "roll": 0, "slip_skid": 0}

# Cache to store the last-loaded data and timestamp
cached_data = {
    "timestamp": 0,  # Last time the data was refreshed
    "data": None     # Cached JSON data
}

def process_ahrs_data(message):
    """ Parses incoming AHRS data from the WebSocket and applies smoothing. """
    global attitude_data, pitch_buffer, roll_buffer, slip_buffer
    try:
        data = json.loads(message)

        # Add new data to buffers for smoothing
        pitch_buffer.append(data.get("AHRSPitch", 0))
        roll_buffer.append(data.get("AHRSRoll", 0))
        slip_buffer.append(data.get("AHRSSlipSkid", 0))

        # Apply moving average filter
        attitude_data["pitch"] = sum(pitch_buffer) / len(pitch_buffer)
        attitude_data["roll"] = sum(roll_buffer) / len(roll_buffer)
        attitude_data["slip_skid"] = sum(slip_buffer) / len(slip_buffer)

    except Exception as e:
        print(f"Error processing AHRS data: {e}")


async def fetch_ahrs_data():
    """ Connects to Stratux WebSocket and listens for AHRS updates. """
    while True:
        try:
            async with websockets.connect(STRATUX_WS_URL, ping_interval=10, ping_timeout=None) as ws:
                print("Connected to Stratux WebSocket.")
                while True:
                    try:
                        message = await ws.recv()  # No timeout; keeps waiting for new messages
                        process_ahrs_data(message)
                    except websockets.ConnectionClosed:
                        print("WebSocket closed unexpectedly. Reconnecting...")
                        break  # Exit loop and reconnect
        except Exception as e:
            print(f"WebSocket error: {e}. Retrying in 2 seconds...")
            await asyncio.sleep(2)

def parse_gps_data():
    global gps_update_success
    global own_ship, own_ship_lat, own_ship_lon, own_ship_alt, own_ship_speed, own_ship_track
    global own_ship_selected
    global receiver_lat, receiver_lon, receiver_altitude, receiver_speed, receiver_track, receiver_quality
    global receiver_time, receiver_epv, receiver_ept, receiver_mode, uSat
    global GPSSatellitesSeen, GPSSatellitesTracked
    global BaroPressureAltitude



    url = "http://localhost/getSituation"  # Ensure correct protocol (http or https)

    while True:
        try:
            response = requests.get(url, verify=False)  # Set verify to True in production for SSL verification
            data = response.json()
            print(data)
            print("Received stratux GPS data:")
            #print(json.dumps(data, indent=2))

            receiver_lat = data.get("GPSLatitude", 0.0)
            receiver_lon = data.get("GPSLongitude", 0.0)
            receiver_time = data.get("GPSTime", "")
            receiver_altitude = data.get("GPSAltitudeMSL", 0.0)
            receiver_epv = data.get("GPSVerticalAccuracy", 0.0)
            receiver_ept = data.get("GPSHorizontalAccuracy", 0.0)
            receiver_speed = data.get("GPSGroundSpeed", 0.0)
            receiver_track = data.get("GPSTrueCourse", 0.0)
            receiver_quality = data.get("GPSFixQuality", 0)
            receiver_mode = 3 if receiver_quality else 0
            GPSSatellitesTracked = data.get("GPSSatellitesTracked", 0)
            GPSSatellitesSeen = data.get("GPSSatellitesSeen", 0)
            uSat = data.get("GPSSatellites", 0)

            BaroPressureAltitude = data.get("BaroPressureAltitude", 0.0)

            gps_update_success = True if receiver_quality else False
            #print(f"GPS data: lat {receiver_lat} lon {receiver_lon} time {receiver_time} track {receiver_track} alt {receiver_altitude} sats {uSat}")

        except Exception as e:
            print(f"Failed to retrieve or parse data: {e}")
            gps_update_success = False

        # You can adjust the sleep time to control how often you poll the server
        time.sleep(1)


def print_own_ship_adsb():
    global own_ship
    global own_ship_alt
    global own_ship_speed
    global own_ship_squawk
    global own_ship_track
    global own_ship_lat
    global own_ship_lon
    global own_ship_climb
    global own_ship_hex

    head_font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 24)
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 20)

    with canvas(device) as draw:
      draw.rectangle((0, 0, 320, 240),  fill="BLACK")

      draw.text((1,0), f"ADS-B Data:", fill="white", font=head_font)
      draw.text((175,0), f"{own_ship}", fill="yellow", font=head_font)
      if own_ship:
        draw.text((1,30), "Squawk:", fill="white", font= head_font)
        draw.text((175,30), f"{own_ship_squawk}", fill="yellow", font= head_font)
        draw.text((1,60), "Heading:", fill="white", font= head_font)
        draw.text((175,60), f"{own_ship_track}", fill="yellow", font= head_font)
        draw.text((1,90), "Climb:", fill="white", font=head_font)
        draw.text((175,90), f"{int(own_ship_climb)}", fill="yellow", font=head_font)
        draw.text((1,140), f"Lat: {own_ship_lat}", font=font)
        draw.text((200,140), "clear --->", fill="red", font=font)
        draw.text((1,160), f"Lon: {own_ship_lon}", font=font)
        draw.text((1,180), f"Hex: {own_ship_hex}", font=font)

def print_current_gps_loc():
    global gps_update_success
    global own_ship_selected
    global receiver_lat, receiver_lon, receiver_altitude, receiver_fix, receiver_quality, receiver_time, receiver_epv, receiver_ept, receiver_speed, receiver_climb, receiver_mode, receiver_track
    global GPSSatellitesSeen, GPSSatellitesTracked, uSat

    head_font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 24)
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 24)
    
    with canvas(device) as draw:
      draw.rectangle((0, 0, 320, 240),  fill="BLACK")

      if gps_update_success: 
           draw.text((75,0), "GPS Data", fill="GREEN", font=head_font)
           draw.text((1,40), f"Lat: {receiver_lat}", font=font)
           draw.text((1,65), f"Lon: {receiver_lon}", font=font)
           if receiver_mode > 2:
               draw.text((1,100), f"Alt: {int(receiver_altitude)} Climb: {int(receiver_climb)}", font=font)
               draw.text((1,125), f"Speed: {receiver_speed:.1f} Track: {int(receiver_track)}", font=font)

           draw.text((1,170), f"EPT: {int(receiver_ept)} EPV: {int(receiver_epv)}", fill="YELLOW", font=font)

         # Parse the timestamp
         #  parsed_time = datetime.strptime(receiver_time, "%Y-%m-%dT%H:%M:%S.%fZ")

           try:
               parsed_time = datetime.strptime(receiver_time, "%Y-%m-%dT%H:%M:%S.%fZ")
               print("Parsed time:", parsed_time)
           except ValueError:
               parsed_time = datetime.strptime(receiver_time, "%Y-%m-%dT%H:%M:%SZ")
           except ValueError as e:
               print("Error parsing time:", e)
               parsed_time = None  # Set a default value or take other corrective actions

         # Extract and format date and time
           try:
               date_str = parsed_time.strftime("%Y-%m-%d")
               time_str = parsed_time.strftime("%H:%M:%S")
               draw.text((1,205), f"{date_str} {time_str}", fill="YELLOW", font=font)
           except ValueError as e:
               print("Error parsing time:", e)
               parsed_time = None  # Set a default value or take other corrective actions
         
      else:
           draw.text((15,2), "NO GPS Location", fill="RED", font=font)
           draw.text((15,40), f"Satallites Seen:    {GPSSatellitesSeen}", fill="White", font=font)
           draw.text((15,65), f"Satallites Tracked: {GPSSatellitesTracked}", fill="White", font=font)
           draw.text((15,90), f"Satallites Used: {uSat}", fill="White", font=font)




def monitor_buttons():
    global radar_range_nm
    global display_mode
    global max_ac
    global trackup
    global awaiting_response, current_prompt_flight, own_ship, own_ship_selected
    global not_own_ship
    global backlight
    global own_ship_last_update_time
    global filter_mode
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    

    # Define the BCM pin number of the button
    KEY1_PIN = 4
    KEY2_PIN = 17
    KEY3_PIN = 23
    KEY4_PIN = 24

# Set the button pin to input mode and use a pull-up resistor
    GPIO.setup(KEY1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(KEY2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(KEY3_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(KEY4_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

#Initialize button state
    prev_state_key1 = GPIO.input(KEY1_PIN)
    prev_state_key2 = GPIO.input(KEY2_PIN)
    prev_state_key3 = GPIO.input(KEY3_PIN)
    prev_state_key4 = GPIO.input(KEY4_PIN)
    key4_pressed_time = None
    key3_pressed_time = None
    last_dim_time = None


  
    while True:

      current_state_key1 = GPIO.input(KEY1_PIN)
      current_state_key2 = GPIO.input(KEY2_PIN)
      current_state_key3 = GPIO.input(KEY3_PIN)
      current_state_key4 = GPIO.input(KEY4_PIN)

      if prev_state_key1 and not current_state_key1 and not awaiting_response:
             display_mode = (display_mode + 1) % 6
             print(f"display mode {display_mode}")
             time.sleep(0.3)  # Debounce delay

      if prev_state_key2 and not current_state_key2:
          key2_press_time = time.time()  # Record the time when button was pressed
          key2_last_filter_time = None

      elif not prev_state_key2 and current_state_key2:
          
          press_duration = time.time() - key2_press_time
          if press_duration < 1:  #quick press 


            if display_mode == 2 and not trackup and not awaiting_response:
              trackup = True
              print (f"button trackup = {trackup}")
              key2_press_time = None
              time.sleep(0.3)
            elif display_mode == 2 and trackup and not awaiting_response:
              trackup = False
              print (f"button trackup = {trackup}")
              key2_press_time = None
              time.sleep(0.3)
            elif display_mode != 2 and not awaiting_response:
              display_mode = 2
              print("display mode 2")
              key2_press_time = None
              time.sleep(0.3)  # Debounce delay
            elif awaiting_response:
            # User pressed '2', indicating 'No'
              not_own_ship.add(current_prompt_flight)
            # Optionally clear the prompt from the screen
              key2_press_time = None
              awaiting_response = False  # Reset the awaiting response flag
              time.sleep(.3)  # Debounce delay

      elif not current_state_key2 and key2_press_time and not awaiting_response:
          #button is being held down
          if (time.time() - key2_press_time) >= 1:
             if key2_last_filter_time is None or (time.time() - key2_last_filter_time) >= 1: 
                if not filter_mode:
                  filter_mode = True
                else:
                  filter_mode = False

             print(f"Filter changed {filter_mode}")
             key2_last_filter_time = time.time()
             time.sleep(0.3)


      if prev_state_key3 and not current_state_key3:
          key3_pressed_time = time.time()  # Record the time when button was pressed
      elif not prev_state_key3 and current_state_key3:
           if key3_pressed_time:
              press_duration = time.time() - key3_pressed_time
              if press_duration >= 1:
                  # Long press detected, decrement backlight.value by 0.1
                  new_value = 1 # Full bright LCD
                  backlight.value = new_value
                  print(f"Backlight at 100% brightness")
                  key3_pressed_time = None  # Reset the timer
              elif press_duration <= 1:
                if display_mode == 0:
                   own_ship_selected = False
                   own_ship = False
                   display_mode = 2
                   key3_pressed_time = None  # Reset the timer
                   time.sleep(.3)
                else:
                   radar_range_nm = max(5, radar_range_nm + 5)
                   print("Increasing radar range to", radar_range_nm, "nm")
                   key3_pressed_time = None  # Reset the timer
                   time.sleep(0.3)  # Debounce delay

        # KEY4 button handling for dimming
      if prev_state_key4 and not current_state_key4:
          # Button was just pressed
          key4_press_time = time.time()
          key4_last_dim_time = None
      elif not prev_state_key4 and current_state_key4:
          # Button was just released
          press_duration = time.time() - key4_press_time
          if display_mode == 0:
                 own_ship_selected = False
                 own_ship = False
                 display_mode = 2
                 key4_press_time = None  # Reset the timer
                 time.sleep(.3)
          elif press_duration < 1:
                 radar_range_nm = max(5, radar_range_nm - 5)
                 print("Decreasing radar range to", radar_range_nm, "nm")
                 key4_pressed_time = None  # Reset the timer
                 time.sleep(0.3)  # Debounce delay
                 key4_press_time = None
                 key4_last_dim_time = None

      elif not current_state_key4 and key4_press_time:
          # Button is being held down
          if (time.time() - key4_press_time) >= 1:
              # It's been at least 2 seconds since the button was pressed
              if key4_last_dim_time is None or (time.time() - key4_last_dim_time) >= 1:
                  new_value = max(.1, backlight.value - .2)  # Dim by 20% of current value
                  backlight.value = new_value
                  print(f"Backlight dimmed to {new_value * 100}% brightness")
                  key4_last_dim_time = time.time()


      elif prev_state_key1 and not current_state_key1 and awaiting_response:
             # User pressed '1', indicating 'Yes'
              own_ship = (current_prompt_flight)
              own_ship_selected = True
              own_ship_last_update_time = time.time()
              awaiting_response = False  # Reset the awaiting response flag
              #Optionally clear the prompt from the screen
              time.sleep(0.3)  # Debounce delay


      prev_state_key1 = current_state_key1
      prev_state_key2 = current_state_key2
      prev_state_key3 = current_state_key3
      prev_state_key4 = current_state_key4

      time.sleep(0.05)  # This sleep reduces CPU usage by limiting the loop execution rate

def haversine(lat1, lon1, lat2, lon2):  # Nautical Miles
    R = 3440  # Radius of the Earth in nautical miles
    dLat = math.radians(lat2 - lat1)
    dLon = math.radians(lon2 - lon1)
    a = math.sin(dLat / 2) * math.sin(dLat / 2) + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dLon / 2) * math.sin(dLon / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance


def calculate_initial_compass_bearing(lat1, lon1, lat2, lon2):
    dLon = math.radians(lon2 - lon1)
    x = math.sin(dLon) * math.cos(math.radians(lat2))
    y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(dLon)
    initial_bearing = math.atan2(x, y)
    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360
    return compass_bearing

def process_aircraft_data(file_path, receiver_lat, receiver_lon):
    global awaiting_response, current_prompt_flight
    global own_ship, own_ship_lat, own_ship_lon, own_ship_alt, own_ship_speed, own_ship_track, not_own_ship, own_ship_squawk, own_ship_climb
    global own_ship_hex, own_ship_last_update_time

    aircraft_info = []
    current_time = time.time()

    # Check if we should refresh the JSON data
    if current_time - cached_data["timestamp"] > 1:
        try:
            print("aircraft file accessed")
            with open(file_path, 'r') as file:
                cached_data["data"] = json.load(file)  # Load new data
                cached_data["timestamp"] = current_time  # Update timestamp
        except Exception as e:
            print(f"Error reading JSON file: {e}")
            return []  # Return an empty list on error

    # Use the cached data
    data = cached_data["data"]

    if data is None:
        return []  # If data is still None, return an empty list

    try:
        for aircraft in data.get('aircraft', []):
            if 'lat' in aircraft and 'lon' in aircraft and 'flight' in aircraft:
                lat = aircraft['lat']
                lon = aircraft['lon']
                flight = aircraft.get('flight', 'Unknown').strip()
                track = aircraft['track']
                speed = aircraft['speed']
                altitude = aircraft.get('altitude', '0')  # Ensure this key exists or use a placeholder
                vert_rate = aircraft.get('vert_rate', '0')
                print(f"flight = {flight}, Own_ship = {own_ship}")

                if own_ship_selected and flight == own_ship:  # Set globals and skip if this flight is filtered
                    own_ship_lat = lat
                    own_ship_lon = lon
                    own_ship_alt = altitude
                    own_ship_speed = speed
                    own_ship_track = track
                    own_ship_squawk = aircraft['squawk']
                    own_ship_climb = aircraft['vert_rate']
                    # own_ship_hex = aircraft['hex']
                    own_ship_last_update_time = time.time()  # Start the timer when own_ship is seen
                    print("own ship time seen reset")
                elif own_ship_selected and flight != own_ship and (time.time() - own_ship_last_update_time) <= 10:
                    distance = haversine(own_ship_lat, own_ship_lon, lat, lon)
                    bearing = calculate_initial_compass_bearing(own_ship_lat, own_ship_lon, lat, lon)
                    aircraft_info.append((flight, distance, bearing, altitude, track, speed, vert_rate))
                elif not own_ship_selected or (time.time() - own_ship_last_update_time) > 10:
                    distance = haversine(receiver_lat, receiver_lon, lat, lon)
                    bearing = calculate_initial_compass_bearing(receiver_lat, receiver_lon, lat, lon)
                    aircraft_info.append((flight, distance, bearing, altitude, track, speed, vert_rate))

                # See if this flight is an own_ship candidate
                if not own_ship_selected and flight not in not_own_ship and distance <= 0.03048 and abs(altitude - receiver_altitude) < 1000:
                    display_filter_prompt(flight)
                    awaiting_response = True
                    current_prompt_flight = flight
                    while awaiting_response:  # Wait here until a response is provided
                        time.sleep(0.1)

                    continue  # Skip the current iteration and wait for user response

    except Exception as e:
        print(f"process_aircraft_data - An error occurred: {e}")

    return sorted(aircraft_info, key=lambda x: x[1])

def display_filter_prompt(flight):
   font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 28)
   with canvas(device) as draw:
      draw.rectangle((0, 0, 320, 240),  fill="BLACK")
      draw.text((10, 0), f"Own ship {flight}?", fill="yellow", font=font)
      draw.text((10, 80), "Key 1: Yes", fill="WHITE", font=font)
      draw.text((10, 120), "Key 2: No", fill="WHITE", font=font)

def find_closest_aircraft(file_path, receiver_lat, receiver_lon):
    """
    Finds the closest aircraft to the receiver's location.

    Parameters:
    - file_path: Path to the JSON file containing the aircraft data.
    - receiver_lat: Receiver's latitude.
    - receiver_lon: Receiver's longitude.

    Returns:
    A tuple containing the flight ID, distance, and bearing of the closest aircraft, or None if no aircraft are found.
    """
    # Process the aircraft data to get a sorted list of aircraft by distance
    aircraft_list = process_aircraft_data(file_path, receiver_lat, receiver_lon)

    # If the list is not empty, return the first element (closest aircraft)
    if aircraft_list:
        closest_aircraft = aircraft_list[0]
        return closest_aircraft
    else:
        return None


def draw_arrow(draw, bearing, center=(64, 32), length=30):
    """
    Draws an arrow pointing in the specified bearing direction on the OLED display.
    Adjusts for track-up mode based on the global trackup variable.

    Parameters:
    - draw: The ImageDraw object to draw on.
    - bearing: The bearing angle (in degrees) to point the arrow. 0 degrees is north.
    - center: The center position (tuple) from which the arrow originates.
    - length: The length of the arrow.
    """
    global receiver_track
    global trackup

    # Adjust the bearing for track-up mode if enabled
    if trackup:
        # Normalize the bearing by subtracting receiver_track, ensuring it remains within 0-360 degrees
        adjusted_bearing = (bearing - receiver_track) % 360
    else:
        adjusted_bearing = bearing

    # Convert adjusted bearing from degrees to radians
    bearing_radians = math.radians(adjusted_bearing)

    # Adjust drawing angle to point upwards and subtract 90 degrees since 0 radians points right
    draw_angle = bearing_radians - math.pi / 2

    # Calculate arrow endpoint based on adjusted bearing
    end_point = (
        center[0] + length * math.cos(draw_angle),
        center[1] + length * math.sin(draw_angle)
    )

    # Arrowhead points
    arrow_points = [
        (end_point[0] + 10 * math.cos(draw_angle + math.radians(150)), end_point[1] + 10 * math.sin(draw_angle + math.radians(150))),
        (end_point[0] + 10 * math.cos(draw_angle - math.radians(150)), end_point[1] + 10 * math.sin(draw_angle - math.radians(150))),
        end_point
    ]

    # Draw the arrow line
    draw.line([center, end_point], fill="black", width=3)

    # Draw the arrowhead
    draw.polygon(arrow_points, fill="black")


def oled_displays_with_arrow(file_path, receiver_lat, receiver_lon):
    global gps_update_success  # Make sure to access the global flag
    global own_ship_selected
    global own_ship_speed
    global own_ship
    global own_ship_alt
    global receiver_mode
    global receiver_speed
    global receiver_alt
    global uSat
    global own_ship_last_update_time
    disp_width, disp_height = 128, 64  # Replace with your LCD's resolution

    """
    Updates the OLED displays. Shows an arrow towards the closest aircraft on one of the OLEDs
    if there is an aircraft within 10 miles.
    """
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 12)
        acfont = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 34)
        searchfont = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 18)
        gpsfont = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 18)
    except IOError:
        font = ImageFont.load_default()

    if own_ship_selected:
      print (f"Own ship time: {(time.time() - own_ship_last_update_time)}")

    # Find the closest aircraft
    closest_aircraft = find_closest_aircraft(file_path, receiver_lat, receiver_lon)
    distance = 99

    
    if closest_aircraft:
       _, distance, bearing, altitude, _, _, _ = closest_aircraft

        # Check if the closest aircraft is within 2 miles
    if distance <= 2 and abs(altitude - receiver_altitude) <= 2000:
        # Create a new image for drawing
        with canvas(device_64) as draw:
            draw.rectangle(device_64.bounding_box, fill="white")
            # Adjust the draw_arrow_inverted and draw.text to use black for the drawing operations
            draw_arrow(draw, bearing)
            draw.text((1, 0), f"{distance:.1f}", fill="black", font=gpsfont)
            # Adjust the draw_arrow_inverted and draw.text to use black for the drawing operations

        if gps_update_success and (time.time() - own_ship_last_update_time) > 10:
          with canvas(device_32) as draw:
            draw.text((1, 0), f"GPS LOCK: {receiver_mode}D / {uSat}", fill="white", font= font)
            draw.text((1, 25), f"SPD: {receiver_speed:.1f}", fill="white", font= gpsfont)
            draw.text((1, 45), f" ALT: {int(receiver_altitude)}'", fill="white", font= gpsfont)

        elif own_ship_selected and (time.time() - own_ship_last_update_time) <= 10:
            with canvas(device_32) as draw:
              draw.text((1, 0), f"ADS-B: {own_ship}", fill="white", font= font)
              draw.text((1, 25), f"SPD: {int(own_ship_speed)}", fill="white", font= gpsfont)
              draw.text((1, 45), f" ALT: {int(own_ship_alt)}'", fill="white", font= gpsfont)


    else:
    # If the closest aircraft is farther than 2 miles, display the count as before
        count = len(process_aircraft_data(file_path, receiver_lat, receiver_lon))
        if gps_update_success and (time.time() - own_ship_last_update_time) > 10:
          with canvas(device_64) as draw:
            draw.text((5, 0), "Aircraft Tracking", fill="white", font=font)
            draw.text((40, 30), f" {count}", fill="white", font = acfont)
          with canvas(device_32) as draw:
            draw.text((1, 0), f"GPS LOCK: {receiver_mode}D / {uSat}", fill="white", font= font)
            draw.text((1, 25), f"SPD: {receiver_speed:.1f}", fill="white", font= gpsfont)
            draw.text((1, 45), f" ALT: {int(receiver_altitude)}'", fill="white", font= gpsfont)
        elif own_ship_selected and (time.time() - own_ship_last_update_time) <= 10:
          with canvas(device_64) as draw:
            draw.text((5, 0), "Aircraft Tracking", fill="white", font=font)
            draw.text((40, 30), f" {count}", fill="white", font = acfont)
          with canvas(device_32) as draw:
            draw.text((1, 0), f"ADS-B: {own_ship}", fill="white", font= font)
            draw.text((1, 25), f"SPD: {int(own_ship_speed)}", fill="white", font= gpsfont)
            draw.text((1, 45), f" ALT: {int(own_ship_alt)}'", fill="white", font= gpsfont)
        elif not gps_update_success:
          with canvas(device_64) as draw:
            draw.text((5, 0), "Aircraft Tracking", fill="white", font=font)
            draw.text((40, 30), f" {count}", fill="white", font = acfont)
          with canvas(device_32) as draw:
            draw.text((20, 0),"NO GPS LOCK", fill="WHITE", font=font)  # Adjust position as needed
            draw.text((1,30), "GPS SEARCH", fill="WHITE", font=searchfont)
    

def display_closest_aircraft_on_lcd(aircraft_info):
    global max_ac
    global gps_update_success  # Make sure to access the global flag
    with canvas(device) as draw:
      draw.rectangle((0, 0, 320, 240),  fill="BLACK")

      try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 18)
        headerfont = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 24)
      except IOError:
        font = ImageFont.load_default()
    
# Assuming aircraft_info is sorted and you want to display up to the closest 6 aircraft
      if gps_update_success or own_ship_selected:
        draw.text ((2,1),f"Closest Aircraft ", fill="YELLOW", font=headerfont)
        for i, (flight, distance, bearing, altitude, track, speed, vert_rate) in enumerate(aircraft_info[:6], start=2):  # Slice the list to the first 5 elements
            text = f"{flight}: {distance:.1f}nm  {bearing:.0f}°  {int(altitude / 100)}'"
            draw.text((2, i * 25 ), text, fill="WHITE", font=font)

      if not gps_update_success and not own_ship_selected:
        draw.text ((2,2),"Tracked Aircraft", fill="YELLOW", font=headerfont)
        for i, (flight, distance, bearing, altitude, track, speed, vert_rate) in enumerate(aircraft_info[:6], start=2):  # Slice the list to the first 5 elements
            text = f"{flight}: XXX XXX° {int(altitude / 100)}"
            draw.text((2, i * 25), text, fill="WHITE", font=font)

def plot_aircraft_on_radar(aircraft_info, max_range_nm):
    global gps_update_success  # Make sure to access the global flag
    global receiver_track
    global max_ac
    global trackup
    global receiver_altitude
    global reciever_speed
    global own_ship_last_update_time
    global filter_mode
    
    # Switch to trackup mode if in North up mode and speed is over 20kts

    if not trackup and receiver_speed > 20: 
      trackup = True


    # Initialize an image in memory
    disp_width, disp_height = 320, 240  # Replace with your LCD's resolution

    center_x, center_y = ((disp.width // 2.5) + 37), ((disp.height // 2.5) + 20)
    radar_radius = min(center_x, center_y) - 0  # Leave some margin

    # Define fonts and sizes
    main_font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 18)
    larger_font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 24)
    small_font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 10)
    alt_font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 20)
    diamond_size = 15
    base_speed = 75  # Define a base speed for scaling the line length
   
    if own_ship_selected and (time.time() - own_ship_last_update_time) <= 10:
      receiver_track = own_ship_track
      receiver_altitude = own_ship_alt
      print (f"plot {receiver_track}  {receiver_altitude}")
      print ("own ship based plot")

    # Draw radar, crosshairs, and degree markers
    with canvas(device) as draw:
      draw.ellipse((center_x - radar_radius, center_y - radar_radius, center_x + radar_radius, center_y + radar_radius), outline="yellow")


    # Vertical line: from top to bottom through the center

      draw.line((center_x, center_y - (radar_radius - 25), center_x, center_y + radar_radius), fill="yellow")


    # Horizontal line: from left to right through the center
      draw.line((center_x - radar_radius, center_y, center_x + radar_radius, center_y), fill="yellow")

    # Show own_ship or GPS based radar plots
      if own_ship_selected and (time.time() - own_ship_last_update_time) <= 10:
        draw.text((1, 195), "ADS-B", fill="YELLOW", font=main_font)
      else:
        draw.text((1, 195), "GPS", fill="YELLOW", font=main_font)


    # Filter mode indication
      if filter_mode: draw.text((1, 215), "Filter ON", fill="YELLOW", font=main_font)

    # Version Number
      draw.text((250, 220), "B2.2.2", fill="YELLOW", font=main_font)


      if not trackup:
        draw.text((157, 1), 'N', fill="white", font=larger_font)
        draw.text((285, 105), 'E', fill="white", font=larger_font)
        draw.text((20, 105), 'W', fill="white", font=larger_font)
   
      if trackup:
        draw.text((center_x - 17, 1),f"{int(receiver_track)}°", fill="YELLOW", font=larger_font)
        draw.text((250,0),"TRACK", fill="yellow", font=main_font)
        draw.text((255,20),"   UP", fill="yellow", font=main_font)

      else: 
        draw.text((250, 0), "NORTH", fill="yellow", font=main_font)
        draw.text((255,20),"   UP", fill="yellow", font=main_font)


      draw.text((1, 1), f"{max_range_nm}nm", fill="YELLOW", font=main_font)
      draw.text((1, 20), f"{max_ac} Max", fill="YELLOW", font=main_font)

      reverence_track_rad = math.radians(receiver_track - 90)

    # Plot aircraft as diamonds and draw direction lines
      for i, (flight, distance, bearing, altitude, track, speed, vert_rate) in enumerate(aircraft_info[:max_ac]):

    # Filter aircraft based on altitude difference if filter_mode is enabled
        if filter_mode and abs(altitude - receiver_altitude) > 5000:
          continue  # Skip this aircraft if it doesn't meet the altitude criteria

        if distance <= max_range_nm:
          scaled_distance = min(distance / max_range_nm, 1) * radar_radius

        # Adjust bearing relative to reference_track for track up orientation
          if trackup:
            relative_bearing_rad = math.radians(bearing - receiver_track - 90)
          else:
            relative_bearing_rad = math.radians(bearing - 90)

          x = center_x + scaled_distance * math.cos(relative_bearing_rad)
          y = center_y + scaled_distance * math.sin(relative_bearing_rad)

        # Calculate points for diamond
          points = [
              (x, y - diamond_size),
              (x + diamond_size, y),
              (x, y + diamond_size),
              (x - diamond_size, y)
          ]
          rel_alt = ((altitude - receiver_altitude) / 100)


          #Highlight closest aircraft
          #print (f"Distance = {int(distance)} Rel_Alt = {int(rel_alt)}")
          
          if abs(rel_alt) <= 20 and distance <= 5:
            draw.polygon(points, fill="RED", outline="WHITE")
          else:
            draw.polygon(points, fill="green", outline="WHITE")

        # Draw direction line based on track and speed
          if not trackup:
            track_rad = math.radians(track - 90)  # Convert track to radians and adjust
            line_length = (speed / base_speed) * 10  # Scale line length by speed
            end_x = x + line_length * math.cos(track_rad)
            end_y = y + line_length * math.sin(track_rad)
            draw.line((x, y, end_x, end_y), fill="WHITE", width=1)


        # Calculate direction line with adjusted bearing
          if trackup:
            relative_track_rad = math.radians(track - receiver_track - 90)
            line_length = (speed / base_speed) * 10
            end_x = x + line_length * math.cos(relative_track_rad)
            end_y = y + line_length * math.sin(relative_track_rad)
            draw.line((x, y, end_x, end_y), fill="WHITE", width=1)


          # Display altitude
          #rel_alt = ((altitude - receiver_altitude) / 100)
          signed_rel_alt = "{:+.0f}".format(rel_alt)
          draw.text((x + 13, y - 13), f"{signed_rel_alt}", fill="WHITE", font=alt_font)

          # Measure text size
          # text_width, text_height = draw.textsize(signed_rel_alt, font=alt_font)

          # Now, use text_width to position the arrow correctly
          if abs(vert_rate) > 500:
              arrow = '↑' if vert_rate > 0 else '↓'
              arrow_x = x - 30 # Adjust position based on the width of the altitude text
              arrow_y = y - 13
              draw.text((arrow_x, arrow_y), arrow, fill="WHITE", font=alt_font)

def draw_attitude_indicator():
    """
    Efficiently updates the attitude indicator:
    - Runs in a loop while display_mode == 4.
    - Uses double buffering to prevent flickering.
    - Smoothly updates pitch, roll, and slip/skid ball.
    """
    global display_mode  # Ensure we can check if mode changes

    screen_width, screen_height = device.width, device.height
    crosshair_size = 10  # Size of the center crosshair

    # Slip/Skid Indicator Configuration (Larger & Less Sensitive)
    skid_tube_width = int(screen_width * 0.75)  # Tube width = 75% of screen width
    skid_tube_height = 14  # Height of the tube
    skid_ball_radius = 8  # Ball size
    max_slip_angle = 10  # Maximum slip angle (degrees)
    max_skid_range = (skid_tube_width // 2) - skid_ball_radius  # Max left/right ball movement

    while display_mode == 4:  # Stay in this loop while in attitude display mode
        roll = -attitude_data["roll"]
        pitch = attitude_data["pitch"]  # Reverse pitch (correct behavior)
        slip_skid = -attitude_data["slip_skid"]

        # Scale slip/skid to limit max deflection to 10° range
        slip_ratio = max(-1, min(slip_skid / max_slip_angle, 1))  # Normalize (-1 to 1 range)
        ball_x = screen_width // 2 + int(slip_ratio * max_skid_range)

        # Ensure ball stays inside the slip/skid tube
        ball_x = max((screen_width - skid_tube_width) // 2 + skid_ball_radius, 
                     min(ball_x, (screen_width + skid_tube_width) // 2 - skid_ball_radius))
        ball_y = screen_height - 25  # Position slightly higher

        # Double buffering
        img = Image.new("RGB", (screen_width, screen_height), "black")
        draw = ImageDraw.Draw(img)

        # Horizon rendering
        roll_rad = math.radians(roll)
        horizon_y = screen_height // 2 + int(pitch * 5)
        x1, y1 = 0, horizon_y - int(math.tan(roll_rad) * screen_width // 2)
        x2, y2 = screen_width, horizon_y + int(math.tan(roll_rad) * screen_width // 2)
        draw.polygon([(x1, y1), (x2, y2), (screen_width, 0), (0, 0)], fill="blue")
        draw.polygon([(x1, y1), (x2, y2), (screen_width, screen_height), (0, screen_height)], fill="brown")

        # Draw static crosshair
        center_x = screen_width // 2
        center_y = screen_height // 2
        draw.line([(center_x - crosshair_size, center_y), (center_x + crosshair_size, center_y)], fill="white", width=2)
        draw.line([(center_x, center_y - crosshair_size), (center_x, center_y + crosshair_size)], fill="white", width=2)

        # Draw slip/skid tube and ball
        draw.rectangle([(screen_width//2 - skid_tube_width//2, ball_y - skid_tube_height//2),
                        (screen_width//2 + skid_tube_width//2, ball_y + skid_tube_height//2)], outline="white", width=2)
        draw.ellipse((ball_x - skid_ball_radius, ball_y - skid_ball_radius,
                      ball_x + skid_ball_radius, ball_y + skid_ball_radius), fill="black", outline="white")

        # Update display
        device.display(img)

        # Exit loop if display mode changes
        if display_mode != 4:
            break  

        time.sleep(UPDATE_RATE)  # Keep refreshing at 20 FPS

def draw_pressure():
    global BaroPressureAltitude
    with canvas(device) as draw:
        draw.rectangle((0, 0, 320, 240), fill="BLACK")

        try:
            font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 18)
            headerfont = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 24)
        except IOError:
            font = ImageFont.load_default()
            headerfont = font  # fallback


        # add a check for gps ()
        draw.text((2, 2), "Pressure", fill="YELLOW", font=headerfont)
        draw.text((2, 40), str(BaroPressureAltitude), fill="WHITE", font=font)



def main_loop(file_path):
    global radar_range_nm  # Declare radar_range_nm as global
    global display_mode    # Declare display_mode as global
    global receiver_lat    # Declare receiver_lat as global
    global receiver_lon    # Declare receiver_lon as global
    global gps_update_success  # Declare the flag as global in the function
    global trackup
    global own_ship_selected
    global own_ship_last_update_time

    display_mode = 2

    while True:
        if display_mode == 0 and own_ship_selected:
            unique_count = len(process_aircraft_data(file_path, receiver_lat, receiver_lon))
            closest_aircraft = process_aircraft_data(file_path, receiver_lat, receiver_lon)
            oled_displays_with_arrow(file_path, receiver_lat, receiver_lon)
            print_own_ship_adsb()

        elif display_mode == 0 and (time.time() - own_ship_last_update_time) > 10: display_mode = 1

        elif display_mode == 3:
            unique_count = len(process_aircraft_data(file_path, receiver_lat, receiver_lon))
            closest_aircraft = process_aircraft_data(file_path, receiver_lat, receiver_lon)
            display_closest_aircraft_on_lcd(closest_aircraft)
            oled_displays_with_arrow(file_path, receiver_lat, receiver_lon)

        elif display_mode == 2:
            #print(f"main {receiver_lat} {receiver_lon}")
            if gps_update_success or own_ship_selected and (time.time() - own_ship_last_update_time) <= 10:
                unique_count = len(process_aircraft_data(file_path, receiver_lat, receiver_lon))
                closest_aircraft = process_aircraft_data(file_path, receiver_lat, receiver_lon)
                oled_displays_with_arrow(file_path, receiver_lat, receiver_lon)
                plot_aircraft_on_radar(closest_aircraft, radar_range_nm)
            else:
                unique_count = len(process_aircraft_data(file_path, receiver_lat, receiver_lon))
                oled_displays_with_arrow(file_path, receiver_lat, receiver_lon)
                print_current_gps_loc()

        elif display_mode == 1:
            unique_count = len(process_aircraft_data(file_path, receiver_lat, receiver_lon))
            oled_displays_with_arrow(file_path, receiver_lat, receiver_lon)
            closest_aircraft = process_aircraft_data(file_path, receiver_lat, receiver_lon)
            print_current_gps_loc()

        elif display_mode == 4:
            unique_count = len(process_aircraft_data(file_path, receiver_lat, receiver_lon))
    
            oled_displays_with_arrow(file_path, receiver_lat, receiver_lon)
    
            # Run attitude indicator continuously until display_mode changes
            draw_attitude_indicator()
        
        # Now show pressure
        elif display_mode == 5: 
          oled_displays_with_arrow(file_path, receiver_lat, receiver_lon)
          
          draw_pressure()

        time.sleep(.25)  # Adjust as needed


if __name__ == "__main__":
    file_path = '/run/dump1090-mutability/aircraft.json'
    #json_file_path = '/run/gpsout.json'
    disp = device
    # Start GPS thread
    gps_thread = threading.Thread(target=parse_gps_data, args=())
    gps_thread.daemon = True
    gps_thread.start()   
    # Start the button monitoring thread
    button_thread = threading.Thread(target=monitor_buttons, args=())
    button_thread.daemon = True  # Ensures the thread exits when the main program does
    button_thread.start()
    # Start WebSocket thread for Attitude Data
    ws_thread = threading.Thread(target=lambda: asyncio.run(fetch_ahrs_data()), daemon=True)
    ws_thread.start()



    # Now start the main loop
    main_loop(file_path)
