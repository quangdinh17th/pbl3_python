def calculate_zambretti(bmp_sensor, month)
 current_pressure = bmp_sensor.pressure
 altitude = bmp_sensor.altitude
 previous_pressure = current_pressure
 lapse_rate = 0.0065  # Standard lapse rate for temperature in K/m
 temperature_kelvin = temperature + 273.15  # Convert temperature to Kelvin
 while True:
    P0 = current_pressure * (1 - (lapse_rate * altitude) / (temperature_kelvin + (lapse_rate * altitude))) ** -5.275
  if current_pressure < previous_pressure:
      z = 127 - 0.12*P0
      if month >=6 and month < 8:
          z = z - 1
  elif current_pressure == previous_pressure:
      z = 144 - 0.13*P0  
  elif current_pressure > previous_pressure:
      z = 185 - 0.16*P0
      if month >=12 and month month < 2:
          z = z + 1
          
 z = round(z)