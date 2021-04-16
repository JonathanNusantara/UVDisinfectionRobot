import qwiic
import time

ToF = qwiic.QwiicVL53L1X()
if (ToF.sensor_init() == None):                  # Begin returns 0 on a good init
    print("Sensor online!\n")
ToF.set_distance_mode(2)

while True:
    try:
        ToF.start_ranging()                      # Write configuration bytes to initiate measurement
        time.sleep(.005)
        distance = ToF.get_distance()    # Get the result of the measurement from the sensor
        time.sleep(.005)
        ToF.stop_ranging()

        distanceInches = distance / 25.4
        distanceFeet = distanceInches / 12.0

        print("Distance(mm): %s Distance(ft): %s" % (distance, distanceFeet))

    except Exception as e:
        print(e)
