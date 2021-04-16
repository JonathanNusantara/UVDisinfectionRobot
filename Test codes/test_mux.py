import qwiic_tca9548a
import qwiic_vl53l1x
import time

mux = qwiic_tca9548a.QwiicTCA9548A()
mux.disable_all()
mux.enable_channels(0)

ToF = qwiic_vl53l1x.QwiicVL53L1X()
if (ToF.sensor_init() == None):                  # Begin returns 0 on a good init
    print("Sensor 0 online!\n")
ToF.set_distance_mode(2)
mux.disable_all()
mux.enable_channels(6)
if (ToF.sensor_init() == None):                  # Begin returns 0 on a good init
    print("Sensor 6 online!\n")
ToF.set_distance_mode(2)
mux.disable_all()
mux.enable_channels(7)
if (ToF.sensor_init() == None):                  # Begin returns 0 on a good init
    print("Sensor 7 online!\n")
ToF.set_distance_mode(2)

while True:
    try:
        #ToF 0
        mux.disable_all()
        mux.enable_channels(0)
        
        ToF.start_ranging()                      # Write configuration bytes to initiate measurement
        time.sleep(.005)
        distance = ToF.get_distance()    # Get the result of the measurement from the sensor
        time.sleep(.005)
        ToF.stop_ranging()

        distanceInches = distance / 25.4
        distanceFeet = distanceInches / 12.0

        print("ToF 0 -- Distance(mm): %s Distance(ft): %s" % (distance, distanceFeet))
        
        #ToF 6
        mux.disable_all()
        mux.enable_channels(6)
        
        ToF.start_ranging()                      # Write configuration bytes to initiate measurement
        time.sleep(.005)
        distance = ToF.get_distance()    # Get the result of the measurement from the sensor
        time.sleep(.005)
        ToF.stop_ranging()

        distanceInches = distance / 25.4
        distanceFeet = distanceInches / 12.0

        print("ToF 6 -- Distance(mm): %s Distance(ft): %s" % (distance, distanceFeet))
        
        #ToF 7
        mux.disable_all()
        mux.enable_channels(7)
        
        ToF.start_ranging()                      # Write configuration bytes to initiate measurement
        time.sleep(.005)
        distance = ToF.get_distance()    # Get the result of the measurement from the sensor
        time.sleep(.005)
        ToF.stop_ranging()

        distanceInches = distance / 25.4
        distanceFeet = distanceInches / 12.0

        print("ToF 7 -- Distance(mm): %s Distance(ft): %s" % (distance, distanceFeet))
        
        time.sleep(0.5)
        
    except Exception as e:
        print(e)

#mux.disable_all()
#mux.list_channels()