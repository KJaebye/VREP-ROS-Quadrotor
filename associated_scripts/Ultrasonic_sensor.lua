function sysCall_init()
    -- do some initialization here
    --ultrasonic_sensor=sim.getObjectHandle('Ultrasonic_sensor')
    ultrasonic_sensor=sim.getObjectHandle(sim.handle_self)
    pan_tilt=sim.getObjectParent(ultrasonic_sensor)
    heli=sim.getObjectParent(pan_tilt)
    if simROS then
        local Name=sim.getObjectName(heli)
        local id=string.gsub(Name, "%#", "_")

    ultrasonicDistance_pub = simROS.advertise('/'..id..'/sensor/ultrasonic_distance','sensor_msgs/Range')
    simROS.publisherTreatUInt8ArrayAsString(ultrasonicDistance_pub)
    end
   -- ultrasonicDistance_pub=simROS.advertise('/ultrasonic_distance','sensor_msgs/Range')
    Ultrasonic_sensor_data={}
end

function sysCall_actuation()
    -- put your actuation code here
end

function sysCall_sensing()
    -- ultrasonic sensor gets the veltical distance to object
    result,distance = sim.readProximitySensor(ultrasonic_sensor)
    Ultrasonic_sensor_data.header={seq=0,stamp=simROS.getTime(),frame_id="ultrasonic_sensor_frame"}
    Ultrasonic_sensor_data.radiation_type=0
    Ultrasonic_sensor_data.range=distance
    simROS.publish(ultrasonicDistance_pub,Ultrasonic_sensor_data)
end

function sysCall_cleanup()
    simROS.shutdownPublisher(ultrasonicDistance_pub)
end

-- See the user manual or the available code snippets for additional callback functions and details
