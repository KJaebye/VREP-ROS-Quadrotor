function sysCall_init()
    -- do some initialization here
    -- Retrieve some handles:
    frontSensorHandle=sim.getObjectHandle('Proximity_sensor_front')
    leftSensorHandle=sim.getObjectHandle('Proximity_sensor_left')
    rightSensorHandle=sim.getObjectHandle('Proximity_sensor_right')
    backSensorHandle=sim.getObjectHandle('Proximity_sensor_back')
    --graphHandle=sim.getObjectHandle("Graph")
    
    proximity_sensor=sim.getObjectHandle(sim.handle_self)
    pan_tilt=sim.getObjectParent(proximity_sensor)
    heli=sim.getObjectParent(pan_tilt)
    if simROS then
        local Name=sim.getObjectName(heli)
        local id=string.gsub(Name, "%#", "_")

    proximity_front_pub = simROS.advertise('/'..id..'/sensor/proximity/front','sensor_msgs/LaserEcho')
    proximity_left_pub = simROS.advertise('/'..id..'/sensor/proximity/left','sensor_msgs/LaserEcho')
    proximity_right_pub = simROS.advertise('/'..id..'/sensor/proximity/right','sensor_msgs/LaserEcho')
    proximity_back_pub = simROS.advertise('/'..id..'/sensor/proximity/back','sensor_msgs/LaserEcho')

    end
end

function sysCall_actuation()
    -- put your actuation code here
end

function sysCall_sensing()
    local Proximity_front_data={}
    local Proximity_left_data={}
    local Proximity_right_data={}
    local Proximity_back_data={}
    -- Read the proximity sensor:
    local result_front,distance_front,detectedPoint_front,detectedObjectHandle_front,detectedSurfaceNormalVector_front=sim.readProximitySensor(frontSensorHandle)
    local result_left,distance_left,detectedPoint_left,detectedObjectHandle_left,detectedSurfaceNormalVector_left=sim.readProximitySensor(leftSensorHandle)
    local result_right,distance_right,detectedPoint_right,detectedObjectHandle_right,detectedSurfaceNormalVector_right=sim.readProximitySensor(rightSensorHandle)
    local result_back,distance_back,detectedPoint_back,detectedObjectHandle_back,detectedSurfaceNormalVector_back=sim.readProximitySensor(backSensorHandle)
    -- plot data
    if(result_front>0) then
        --sim.setGraphUserData(graphHandle,"sensor_front",distance_front)
        Proximity_front_data.echoes={
            distance_front,
            detectedPoint_front[1],
            detectedPoint_front[2],
            detectedPoint_front[3]}
        simROS.publish(proximity_front_pub,Proximity_front_data)
    end
    if(result_left>0) then
        --print(distance_left)
        --print(detectedSurfaceNormalVector_left)
        --sim.setGraphUserData(graphHandle,"sensor_left",distance_left)
        Proximity_left_data.echoes={
            distance_left,
            detectedPoint_left[1],
            detectedPoint_left[2],
            detectedPoint_left[3]}
        --print(Proximity_left_data)
        simROS.publish(proximity_left_pub,Proximity_left_data)
    end
    if(result_right>0) then
        --sim.setGraphUserData(graphHandle,"sensor_right",distance_right)
        Proximity_right_data.echoes={
            distance_right,
            detectedPoint_right[1],
            detectedPoint_right[2],
            detectedPoint_right[3]}
        simROS.publish(proximity_right_pub,Proximity_right_data)
    end
    if(result_back>0) then
        --sim.setGraphUserData(graphHandle,"sensor_back",distance_back)
        Proximity_back_data.echoes={
            distance_back,
            detectedPoint_back[1],
            detectedPoint_back[2],
            detectedPoint_back[3]}
        simROS.publish(proximity_back_pub,Proximity_back_data)
    end
    
    
end

function sysCall_cleanup()
    -- do some clean-up here
    simROS.shutdownPublisher(proximity_front_pub)
    simROS.shutdownPublisher(proximity_left_pub)
    simROS.shutdownPublisher(proximity_right_pub)
    simROS.shutdownPublisher(proximity_back_pub)
end

-- See the user manual or the available code snippets for additional callback functions and details
