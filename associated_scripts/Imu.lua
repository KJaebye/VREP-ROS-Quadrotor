-- DO NOT WRITE CODE OUTSIDE OF THE if-then-end SECTIONS BELOW!! (unless the code is a function definition)

if (sim_call_type==sim.syscb_init) then

    --    simExtROS_enablePublisher('my_topic_imu',1,simros_strmcmd_get_imu_state,-1,-1,'imu_signal'
    
        d=sim.getObjectHandle(sim.handle_self)
        heli=sim.getObjectParent(d)
        if simROS then
            local Name=sim.getObjectName(heli)
            local id=string.gsub(Name, "%#", "_")
    
        pub = simROS.advertise('/'..id..'/sensor/imu', 'sensor_msgs/Imu')
        simROS.publisherTreatUInt8ArrayAsString(pub)
        end
        
        Imu_data={}
    
        gyroCommunicationTube=sim.tubeOpen(0,'gyroData'..sim.getNameSuffix(nil),1)
        accelCommunicationTube=sim.tubeOpen(0,'accelerometerData'..sim.getNameSuffix(nil),1)
        
        h_Imu = sim.getObjectHandle('Imu')
    end
    
    
    if (sim_call_type==sim.syscb_actuation) then
    
    
    end
    
    
    if (sim_call_type==sim.syscb_sensing) then
        
        -- Put your main SENSING code here
    
        quaternion = sim.getObjectQuaternion(h_Imu,-1)
    
        accele_data=sim.tubeRead(accelCommunicationTube)
        gyro_data=sim.tubeRead(gyroCommunicationTube)
    
        if (accele_data and gyro_data) then
            acceleration=sim.unpackFloatTable(accele_data)
            angularVariations=sim.unpackFloatTable(gyro_data)
            Imu_data['orientation'] = {x=quaternion[1],y=quaternion[2],z=quaternion[3],w=quaternion[4]}
            Imu_data['header']={seq=0,stamp=simROS.getTime(), frame_id="imu_frame"}
            Imu_data['linear_acceleration']= {x=acceleration[1],y=acceleration[2],z=acceleration[3]}
            Imu_data['angular_velocity'] = {x=angularVariations[1],y=angularVariations[2],z=angularVariations[3]}
    
            simROS.publish(pub, Imu_data)
        end
    
    end
    
    
    if (sim_call_type==sim.syscb_cleanup) then
        simROS.shutdownPublisher(pub)
    end
    