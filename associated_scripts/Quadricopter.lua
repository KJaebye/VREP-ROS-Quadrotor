function sysCall_init() 
    -- Make sure we have version 2.4.13 or above (the particles are not supported otherwise)
    v=sim.getInt32Param(sim.intparam_program_version)
    if (v<20413) then
        sim.displayDialog('Warning','The propeller model is only fully supported from V-REP version 2.4.13 and above.&&nThis simulation will not run as expected!',sim.dlgstyle_ok,false,'',nil,{0.8,0,0,0,0,0})
    end

    -- Detatch the manipulation sphere:
    --targetObj=sim.getObjectHandle('Quadricopter_target')
    --sim.setObjectParent(targetObj,-1,true)

    -- This control algo was quickly written and is dirty and not optimal. It just serves as a SIMPLE example

    d=sim.getObjectHandle('Quadricopter_base')
    panTiltBase=sim.getObjectHandle('Pan_tilt')
    panTiltBase_=sim.getObjectHandle('Pan_tilt_')
    pos_panTiltBase={}

    particlesAreVisible=sim.getScriptSimulationParameter(sim.handle_self,'particlesAreVisible')
    sim.setScriptSimulationParameter(sim.handle_tree,'particlesAreVisible',tostring(particlesAreVisible))
    simulateParticles=sim.getScriptSimulationParameter(sim.handle_self,'simulateParticles')
    sim.setScriptSimulationParameter(sim.handle_tree,'simulateParticles',tostring(simulateParticles))

    propellerScripts={6.1,6.1,6.1,6.1}
    for i=1,4,1 do
        propellerScripts[i]=sim.getScriptHandle('Quadricopter_propeller_respondable'..i)
    end
    heli=sim.getObjectHandle(sim.handle_self)

    --particlesTargetVelocities={5.35,5.35,5.35,5.35}
    particlesTargetVelocities={6.1,6.1,6.1,6.1}

    fakeShadow=sim.getScriptSimulationParameter(sim.handle_self,'fakeShadow')
    if (fakeShadow) then
        shadowCont=sim.addDrawingObject(sim.drawing_discpoints+sim.drawing_cyclic+sim.drawing_25percenttransparency+sim.drawing_50percenttransparency+sim.drawing_itemsizes,0.2,0,-1,1)
    end


    
    
    if simROS then
    
        local Name=sim.getObjectName(heli)
        local id=string.gsub(Name, "%#", "_")
        
        odometry_pub=simROS.advertise('/'..id..'/sensor/gps', 'nav_msgs/Odometry')
        simROS.publisherTreatUInt8ArrayAsString(odometry_pub)
        Odometry_data={}
        pose={}

        motorSpeed_sub=simROS.subscribe('/'..id..'/command/motor_speed','mav_msgs/CommandMotorSpeed','setMotorSpeed')
    end
end

function setMotorSpeed(msg)
    particlesTargetVelocities=msg.motor_speed
end

function sysCall_sensing()
    
    -- publish the real odometry in the simulator
    -- including the current stamp, current position, current orientation,
    -- and the current linear velocity, current angular velocity. 
    -- Note: All the values are relative to the world frame
    quaternion = sim.getObjectQuaternion(d,-1)
    position=sim.getObjectPosition(d,-1)
    linear_vel,angular_vel=sim.getObjectVelocity(d)

    if simROS then
    
    Odometry_data.header={seq=0,stamp=simROS.getTime(),frame_id="odometry_frame"}
    Odometry_data.pose={
                pose={
                    position={x=position[1],y=position[2],z=position[3]},
                    orientation={x=quaternion[1],y=quaternion[2],z=quaternion[3],w=quaternion[4]}
                    }}
    Odometry_data.twist={
                twist={
                    linear={x=linear_vel[1],y=linear_vel[2],z=linear_vel[3]},
                    angular={x=angular_vel[1],y=angular_vel[2],z=angular_vel[3]}
                    }}
    
    simROS.publish(odometry_pub, Odometry_data)
    end


-- set the pan tilt position and orientation
    pos_=sim.getObjectPosition(panTiltBase_,-1)
    
    -- this is for special type, that the pan tilt position z is absolute the
    -- drone position, ignoring the influence of pitch and roll
    pos_panTiltBase[1]=pos_[1]
    pos_panTiltBase[2]=pos_[2]
    pos_panTiltBase[3]=pos_[3]
    
    -- this is the pan tilt type
    --pos_panTiltBase[1]=pos_[1]
    --pos_panTiltBase[2]=pos_[2]
    --pos_panTiltBase[3]=pos_[3]
    
    sim.setObjectPosition(panTiltBase,-1,pos_panTiltBase)
    sim.setObjectOrientation(panTiltBase,-1,{0,0,0})
end

function sysCall_cleanup() 
    sim.removeDrawingObject(shadowCont)

    simROS.shutdownPublisher(odometry_pub)
    simROS.shutdownSubscriber(motorSpeed_sub)
end 

function sysCall_actuation() 
    s=sim.getObjectSizeFactor(d)
    
    pos=sim.getObjectPosition(d,-1)
    if (fakeShadow) then
        itemData={pos[1],pos[2],0.002,0,0,1,0.2*s}
        sim.addDrawingObjectItem(shadowCont,itemData)
    end

    -- Send the desired motor velocities to the 4 rotors:
    for i=1,4,1 do
        sim.setScriptSimulationParameter(propellerScripts[i],'particleVelocity',particlesTargetVelocities[i])
    end
end 
