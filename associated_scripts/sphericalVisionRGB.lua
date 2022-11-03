function sysCall_init() 
    p=sim.getObjectHandle('sphericalVisionRGB_sensor')
    a={-1,-1,-1,-1,-1,-1}
    a[1]=sim.getObjectHandle('sphericalVisionRGB_front')
    a[2]=sim.getObjectHandle('sphericalVisionRGB_top')
    a[3]=sim.getObjectHandle('sphericalVisionRGB_back')
    a[4]=sim.getObjectHandle('sphericalVisionRGB_bottom')
    a[5]=sim.getObjectHandle('sphericalVisionRGB_left')
    a[6]=sim.getObjectHandle('sphericalVisionRGB_right')

    resX=sim.getScriptSimulationParameter(sim.handle_self,'sensorResolutionX')
    resY=sim.getScriptSimulationParameter(sim.handle_self,'sensorResolutionY')
    resXY=sim.getScriptSimulationParameter(sim.handle_self,'boxSensorResolutionXY')
    horizontalAngle=sim.getScriptSimulationParameter(sim.handle_self,'horizontalAngle')
    verticalAngle=sim.getScriptSimulationParameter(sim.handle_self,'verticalAngle')
    updateEveryXSimulationPass=sim.getScriptSimulationParameter(sim.handle_self,'updateEveryXSimulationPass')

    for i=1,6,1 do
        sim.setObjectInt32Param(a[i],sim.visionintparam_resolution_x,resXY)
        sim.setObjectInt32Param(a[i],sim.visionintparam_resolution_y,resXY)
    end
    sim.setObjectInt32Param(p,sim.visionintparam_resolution_x,resX)
    sim.setObjectInt32Param(p,sim.visionintparam_resolution_y,resY)

    view=sim.floatingViewAdd(0.8,0.8,0.4,0.4,0)
    sim.adjustView(view,p,64)
    
    pass=0
    
    
    -- Enable an image publisher:
    if simROS then
        print("<font color='#0F0'>ROS interface was found.</font>@html")
        
        base=sim.getObjectParent(p)
        pan_tilt=sim.getObjectParent(base)
        heli=sim.getObjectParent(pan_tilt)
        local Name=sim.getObjectName(heli)
        local id=string.gsub(Name, "%#", "_")
        
        
        pub=simROS.advertise('/'..id..'/vision/panoramic', 'sensor_msgs/Image')
        simROS.publisherTreatUInt8ArrayAsString(pub) -- treat uint8 arrays as strings (much faster, tables/arrays are kind of slow in Lua)
    else
        print("<font color='#F00'>ROS interface was not found. Cannot run.</font>@html")
    end
    
end
-- Note: the spherical vision sensor is supported via the plugin simExtVision, which exports a
-- custom Lua function: simVision.handleSpherical (see further down for the documentation of this function)
-- The source code of the plugin is located in CoppeliaSim's programming folder, and you can freely
-- adapt it. If you add some specific filter or other useful feature to that plugin, let us know, so that we can add it too.


function sysCall_cleanup() 
    if simROS then   
        -- Shut down publisher and subscriber. Not really needed from a simulation script (automatic shutdown)
        simROS.shutdownPublisher(pub)
    end
end 

function sysCall_sensing() 
    pass=pass+1
    
    if pass>=updateEveryXSimulationPass then
        result=simVision.handleSpherical(p,a,horizontalAngle*math.pi/180,verticalAngle*math.pi/180,-1)
        pass=0
    end
    
    if simROS then   
        -- Publish the image of the panoramic vision sensor:
        local data,w,h=sim.getVisionSensorCharImage(p)
        d={}
        d['header']={stamp=simROS.getTime(), frame_id="a"}
        d['height']=h
        d['width']=w
        d['encoding']='rgb8'
        d['is_bigendian']=1
        d['step']=w*3
        d['data']=data
        simROS.publish(pub,d)
    end
    
    -- How to use the simVision.handleSpherical function:
    -- result=simVision.handleSpherical(passiveVisionSensorHandle,tableOfSixVisionSensorHandles,
    --                                     horizontalSweepAngle,verticalSweepAngle)
    -- In total there are 7 vision sensor objects needed: one passive vision sensor, that will be used to
    -- store the calculated image, and 6 active vision sensors that will look into the 6 direction of space
    -- in following order: front, top, back, bottom, left, right
    -- All vision sensors need to be flagged as 'explicit handling'. The active vision sensor's resolution
    -- need to be all the same, and resolutionX==resolutionY
    --
    -- If you open the hierarchy tree branch of the spherical vision model, you will notice a model
    -- named 'sphericalVision_box'. If you open its model properties, you can make it visible, and 
    -- display the images recorded by the 6 active sensors
    --
    -- Currently, the spherical vision model will disable the specular lighting during operation.
end 
