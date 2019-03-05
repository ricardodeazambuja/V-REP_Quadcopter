function sysCall_init()
        
    -- Launch a headless V-REP instance:
    appPath=sim.getStringParameter(sim.stringparam_application_path)
    sim.launchExecutable(appPath..'/vrep','-h -gREMOTEAPISERVERSERVICE_19900_FALSE_TRUE',1)
    connectionDelay=5 -- in seconds
    startTime=sim.getSystemTimeInMs(-1)
    dlgHandle=sim.displayDialog("Info","Please wait for a few seconds, until the headless instance has launched.",sim.dlgstyle_message,false)
    
end


function sysCall_sensing()
    if sim.getSystemTimeInMs(startTime)>connectionDelay*1000 then
        if not alreadyPassed then
            alreadyPassed=true
            sim.endDialog(dlgHandle)

            -- Connect to the headless instance of V-REP:
            clientId=simx.start('',19900,false,false,10000,0)
            -- Load a scene to the headless instance:
            local theOs=sim.getInt32Parameter(sim.intparam_platform)
            if theOs==1 then
                -- MacOS, special: executable is inside of a bundle:
                simx.loadScene(clientId,appPath..'/../../../scenes/proximitySensorDemo.ttt',0,simx.opmode_oneshot)
            else
                simx.loadScene(clientId,appPath..'/scenes/proximitySensorDemo.ttt',0,simx.opmode_oneshot)
            end
            -- Add a vision sensor to the headless scene, so that we can see what is going on:
            local remoteSensor=sim.getObjectHandle('remoteVisionSensor')
            sim.setObjectInt32Parameter(remoteSensor,sim.visionintparam_rendering_attributes,sim.displayattribute_renderpass+sim.displayattribute_forvisionsensor+sim.displayattribute_ignorerenderableflag) -- make it behave as a camera
            local remoteSensorScript=sim.getScriptAssociatedWithObject(remoteSensor)
            sim.setScriptAttribute(remoteSensorScript,sim.childscriptattribute_enabled,true) -- enabled in the other scene
            sim.saveModel(remoteSensor,'tmp_remoteSensor.ttm')
            sim.setScriptAttribute(remoteSensorScript,sim.childscriptattribute_enabled,false) -- disabled in this scene
            ret,remoteSensorHandle=simx.loadModel(clientId,'tmp_remoteSensor.ttm',0,simx.opmode_oneshot_wait)
            -- Start the remote simulation:
            simx.startSimulation(clientId,simx.opmode_oneshot)
            -- Prepare the streaming of the remote sensor's image:
            simx.getVisionSensorImage(clientId,remoteSensorHandle,0,simx.opmode_streaming)
            -- Get the handle of the passive vision sensor in this scene (will be used to visualize the remote image):
            passiveSensorHandle=sim.getObjectAssociatedWithScript(sim.handle_self)
        end

        -- Read the image from the remote sensor:
        local ret,img=simx.getVisionSensorImage(clientId,remoteSensorHandle,0,simx.opmode_buffer)
        if ret==simx.return_ok then
            -- Apply the image to the passive vision sensor in this scene:
            sim.setVisionSensorCharImage(passiveSensorHandle,img)
        end
    end
end


function sysCall_cleanup()
    -- Stop the remote simulation:
    simx.stopSimulation(clientId,simx.opmode_oneshot_wait) -- this just sends the stop request
    -- Wait until the simulation has really stopped (otherwise we cannot kill the headless instance):
    while true do
        local res,info=simx.getInMessageInfo(clientId,simx.headeroffset_server_state)
        if sim.boolAnd32(info,1)==0 then
            break
        end
    end
    -- Kill the headless V-REP instance: 
    simx.setBooleanParameter(clientId,sim.boolparam_exit_request,true,simx.opmode_oneshot_wait)
end

