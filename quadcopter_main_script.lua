-- Adapted from V-REP quadcopter model


-- I would like to add a battery simulator to show the effects of different batteries:
-- https://oscarliang.com/how-to-choose-battery-for-quadcopter-multicopter/


-- Explanation about what is returned by sim.GetObjectPosition, sim.GetObjectMatrix, sim.GetObjectVelocity and sim.AddForceAndTorque:
-- http://www.forum.coppeliarobotics.com/viewtopic.php?t=4753


print('SIMULATION START @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@')
-- result=sim.addStatusbarMessage("Starting")
-- sim.displayDialog('Title','First line....&&nSecond line...',sim.dlgstyle_ok,false,'',nil,{0.8,0,0,0,0,0})


function sysCall_init()

    -- Detatch the manipulation sphere:
    targetObj=sim.getObjectHandle('Quadcopter_target')
    sim.setObjectParent(targetObj,-1,true)

    
    Quad_base=sim.getObjectHandle('Quadcopter')
    -- Quad=sim.getObjectAssociatedWithScript(sim.handle_self) --sim.handle_self returns a handle for the current script


    propellerScripts={-1,-1,-1,-1}
    for i=1,4,1 do
        propellerScripts[i]=sim.getScriptHandle('Propeller'..i)
    end


    dt=sim.getSimulationTimeStep()


    thrust_gravity_compensation = 0 -- compensation for gravity, a bias value for thrust
    thrust         = 0
    pitch_corr     = 0
    yaw_corr       = 0
    roll_corr      = 0

    -- elevation
    Kzref         = 3
    Dzref         = 1
    Izref         = 0.1
    z_error       = 0
    z_error_sum   = 0
    z_pv          = 0

    -- pitch (x)
    Kxref         = 1
    Dxref         = 0.5
    Ixref         = 0
    x_error       = 0
    x_error_sum   = 0
    x_pv          = 0

    pitch_pv      = 0
    Kpitch        = 2
    Dpitch        = 0.5

    pitch_max     = 0.1
    pitch_min     = -0.1


    -- roll (y)
    Kyref         = 1
    Dyref         = 0.5
    Iyref         = 0
    y_error       = 0
    y_error_sum   = 0
    y_pv          = 0
    
    roll_pv       = 0
    Kroll         = 2
    Droll         = 0.5
    
    roll_max     = 0.1
    roll_min     = -0.1

    -- yaw
    yaw_pv       = 0
    Kyaw         = 0.8
    Dyaw         = 0.1

    -- propeller indices
    prop_front_left  = 1
    prop_back_left   = 4
    prop_back_right  = 3
    prop_front_right = 2
    propeller_cmds = {0,0,0,0}

end

function sysCall_cleanup() 
    --sim.removeDrawingObject(shadowCont)
end 

function sysCall_actuation() 


    -- Sensing
    targetPos=sim.getObjectPosition(targetObj,-1) --dummy sphere position
    targetOri=sim.getObjectPosition(targetObj,-1) --dummy sphere orientation
    position=sim.getObjectPosition(Quad_base,-1) --quadcopter position
    orientation=sim.getObjectOrientation(Quad_base,-1) --quadcopter orientation

    z_curr = position[3]
    dz = (z_curr-z_pv)/dt
    z_pv = z_curr
    z_error = targetPos[3]-z_curr
    z_error_sum = z_error_sum + z_error
    thrust = thrust_gravity_compensation + Kzref*z_error + Izref*z_error_sum - Dzref*dz


    -- roll correction (also y)
    roll = orientation[1]
    droll = (roll-roll_pv)/dt
    roll_pv = roll

    y_curr = position[2]
    dy = (y_curr-y_pv)/dt
    y_pv = y_curr
    y_error = targetPos[2]-y_curr
    y_error_sum = y_error_sum + y_error
    
    -- roll_corr = -Kroll*roll-Droll*droll
    roll_corr = -Kroll*roll-Droll*droll-(Kyref*y_error+Iyref*y_error_sum-Dyref*dy)
    
    if roll_corr>roll_max then
        roll_corr=roll_max
    end
    
    if roll_corr<roll_min then
        roll_corr=roll_min
    end

    -- pitch correction (also x)
    pitch = orientation[2]
    dpitch = (pitch-pitch_pv)/dt
    pitch_pv = pitch

    x_curr = position[1]
    dx = (x_curr-x_pv)/dt
    x_pv = x_curr
    x_error = targetPos[1]-x_curr
    x_error_sum = x_error_sum + x_error
    
    -- pitch_corr = -Kpitch*pitch-Dpitch*dpitch
    pitch_corr = -Kpitch*pitch-Dpitch*dpitch+(Kxref*x_error+Ixref*x_error_sum-Dxref*dx)

    if pitch_corr>pitch_max then
        pitch_corr=pitch_max
    end
    
    if pitch_corr<pitch_min then
        pitch_corr=pitch_min
    end


    -- yaw correction
    yaw = orientation[3]
    dyaw = (yaw-yaw_pv)/dt
    yaw_pv = yaw
    yaw_corr = -Kyaw*yaw+Dyaw*dyaw


    -- propellers velocities computation
    propeller_cmds[prop_front_left] = thrust*(1 - pitch_corr + roll_corr - yaw_corr)
    propeller_cmds[prop_back_left]  = thrust*(1 + pitch_corr + roll_corr + yaw_corr)
    propeller_cmds[prop_back_right] = thrust*(1 + pitch_corr - roll_corr - yaw_corr)
    propeller_cmds[prop_front_right]= thrust*(1 - pitch_corr - roll_corr + yaw_corr)
    
    -- Send the desired motor velocities to the 4 rotors:
    for i=1,4,1 do
        sim.setScriptSimulationParameter(propellerScripts[i],'ang_vel',propeller_cmds[i])
    end

    print('thrust:'..thrust)
    print('roll='..orientation[1]..' - pitch='..orientation[2]..' - yaw='..orientation[3])
    print('roll_corr='..roll_corr..' - pitch_corr='..pitch_corr..' - yaw_corr='..yaw_corr)
    print('x_curr='..x_curr..' - y_curr='..y_curr..' - z_curr='..z_curr)
    print('x_error='..x_error..' - y_curr='..y_error..' - z_curr='..z_error)

end 
