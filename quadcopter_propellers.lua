-- If I decide using an external script called myRoutines.lua
--require('propellers')

function sysCall_init()
    curr_name=sim.getObjectName(sim.getObjectAssociatedWithScript(sim.handle_self))
    print('Initializing '..curr_name)

    i=string.sub(curr_name, -1)

    propeller=sim.getObjectHandle('Propeller'..i)
end

function sysCall_cleanup() 
    sim.setShapeColor(propeller,nil,0,{0,0,1.0}) 
end 

function sysCall_actuation()
    local t=sim.getSimulationTime()
    -- local ts=sim.getSimulationTimeStep()

    ang_vel=sim.getScriptSimulationParameter(sim.handle_self,'ang_vel')
    k_lift=sim.getScriptSimulationParameter(sim.handle_self,'k_lift')
    b_drag=sim.getScriptSimulationParameter(sim.handle_self,'b_drag')
    max_vel=sim.getScriptSimulationParameter(sim.handle_self,'max_vel')


    if ang_vel>max_vel then
        ang_vel=max_vel
        print('Actuator Saturated='..max_vel..' t='..t)
    end
        
    m=sim.getObjectMatrix(propeller,-1)

    -- Apply a reactive force onto the propeller's body:
    -- Considering w (angular velocity, ang_vel) and k (lift constant): f=k*w^2
    force={0,0,k_lift*math.pow(ang_vel,2)}
    m[4]=0
    m[8]=0
    m[12]=0
    force=sim.multiplyVector(m,force)

    -- Apply a torque onto the propeller's body:
    -- Considering w (angular velocity, ang_vel) and b (drag constant): T=b*w^2
    if i%2==0 then 
        direction = 1
    else
        direction = -1 -- propellers 1 and 3, negative value
    end
    torque={0,0,direction*b_drag*math.pow(ang_vel,2)}
    torque=sim.multiplyVector(m,torque)

    sim.addForceAndTorque(propeller,force,torque)

    -- Change the color according to the applied signal
    sim.setShapeColor(propeller,nil,0,{ang_vel/max_vel,0,1-ang_vel/max_vel})
end 
