from dronekit import connect, VehicleMode
from dronekit import LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import dronekit
import time
import math
import socket
import struct
import GPStolocal
import numpy as np
# import pacv2 as pac
# import pacvel
# import pacvely
import pacvelNew
#-----------------------------------
def set_attitude(roll_angle, pitch_angle, yaw_rate, thrust, duration):
    """
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version
    (sending the message multiple times does not cause problems).
    """
    
    """
    The roll and pitch rate cannot be controllbed with rate in radian in AC3.4.4 or earlier,
    so you must use quaternion to control the pitch and roll for those vehicles.
    """
    
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, # time_boot_ms
        1, # Target system
        1, # Target component
        0b00000000, # Type mask: bit 1 is LSB
        to_quaternion(roll_angle, pitch_angle), # Quaternion
        0, # Body roll rate in radian
        0, # Body pitch rate in radian
        math.radians(yaw_rate), # Body yaw rate in radian
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)

    start = time.time()
    while time.time() - start < duration:
        vehicle.send_mavlink(msg)
        time.sleep(0.1)
#----------------------------------------------
def to_quaternion(roll, pitch, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5
    print ("quat", w,x,y,z)

    return [w, x, y, z]
#--------------------------------------------
#----- Giving motor command---- a=1 for ARM, 0 to disarm
def motor_rotate(a):
    # create the MAV_CMD_COMPONENT_ARM_DISARM command
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, #command
        0, #confirmation
        a, 21196, 0, 0, #params 1-4
        0,
        0,
        0
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)
#------------------------------------------------------------------------------
#----- Servo control
def servo(a):
    # create the MAV_CMD_COMPONENT_ARM_DISARM command
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, #command
        0, #confirmation
        a, 1800, 0, 0, #params 1-4
        0,
        0,
        0
        )
    # send command to vehicle
    vehicle.send_mavlink(msg)
#------------------------------------------------------------------------------
#--------------------------------------------
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    ##### CONSTANTS #####
    DEFAULT_TAKEOFF_THRUST = 0.9
    SMOOTH_TAKEOFF_THRUST = 0.6

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    # If you need to disable the arming check,
    # just comment it with your own responsibility.
    #while not vehicle.is_armable:
        #print(" Waiting for vehicle to initialise...")
        #time.sleep(1)


    print("Arming motors")
    # Copter should arm in GUIDED_NOGPS mode
    #vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        vehicle.armed = True
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    thrust = DEFAULT_TAKEOFF_THRUST
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(" Altitude: %f  Desired: %f" %
              (current_altitude, aTargetAltitude))
        if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
            print("Reached target altitude")
            break
        elif current_altitude >= aTargetAltitude*0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude(0,0,0,thrust,0.5)
        time.sleep(0.2)
#----------------------------------------------

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, velocity_y, velocity_z, # XYZ velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    then = time.time()
    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        #while((time.time() - then) < dt):
         #   pass
        #then = time.time()
        time.sleep(0.1)
#----------------------------------------------------------

homelat=1.347661
homelon=103.688615
homealt=0.0

vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)
# vehicle = connect ('/dev/ttyUSB0', wait_ready=True,baud=921600)


# Copter should arm in GUIDED mode
vehicle.mode    = VehicleMode("STABILIZE") ## modify to ALTHOLD
vehicle.armed   = True

#vehicle.simple_takeoff(take_off_alt)

# Confirm vehicle armed before attempting to take off
while not vehicle.armed:
    print " Waiting for arming..."
    time.sleep(1)

# Take off
vehicle.mode = VehicleMode("GUIDED")
print "Taking off!"
take_off_alt = 3.0
yaw_bias =  -0.148798823357
vehicle.simple_takeoff(take_off_alt) # Take off to target altitude

flag_alt = False
while flag_alt==False:
   print " Global Altitude: %s" % vehicle.location.global_frame.alt    
   if (vehicle.location.global_relative_frame.alt > take_off_alt*0.95):         
        flag_alt = True
    
print "Reached target altitude"

time.sleep(5)

# Closed loop control initialize
vx_err = 0.0
vx_sumerr = 0.0
vx_filtered=0.0

vy_err=0.0
vy_filtered=0.0
vy_sumerr=0

k = 1
dt = 0.5
target=1

pac_vx    = pacvelNew.structures()
pac_vx.dt = dt
pac_vy    = pacvelNew.structures()
pac_vy.dt = dt

tim = []
runtime = 0

xpos_reflog = []
ypos_reflog = []
pos_x = []
pos_y = []
height = []

vx_reflog = []
vy_reflog = []
vx_uavlog = []
vy_uavlog = []

yawlog = []
yaw_reflog = []
rollLog = []
pitchLog = []

ucplog = []
ucrlog = []
rul_vx = []
rulwin_vx = []

rul_vy = []
rulwin_vy = []

plotgraph    = True
if plotgraph == True:
    import matplotlib.pyplot as plt
    plt.ion()
    plt.figure("Flightplot")

# vehicle.mode  = VehicleMode("LOITER")
vehicle.mode  = VehicleMode("GUIDED")
# vehicle.channels.overrides['3']=1500

while True:
        
        start = time.time()
        #print (" Global Latitude: %s" % vehicle.location.global_frame.lat)
        #print (" Global Longitude: %s" % vehicle.location.global_frame.lon)
        #print (" Global Altitude: %s" % vehicle.location.global_frame.alt)
        #print ("Local Location: %s" % vehicle.location.local_frame)    #NED
        #print ("Local Location NORTH: %s" % vehicle.location.local_frame.north)
        #print ("Local Location EAST: %s" % vehicle.location.local_frame.east)
        #print ("Local Location DOWN: %s" % vehicle.location.local_frame.down)
        #print ("Attitude: %s" % vehicle.attitude)
        #print ("pitch: %s" % vehicle.attitude.pitch)
        #print ("yaw: %s" % vehicle.attitude.yaw)
        #print ("roll: %s" % vehicle.attitude.roll)
        #print ("Velocity: %s" % vehicle.velocity)
#--------------------------------SAVING VEHICLE STATES INTO VARIABLES-------------
        phi1  = vehicle.location.global_frame.lat
        long1 = vehicle.location.global_frame.lon
        alt   = vehicle.location.global_relative_frame.alt
        local_x,local_y = GPStolocal.GPStolocalr(phi1,long1,homealt,homelat,homelon,homealt)
        roll  = vehicle.attitude.roll
        pitch = vehicle.attitude.pitch
        yaw   = vehicle.attitude.yaw
        vx_uav = vehicle.velocity[0]
        vy_uav = vehicle.velocity[1]
        vz_uav = vehicle.velocity[2]

        #Safety condition check
        if alt > 7:
            print("Altitude is overlimit!!")
            break
        if alt < 1.0:
            print("Altitude is underlimit")
            break

        print (" Ch1: %s" % vehicle.channels['1'])   #Roll
        print (" Ch2: %s" % vehicle.channels['2'])   #Pitch
        print (" Ch3: %s" % vehicle.channels['3'])   #Throttle
        print (" Ch4: %s" % vehicle.channels['4'])   #Yaw
        print (" Ch5: %s" % vehicle.channels['5'])

        # Position Target
        # xpos_ref=15.0
        # ypos_ref=-15.0

        # 1) square target
        # distance=5.0
        # if target==1 :
        #     xpos_ref=distance
        #     ypos_ref=0.0

        # elif target==2 :
        #    xpos_ref=distance
        #    ypos_ref=distance

        # elif target==3 :
        #    xpos_ref=0.0
        #    ypos_ref=distance

        # else:
        #     xpos_ref=0.0
        #     ypos_ref=0.0
        
        # 2) Circle target
        xpos_ref=0+6.0*math.cos(2*math.pi*k*dt/120)
        ypos_ref=0+6.0*math.sin(2*math.pi*k*dt/120)

        xpos_err= xpos_ref-local_x
        ypos_err= ypos_ref-local_y

        
        # Position target condition 
        sqroot_err=math.sqrt(xpos_err**2+ypos_err**2)
        # if sqroot_err < 1: 
        #     print "Position Target Achieved.."
        #     target+=1


        # PAC Velocity X
        vx_errlast = vx_err
        vx_err     = xpos_err
        vx_derr    = (vx_err-vx_errlast)/dt
        vx_lastfilter = vx_filtered
        vx_filtered  += ((-1*vx_lastfilter*0.5)+vx_derr)*dt
        vx_sumerr    += vx_err*dt
        
        pac_vx.process(vx_err,vx_filtered,vx_sumerr,local_x,xpos_ref)
        ucp  = 1*pac_vx.u

        # vx control action limiter
        if ucp <= -1.0:
		    ucp = -1.0

        if ucp >= 1.0:
		    ucp = 1.0

        # PAC Velocity Y
        vy_errlast = vy_err
        vy_err     = ypos_err
        vy_derr    = (vy_err-vy_errlast)/dt
        vy_lastfilter = vy_filtered
        vy_filtered  += ((-1*vy_lastfilter*0.5)+vy_derr)*dt
        vy_sumerr    += vy_err*dt
        
        pac_vy.process(vy_err,vy_filtered,vy_sumerr,local_y,ypos_ref)
        ucr  = 1*pac_vy.u
        
        # vy control action limiter
        if ucr <= -1.0:
            ucr = -1.0

        if ucr >= 1.0:
            ucr = 1.0

        # Wind disturbances
        # wind = -0.5+0.05*math.sin(2*math.pi*k*dt/1) # High
        # wind = -0.3+0.05*math.sin(2*math.pi*k*dt/1) # Medium
        wind = -0.1+0.05*math.sin(2*math.pi*k*dt/1) # Low

        # Send velocity control signal
        uav_vx_t_comm = ucp + wind
        uav_vy_t_comm = ucr + wind
        uav_vz_t_comm = 0.0
        send_ned_velocity(uav_vx_t_comm,uav_vy_t_comm,uav_vz_t_comm,1)

        # Overriding Channel
        # utp = 1500+(1*float(ucp))
        # utr = 1500+(1*float(ucr))
        # vehicle.channels.overrides['2']=utp ##change to pitch channel      
        # vehicle.channels.overrides['1']=utr ##change to roll channel
        
        runtime = k*dt
        # Print all states
        print ("Error Pos:", xpos_err, ypos_err)
        print ("Reference Pos:", xpos_ref, ypos_ref)
        print ("Current Pos:", local_x, local_y)
        # print ("Vx ref:",vx_ref)
        print ("Vx:", vx_uav)
        # print ("Vy ref:",vy_ref)
        print ("Vy:", vy_uav)
        print ("Altitude:", alt)
        print ("Roll angle:", roll)
        print ("Pitch angle:", pitch)
        # print ("Yaw ref:",yaw_ref)
        print ("Yaw angle:", yaw)
        # print ("Control signal Roll, Pitch", utr,utp)
        print ("Running time", runtime)
        # time.sleep(0.2)
        
        #safe required data      
        tim.append(runtime)
        
        xpos_reflog.append(xpos_ref)
        ypos_reflog.append(ypos_ref)
        pos_x.append(local_x)
        pos_y.append(local_y)
        height.append(alt)

        #vx_reflog.append(vx_ref)
        #vy_reflog.append(vy_ref)
        vx_uavlog.append(vx_uav)
        vy_uavlog.append(vy_uav)

        # vx_reflog.append(v_thetaref)
        # vy_reflog.append(v_phiref)
        # vx_uavlog.append(v_theta)
        # vy_uavlog.append(v_phi)

        yawlog.append(yaw)
        # yaw_reflog.append(yaw_ref)

        rollLog.append(roll)
        pitchLog.append(pitch)
        
        ucplog.append(ucp)
        ucrlog.append(ucr)
        rul_vx.append(pac_vx.R)
        rulwin_vx.append(pac_vx.Rwin)

        rul_vy.append(pac_vy.R)
        rulwin_vy.append(pac_vy.Rwin)

        if plotgraph==True:
            plt.subplot2grid((5,3),(0,2), rowspan=2)
            plt.plot(tim,ucplog)
            plt.title('velocity X PAC Dronekit')
            plt.grid(True)
            plt.xlabel ("time(s)")
            plt.ylabel('velocity(m/s)')

            plt.subplot2grid((5,3),(0,0), colspan=2, rowspan=5)
            plt.plot(pos_y,pos_x)
            plt.title('Position PAC Dronekit')
            plt.grid(True)
            plt.xlabel ("East(m)")
            plt.ylabel("North(m)")

            plt.subplot2grid((5,3),(3,2), rowspan=2)
            plt.plot(tim,pos_x,tim,xpos_reflog)
            # plt.plot()
            plt.title('velocity Y PAC Dronekit')
            plt.grid(True)
            plt.xlabel ("time(s)")
            plt.ylabel('velocity(m/s)')

            plt.show()
            plt.pause(0.0001)

        duration = time.time()-start
        print("iteration time=",duration)
        print ("-----------------------------------------------------------------------")
        
        #Break condition
        if runtime>280:
            print("Control task timeout achieved")
            break
        # dt = time.time() - start
        k += 1

# Autoland	
print("Now let's land")
# vehicle.channels.overrides['2']=1500
# vehicle.channels.overrides['1']=1500


with open("SMCCircle6m_withWindLow.txt", "w") as output:
    output.writelines(map("{};{};{};{};{};{};{};{};{};{};{};{};{};{}\n".format, tim, xpos_reflog, pos_x, ypos_reflog, pos_y, vx_uavlog, vy_uavlog, rul_vx, rul_vy, ucplog, ucrlog, rollLog, pitchLog, yawlog))

if plotgraph==True:
    plt.figure('Flightplot').savefig('PAC_flight0.png')    

vehicle.mode = VehicleMode("LAND")
time.sleep(3)

print("Close vehicle object")
vehicle.close()


