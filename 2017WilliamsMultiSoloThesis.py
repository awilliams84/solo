__author__ = "Alexander Williams"
__email__ = "alexander.williams@gmail.com"
__status__ = "Production" 

'''
This code was developed as part of the thesis:

Naval Postgraduate School
FEASIBILITY OF AN EXTENDED-DURATION AERIAL PLATFORM USING AUTONOMOUS 
MULTI-ROTOR VEHICLE SWAPPING AND BATTERY MANAGEMENT

By LCDR Alexander Williams

This code was written and tested in 2017 at Camp Roberts. All  
code has parameters set for testing at a specific site. If you  
intend to use this code, ensure you change the parameters,  
especially the latitude and longitude.  

'''

# Import necessary libraries
from dronekit import connect, VehicleMode
from dronekit import LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import signal
import sys
from datetime import datetime, timedelta
from collections import defaultdict
import csv

from mpl_toolkits.mplot3d import Axes3D

# Initialize global variables
sortiedata=defaultdict(list)
solos=defaultdict(list)
sortie=0

#####################################################################
####################### ADJUSTABLE VARIABLES ########################
#####################################################################

# Define all Solo ports and names (ports and names)
# If you add vehicles, this is where to add their ports and assign names for
# inclusion
soloports=['udpin:0.0.0.0:15550','udpin:0.0.0.0:16550','udpin:0.0.0.0:17550','udpin:0.0.0.0:14550']
soloids=['redleader','blueleader','goldleader','greenleader']

# Static test parameters

# Min voltage we want to see from any vehicle
battery_volt_limit = 10	
# Min battery level we want to see from any vehicle
battery_level_critical_limit = 30

# Test location - modify for future application not at Camp Roberts
loiterlat = 35.7167982
loiterlon = -120.7625160
loiteralt = 100 #meters
# loiterlat = 35.716014
# loiterlon = -120.763119
# loiteralt = 30 #meters

# Adjustable variables
takeoffalt = 15				# Altitude vehicles will go to on takeoff
vehiclealtseparation = 5	# Seperation distance (in meters) at swap

#####################################################################
#################### END ADJUSTABLE VARIABLES #######################
#####################################################################

# Initialize time for data collection
# DELETE
# starttime = datetime.now() 


# Function for arming and takeoff to set altitude
def arm_and_takeoff(vehicle,aTargetAltitude,vehiclename):

    print "Basic pre-arm checks"
    # Do not try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

        
    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off! Heading to ",aTargetAltitude

    takeofftime = datetime.now()  
    landedalt=vehicle.location.global_relative_frame.alt
    global sortie
    sortie+=1

    # For data logging
    storesortiedata(vehiclename,vehicle)    	    

    # Wait until the vehicle reaches a safe height before 
    # processing the goto (otherwise the command 
    # after Vehicle.simple_takeoff will execute immediately).
    while True:
        vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
        timeelapsed=((datetime.now()-takeofftime).total_seconds())
        print " Altitude: ", vehicle.location.global_relative_frame.alt, " Time Elapsed: ",timeelapsed, " sec"      
		#Trigger just below target alt
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print "Reached target altitude in ",timeelapsed," sec"
            status="success"
            break
        if timeelapsed>3 and vehicle.location.global_relative_frame.alt<landedalt+1:
        	print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Launch ERROR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
        	print "Land vehicle (%s)" %(vehiclename)
        	vehicle.mode = VehicleMode("LAND")
        	status="fail"
        	break
        time.sleep(1)

    # For data logging
    storesortiedata(vehiclename,vehicle)    	

    return status


# Function to calculate the distance in meters between two position
def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over 
    large distances and close to the earth's poles. It comes from the 
    ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


# Function to store data during testing
def storesortiedata(vehiclename,vehicle):
	timenow = datetime.now()
	global sortie

	sortiename=vehiclename
	sortiedata[sortiename].append( 
		(timenow,
			vehicle.location.global_relative_frame.lat,
			vehicle.location.global_relative_frame.lon,
			vehicle.location.global_relative_frame.alt,
			vehicle.battery.level,
			vehicle.battery.voltage,
			sortiename
			))
	
	print "Stored data for %s in dictionary"%(vehiclename)


# Function to write data to csv file and plot data
def showplot():
	if len(sortiedata)>0:
		fig=plt.figure()
		fig.suptitle('Battery Management Testing')
		plt1=fig.add_subplot(211)
		plt2=fig.add_subplot(212)
		fig2=plt.figure()
		plt3=fig2.add_subplot(111,projection='3d')

		# Open csv file and append
		f = open('data.csv','a')

		fulltime=[]
		fulllatitude=[]
		fulllongitude=[]
		fullaltitude=[]
		fullbatterypct=[]
		fullbatteryvolt=[]
		fullsolo=[]

		global starttime

		for key, value in sorted(sortiedata.iteritems()):
			soloid=key
			time=[]
			latitude=[]
			longitude=[]
			altitude=[]
			batterypct=[]
			batteryvolt=[]
			solo=[]

			for var in value:
				time.append((var[0]-starttime).total_seconds())
				latitude.append(var[1])
				longitude.append(var[2])
				altitude.append(var[3])
				batterypct.append(var[4])
				batteryvolt.append(var[5])
				solo.append(var[6])

			plt1.plot(time,altitude,label=soloid)
			plt1.set_xlabel('Time (sec)')
			plt1.set_ylabel('Altitude (m)')
			plt2.plot(time,batterypct,label=soloid)
			plt2.set_xlabel('Time (sec)')
			plt2.set_ylabel('Battery Life (%)')
			plt3.plot(latitude,longitude,altitude)
			plt3.set_xlabel('Latitude (m)')
			plt3.set_xlabel('Longitude (m)')
			plt3.set_xlabel('Altitude (m)')

			plt1.scatter(time[0],altitude[0],marker='o',color='g',s=50)
			plt1.scatter(time[-1],altitude[-1],marker='o',color='r',s=50)
			plt2.scatter(time[0],batterypct[0],marker='o',color='g',s=50)
			plt2.scatter(time[-1],batterypct[-1],marker='o',color='r',s=50)

			fulltime.extend(time)
			fulllatitude.extend(latitude)
			fulllongitude.extend(longitude)
			fullaltitude.extend(altitude)
			fullbatterypct.extend(batterypct)
			fullbatteryvolt.extend(batteryvolt)
			fullsolo.extend(solo)

		duration=math.ceil(fulltime[-1]/60)
		start=starttime.strftime('%Y%m%d %H%M')
		stoptime=var[0]
		stop=stoptime.strftime('%Y%m%d %H%M')
		flightdata=['Start: '+start,'Stop: '+stop,'Duration: '+`duration`+ ' minutes','Sortie Count: '+`len(sortiedata)`]

		# Add row titles
		fulltime.insert(0,'Time (sec)')
		fullsolo.insert(0,'Vehicle ID')
		fulllatitude.insert(0,'Latitude')
		fulllongitude.insert(0,'Longitude')
		fullaltitude.insert(0,'Altitude (m)')
		fullbatterypct.insert(0,'Battery (%)')
		fullbatteryvolt.insert(0,'Battery (V)')

		# Write to csv file			
		w = csv.writer(f, delimiter=',')
		w.writerow(flightdata)
		w.writerow(fullsolo)
		w.writerow(fulltime)
		w.writerow(fulllatitude)	
		w.writerow(fulllongitude)	
		w.writerow(fullaltitude)	
		w.writerow(fullbatterypct)
		w.writerow(fullbatteryvolt)

		print "############# Data stored to CSV #############"

		# Close csv file
		f.close()	

		plt1.autoscale(enable=True, axis='both', tight=False)
		plt2.autoscale(enable=True, axis='both', tight=False)



# Initialize lists of available Solos (connect and names) set in 
# connection (set empty here)
solo=[]
soloname=[]

print "======================== CONNECTING TO SOLOS ========================"

i=0

# Loop through solo ports to attempt connection with each vehicle
for soloport in soloports:

	index=soloports.index(soloport)
	soloid=soloids[index]

	print("Connecting to solo: %s (%s)" % (soloport,soloid))

	# Try to connect to the solo for XX heartbeat_timeout seconds
	try:
		vehicle=connect(soloport, wait_ready=True, heartbeat_timeout=90)
		solo.append(vehicle)
		soloname.append(soloid)

		solos[soloid]=vehicle

		# Print some vehicle attributes
		print "Get some vehicle attribute values:"
		print " GPS: %s" % solo[i].gps_0
		print " Battery: %s" % solo[i].battery
		print " Last Heartbeat: %s" % solo[i].last_heartbeat
		print " Is Armable?: %s" % solo[i].is_armable
		print " System status: %s" % solo[i].system_status.state
		print " Mode: %s" % solo[i].mode.name

		# Increment 1 to look on the next solo port
		i+=1

	except:
		print (soloid+" vehicle not found.")


print "////////////////////// END CONNECTING TO SOLOS //////////////////////"


# Write which solos were found
print "%s Solos available: " %len(solo)
print soloname

# A check to see if the script should continue running in loop to follow
continuecheck = 1

# Ask user whether to continue where 'y' and 'n' are only acceptable answers
while continuecheck != 'y' and continuecheck != 'n':
	continuecheck = raw_input('Continue? (y/n):')

# Create new dictionary variables for home lat and longs
homelat=defaultdict(list)
homelon=defaultdict(list)

vehicle=0
sortienum=-1
unavailablevehicles=0
takeoffstatus="true"



# Main script for flight operations
try:

	vehicleindex=0
	vehicleindex2=1

	# Initialize time for data collection
	starttime = datetime.now()

	# Loop through vehicles
	while True and continuecheck == 'y':

		for thisvehicle in solo:

			if vehicleindex>len(solo)-1:
				vehicleindex=0

			print "vehicleindex=%s" %(vehicleindex)
			print "vehicleindex2=%s" %(vehicleindex2)

			if vehicle==solo[vehicleindex]:
				if takeoffstatus=="fail":
					print "Sleep to standby for another attempt. Same vehicle."
					time.sleep(15)
				else:
					print 'Vehicle Check: %s already flying, cannot use same vehicle to start while airborne' %(soloname[vehicleindex])
					continuecheck='n'
					break
			if solo[vehicleindex].battery.voltage < battery_volt_limit or solo[vehicleindex].battery.level < battery_level_critical_limit:
				print 'Vehicle Check: %s battery too low for operation, skip to next' %(soloname[vehicleindex])
				vehicleindex+=1
				unavailablevehicles+=1
				break
			elif unavailablevehicles>len(solo):
				print 'Vehicle Check: No vehicles available.'
				continuecheck='n'
				break

			vehicle=solo[vehicleindex]
			sortienum+=1

			# Dynamic test parameters
			battery_level_limit=0 # Set higher for testing

			print("Connected to solo: %s" % (soloname[vehicleindex]))

			# If connection is not current, skip vehicle and go to next
			if vehicle.last_heartbeat>10:
				vehicleindex+=1
				break

			# If the next vehicle does not have enough battery
			if vehicle.battery.level < battery_level_critical_limit:
				print " %s battery too low for designated flight." %(soloname[vehicleindex])

			# Else, continue
			else:

				# If vehicle is not already airborne (likely same vehicle)
				if vehicle.location.global_relative_frame.alt < takeoffalt-1:	

					try:						
						cmds = vehicle.commands
						cmds.download()
						cmds.wait_ready()

						# Arm and take off - receive status
						takeoffstatus=arm_and_takeoff(vehicle,takeoffalt,soloname[vehicleindex])
					
						# If launch fail
						if takeoffstatus=="fail":
							print "Launch failed, vehicleindex=%s" %(vehicleindex)
							vehicleindex+=1
							print "Find next vehicle, vehicleindex=%s" %(vehicleindex)
							break

					except:
						if takeoffstatus=="fail":
							print "Take off aborted, vehicleindex=%s" %(vehicleindex)
						vehicleindex+=1
						print "Find next vehicle, new vehicleindex=%s" %(vehicleindex)
						break

					# Increasing vehicle airspeed to max (30)
					print "Set vehicle airspeed to 30"
					vehicle.airspeed = 30

					# Set home location as the location above takeoff
					# (for return later)
					homelat[vehicleindex]=(vehicle.location.global_frame.lat)
					homelon[vehicleindex]=(vehicle.location.global_frame.lon)

					# Print for debug
					print " Home Latitude: %s" % homelat[vehicleindex]
					print " Home Longitude: %s" % homelon[vehicleindex]

				else:
					print "Vehicle (%s) already airborne" %(soloname[vehicleindex])

				# Mission location
				loiterpoint = LocationGlobalRelative(loiterlat,loiterlon, loiteralt)
				vehicle.simple_goto(loiterpoint)


				print "*****************Go to loiter point*****************"

				storesortiedata(soloname[vehicleindex],vehicle)    	

				# Loop to track distance to loiter. Print distance to go until
				# at loiter
				while True: 
					currentaltitude=vehicle.location.global_relative_frame.alt

					dist = get_distance_metres(vehicle.location.global_frame, loiterpoint)
					print " Distance to go: %s Altitude goal: %s Current: %s"%(dist,loiteralt,currentaltitude)

					if currentaltitude>loiteralt:
						goalpct=1.02
					else: 
						goalpct=0.98

					if dist < 1 and currentaltitude>=(loiteralt)*0.98:
						print 'Within 1m of waypoint'
						break

					time.sleep(1)

				storesortiedata(soloname[vehicleindex],vehicle)    	


				print "*******************Maintain loiter point*******************"

				# Maintain loiter position until battery level too low
				while True:

					try:
						print ' %s Battery limit: %s  Current battery: %s'%(soloname[vehicleindex],max(battery_level_limit,battery_level_critical_limit),vehicle.battery.level)
						time.sleep(1)

						# If battery level below limit or voltage level below limit, break loop
						if vehicle.battery.voltage < battery_volt_limit or vehicle.battery.level < battery_level_limit or vehicle.battery.level < battery_level_critical_limit:
							print 'Battery below limit'					
							break	

					except:
						# When battery loop is manually exited (CTRL+C)
						print " BATTERY CHECK ABORTED"
						break

				storesortiedata(soloname[vehicleindex],vehicle)    	

				if vehicleindex==len(solo)-1:
					vehicle2=solo[0]
					vehicleindex2=0
				else: 					
					vehicle2=solo[vehicleindex+1]
					vehicleindex2=vehicleindex+1

				# Look for replacement if more than one solo online
				if len(solo) > 1:
					print 'Checking for replacement'

					vehiclecheck=0
					while vehiclecheck <= len(solo) and (vehicle2.battery.level < battery_level_critical_limit or vehicle==vehicle2 or vehicle2.last_heartbeat>5):
						if vehicleindex==len(solo)-1:
							vehicle2=solo[0]
							vehicleindex2=0
						else: 					
							vehicle2=solo[vehicleindex+1]
							vehicleindex2=vehicleindex+1
						vehiclecheck+=1
						print "Vehicle check count: %s"%(vehiclecheck)

					# Check if replacements are able to replace (battery checks)
					if vehiclecheck > len(solo) or vehicle2.battery.level < battery_level_critical_limit:
						replacementfails=vehiclecheck
						# break
					else:
						replacementfails=0

					# Reset takeoffstatus before trying another takeoff
					takeoffstatus="fail"

					# Start takeoff sequence until a replacement is launched or all fail
					while takeoffstatus=="fail" and replacementfails<len(solo):
						if vehicleindex2==vehicleindex:
							vehicleindex2+=1
						if vehicleindex2>len(solo)-1:
							vehicleindex2=0
						if solo[vehicleindex2].last_heartbeat<5:
							print 'Launch replacement'
							print 'Launch vehicleindex2=%s' %(vehicleindex2)
							try:
								takeoffstatus=arm_and_takeoff(solo[vehicleindex2],takeoffalt,soloname[vehicleindex2])
							except:
								takeoffstatus="fail"

						nextvehicle=vehicleindex2+1
						if nextvehicle>len(solo):
							nextvehicle=0

						if nextvehicle==vehicleindex2 or nextvehicle==vehicleindex:
							time.sleep(15)

						if takeoffstatus=="fail":
							print 'Launch replacement failed'
							vehicleindex2+=1
							replacementfails+=1
							# break

					vehicle2=solo[vehicleindex2]

					if replacementfails>len(solo):
						print "Replacement Launch Failed %s times, quitting" %(replacementfails)
						continuecheck='n'
					else:

						print "Set vehicle airspeed to 30"
						vehicle2.airspeed = 30

						storesortiedata(soloname[vehicleindex],vehicle)
						storesortiedata(soloname[vehicleindex2],vehicle2)

						if vehicleindex2>len(homelat)-1:
							print "Adding home point for %s" %(soloname[vehicleindex2])

							cmds = vehicle2.commands
							cmds.download()
							cmds.wait_ready()

							homelat[vehicleindex2]=(vehicle2.location.global_frame.lat)
							homelon[vehicleindex2]=(vehicle2.location.global_frame.lon)

							print 'Home: (%s,%s)' %(homelat[vehicleindex2],homelon[vehicleindex2])

						else:
							print "Home already exists, ignoring!"

					print "**************Vehicle: %s airborne, takeoffstatus=%s"%(soloname[vehicleindex2],takeoffstatus)

					# Raise original vehicle
					print 'Raising vehicle from %s to %s' %(vehicle.location.global_relative_frame.alt,loiteralt+vehiclealtseparation)
					highloiterpoint = LocationGlobalRelative(loiterlat,loiterlon, loiteralt+vehiclealtseparation)
					vehicle.simple_goto(highloiterpoint)

					# Loop to track raised altitude
					while True and takeoffstatus!="fail":	
						currentaltitude=vehicle.location.global_relative_frame.alt

						print ' Altitude goal: %s Current: %s' %(loiteralt+vehiclealtseparation,currentaltitude)
						if currentaltitude>loiteralt+vehiclealtseparation:
							goalpct=1.02
						else: 
							goalpct=0.98
						if currentaltitude>=(loiteralt+vehiclealtseparation)*goalpct:
							break

						time.sleep(1)

					# Changing to guided mode in case vehicle coming online is not already
					print 'Change mode to guided'
					vehicle.mode = VehicleMode("GUIDED")

					# If repacement launched, send to loiter point
					if len(solo) > 1 and takeoffstatus!="fail":

						print 'Send replacement'

						storesortiedata(soloname[vehicleindex],vehicle)
						storesortiedata(soloname[vehicleindex2],vehicle2)

						while True: 
							vehicle2.simple_goto(loiterpoint)

							currentaltitude=vehicle2.location.global_relative_frame.alt

							dist = get_distance_metres(vehicle2.location.global_frame, loiterpoint)
							print " Distance to loiter pt: %s Altitude goal: %s Current: %s"%(dist,loiteralt,currentaltitude)

							if currentaltitude>loiteralt:
								goalpct=1.02
							else: 
								goalpct=0.98

							if dist < 1 and currentaltitude>=(loiteralt)*goalpct:
								print 'Within 1m of waypoint'
								break

							time.sleep(1)


				print 'Check altitude: %s' %(vehicle.location.global_relative_frame.alt)
				print 'Take off alt minus 2: %s' %(takeoffalt-2)

			# If original vehicle still airborn (should be), return to launch position
			# this check is conducted using vehicle's altitude. If loiter position is lower
			# than takeoff alt, this will need to be rewritten
			if vehicle.location.global_relative_frame.alt > takeoffalt-2:	

				# Return to launch point
				print 'Vehicle index=%s' %(vehicleindex)
				print 'Returning to launch position: (%s,%s)' %(homelat[vehicleindex],homelon[vehicleindex])

				homepoint = LocationGlobalRelative(homelat[vehicleindex],homelon[vehicleindex], takeoffalt+vehiclealtseparation)
				vehicle.simple_goto(homepoint)

				print " Go to %s" %(homepoint)

				storesortiedata(soloname[vehicleindex],vehicle)
				storesortiedata(soloname[vehicleindex2],vehicle2)			

				while True: 
					currentaltitude=vehicle.location.global_relative_frame.alt

					dist = get_distance_metres(vehicle.location.global_frame, homepoint)
					print " Distance to home: %s Altitude goal: %s Current: %s"%(dist,takeoffalt+vehiclealtseparation,currentaltitude)

					if currentaltitude>loiteralt+vehiclealtseparation:
						goalpct=1.02
					else: 
						goalpct=0.98

					if dist < 1 and currentaltitude<=(loiteralt+vehiclealtseparation)*goalpct:
						print 'Within 1m of waypoint'
						break

					time.sleep(1)

				# Land original vehicle
				print ' Landing'
				vehicle.mode = VehicleMode("LAND")

				storesortiedata(soloname[vehicleindex],vehicle)
				storesortiedata(soloname[vehicleindex2],vehicle2)			

				# Loop to track landing. Was used for data storing
				while True:
					print " Landing %s Altitude: %s" %(soloname[vehicleindex],vehicle.location.global_relative_frame.alt)      

					if vehicle.location.global_relative_frame.alt<=2: #Trigger just below target alt.
						print "Vehicle On Deck"
						break
					time.sleep(1)

				storesortiedata(soloname[vehicleindex],vehicle)

			# showplot()
			vehicleindex+=1

		if unavailablevehicles >= len(solo):
			break


# If interrupt
except:

	print "\n\n\n**************************Exited script early.****************************"

	vehicleindex=0
	for vehicle in solo:
		print "%s control released. Returned to LOITER mode"%(soloname[vehicleindex])
		vehicle.mode = VehicleMode("LOITER")
		vehicleindex+=1

	showplot()
	plt.show(block=True)
	sys.exit(0)


# If script errors out
if continuecheck=='y' and len(solo)==0:
	print 'No solo present for operation. Exited script.'
elif continuecheck=='n':
	print 'Exited script.'	
else:
	print "\n\n\n*************************Errored out of script early.****************************"

	vehicleindex=0
	for vehicle in solo:
		print "%s control released. Returned to LOITER mode"%(soloname[vehicleindex])
		vehicle.mode = VehicleMode("LOITER")
		vehicleindex+=1

if len(solo)!=0:
	showplot()
	plt.show(block=True)
	sys.exit(0)
