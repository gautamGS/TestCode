#!/usr/bin/env python
import rospy ,rospkg
import math
import serial
from std_msgs.msg import Float64 ,String
import time
from lawn_mower.srv import *
import datetime
#from geometry_msgs.msg import Twist
#from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import numpy as np
import ast

rospack = rospkg.RosPack()



class control_compass_gps:
	
	def __init__(self):
		# function to initialize stuff
		self.rospack = rospkg.RosPack()
		
		#GPS related
		self.tgt_lat = -1
        self.tgt_long =-1
        self.curr_lat = -1
        self.curr_long = -1
		self.gpsInterupt = False
		self.calTargetDistance = False
		
		#waypoints / distance realted
		self.WAYPOINT_LIST = []
		self.distanceToTarget = -1
        self.originalDistanceToTarget= -1
		self.WAYPOINT_DIST_TOLERANE = 0.10
		self.WAYPOINT_INDEX = -1
		
		#compass related
		self.DEC_ANGLE = 0.2225
		self.HEADING_TOLERANCE = 2
		self.TWO_PI = 6.2831855
		self.targetHeading = -1
		self.currentHeading = -1
		self.headingError = None
		self.compasInterupt = False
		
		# subscriptions
		
		self.gps_sub = rospy.Subscriber('/fix', NavSatFix, self.getGPS)
		self.compas_sub =  rospy.Subscriber('/compass_angle', Float64, self.getCompass)
		
		# waiting for service to de activated
		print "control_compass_gps.py : Info :  " , datetime.datetime.now() , " waiting for service CONTROLLER_CMD to be available."
		rospy.wait_for_service('controller_cmd')
		
		try:
			self.controller_cmd= rospy.ServiceProxy('controller_cmd',controllerCMD)
			print "control_compass_gps.py : Info :  " , datetime.datetime.now() , " connected to service CONTROLLER_CMD."
		except as e :
			print "control_compass_gps.py : ERROR :  " , datetime.datetime.now() , " issue connecting to service CONTROLLER_CMD." , e
			
	# function to deal with service request 	
	def send_srv_control_cmd(self,cmd_srv):
		try:
			response_= self.controller_cmd(cmd_srv)
			return response_.cmd_response
		except rospy.ServiceException, e:
			print "control_compass_gps.py : ERROR :  " , datetime.datetime.now() , " issue sending or recieving data from service CONTROLLER_CMD %s" %e
	
	def getGPS(self,data):
        
		self.curr_lat = data.latitude
        self.curr_long = data.longitude
        self.gpsInterupt = True
		self.calCurrentDistance()
		print "control_compass_gps.py : INFO :  " , datetime.datetime.now() , " new current Distance Calculated " , self.distanceToTarget
		self.calTargetHeading()
		print "control_compass_gps.py : INFO :  " , datetime.datetime.now() , " new Target Heading Calculated " , self.targetHeading
		if self.calTargetDistance :
			self.calTargetDistance()
			print "control_compass_gps.py : INFO :  " , datetime.datetime.now() , " new Target Distance Calculated " , self.originalDistanceToTarget
			self.calTargetDistance= False
	
	def getCompass(self,data):
        
        self.currentHeading = data.data
        self.compasInterupt = True
	
	def calCurrentDistance(self):
	
		delta = math.radians(self.curr_long - self.tgt_long)
		sdlong = math.sin(delta)
        cdlong = math.cos(delta)
        lat1 = math.radians(self.curr_lat)
        lat2 = math.radians(self.tgt_lat)
        slat1 = math.sin(lat1)
        clat1 = math.cos(lat1)
        slat2 = math.sin(lat2)
        clat2 = math.cos(lat2)
        delta = (clat1 * slat2) - (slat1 * clat2 * cdlong)
        delta = math.pow(delta,2)
        delta += math.pow(clat2 * sdlong,2)
        delta = math.sqrt(delta) 
        denom = (slat1 * slat2) + (clat1 * clat2 * cdlong)
        delta = math.atan2(delta, denom)
		self.distanceToTarget = delta * 6372795     #In meters
	
	def calTargetDistance(self):
	
		delta = math.radians(self.curr_long - self.tgt_long)
		sdlong = math.sin(delta)
        cdlong = math.cos(delta)
        lat1 = math.radians(self.curr_lat)
        lat2 = math.radians(self.tgt_lat)
        slat1 = math.sin(lat1)
        clat1 = math.cos(lat1)
        slat2 = math.sin(lat2)
        clat2 = math.cos(lat2)
        delta = (clat1 * slat2) - (slat1 * clat2 * cdlong)
        delta = math.pow(delta,2)
        delta += math.pow(clat2 * sdlong,2)
        delta = math.sqrt(delta) 
        denom = (slat1 * slat2) + (clat1 * clat2 * cdlong)
        delta = math.atan2(delta, denom)
		self.originalDistanceToTarget = delta * 6372795     #In meters
	
	def calTargetHeading(self):
		
		dlon = math.radians(self.tgt_long-self.curr_long)
        cLat = math.radians(self.curr_lat)
        
        tLat = math.radians(self.tgt_lat)
        a1 = math.sin(dlon) * math.cos(tLat)
        a2 = math.sin(cLat) * math.cos(tLat) * math.cos(dlon)
        a2 = math.cos(cLat) * math.sin(tLat) - a2
        a2 = math.atan2(a1, a2)
        
        self.targetHeading = math.degrees(a2)
        if self.targetHeading < 0:
			self.targetHeading += 360
		
	
	def calHeadingError(self):
		if self.compasInterupt :
			self.headingError= self.targetHeading - self.currentHeading
			if self.headingError <= -180 :
				self.headingError +=360
			elif self.headingError > 180 :
				self.headingError -= 360
				
	
	def setWaypoints(self):
		abs_path = rospack.get_path("lawn_mower") + "/script/ServerRequestHandler/" + "saved_points_sim.txt"
        with open(abs_path,'r') as pts:
		data = pts.readlines() 
        
        for i in data:
            #x,y = i.split(';')
            co = ast.literal_eval(i)
            #self.WAYPOINT_LIST.append([float(x),float(y)])
            self.WAYPOINT_LIST.append([co[0],co[1]])
		print "control_compass_gps.py : INFO :  " , datetime.datetime.now() , " All Waypoints to Follow " , self.WAYPOINT_LIST
		
	def getNextWayPoint(self):
		
		self.WAYPOINT_INDEX = self.WAYPOINT_INDEX + 1
		if (self.WAYPOINT_INDEX >= len(self.WAYPOINT_LIST) ):
			print "control_compass_gps.py : INFO :  " , datetime.datetime.now() , " Last Waypoint Processed , No More Points to Process , Shutting Down Node " 
			rospy.signal_shutdown("Node Completed")
			return
		
		print "control_compass_gps.py : INFO :  " , datetime.datetime.now() , " Picking waypoint at Index " , self.WAYPOINT_INDEX , " with value as " ,self.WAYPOINT_LIST[self.WAYPOINT_INDEX]
		self.tgt_lat = self.WAYPOINT_LIST[self.WAYPOINT_INDEX][0]
		self.tgt_long= self.WAYPOINT_LIST[self.WAYPOINT_INDEX][1]
		
		self.calTargetDistance = True
		print "control_compass_gps.py : INFO :  " , datetime.datetime.now() , " Waiting for New GPS signal as new Waypoint is selected."
		while self.gpsInterupt :
			pass
		print "control_compass_gps.py : INFO :  " , datetime.datetime.now() , " Recieved New GPS signal as new Waypoint is selected."	
		
	
def pipeline():
	
	# declare object 
	print "pipeline : INFO :  " , datetime.datetime.now() , " Called for first time setting up data."	
	objControl = control_compass_gps()
	
	# set calTargetDistance as true for the first time to get target value set in variable
	objControl.calTargetDistance = True
	# check that GPS and Compass Reading have been recieved atleast once to begin with
	
	if objControl.compasInterupt and objControl.gpsInterupt :
		print "pipeline : INFO :  " , datetime.datetime.now() , " GPS and Compass Data ready ...."	
		
	else:
		if not objControl.compasInterupt : 
			# wait for compass angle data to come
			print "pipeline : INFO :  " , datetime.datetime.now() , " waiting for compass data to be recieved...."	
			while not objControl.compasInterupt:
				pass
			print "pipeline : INFO :  " , datetime.datetime.now() , " Recieved compass data...."		
		
		if not objControl.gpsInterupt : 
			# wait for gps data to come
			print "pipeline : INFO :  " , datetime.datetime.now() , " waiting for GPS data to be recieved...."	
			while not objControl.gpsInterupt:
				pass
			print "pipeline : INFO :  " , datetime.datetime.now() , " Recieved GPS data...."		
		

if __name__ == '__main__':
    try:
        # initialize ros node
        rospy.init_node('controllerGPSCompass')
		pipeline()
		rospy.spin()
	except as e:
		print "pipeline : ERROR :  " , datetime.datetime.now() , " Error while running pipeline...... %s" , %e		
	