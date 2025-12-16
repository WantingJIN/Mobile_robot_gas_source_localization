import numpy as np
import math 
import time

class Virtual_Rover:
	def __init__(self,initial_pose, parameter):
		self.state={}
		self.state['x']=initial_pose.get('x',0.)
		self.state['y']=initial_pose.get('y',0.)
		self.state['yaw']=initial_pose.get('yaw',0.)

		self.state['speed_left']=0.;
		self.state['speed_right']=0.;

		self.state['nominal_speed_left']=0.;
		self.state['nominal_speed_right']=0.;
		
		self.acc_motor=parameter.get('acc_motor',100.);
		self.diameter_wheel=parameter.get('diameter_wheel',0.05);
		self.dist_tracks=parameter.get('dist_tracks',0.16);
		self.P_factor=parameter.get('control_P',8.);
		
		self.last_time=time.time()
		self.last_yaw=0

	def get_pose(self):
		return [self.state['x'],self.state['y'],self.state['yaw']]

	def set_nominal_speed(self,left,right):
		self.state['nominal_speed_left']=left;
		self.state['nominal_speed_right']=right;

	def update(self,dt):
		#print self.state['yaw'],self.state['speed_left'],self.state['nominal_speed_left']
		self.integrate_acc(dt);
		self.integrate_vel(dt);
		

	def integrate_acc(self,dt):
		fl=min(self.acc_motor,self.P_factor*abs(self.state['nominal_speed_left']-self.state['speed_left']))
		fr=min(self.acc_motor,self.P_factor*abs(self.state['nominal_speed_right']-self.state['speed_right']))

		if self.state['nominal_speed_left']>self.state['speed_left']:
			self.state['speed_left']+=dt*fl
		if self.state['nominal_speed_left']<self.state['speed_left']:
			self.state['speed_left']-=dt*fl
		if self.state['nominal_speed_left']==0:
			self.state['speed_left']=0;		

		if self.state['nominal_speed_right']>self.state['speed_right']:
			self.state['speed_right']+=dt*fr
		if self.state['nominal_speed_right']<self.state['speed_right']:
			self.state['speed_right']-=dt*fr
		if self.state['nominal_speed_right']==0:
			self.state['speed_right']=0;
			

		#self.state['speed_left']=self.state['nominal_speed_left'];
		#self.state['speed_right']=self.state['nominal_speed_right'];

	def integrate_vel(self,dt):
		mean=0.5*(self.state['speed_left']+self.state['speed_right'])*self.diameter_wheel*math.pi/60
		dir_x=math.cos(self.state['yaw']);
		dir_y=math.sin(self.state['yaw']);
		self.state['x']+=mean*dt*dir_x;
		self.state['y']+=mean*dt*dir_y;
		
		dif=(-self.state['speed_left']+self.state['speed_right'])*self.diameter_wheel*math.pi/60
		self.state['yaw']+=dif/(self.dist_tracks)*dt;
		while(self.state['yaw']>math.pi):
			self.state['yaw']-=2.*math.pi;
		while(self.state['yaw']<-math.pi):
			self.state['yaw']+=2.*math.pi;
		
		
