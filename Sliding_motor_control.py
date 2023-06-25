#!/usr/bin/env python3
from math import pi, sqrt, atan2, cos, sin, asin, acos
from turtle import position
import numpy as np
from numpy import NaN
import rospy
import tf
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Twist, Pose2D
import pickle
import os

class Quadrotor():
	def __init__(self):
		# publisher for rotor speeds
		self.motor_speed_pub = rospy.Publisher("/crazyflie2/command/motor_speed", Actuators, queue_size=10)
		# subscribe to Odometry topic
		self.odom_sub = rospy.Subscriber("/crazyflie2/ground_truth/odometry",Odometry, self.odom_callback)
		self.t0 = None
		self.t = None
		self.t_series = []
		self.x_series = []
		self.y_series = []
		self.z_series = []
		self.mutex_lock_on = False
		rospy.on_shutdown(self.save_data)

		self.m = 27*1e-3 
		self.l = 46*1e-3 
		self.Ix = 16.571710*1e-6
		self.Iy = 16.571710*1e-6
		self.Iz = 29.261652*1e-6
		self.Ip = 12.65625*1e-8
		self.kF = 1.28192*1e-8
		self.kM = 5.964552*1e-3
		self.g = 9.81
		self.omega=0

		# 6 positions at the end of each time interval, and initial and final velocities and accelerations
		p0 = np.array([0,0,0])
		p1 = np.array([0,0,1])
		p2 = np.array([1,0,1])
		p3 = np.array([1,1,1])
		p4 = np.array([0,1,1])
		p5 = np.array([0,0,1])
		vi = np.array([0,0,0])
		vf = np.array([0,0,0])
		ai = np.array([0,0,0])
		af = np.array([0,0,0])

		self.c1=self.quintic(p0,p1,vi,vf,ai,af,0,5);
		self.c2=self.quintic(p1,p2,vi,vf,ai,af,5,20);
		self.c3=self.quintic(p2,p3,vi,vf,ai,af,20,35);
		self.c4=self.quintic(p3,p4,vi,vf,ai,af,35,50);
		self.c5=self.quintic(p4,p5,vi,vf,ai,af,50,65);

	def quintic(self, pi, pf, vi, vf, ai, af, ti, tf):

		A =np.array([[1, ti,   ti**2,   ti**3,    ti**4,     ti**5],
					[1, tf,   tf**2,   tf**3,    tf**4,     tf**5 ],
					[0, 1 ,   2*ti,   3*ti**2,  4*ti**3,   5*ti**4],
					[0, 1 ,   2*tf,   3*tf**2,  4*tf**3,   5*tf**4],
					[0, 0 ,   2   ,   6*ti  ,  12*ti**2,  20*ti**3],
					[0, 0 ,   2   ,   6*tf  ,  12*tf**2,  20*tf**3]])

		b = np.stack([pi,pf,vi,vf,ai,af])

		traj = np.linalg.inv(A).dot(b)
		return traj


	def traj_evaluate(self):

		if(self.t<=5):
			c=self.c1
		elif(self.t<=20):
			c=self.c2
		elif(self.t<=35):
			c=self.c3
		elif(self.t<=50):
			c=self.c4
		elif(self.t<=65):
			c=self.c5
		else:
			self.t=65
			c=self.c5


		traj_time = np.array([[1, self.t,   self.t**2,   self.t**3,    self.t**4,     self.t**5],
							 [0, 1 ,   2*self.t,   3*self.t**2,  4*self.t**3,   5*self.t**4],
							 [0, 0 ,   2   ,   6*self.t  ,  12*self.t**2,  20*self.t**3]])

		a = traj_time.dot(c)
		pos = a[0,:]
		vel = a[1,:]
		acc = a[2,:]

		return pos,vel,acc

	def smc_control(self, xyz, xyz_dot, rpy, rpy_dot):
		# obtain the desired values by evaluating the corresponding trajectories
		
		pos,vel,acc=self.traj_evaluate()
		bdry =1

		## z tuning
		lambda_z=10
		K_z=5

		e=xyz[2]-pos[2]
		e_dot=xyz_dot[2]-vel[2]
		s_z=e_dot + lambda_z*e
		sat_z=min(max(s_z/bdry, -1), 1)

		u1=((self.m*(self.g + acc[2] - lambda_z*e_dot - K_z*sat_z))/(cos(rpy[0])*cos(rpy[1]))) 

		K_p=100
		K_d=5

		Fx=self.m*(-K_p*(xyz[0]-pos[0])-K_d*(xyz_dot[0]-vel[0])+acc[0])
		Fy=self.m*(-K_p*(xyz[1]-pos[1])-K_d*(xyz_dot[1]-vel[1])+acc[1])
		theta_d=asin(Fx/u1)
		phi_d=asin(-Fy/u1)
		
		## phi tuning
		lambda_phi=16
		K_phi=141

		e=rpy[0]-phi_d
		e=(e + np.pi) % (2 * np.pi) - np.pi

		s_phi=rpy_dot[0] + lambda_phi*e
		sat_phi=min(max(s_phi/bdry, -1), 1)

		u2= -rpy_dot[1]*rpy_dot[2]*(self.Iy-self.Iz) + self.Ip*self.omega*rpy_dot[1] - self.Ix*lambda_phi*rpy_dot[0] - self.Ix*K_phi*sat_phi

		# theta tuning
		lambda_theta=20
		K_theta=109

		e=rpy[1]-theta_d
		e=(e + np.pi) % (2 * np.pi) - np.pi
		s_theta=rpy_dot[1] + lambda_theta*e
		sat_theta=min(max(s_theta/bdry, -1), 1)

		u3= -rpy_dot[0]*rpy_dot[2]*(self.Iz-self.Ix) - self.Ip*self.omega*rpy_dot[0] - self.Iy*lambda_theta*rpy_dot[1] - self.Iy*K_theta*sat_theta

		# psi tuning
		lambda_psi=5
		K_psi=25
		e=(rpy[2] + np.pi) % (2 * np.pi) - np.pi
		s_psi=rpy_dot[2] + lambda_psi*e
		sat_psi=min(max(s_psi/bdry, -1), 1)

		u4= -rpy_dot[0]*rpy_dot[1]*(self.Ix-self.Iy) - self.Iz*lambda_psi*rpy_dot[2] - self.Iz*K_psi*sat_psi

		amat=np.array([[1/(4*self.kF),-sqrt(2)/(4*self.kF*self.l),-sqrt(2)/(4*self.kF*self.l),-1/(4*self.kM*self.kF)],
			  		   [1/(4*self.kF),-sqrt(2)/(4*self.kF*self.l),sqrt(2)/(4*self.kF*self.l),1/(4*self.kM*self.kF)],
					   [1/(4*self.kF),sqrt(2)/(4*self.kF*self.l),sqrt(2)/(4*self.kF*self.l),-1/(4*self.kM*self.kF)],
					   [1/(4*self.kF),sqrt(2)/(4*self.kF*self.l),-sqrt(2)/(4*self.kF*self.l),1/(4*self.kM*self.kF)]])

		u=np.squeeze(np.array([[u1],[u2],[u3],[u4]]), axis=2)
		
		motor_v =np.sqrt(np.matmul(amat,u))
		motor_v = np.clip(motor_v, a_min=0, a_max=2618.0)
		
		motor_speed = Actuators()
		motor_speed.angular_velocities = [motor_v[0,0], motor_v[1,0],motor_v[2,0], motor_v[3,0]]
		self.omega=motor_v[0,0] - motor_v[1,0] + motor_v[2,0] - motor_v[3,0]
		self.motor_speed_pub.publish(motor_speed)


	# odometry callback function (DO NOT MODIFY)
	def odom_callback(self, msg):
		if self.t0 == None:
			self.t0 = msg.header.stamp.to_sec()
		self.t = msg.header.stamp.to_sec() - self.t0

		# convert odometry data to xyz, xyz_dot, rpy, and rpy_dot
		w_b = np.asarray([[msg.twist.twist.angular.x], [msg.twist.twist.angular.y], [msg.twist.twist.angular.z]])
		v_b = np.asarray([[msg.twist.twist.linear.x], [msg.twist.twist.linear.y], [msg.twist.twist.linear.z]])
		xyz = np.asarray([[msg.pose.pose.position.x], [msg.pose.pose.position.y], [msg.pose.pose.position.z]])
		q = msg.pose.pose.orientation
		T = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
		T[0:3, 3] = xyz[0:3, 0]
		R = T[0:3, 0:3]
		xyz_dot = np.dot(R, v_b)
		rpy = tf.transformations.euler_from_matrix(R, 'sxyz')
		rpy_dot = np.dot(np.asarray([[1, np.sin(rpy[0])*np.tan(rpy[1]), np.cos(rpy[0])*np.tan(rpy[1])],
									 [0, np.cos(rpy[0]), -np.sin(rpy[0])],
									 [0, np.sin(rpy[0])/np.cos(rpy[1]), np.cos(rpy[0])/np.cos(rpy[1])]]), w_b)
		rpy = np.expand_dims(rpy, axis=1)

		# store the actual trajectory to be visualized later
		if (self.mutex_lock_on is not True):
			self.t_series.append(self.t)
			self.x_series.append(xyz[0, 0])
			self.y_series.append(xyz[1, 0])
			self.z_series.append(xyz[2, 0])

		# call the controller with the current states
		self.smc_control(xyz, xyz_dot, rpy, rpy_dot)

	# save the actual trajectory data
	def save_data(self):
		# TODO: update the path below with the correct path
		with open("/home/sumeet/rbe502_project/src/project/scripts/log.pkl","wb") as fp:
			self.mutex_lock_on = True
			pickle.dump([self.t_series,self.x_series,self.y_series,self.z_series], fp)

if __name__ == '__main__':
	rospy.init_node("quadrotor_control")
	rospy.loginfo("Press Ctrl + C to terminate")
	whatever = Quadrotor()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down")
