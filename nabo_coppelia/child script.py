#python
'''========= ***doc description @ yyp*** ==========
This is part of Nabo (Naughty Boy), an open project for the control of biped robot
Copyright (C) 2024 YYP with MIT License.
Feel free to use in any purpose, and cite Nabo or in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

remoteAPI has com delay, so here using python to call ".dll" compiled from c++
==================================================='''
import sys
import ctypes as c

NLegDof=6
NMot=NLegDof*2

def clip(x,lim):
	if(lim<0):
		lim=-lim
	if(x<-lim):
		return -lim
	elif(x>lim):
		return lim
	else:
		return x

class inClass(c.Structure):
	_fields_=[	("cnt",c.c_int),
				("cmdVx",c.c_double),
				("cmdVy",c.c_double),
				("cmdWz",c.c_double),
				("supP",c.c_double*3),
				("supV",c.c_double*3),
				("rpy",c.c_double*3),
				("gyr",c.c_double*3),
				("acc",c.c_double*3),
				("j", c.c_double*NMot),
				("w", c.c_double*NMot),
				("t", c.c_double*NMot)
			]
class outClass(c.Structure):
	_fields_=[	("j", c.c_double*NMot),
				("w", c.c_double*NMot),
				("t", c.c_double*NMot)
			]

a=inClass()
b=outClass()
if(sys.platform.startswith('win')):
	nabo=c.CDLL('./nabo.dll')
else:
	nabo=c.CDLL('./libnabo.so')

def sysCall_init():
	sim=require('sim')
	self.dt=sim.getSimulationTimeStep()
	nabo.setDt(c.c_double(self.dt))
	self.mt=[sim.getObject("./mt00"),
			sim.getObject("./mt01"),
			sim.getObject("./mt02"),
			sim.getObject("./mt03"),
			sim.getObject("./mt04"),
			sim.getObject("./mt05"),
			sim.getObject("./mt10"),
			sim.getObject("./mt11"),
			sim.getObject("./mt12"),
			sim.getObject("./mt13"),
			sim.getObject("./mt14"),
			sim.getObject("./mt15")]
	self.bodyDummy=sim.getObject("./bodyDummy")
	self.bodyForceSensor=sim.getObject('./bodyForceSensor')
	self.bodyDummyMass=sim.getObjectFloatParam(sim.getObject("./bodyDummyMass"),sim.shapefloatparam_mass)
	self.tLast=0
	self.RLast=sim.getObjectMatrix(self.bodyDummy)


def sysCall_actuation():
	for i in range(NMot):
		#sim.setJointTargetPosition(self.mt[i], b.j[i])
		#sim.setJointTargetVelocity(self.mt[i], b.w[i])
		sim.setJointTargetForce(self.mt[i], b.t[i])
		pass

def sysCall_sensing():
	t=sim.getSimulationTime()
	dt=t-self.tLast
	self.tLast=t
	a.cnt=int(t/self.dt)#"self.dt" is different from "dt"
	# ==keyboard===========
	msg=sim.getSimulatorMessage()#click in the sense only then the keyboard could work!!
	if(msg[0]==sim.message_keypress):
		key=msg[1][0]
		print(key)
		if(key==ord('w')):
			a.cmdVx+=0.1
		elif(key==ord('s')):
			a.cmdVx-=0.1
		elif(key==ord('a')):
			a.cmdVy+=0.1
		elif(key==ord('d')):
			a.cmdVy-=0.1
		elif(key==ord('j')):
			a.cmdWz+=0.1
		elif(key==ord('l')):
			a.cmdWz-=0.1
		elif(key==ord(' ')):#'space'
			a.cmdVx=0
			a.cmdVy=0
			a.cmdWz=0
		a.cmdVx=clip(a.cmdVx,0.5)
		a.cmdVy=clip(a.cmdVy,0.2)
		a.cmdWz=clip(a.cmdWz,0.5)
		print(f"cmd: vx={a.cmdVx:.2f}, vy={a.cmdVy:.2f}, wz={a.cmdWz:.2f}")
	# ==body pos,vel=======
	p=sim.getObjectPosition(self.bodyDummy)
	a.supP[0]=p[0]; a.supP[1]=p[1]; a.supP[2]=p[2]
	[v,w]=sim.getObjectVelocity(self.bodyDummy)
	a.supV[0]=v[0]; a.supV[1]=v[1]; a.supV[2]=v[2]
	# == rpy ==============
	R=sim.getObjectMatrix(self.bodyDummy)
	rpy=sim.getEulerAnglesFromMatrix(R)
	a.rpy[0]=rpy[0];a.rpy[1]=rpy[1];a.rpy[2]=rpy[2]
	# == gyr, by diff of R ==
	dR=sim.multiplyMatrices(sim.getMatrixInverse(self.RLast),R)
	self.RLast=R
	if(dt>1e-4):
		gyr=sim.getEulerAnglesFromMatrix(dR)
		a.gyr[0]=gyr[0]/dt; a.gyr[1]=gyr[1]/dt; a.gyr[2]=gyr[2]/dt
	# == acc, by (force/mass) ==
	[re,foc,tor]=sim.readForceSensor(self.bodyForceSensor)
	a.acc[0]=foc[0]/self.bodyDummyMass; a.acc[1]=foc[1]/self.bodyDummyMass; a.acc[2]=foc[2]/self.bodyDummyMass
	# == joint ==============
	for i in range(NMot):
		a.j[i]=sim.getJointPosition(self.mt[i])
		a.w[i]=sim.getJointVelocity(self.mt[i])
		a.t[i]=sim.getJointForce(self.mt[i])


	nabo.run(c.byref(a), c.byref(b))

def sysCall_cleanup():

	pass
