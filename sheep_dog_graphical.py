# Boids Simulation - Sheep Flocking with Predator
# -openGl Version
#
# Written by Jason Fu and Yu Tomita
# 2009

import sys
import math
import getpass
from random import *
from vectoring import *

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

ESCAPE = '\033'
blah = 0                                # trash variable

sheepSize = 1.                          # m radius of ions (for display)
wolfSize = 1.
numSheep = 20
numWolf = 1
homeSize = 15.


# All information stored as pixels, not SI units.
class sheep(object):
	__slots__=['pos','vel','state','c_s','s_s','a_s','c_w','s_w','a_w','vlim','agitate_count'] #c_s,s_s,a_s,c_w,s_w,a_w are modifiers for boid/boid interactions
	# agitate count keeps track of how long a sheep has been agitated
	# starts from highest position & zero velocity, with center xyz
	def __init__(self,(px,py,pz),(vx,vy,vz)):
		self.pos=vec3(px,py,pz)
		self.vel=vec3(vx,vy,vz)
		self.state=0
		self.c_s=1
		self.s_s=1
		self.a_s=1
		self.c_w=1
		self.s_w=1
		self.a_w=1
		self.vlim = vlim_s * 0.75
		self.agitate_count = 0

class wolf(object):
	__slots__=['pos','vel','state','vlim','c_s','s_s','a_s','ret','turn'] #c_s,s_s,a_s are modifiers for wolf/sheep interactions, ret is logical for returnHome?
	# starts from highest position & zero velocity, with center xyz
	def __init__(self,(px,py,pz),(vx,vy,vz)):
		self.pos=vec3(px,py,pz)
		self.vel=vec3(vx,vy,vz)
		self.state=0
		self.vlim = vlim_w
		self.c_s = 1
		self.s_s = 1
		self.a_s = 1
		self.ret = 0
		self.turn = 0

class home(object):
	__slots__=['pos'] # home only has position
	# home at bottom left corner
	def __init__(self,(px,py,pz)):
		self.pos=vec3(px,py,pz)

#for sim
step = 0.15                                    # stepsize
pause = 0                                       # pause variable

# For animation -opengl stuff and SI units converter
window = 0
ratiof = 0
zoom = 0
oneGridm = 5                            # m per grid
oneGridpx = 20                          # pixels per grid
pxPerm = oneGridpx/oneGridm             # pixels per meter
fieldSize = 10
n = 1.64485
sd = 0.1

sheeps=[]
wolves=[]
homes=[]

#Boids constants
cohesion_ssc = 0.01                        # 1% closer to the center of mass
separation_ssc = 10                        # px separated
alignment_ssc = 0.15                       # 15% closer to other sheep vel
cohesion_swc = -0.01                       # 1% away from wolf
separation_swc = 10                        # px separated
alignment_swc = 0                          # 0% closer to wolf vel
random_sc = 0.5                            # px velocity
 
cohesion_wsc = 0.02                        # 2% closer to the center of mass
separation_wsc = 0                         # px separated
alignment_wsc = 0.01                       # 1% closer to other sheep vel
random_wc = 0.75                           # px velocity


#Sheep variables
c_ss = 1                                   # cohesion multiplier to sheep
s_ss = 1                                   # separation multiplier to sheep
a_ss = 1                                   # alignment multiplier to sheep
c_sw = 1                                   # cohesion multiplier to wolves
s_sw = 1                                   # separation multiplier to wolves
a_sw = 1                                   # alignment multiplier to wolves
vlim_s = 4                                 # maximum velocity for sheep
aov_s = 145                                # angle of view for one side of the sheep 145
ros_s = 20*oneGridpx                       # range of sight 20 
threat_range = 4*oneGridpx                 # range when sheep become agitated by non-sheep 4 
ag_count = 200                             # agitate count (steps)

#Wolf Variables
c_ws = 1                                   # cohesion multiplier to wolves
s_ws = 1                                   # separation multiplier to wolves
a_ws = 1                                   # alignment multiplier to wolves
aov_w = 110                                # angle of view for one side of the wolves
ros_w = 20*oneGridpx                       # range of sight
vlim_w = 6                                 # maximum velocity for wolves


def initStates():
	global t,sheeps,oneGridpx,fieldSize
	t=0
	p=[0]*3
	v=[0]*3
	for i in range(numSheep):
		for j in range(3):
			p[j]=uniform(-fieldSize*oneGridpx,fieldSize*oneGridpx)
			v[j]=uniform(-oneGridpx,oneGridpx)
		sheeps.append(sheep((p[0],0,p[2]),(v[0],0,v[2])))
	#start at house with 0 velocity
	wolves.append(wolf((-fieldSize*oneGridpx,0,fieldSize*oneGridpx),(0,0,0)))
	homes.append(home((-fieldSize*oneGridpx,0,fieldSize*oneGridpx)))

def dist(a,b):#both vec3
	return math.sqrt((a.x-b.x)**2+(a.y-b.y)**2+(a.z-b.z)**2)

def magnitude(a):#input is vec3
        return math.sqrt(a.x**2 + a.y**2 + a.z**2)

def angle(main_s,target_s,ros):#gives angle of main boid and target boid
        # if boid is stationary, it can see all around him
        if magnitude(main_s.vel) > 0.01 or magnitude(main_s.vel) < -0.01:
                temp_point = main_s.pos + (main_s.vel/magnitude(main_s.vel))*ros
                length_a = dist(temp_point,target_s.pos)
                length_b = dist(main_s.pos,target_s.pos)
                length_c = ros
                return (180/math.pi)*math.acos((length_b**2+length_c**2-length_a**2)/(2*length_b*length_c))
        else:
                return 180

## START OF SHEEP/SHEEP INTERACTION ##
def cohesion_ss(sn, const):
	global sheeps
	center = vec3(0.,0.,0.)
	count = 0
	diff = vec3(0.,0.,0.)
	for i,s in enumerate(sheeps):
		if i is sn:
			continue
		if dist(s.pos,sheeps[sn].pos) > ros_s  or angle(sheeps[sn],s,ros_s) > aov_s: # skip if target sheep not in view
			continue
		center = center + s.pos
		count = count + 1
	if count is not 0:
		center = center/(count)
		diff = center-sheeps[sn].pos
	return diff*const

def separation_ss(sn, const):
	global sheeps
	diff = vec3(0.,0.,0.)
	for i,s in enumerate(sheeps):
		if i is sn:
			continue
		if dist(s.pos,sheeps[sn].pos) < const:
			diff = diff - s.pos + sheeps[sn].pos
	return diff

def alignment_ss(sn,const):
	center = vec3(0.,0.,0.)
	count = 0
	diff = vec3(0.,0.,0.)
	for i,s in enumerate(sheeps):
		if i is sn:
			continue
		if dist(s.pos,sheeps[sn].pos) > ros_s or angle(sheeps[sn],s,ros_s) > aov_s: # skip if target wolf not in view
			continue
		center = center + s.vel
		count = count + 1
	if count is not 0:
		center = center/(count)
		diff = center-sheeps[sn].vel
	return diff*const

## START OF SHEEP/WOLF INTERACTION ##
def cohesion_sw(sn, const):
	global wolves
	center = vec3(0.,0.,0.)
	count = 0
	diff = vec3(0.,0.,0.)
	for i,w in enumerate(wolves):
		if dist(w.pos,sheeps[sn].pos) > ros_s or angle(sheeps[sn],w,ros_s) > aov_s: # or dist(w.pos,sheeps[sn].pos) > threat_range: # skip if target wolf not in view
			continue
		center = center + w.pos
		count = count + 1
	if count is not 0:
		center = center/(count)
		diff = center-sheeps[sn].pos
	return diff*const


def separation_sw(sn, const):
	diff = vec3(0.,0.,0.)
	for i,w in enumerate(wolves):
		if dist(w.pos,sheeps[sn].pos) < const:
			diff = diff - w.pos + sheeps[sn].pos
	return diff

def alignment_sw(sn,const):
	center = vec3(0.,0.,0.)
	count = 0
	diff = vec3(0.,0.,0.)
	for i,w in enumerate(wolves):
		if dist(w.pos,sheeps[sn].pos) > ros_s or angle(sheeps[sn],w,ros_s) > aov_s: # skip if target sheep not in view
			continue
		center = center + w.vel
		count = count + 1
	if count is not 0:
		center = center/(count)
		diff = center-sheeps[sn].pos
	return diff*const

## START OF WOLF/SHEEP INTERACTION ##
def cohesion_ws(wn, const):
	global wolves
	center = vec3(0.,0.,0.)
	count = 0
	diff = vec3(0.,0.,0.)
	for i,s in enumerate(sheeps):
		if dist(s.pos,wolves[wn].pos) > ros_w or angle(wolves[wn],s,ros_w) > aov_w: # skip if target sheep not in view
			continue
		center = center + s.pos
		count = count + 1
	if count is not 0:
		center = center/(count)
		diff = center-wolves[wn].pos
	return diff*const


def separation_ws(wn, const):
	diff = vec3(0.,0.,0.)
	for i,s in enumerate(sheeps):
		if dist(s.pos,wolves[wn].pos) < const:
			diff = diff - s.pos + wolves[wn].pos
	return diff

def alignment_ws(wn,const):
	center = vec3(0.,0.,0.)
	count = 0
	diff = vec3(0.,0.,0.)
	for i,s in enumerate(sheeps):
		if dist(s.pos,wolves[wn].pos) > ros_w or angle(wolves[wn],s,ros_w) > aov_w: # skip if target sheep not in view
			continue
		center = center + s.vel
		count = count + 1
	if count is not 0:
		center = center/(count)
		diff = center-wolves[wn].pos
	return diff*const


#Boids Functions
def limitVelocity(boid):
	if magnitude(boid.vel) > boid.vlim:
		boid.vel = (boid.vel / magnitude(boid.vel)) * normalvariate(boid.vlim,boid.vlim*sd*n)

def turnBack(boid):
	global fieldSize,oneGridpx
	# make them come back when they run out of bounds
	if boid.pos.x > fieldSize*oneGridpx:
		boid.vel.x = boid.vel.x - oneGridpx/2
	if boid.pos.x < -fieldSize*oneGridpx:
		boid.vel.x = boid.vel.x + oneGridpx/2
	if boid.pos.z > fieldSize*oneGridpx:
		boid.vel.z = boid.vel.z - oneGridpx/2
	if boid.pos.z < -fieldSize*oneGridpx:
		boid.vel.z = boid.vel.z + oneGridpx/2

def random_vel(const):
	v = vec3(0.,0.,0.)
	v.x = uniform(-const,const)
	v.z = uniform(-const,const)
	return v

def checkStateSheep(boid):
        # sheep become agitated when unknown boid goes into its threat_range for a certain number of ticks
        if boid.agitate_count <= 0:
                boid.state = 0
        else:
                boid.agitate_count = boid.agitate_count - 1
        for wn, w in enumerate(wolves):
                if dist(boid.pos,w.pos) < threat_range and dist(w.pos,boid.pos) < ros_s and angle(boid,w,ros_s) < aov_s:
                        boid.state = 1
                        boid.agitate_count = normalvariate(ag_count,ag_count*sd*n)

def updateModifierSheep(boid):
	if boid.state is 0: #default state
		boid.c_s = 0
		boid.s_s = 1
		boid.a_s = 0
		boid.c_w = 0
		boid.s_w = 1
		boid.a_w = 0
		boid.vlim = vlim_s * 0.75
	elif boid.state is 1: #agitated state
		boid.c_s = 2 # get closer to center of all sheep
		boid.s_s = 1
		boid.a_s = 2 # flock more with sheep
		boid.c_w = 3 # get further away from wolf
		boid.s_w = 1
		boid.a_w = 1
		boid.vlim = vlim_s

def resetStateWolf(boid): #reset wolf to default state for instantaneous states
	if boid.state is 5:
		boid.state = 0
        

def updateModifierWolf(boid):
	if boid.state is 0: #default state
		boid.c_s = 1
		boid.s_s = 1
		boid.a_s = 1
		boid.vlim = vlim_w * 0.5
		boid.ret = 0
	elif boid.state is 1: # stop command
		boid.c_s = 0
		boid.s_s = 0
		boid.a_s = 0
		boid.vlim = 0
		boid.ret = 0
	elif boid.state is 2: # slow down command
		boid.vlim = boid.vlim * 0.9
	elif boid.state is 3: # move in closer command
		boid.c_s = 1.5
		boid.s_s = 0.5
		boid.a_s = 0.5
		boid.vlim = vlim_w * 0.75
		boid.ret = 0
	elif boid.state is 4: # stop working, return home
		boid.c_s = 0
		boid.s_s = 0
		boid.a_s = 0
		boid.vlim = vlim_w * 0.5
		boid.ret = 1
	elif boid.state is 5: # bark at stock, effectively agitates all sheep on board
		for sn, s in enumerate(sheeps):
			s.state = 1
			s.agitate_count = normalvariate(ag_count,ag_count*sd*n)
	elif boid.state is 6: # rotate to left of stock
		boid.turn = -1
	elif boid.state is 7: #rotate to right of stock
		boid.turn = 1
	elif boid.state is 8: #stop rotating
		boid.turn = 0
	boid.state=-1

def returnHome(boid):
	if boid.ret is 1:
		boid.vel = homes[0].pos - boid.pos

def rotate(boid): #input should be a wolf
	global wolves
	
	center = vec3(0.,0.,0.)
	count = 0
	diff = vec3(0.,0.,0.)
	for i,s in enumerate(sheeps):
		if dist(s.pos,boid.pos) > ros_w or angle(boid,s,ros_w) > aov_w: # skip if target sheep not in view
			continue
		center = center + s.pos
		count = count + 1
	if count is not 0:
		center = center/(count)
		diff = center - boid.pos

	#now rotate it 90 degrees to the left
	#rotated matrix = [cos(th) -sin(th);sin(th) cos(th)] * vec
	rotation = math.pi/2*boid.turn
	xvec = diff.x * math.cos(rotation) - diff.z * math.sin(rotation)
	zvec = diff.x * math.sin(rotation) + diff.z * math.cos(rotation)
	rvec = vec3(xvec,0.,zvec)
	return rvec

def scaletomax(vec): #averages 2 vectors
        if magnitude(vec) > 0.01 or magnitude(vec) < -0.01:
                return vec / magnitude(vec) * vlim_w
        
	

def computeStep():
	global t, cohesion_ssc, separation_ssc, alignment_ssc, step
	global c_ss, s_ss, a_ss, c_sw, s_sw, a_sw, c_ws, s_ws, a_ws
	global c_s,s_s,a_s
	global cohesion_swc, separation_swc, alignment_swc
	global cohesion_wsc, separation_wsc, alignment_wsc
	if not pause:
		t = t + step

		# update state of sheep 
		for sn, s in enumerate(sheeps):
			checkStateSheep(s)
			updateModifierSheep(s)

		# update state of wolves 
		for wn, w in enumerate(wolves):
			updateModifierWolf(w)
			resetStateWolf(w)

		
                # the coefficients are drawn from a normal distribution with mean at the value and standard deviation
                # set to provide 90% confidence (n = 1.64485)    80% would be n = 1.28155     95% would be n = 1.95996

		# update movement vector and position for sheep
		for sn, s in enumerate(sheeps):
			v1 = cohesion_ss(sn,normalvariate(cohesion_ssc*c_ss*s.c_s,cohesion_ssc*c_ss*s.c_s*sd*n))
			v2 = separation_ss(sn,normalvariate(separation_ssc*s_ss*s.s_s,separation_ssc*s_ss*s.s_s*sd*n))
			v3 = alignment_ss(sn,normalvariate(alignment_ssc*a_ss*s.a_s,alignment_ssc*a_ss*s.a_s*sd*n))
			v4 = cohesion_sw(sn,normalvariate(cohesion_swc*c_sw*s.c_w,cohesion_swc*c_sw*s.c_w*sd*n))
			v5 = separation_sw(sn,normalvariate(separation_swc*s_sw*s.s_w,separation_swc*s_sw*s.s_w*sd*n))
			v6 = alignment_sw(sn,normalvariate(alignment_swc*a_sw*s.a_w,alignment_swc*a_sw*s.a_w*sd*n))
			v7 = random_vel(normalvariate(random_sc,random_sc*sd*n))
			s.vel = s.vel + v1 + v2 + v3 + v4 + v5 + v6 + v7
				
			turnBack(s)
			limitVelocity(s)
				
			s.pos = s.pos + s.vel*step

		# update movement vector and position for wolves 
		for wn, w in enumerate(wolves):
			v1 = cohesion_ws(wn,normalvariate(cohesion_wsc*c_ws*w.c_s,cohesion_wsc*c_ws*w.c_s*sd*n))
			v2 = separation_ws(wn,normalvariate(separation_wsc*s_ws*w.s_s,separation_wsc*s_ws*w.s_s*sd*n))
			v3 = alignment_ws(wn,normalvariate(alignment_wsc*a_ws*w.a_s,alignment_wsc*a_ws*w.a_s*sd*n))
			v4 = random_vel(normalvariate(random_wc,random_wc*sd*n))

			w.vel = w.vel + v1 + v2 + v3 + v4
			
			if w.turn is not 0:
				v5 = rotate(w)

				scaletomax(v5)
				w.vel = w.vel + v5
				limitVelocity(w)

			returnHome(w)
			turnBack(w)
			limitVelocity(w)
			
			w.pos = w.pos + w.vel*step
		


#--------- below is more graphics related --------#
def InitGL(Width, Height):
	glClearColor(0.0, 0.0, 0.0, 0.0)
	glClearDepth(1.0)
	glDepthFunc(GL_LESS)
	glEnable(GL_DEPTH_TEST)
	glShadeModel(GL_SMOOTH)

	glMatrixMode(GL_PROJECTION)
	glLoadIdentity()
	gluPerspective(45.0, float(Width)/float(Height), 0.1, 1000.0)
	glMatrixMode(GL_MODELVIEW)

def ReSizeGLScene(Width, Height):
	if Height == 0:                                          
		Height = 1
	glViewport(0, 0, Width, Height)         
	glMatrixMode(GL_PROJECTION)
	glLoadIdentity()
	gluPerspective(45.0, float(Width)/float(Height), 0.1, 1000.0)
	glMatrixMode(GL_MODELVIEW)

def drawSheep():
	global pxPerm, sheepSize
	glPushMatrix()
	glutWireSphere(sheepSize*pxPerm,16,8)   
	glPopMatrix()

def drawWolf():
        global pxPerm, wolfSize
        glPushMatrix()
        glutWireTorus(0.5*wolfSize*pxPerm,1*wolfSize*pxPerm,10,16)
        glPopMatrix()

def drawHome():
        global pxPerm, homeSize
        glPushMatrix()
        glutSolidCube(homeSize)
        glPopMatrix()

# numx = number of lines from x=0 to edge (length is twice)
def drawGrid(numx):
	global oneGridpx
	xi = -(numx)*oneGridpx
	xf = -xi
	for i in range(xi,xf+oneGridpx,oneGridpx):
		glBegin(GL_LINES)
		glVertex3i(i,0,xi-oneGridpx)
		glVertex3i(i,0,xf+oneGridpx)
		glVertex3i(xi-oneGridpx,0,i)
		glVertex3i(xf+oneGridpx,0,i)
		glEnd()

def drawField(numx):
	global oneGridpx
	numx = numx*2
	glBegin(GL_QUAD_STRIP)
	glVertex3i(numx*fieldSize,-1,numx*fieldSize)
	glVertex3i(-numx*fieldSize,-1,numx*fieldSize)
	glVertex3i(numx*fieldSize,-1,-numx*fieldSize)
	glVertex3i(-numx*fieldSize,-1,-numx*fieldSize)
	glEnd()

def DrawGLScene():
	global pxPerm, fieldSize, sheeps, numSheep, ag_count

	computeStep()

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
	glLoadIdentity()# Reset The View

	glTranslatef(0.,-1.,-800+zoom*5)
	glRotate(50,1,0,0)      # 26.56=atan(5/10)

    #draw grid
	glPushMatrix()
	glTranslatef(0.,-5.,0.)
	glColor3f(0.,0.,0.)
	drawGrid(fieldSize)
	#draw field
	glColor3f(0.,0.5,0.)
	drawField(fieldSize)
	glPopMatrix()

	#draw sheeps
	for i in range(numSheep):
		glPushMatrix()
		glColor3f(1.,(ag_count-sheeps[i].agitate_count)/ag_count,(ag_count-sheeps[i].agitate_count)/ag_count)
		x,y,z=(sheeps[i].pos.x,sheeps[i].pos.y,sheeps[i].pos.z) # position set here
		glTranslatef(x,y,z)
		drawSheep()
		glPopMatrix()

	#draw wolf
	for i in range(numWolf):
		glPushMatrix()
		glColor3f(139/255.,69/255.,19/255.)
		x,y,z=(wolves[i].pos.x,wolves[i].pos.y,wolves[i].pos.z) # position set here
		glTranslatef(x,y,z)
		drawWolf()
		glPopMatrix()

        #draw house
	glPushMatrix()
	glColor3f(0.3,0.3,0.3)
	x,y,z=(homes[0].pos.x,homes[0].pos.y,homes[0].pos.z) # position set here
	glTranslatef(x,y,z)
	drawHome()
	glPopMatrix()	

	glutSwapBuffers()

def keyPressed(*args):
	global vc, zoom, a_ss, c_ss, a_ss, c_sw, s_sw, a_sw, c_ws, s_ws, a_ws, pause,blah
	if args[0] == ESCAPE:
		glutDestroyWindow(window)
		sys.exit()
	elif args[0] == 'z':
		zoom = zoom + 1
	elif args[0] == 'Z':
		zoom = zoom - 1
	elif args[0] == 't':
		a_ss = -a_ss
		c_ss = -c_ss
	elif args[0] == 'p':
		if pause is 0:
			pause = 1
		else:
			pause = 0
	elif args[0] == '0':
		for wn,w in enumerate(wolves):
			w.state = 0
	elif args[0] == '1':
		for wn,w in enumerate(wolves):
			w.state = 1
	elif args[0] == '2':
		for wn,w in enumerate(wolves):
			w.state = 2
	elif args[0] == '3':
		for wn,w in enumerate(wolves):
			w.state = 3
	elif args[0] == '4':
		for wn,w in enumerate(wolves):
			w.state = 4
	elif args[0] == '5':
		for wn,w in enumerate(wolves):
			w.state = 5
	elif args[0] == '6':
		for wn,w in enumerate(wolves):
			w.state = 6
	elif args[0] == '7':
		for wn,w in enumerate(wolves):
			w.state = 7
	elif args[0] == '8':
		for wn,w in enumerate(wolves):
			w.state = 8
		

def vsimulate():
	global window

	initStates()

	glutInit(sys.argv)
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH)
	glutInitWindowSize(960, 720)
	glutInitWindowPosition(0, 0)

	window = glutCreateWindow("Boids - Sheep Flocking with Predator")

	glutDisplayFunc(DrawGLScene)
	glutIdleFunc(DrawGLScene)
	glutReshapeFunc(ReSizeGLScene)
	
	glutKeyboardFunc(keyPressed)
	InitGL(960, 720)        

vsimulate()
glutMainLoop()
