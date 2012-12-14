import math

class vec3(object):
	__slots__=['x','y','z']
	def __init__(self,x,y,z):
		self.x=x
		self.y=y
		self.z=z
	def __add__(self,other):
		x = self.x+other.x
		y = self.y+other.y
		z = self.z+other.z
		return vec3(x,y,z)
	def __sub__(self,other):
		x = self.x-other.x
		y = self.y-other.y
		z = self.z-other.z
		return vec3(x,y,z)
	def __mul__(self,num):
		x = self.x*num
		y = self.y*num
		z = self.z*num
		return vec3(x,y,z)
	def __div__(self,num):
		x = self.x/num
		y = self.y/num
		z = self.z/num
		return vec3(x,y,z)
	def normalize(self):
		n = math.sqrt(self.x**2+self.y**2+self.z**2)
		if n < 1e-20:
			return vec3(0,0,0)
		x = self.x/n
		y = self.y/n
		z = self.z/n
		return vec3(x,y,z)
	def dot(self,other):
		x = self.x*other.x
		y = self.y*other.y
		z = self.z*other.z
		return x+y+z
	def norm(self): #2-norm
		return math.sqrt(self.x**2+self.y**2+self.z**2)
	def __repr__(self):
		return '%.2f, %.2f, %.2f'%(self.x,self.y,self.z)
	def __str__(self):
		return '%.2f, %.2f, %.2f'%(self.x,self.y,self.z)

