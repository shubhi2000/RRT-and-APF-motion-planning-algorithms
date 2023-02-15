import random 
import matplotlib.pyplot as plt 

# given environment 
p=plt.figure()

obs1=plt.Circle((4.5,3),2,color='r',fill=False)
obs2=plt.Circle((3,12),2,color='g',fill=False)
obs3=plt.Circle((15,15),3,color='b',fill=False)  

ax=plt.gca() 
ax.set_xlim((0,30))
ax.set_ylim((0,30)) 

ax.add_artist(obs1) 
ax.add_artist(obs2)
ax.add_artist(obs3)

p1=plt.Circle((1,1),0.3)
p2=plt.Circle((20,20),0.3)
ax.add_artist(p1)
ax.add_artist(p2)  

# the two locations given
root_start=(1,1)
root_goal=(20,20)

obstacles_centres=[(4.5,3),(3,12),(15,15)]
obstacles_radii=[2,2,3]

# parameter -> attractive 
ka=0.1
						
point=[1,1]

# x and y stores the coordinates in order 
x=[1]
y=[1]

while(((point[0]-root_goal[0])**2+(point[1]-root_goal[1])**2)**0.5>0.1):
	# velocities 
	#negative gradient for x (attractive potential)
	x_vel=-ka*(root_goal[0]-point[0])
	# negative gradient in y (attractive potential)
	y_vel=-ka*(root_goal[1]-point[1])  

	# parameters for repulsive potential
	n_oi=1
	K=0.1
	gamma=2     

	for j in range(3): 
		d=((obstacles_centres[j][0]-point[0])**2+(obstacles_centres[j][1]-point[1])**2)**0.5   
		n=d-obstacles_radii[j]
		if n<=n_oi:
			# if near the obstacle, a repulsive gradient will be added
			x_vel=x_vel-K*((1/n-1/n_oi)**(gamma-1))*(-1/n**2)*(1/(2*d**0.5))*2*(-point[0]+obstacles_centres[j][0])     
			y_vel=y_vel-K*((1/n-1/n_oi)**(gamma-1))*(-1/n**2)*(1/(2*d**0.5))*2*(-point[1]+obstacles_centres[j][1])
	
	# find next proceeding point
	point[0]=point[0]-0.01*x_vel
	point[1]=point[1]-0.01*y_vel
	
	x.append(point[0])
	y.append(point[1])

plt.plot(x,y)
p.savefig('figure.jpg')