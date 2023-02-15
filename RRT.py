import random 
import matplotlib.pyplot as plt 

# drawing the given surrounding  
p=plt.figure()

#obstacles
obs1=plt.Circle((4.5,3),2,color='r',fill=False)
obs2=plt.Circle((3,12),2,color='g',fill=False)
obs3=plt.Circle((15,15),3,color='b',fill=False)  

ax=plt.gca() 
ax.set_xlim((0,30))
ax.set_ylim((0,30)) 

ax.add_artist(obs1) 
ax.add_artist(obs2)
ax.add_artist(obs3)

# marking locations (1,1) and (20,20)
p1=plt.Circle((1,1),0.3)
p2=plt.Circle((20,20),0.3)
ax.add_artist(p1)
ax.add_artist(p2)  

# start and goal points
root_start=(1,1)
root_goal=(20,20)

nodes_start=[(1,1)]   # list of nodes in the tree from start 
nodes_goal=[(20,20)]  # list of nodes in the tree from goal 

obstacles_centres=[(4.5,3),(3,12),(15,15)]
obstacles_radii=[2,2,3]

parent_nodes_start=[] #list of parent nodes for tree from start
parent_nodes_goal=[]  #list of parent nodes for tree from goal 

delta=2

def nearest_node(point_x,point_y,nodes_list):
	#returns the nearest node from nodes_list to the point(pint_x,point_y) 
	min_dist=((nodes_list[0][0]-point_x)**2 + (nodes_list[0][1]-point_y)**2)**0.5  
	node=nodes_list[0]
	i=0
	for j in nodes_list:
		# when found a nearer node, update the variables
		dist=((j[0]-point_x)**2+(j[1]-point_y)**2)**0.5
		if min_dist>dist:
			min_dist=dist
			node=nodes_list[i]
		i+=1
	return node

def collision(node,new_point):
	#determines if a collision with obstacle happens in the line joining the node and new_point 
	slope=(new_point[1]-node[1])/(new_point[0]-node[0])
	t=0
	for j in range(len(obstacles_centres)):
		x0=obstacles_centres[j][0]
		y0=obstacles_centres[j][1]

		# coefficients of line joining the two points
		a=(node[1]-new_point[1])/(node[0]-new_point[0])
		b=-1
		c=new_point[1]-new_point[0]*((node[1]-new_point[1])/(node[0]-new_point[0]))
		# distance of line from center
		dist=abs(a*x0+b*y0+c)/(a**2+b**2)**0.5

		# if line is at a distance less than the radius of the particular obstacle there can be a collision 
		if dist<obstacles_radii[j]:
			#  coefficients of line joining center to the above line
			a=1/slope
			b=1
			c=-(y0+x0/slope)

			# check for whether the points are on the same side and also their distance
			if a*new_point[0]+b*new_point[1]+c<0 and a*node[0]+b*node[1]+c<0:
				if ((new_point[0]-obstacles_centres[j][0])**2+(new_point[1]-obstacles_centres[j][1])**2)**0.5<obstacles_radii[j]:
					t=1
				elif ((node[0]-obstacles_centres[j][0])**2+(node[1]-obstacles_centres[j][1])**2)**0.5<obstacles_radii[j]:
					t=1
			elif a*new_point[0]+b*new_point[1]+c>0 and a*node[0]+b*node[1]+c>0:
				if ((new_point[0]-obstacles_centres[j][0])**2+(new_point[1]-obstacles_centres[j][1])**2)**0.5<obstacles_radii[j]:
					t=1
				elif ((node[0]-obstacles_centres[j][0])**2+(node[1]-obstacles_centres[j][1])**2)**0.5<obstacles_radii[j]:
					t=1
			if t==1:
				break
	# t=1 says that collision is occuring 
	return t 

def node_new(node,point):   
	# determines the new node at a distance delta from a node already in tree and nearest to the random generated point     
	slope=(point[1]-node[1])/(point[0]-node[0])
	# new node can be found by using these 2 info : 
	# slope is same as of random point to nearest point
	# and that the distance has to be delta for the new point
	new_point=[0,0]
	new_point[0]=node[0]-((delta**2/(1+slope**2)))**0.5
	if new_point[0]<0:
		new_point[0]=node[0]+((delta**2/(1+slope**2)))**0.5
	if new_point[0]>30:
		new_point[0]=-1
	new_point[1]=node[1]-slope*(node[0]-new_point[0])

	# conditions for collision and whether coordinate is going out of the frame
	if new_point[1]<0 or new_point[1]>30:
		new_point[1]=-1
	if new_point[0]!=-1 and new_point[1]!=-1:
		t=collision(node,new_point)
		if t==1:
			new_point=[-1,-1]
		
		p=0
		for m in range(len(obstacles_centres)):
			if obstacles_centres[m][0]-obstacles_radii[m]<new_point[0]<obstacles_centres[m][0]+obstacles_radii[m]:
				if obstacles_centres[m][1]-obstacles_radii[m]<new_point[1]<obstacles_centres[m][1]+obstacles_radii[m]:
					p=1     
		if p==1: 
			new_point[0]=-1

	return new_point

for i in range(1000):
	# find a random coordinated in the xy plane
	x_rand=random.random()*30  
	y_rand=random.random()*30

	#find the nearest node to the random coordinates found for both start and goal trees 
	node_s=nearest_node(x_rand,y_rand,nodes_start)
	node_g=nearest_node(x_rand,y_rand,nodes_goal)

	#find the new node for both trees
	rand=(x_rand,y_rand)
	new_node_of_s=node_new(node_s,rand)
	new_node_of_g=node_new(node_g,rand)

	# see if valid new_node is found or not  
	if new_node_of_s[0]!=-1 and new_node_of_s[1]!=-1:
		nodes_start.append(new_node_of_s)
		parent_nodes_start.append(node_s)
		plt.plot([node_s[0],new_node_of_s[0]],[node_s[1],new_node_of_s[1]],color='b')

	if new_node_of_g[0]!=-1 and new_node_of_g[1]!=-1:
		nodes_goal.append(new_node_of_g) 
		parent_nodes_goal.append(node_g) 
		plt.plot([node_g[0],new_node_of_g[0]],[node_g[1],new_node_of_g[1]],color='b') 

# connect the two trees
# from all the nodes of the trees whose linkage do not lead to collision find the nearest pair of nodes      
dist=-1
a=-1
b=-1
for i in range(len(nodes_start)):
	for j in range(len(nodes_goal)): 
		if (collision(nodes_start[i],nodes_goal[j]))==0:   
			d=((nodes_goal[j][0]-nodes_start[i][0])**2+(nodes_goal[j][1]-nodes_start[i][1])**2)**0.5  
			if dist==-1:
				dist=d 
				a=i
				b=j
			else:
				if d<dist:
					dist=d
					a=i 
					b=j 

#connecting the trees
nodes_goal.append(nodes_start[a])
nodes_start.append(nodes_goal[b])

parent_nodes_goal.append(nodes_goal[b])
parent_nodes_start.append(nodes_start[a])

print(nodes_goal[b], nodes_start[a])  

# path from the node in start tree (which gets connected to goal tree) to the start coordinates
curr=nodes_start[a]
c=0
l_start=[curr]
while(c==0):
	curr=parent_nodes_start[nodes_start.index(curr)-1] 
	l_start.append(curr)	
	if curr==root_start:
		c=1
 
#path from the node in goal tree (which gets connected to start tree) to the goal (20,20) 
curr=nodes_goal[b]
c=0
l_goal=[curr]
while(c==0):
	curr=parent_nodes_goal[nodes_goal.index(curr)-1]   
	l_goal.append(curr)
	if curr==root_goal:  
		c=1

# reversing l_start and combining with list l_goal gives the determined path
l_start.reverse()
print(l_start+l_goal)

x=[]
y=[]
l=l_start+l_goal 

# storing the lists for coordinates for plotting 
for j in range(len(l)): 
	x.append(l[j][0])
	y.append(l[j][1])

plt.plot(x,y,color='r')  
# figure will get saved 
p.savefig('figure1.png') 