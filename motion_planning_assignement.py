import numpy as np
import openravepy as orpy
import math 
env = orpy.Environment()
env.SetViewer('qtcoin')
env.Load('osr_openrave/worlds/pick_and_place.env.xml')
def create_box ( T, color = [0,0.6,0]):
	box = orpy.RaveCreateKinBody(env, '')
	box.SetName('box')
	box.InitFromBoxes(np.array([[0,0,0,0.03,0.03,0.005]]),True)
	g = box.GetLinks()[0].GetGeometries()[0]
	g.SetAmbientColor(color)
	g.SetDiffuseColor(color)
	box.SetTransform(T)
	env.Add(box,True)
	return box
T = np.eye(4)
container_center = np.array([0.4,0.2,0.195])
T[:3,3] = container_center + np.array([0,-0.3,0])
destination0 = create_box(T,color =  [0,0,0.6])
T[:3,3] = container_center + np.array([0,-0.4,0])
destination1 = create_box(T, color = [0,0,0.6])

boxes = []
nbox_per_layer = 2
n_layer = 10
h = container_center[2]
list_theta = []
for i in range(n_layer):
	nbox_current_layer = 0
	while nbox_current_layer < nbox_per_layer:
		theta = np.random.rand()*np.pi
			
		T[0,0] = np.cos(theta)
		T[0,1] = -np.sin(theta)
		T[1,0] = np.sin(theta)
		T[1,1] = np.cos(theta)
		T[0,3] = container_center[0] + (np.random.rand()-0.5)*0.2
		T[1,3] = container_center[1] + (np.random.rand()-0.5)*0.06
		T[2,3] = h
		box = create_box(T)
		if env.CheckCollision(box):
			env.Remove(box)
		else:
			boxes.append(box)
			nbox_current_layer += 1
			list_theta.append(theta)
	h += 0.011 * 2



robot = env.GetRobot('Denso')
manipulator = robot.SetActiveManipulator('gripper')
robot.SetActiveDOFs(manipulator.GetArmIndices())
np.set_printoptions(precision = 6, suppress = True)
def signe(a):
	if a >= 0:
		return 1
	else :
		return -1

print("\n\n\n")


for anti_count in range(len( boxes) ):

	count = len(boxes) - anti_count -1
	box = boxes[count]
	solutions = np.array([])
	faire_rot_0 = True
	faire_rot_1 = True
	modif_ang = 0
	abc = [0.,0.,0.]
	while np.size(solutions)== 0 and (faire_rot_0 or faire_rot_1):
	
		box_centroid = box.ComputeAABB().pos()
		
		ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(robot, iktype = orpy.IkParameterization.Type.Transform6D)
		if not ikmodel.load():
			ikmodel.autogenerate()

		Tgrasp = np.zeros((4,4))
	
		rot = 0

		while rot < 2 and np.size(solutions) == 0:
		
			if rot == 0 and not faire_rot_0:
				rot += 1
			if rot ==1 and not faire_rot_1:
				rot += 1
				
			if rot < 2:
				theta2 = - (list_theta[count] % (math.pi /2)) + (1-rot)* math.pi /2 

# On oeut faire varier b pour trouver des solutions mais 
# On garde b proche de math.pi pour rester sur une rotation autour de Y (tete vers le bas)
				b = math.pi * (1 -  modif_ang)

# On garde c = 0  pour garder la condition Yz = 0  
				c = 0

# On modifie la valeur de l angle a pour correspondre a nos contraintes Tgrasp(0,0) = - cos(theta2) et Tgrasp(1,0) = - sin(theta2)		 
				try :
					a = math.acos(- math.cos(theta2) / math.cos(b))
					if theta2 > 0:
						a = -a
				except :
					if rot ==0:
						faire_rot_0 = False
					if rot ==1:
						faire_rot_1 = False
			


				Tgrasp[0,0] = math.cos(a) * math.cos(b)	
				Tgrasp[1,0] = math.sin(a) * math.cos(b)
				Tgrasp[2,0] = - math.sin(b)

				Tgrasp[0,1] = math.cos(a) * math.sin(b) * math.sin(c) - math.sin(a) * math.cos(c)
				Tgrasp[1,1] = math.sin(a) * math.sin(b) * math.sin(c) + math.cos(a) * math.cos(c)
				Tgrasp[2,1] = math.cos(b) * math.sin(c) 

				Tgrasp[0,2] = math.cos(a) * math.sin(b) * math.cos(c) + math.sin(a) * math.sin(c)
				Tgrasp[1,2] = math.sin(a) * math.sin(b) * math.cos(c) - math.cos(a) * math.sin(c)
				Tgrasp[2,2] = math.cos(b) * math.cos(c)
				Tgrasp[:3,3] = box_centroid 
				solutions = manipulator.FindIKSolutions(Tgrasp, orpy.IkFilterOptions.CheckEnvCollisions)
			rot += 1
		if modif_ang > 0 :
			modif_ang = - modif_ang
		else :

			modif_ang = -modif_ang + 0.03
		abc = [a,b,c]

	list_obj = [solutions[0]]
	config_obj_1 = [0.596678, 0.250759, 1.239969, 0., 1.650865, 0.576918]
	objectif_1 = np.array(config_obj_1)
	config_obj_2 = [0.241125, -0.002831, 1.501325, 0., 1.643098, -0.795877]
	objectif_2 = np.array(config_obj_2)
	list_obj.append(objectif_1)
	list_obj.append(objectif_2)
	ajusteur = 0.02
	distance_empilement = 0.001
	for i in range(2):

		if count < nbox_per_layer * n_layer /2:
			box_obj = destination0.ComputeAABB().pos()
			box_obj[2] += (nbox_per_layer * n_layer /2 - count ) * (0.01 + distance_empilement) + ajusteur * (1 - i)
		else : 
			box_obj = destination1.ComputeAABB().pos()
			box_obj[2] += ( nbox_per_layer* n_layer - count) *(0.01 + distance_empilement) + ajusteur * (1-i)
		ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(robot, iktype = orpy.IkParameterization.Type.Transform6D)
		if not ikmodel.load():
			ikmodel.autogenerate()
		Tr = np.zeros((4,4))
	
		a = - math.pi / 2 
		b = abc[1]
		c = abc[2]

		Tr[0,0] = math.cos(a) * math.cos(b)	
		Tr[1,0] = math.sin(a) * math.cos(b)
		Tr[2,0] = - math.sin(b)
		Tr[0,1] = math.cos(a) * math.sin(b) * math.sin(c) - math.sin(a) * math.cos(c)
		Tr[1,1] = math.sin(a) * math.sin(b) * math.sin(c) + math.cos(a) * math.cos(c)
		Tr[2,1] = math.cos(b) * math.sin(c) 
		Tr[0,2] = math.cos(a) * math.sin(b) * math.cos(c) + math.sin(a) * math.sin(c)
		Tr[1,2] = math.sin(a) * math.sin(b) * math.cos(c) - math.cos(a) * math.sin(c)
		Tr[2,2] = math.cos(b) * math.cos(c)
		Tr[:3,3] = box_obj 

		solutions_ = manipulator.FindIKSolutions(Tr, orpy.IkFilterOptions.CheckEnvCollisions)

		list_obj.append(solutions_[0])


	planner = orpy.RaveCreatePlanner(env,'birrt')
	for i in range(5):

		params = orpy.Planner.PlannerParameters()
		params.SetRobotActiveJoints(robot)

		params.SetGoalConfig(list_obj[i])
		params.SetPostProcessing('ParabolicSmoother','<_nmaxiterations>40</_nmaxiterations>')
		planner.InitPlan(robot,params)
		traj = orpy.RaveCreateTrajectory(env, '')
		planner.PlanPath(traj)
		controller = robot.GetController()
		controller.SetPath(traj)
		robot.WaitForController(0)
		if i ==0:
			taskmanip = orpy.interfaces.TaskManipulation(robot)
			robot.Grab(box)
			robot.WaitForController(0)
		if i == 4:
			taskmanip = orpy.interfaces.TaskManipulation(robot)
			
			robot.ReleaseAllGrabbed()
			robot.WaitForController(0)
	




run = True 
while run :
	run = True
