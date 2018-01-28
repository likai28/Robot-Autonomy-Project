import numpy
import math
from DiscreteEnvironment import DiscreteEnvironment

class HerbEnvironment(object):
    
    def __init__(self, herb, resolution):
        
        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        upper_coord = numpy.array([x - 1 for x in self.discrete_env.num_cells])
        upper_config = self.discrete_env.GridCoordToConfiguration(upper_coord)
        for idx in range(len(upper_config)):
            self.discrete_env.num_cells[idx] -= 1

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 0.7], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)
        
        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
    
    def GetSuccessors(self, node_id):
        successors = []
	config = self.discrete_env.NodeIdToGridCoord(node_id)
	for i in range(0,self.discrete_env.dimension):
		config[i] += 1
		if config[i] < self.discrete_env.num_cells[i]:
			successors.append(self.discrete_env.GridCoordToNodeId(config))
		config[i] -= 2
		if config[i] >= 0:
			successors.append(self.discrete_env.GridCoordToNodeId(config))
		config[i] += 1
        return successors

    def GetValidSuccessors(self, node_id):
	successors = self.GetSuccessors(self, node_id)
	successors = [x for x in successors if self.ComputeDistance(node_id,x) != float("inf")]
	return successors

    def ComputeDistanceBetweenIds(self, start_id, end_id):
	epsilon = 0.01
	start = self.discrete_env.NodeIdToConfiguration(start_id) 
	end = self.discrete_env.NodeIdToConfiguration(end_id) 
	with self.robot:
		cfg = self.robot.GetActiveDOFValues()
		vec = end-start
		veclen = numpy.linalg.norm(vec)
		vec = vec/veclen
		travel = 0
		while travel < veclen:
			val = start + vec*travel
			self.robot.SetActiveDOFValues(val)	
			if self.robot.GetEnv().CheckCollision(self.robot) or self.robot.CheckSelfCollision():
				return float("inf") 
			travel += epsilon
	return self.ComputeHeuristicCost(start_id, end_id)	

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0
	start = numpy.array(self.discrete_env.NodeIdToConfiguration(start_id))
	end = numpy.array(self.discrete_env.NodeIdToConfiguration(goal_id)) 	
	return numpy.linalg.norm(end-start)
	
    def SetGoalParameters(self, goal_config, p = 0.05):
		self.goal_config = goal_config
		self.p = p
		

    def GenerateRandomConfiguration(self):
		config = [0] * len(self.robot.GetActiveDOFIndices())

		lower_limits, upper_limits = self.robot.GetActiveDOFLimits()

		import numpy
		lower_limits = numpy.array(lower_limits)
		upper_limits = numpy.array(upper_limits)

		# Generate random configuration
		choice = numpy.random.rand(1)
		if choice < self.p:
			config = self.goal_config
		else:
			COLLISION = True
			while COLLISION:
				config = numpy.random.rand(len(self.robot.GetActiveDOFIndices()))*(upper_limits - lower_limits) + lower_limits
				# Check if it is collision free
				with self.robot:
					robot_pos = self.robot.GetActiveDOFValues()
					robot_pos = config
					self.robot.SetActiveDOFValues(robot_pos)
					if (self.robot.GetEnv().CheckCollision(self.robot) or self.robot.CheckSelfCollision()) == False:
						COLLISION = False
		return self.discrete_env.NodeIdToConfiguration( self.discrete_env.ConfigurationToNodeId(config))


	
    def ComputeDistance(self, start_config, end_config):
		return numpy.linalg.norm(end_config - start_config)


    def Extend(self, start_config, end_config):
		epsilon = .01
		dist = self.ComputeDistance(start_config, end_config)
		numSteps = math.ceil(dist / epsilon)
		step = (end_config - start_config) / numSteps
		best_config = None
		i = 1
		while i <= numSteps:
			cur_config = start_config+step*i
			# Check if it is collision free
			with self.robot:
				robot_pos = self.robot.GetActiveDOFValues()
				robot_pos = cur_config
				self.robot.SetActiveDOFValues(robot_pos)
				## should we also check self.robot.CheckSelfCollision?
				if self.robot.GetEnv().CheckCollision(self.robot):
					return best_config
			#update variables
			best_config = cur_config
			i += 1
		return end_config
		
    def ShortenPath(self, path, timeout=5.0):
		#print('starting path shortening')
		start_time = time.time()
		current_time = 0
		i = 0
		totalDistance = 0
		#TODO: Calculate current path distance
		for i in range(0,len(path)-1):
			curDistance = self.ComputeDistance(path[i],path[i+1])
			totalDistance += curDistance
		print(totalDistance)
		i = 0
		while current_time < timeout and i == 0:
			#print("NOW IN THE FUNCTION LOOP")
			pathLength = len(path)
			p1index = random.randint(0,pathLength-2)
			p2index = p1index
			while p2index == p1index or abs(p2index-p1index) <= 1:
				p2index = random.randint(0,pathLength-2)
			if p1index > p2index:
				p1index, p2index = p2index,p1index
			p1a = numpy.array(path[p1index])
			p1b = numpy.array(path[p1index+1])
			p2a = numpy.array(path[p2index])
			p2b = numpy.array(path[p2index+1])  


			#choose random interpolation between points
			p1Vec = (p1b-p1a)
			p2Vec = (p2b-p2a)

			p1 = p1a + (p1Vec*random.random())
			p2 = p2a + (p2Vec*random.random())

			#TODO use extend to move to the two random points
			extender = self.Extend(p1,p2)
			if extender is not None:
				if self.ComputeDistance(extender,p2) < .01:
					#TODO get new path
					#print('shortening path now')
					badIndeces = [p1index+1,p2index]
					if (badIndeces[1]-badIndeces[0]) != 0:
						path[badIndeces[0]:badIndeces[1]] = []
					else:
						path[badIndeces[0]] = []
					path[badIndeces[0]] = p1
					path.insert(badIndeces[0]+1,p2)
					#print('new path created')

		# repeate until timeout
			current_time = time.time()-start_time
			i = 0
			
		totalDistance = 0
		#TODO: Calculate current path distance
		for i in range(0,len(path)-1):
			curDistance = self.ComputeDistance(path[i],path[i+1])
			totalDistance += curDistance
		print(totalDistance)    
		return path
