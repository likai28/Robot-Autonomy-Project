import numpy 
import matplotlib.pyplot as pl
import time

class SimpleEnvironment(object):
    
    def __init__(self, herb):
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5.], [5., 5.]]

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.0], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        # goal sampling probability
        self.p = 0.0

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        
    def GenerateRandomConfiguration(self):
        config = [0] * 2;
        lower_limits, upper_limits = self.boundary_limits
        
        #
        # TODO: Generate and return a random configuration
        #
        #Peter: just random config in the limitations
        config[0] = lower_limits[0] + (upper_limits[0]-lower_limits[0]) * numpy.random.random_sample()
        config[1] = lower_limits[1] + (upper_limits[1]-lower_limits[1]) * numpy.random.random_sample()
        

        return numpy.array(config)

    def ComputeDistance(self, start_config, end_config):
        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        # 
        #Peter: calculate the Euclidean distance of the two config
        assert ( len(start_config) == 2 and len(end_config) == 2)
        distance =pow(  pow(start_config[0] - end_config[0],2) + pow(start_config[1] - end_config[1],2) , 0.5)

        return distance

    def Extend(self, start_config, end_config):
        
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #The function should return
        #either None or a configuration such that the linear interpolation between the start configuration and
        #this configuration is collision free and remains inside the environment boundaries.
        #self.boundary_limits = [[-5., -5.], [5., 5.]]
        collision = False
        outside = False
        lower_limits, upper_limits = self.boundary_limits
        resolution = 1000;
        move_dir = [float(end_config[0]-start_config[0])/float(resolution), float(end_config[1]-start_config[1])/float(resolution)];
        position_unchange = self.robot.GetTransform();
        for i in range(resolution+1):  
            #get the robot position
            position = self.robot.GetTransform()
            position[0][3] = start_config[0] + i*move_dir[0];
            position[1][3] = start_config[1] + i*move_dir[1];
            self.robot.SetTransform(position);
            if(self.robot.GetEnv().CheckCollision(self.robot)): 
                collision = True
                #print position
                #print "Detected collision!! Will return last safe step"
                #print 
            if( upper_limits[0] < position[0][3] or position[0][3] < lower_limits[0] or upper_limits[1] < position[1][3] or position[1][3] < lower_limits[1]): 
                outside = True
                #print position
                #print "Detected outside of bound!! Will return last safe step"
            self.robot.SetTransform(position_unchange);
        #check the collision == not touch the table && inside the boundary
            if(collision or outside):
                if(i >=1):
                    valid_config = [0,0]
                    valid_config[0] = start_config[0] + (i-1)* move_dir[0]
                    valid_config[1] = start_config[1] + (i-1)* move_dir[1]
                    return valid_config
                else: return None
        #No interpolate with collision and outside
        
        return end_config
                #return NULL

    def ShortenPath(self, path, timeout=5.0):
        
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        #
        #Brad
	#print "Start shortening"
	prev_path = list(path)
	prev_len = self.ComputePathLength(path)
	print "Current path length is %f" % prev_len
	elapsed_time = 0
	start_time = time.time();
	while elapsed_time < timeout:
		end_time = time.time();
		elapsed_time = end_time - start_time
		#print path_short
		if len(prev_path) < 3:
			print "Shortened path length is %f" % self.ComputePathLength(prev_path)
			return prev_path
		else:
			curr_path = list(path)
			for attempt in range(len(path)*len(path)):
				#print attempt
				init_idx = numpy.random.randint(0,len(curr_path)-2)
				end_idx = numpy.random.randint(init_idx+2,len(curr_path))
				init_ms = curr_path[init_idx]
				end_ms = curr_path[end_idx]
				result_ms = self.Extend(init_ms,end_ms)
				if numpy.array_equal(end_ms,result_ms):
					for elem in range(init_idx+1,end_idx):
						del curr_path[init_idx+1]
			curr_len = self.ComputePathLength(curr_path)
			if curr_len < prev_len:
				prev_path = curr_path
				prev_len = curr_len
	print "Shortened path length is %f" % self.ComputePathLength(prev_path)
        return prev_path
	
    def ComputePathLength(self, path):
	#Helper function to compute path length
	length = 0
	for milestone in range(1,len(path)):
		#print path[milestone-1], path[milestone]
		distance = self.ComputeDistance(numpy.array(path[milestone-1]),numpy.array(path[milestone]))
		#print distance
		length += distance
	return length


    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()
