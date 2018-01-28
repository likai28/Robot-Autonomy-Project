import heapq
import time
from Queue import PriorityQueue

class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict() 


    def Plan(self, start_config, goal_config):

        plan = []
        path = dict()
        
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the AStar planner
        #  The return comingFrom should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        # Create the closed set and the open set which contain the node ID
        closedSet = set()
        # openSet = PriorityQueue([])
        openSet = []

        # Give start and goal ID
        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        # plan.append(start_config)

        # Add the start id th the open node
        self.nodes[start_id] = 0
        h_cost = self.planning_env.ComputeHeuristicCost(start_id, goal_id)
        f_cost = self.nodes[start_id] + 5*h_cost #the f(n) or f(x) of the first node which is the start config
        #openSet[start_id]=f_cost
        # openSet.put((f_cost,h_cost,start_id))
        heapq.heappush(openSet,(f_cost,h_cost,start_id))
        


        while(openSet):

              
            x_id = heapq.heappop(openSet)[2]
            
            # x_id = openSet.get()[2]
            # print "x_id =",x_id

            if x_id in closedSet:
                continue

            # Return the comingFrom if the goal is reached
            if x_id == goal_id:
                plan = self.find_path(path,start_id,goal_id)
                return (plan, len(self.nodes))
            
            # Add this to the close set
            closedSet.add(x_id)
            # Create the list of the nearby nodes
            xNearby = self.planning_env.GetSuccessors(x_id)
            # print "Sccessor =",xNearby
            # Loop through all the nearby node and determine their f,g,h
            for y_id in xNearby:
                # See if y is already searched
                if y_id in closedSet:
                    continue

                #Calculate current g_cost which is the distance traveled from start to y, thus, g(y)=g(x)+dis(x,y)
                tentative_g_cost_y = self.nodes[x_id] + self.planning_env.ComputeDistanceBetweenIds(x_id, y_id)
		if(tentative_g_cost_y == float('inf')):
		    continue
                if self.visualize:
                    self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(x_id), self.planning_env.discrete_env.NodeIdToConfiguration(y_id))
                
                if self.nodes.has_key(y_id) and tentative_g_cost_y>=self.nodes[y_id]:
                    continue

                self.nodes[y_id]=tentative_g_cost_y
                h = self.planning_env.ComputeHeuristicCost(y_id, goal_id)
                f = self.nodes[y_id] + 5*h
                # print "y_id = " , y_id ,"f =",f, ", g =" , self.nodes[y_id], ", h =", f-self.nodes[y_id]

                # openSet.put((f,h,y_id))
                heapq.heappush(openSet,(f,h,y_id))
                path[y_id]=x_id
                # time.sleep(5)
                
            # time.sleep(5)
        return None
    #This help function is used to get the path in bfs
    def find_path(self, path, start_id, end_id): #[start_id, end_id)
        id_next_v = end_id
        path2 = []
        node_config = self.planning_env.discrete_env.NodeIdToConfiguration(id_next_v)
        path2.append(node_config)
        while(id_next_v != start_id):
            id_next_v = path[id_next_v]
            node_config = self.planning_env.discrete_env.NodeIdToConfiguration(id_next_v)
            path2.append(node_config)
        path2.reverse()
        return path2

    
