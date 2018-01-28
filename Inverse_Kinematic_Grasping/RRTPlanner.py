import numpy
from RRTTree import RRTTree
import time
class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):

        start_time = time.time()
        #establish the start tree
        tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        
        #plan.append(start_config)
        #plan.append(goal_config)

        # Kai's implementation
        while(1):           
            #import IPython
            #IPython.embed()
            #drop one valid random point that only connect with start tree
            point_drop = self.planning_env.GenerateRandomConfiguration()
            index, near_point = tree.GetNearestVertex(point_drop)
            point_chosen_from_s = self.planning_env.Extend(tree.vertices[index],point_drop)
            point_chosen_from_g = self.planning_env.Extend(goal_config,point_drop)

            #Check whether the point_drop can be directly used for connecting both

            if(point_chosen_from_s == None or point_chosen_from_g == None): continue

            dist_s = self.planning_env.ComputeDistance(point_chosen_from_s, point_drop)
            dist_g = self.planning_env.ComputeDistance(point_chosen_from_g,point_drop)

            if (dist_s <= epsilon and dist_g <= epsilon):
                point_addon_s_final = tree.AddVertex(point_chosen_from_s)
                tree.AddEdge(index,point_addon_s_final)
                point_addon_g_final = tree.AddVertex(point_chosen_from_g)
                tree.AddEdge(point_addon_s_final,point_addon_g_final)
                #self.planning_env.PlotEdge(near_point,point_chosen_from_s)
                #self.planning_env.PlotEdge(point_chosen_from_g,goal_config)
                break

            #If point_drop can only connect to start tree but not connect to goal point
            elif (dist_s <= epsilon and dist_g > epsilon):
                point_addon_s = tree.AddVertex(point_chosen_from_s)
                tree.AddEdge(index,point_addon_s)
                #self.planning_env.PlotEdge(near_point,point_chosen_from_s)

            #If point_drop can connect either the start tree or the goal point
            elif (dist_s > epsilon and dist_g > epsilon):
                point_addon_s = tree.AddVertex(point_chosen_from_s)
                tree.AddEdge(index,point_addon_s)
                #self.planning_env.PlotEdge(near_point,point_chosen_from_s)

        total_time = time.time() - start_time;
        # Find the path        
        path_start_tree = self.find_path(tree, 0, point_addon_g_final)
        path_start_tree.reverse()
        for i in path_start_tree:
            plan.append(i)
        plan.append(goal_config)

        dist_plan = self.planning_env.ComputePathLength(plan)
        print "total plan distance"
        print dist_plan
        
        print "total vertice in tree "
        print len(tree.vertices)
        # end of implement
        print "total plan time = "
        print total_time
        print " "
        return plan


    def find_path(self, tree, start_id, end_id): #[start_id, end_id)
        id_next_v = end_id
        path = []
        while(id_next_v != start_id):
            id_next_v = tree.edges[id_next_v]
            path.append(tree.vertices[id_next_v])
        return path

        
