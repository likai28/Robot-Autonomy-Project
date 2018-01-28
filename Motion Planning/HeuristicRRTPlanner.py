import numpy
import random
from RRTTree import RRTTree

class HeuristicRRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
	self.probfloor = 1
	self.kvertices = 1
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        ftree = RRTTree(self.planning_env, start_config)

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
	costToCome = dict()
	heuristicCost = dict()
        costToCome[ftree.GetRootId()] = 0
	optcost = self.planning_env.ComputeDistance(start_config, goal_config)
	worstcost = 0
	heuristicCost[ftree.GetRootId()] = costToCome[ftree.GetRootId()] + optcost 
	
	# max edge length + a lot of code clean up
        maxDist = .5
	it = 0
        while True:
            sample = self.planning_env.GenerateRandomConfiguration()
            #vertices = ftree.GetNearestKVertices(sample, self.kvertices)
	    f_vid, vtx = ftree.GetNearestVertex(sample)
	    it+=1
	   # for (f_vid, vtx) in vertices: 
	    quality = 1 - (heuristicCost[f_vid] - optcost)/(worstcost-optcost)
	    quality = min(quality, self.probfloor)

	    r = random.random()
	#    print quality, r
	    if quality < r: continue 
	    
	    f_best = self.ExtendFromTree(vtx,sample,maxDist, epsilon)
	   
	    if f_best is not None:
                new_id = self.AddNode(ftree, f_vid, f_best)
		costToCome[new_id] = costToCome[f_vid] + self.planning_env.ComputeDistance(f_best, vtx)
		heuristicCost[new_id] = costToCome[new_id] + self.planning_env.ComputeDistance(f_best,goal_config)
		worstcost = max(heuristicCost[new_id], worstcost)
		r_best = self.planning_env.Extend(goal_config, f_best)
		if r_best is not None:
			dist = self.planning_env.ComputeDistance(f_best, r_best)	
			if(dist<epsilon):
			    g_id = self.AddNode(ftree, new_id, goal_config)
			    return (self.GeneratePlan(ftree, g_id), it)

    def ExtendFromTree(self, vtx, sample, maxDist,epsilon):
        vtx_sample_dist = self.planning_env.ComputeDistance(sample, vtx)
        if(vtx_sample_dist >= epsilon):
            sample = vtx + (sample - vtx)/vtx_sample_dist * min(vtx_sample_dist, maxDist)
            return self.planning_env.Extend(vtx, sample)
        return None

    def AddNode(self, tree, parent_id, cfg):
        new_id = tree.AddVertex(cfg)
        tree.AddEdge(parent_id, new_id)
        if(self.visualize):
            self.planning_env.PlotEdge(tree.vertices[parent_id], cfg)
        return new_id

    def GeneratePlan(self, ftree, f_vid):
        # build plan here
        plan = []
        cursor = f_vid
        while(cursor != ftree.GetRootId()):
            plan.append(ftree.vertices[cursor])
            cursor = ftree.edges[cursor]
        plan.append(ftree.vertices[cursor])
        plan.reverse()
        return plan
