#!/usr/bin/env python

PACKAGE_NAME = 'hw1'

# Standard Python Imports
import os
import copy
import time
import math
import numpy as np
np.random.seed(0)
import scipy

# OpenRAVE
import openravepy
#openravepy.RaveInitialize(True, openravepy.DebugLevel.Debug)
  

curr_path = os.getcwd()
relative_ordata = '/models'
ordata_path_thispack = curr_path + relative_ordata


#this sets up the OPENRAVE_DATA environment variable to include the files we're using
openrave_data_path = os.getenv('OPENRAVE_DATA', '')
openrave_data_paths = openrave_data_path.split(':')
if ordata_path_thispack not in openrave_data_paths:
  if openrave_data_path == '':
      os.environ['OPENRAVE_DATA'] = ordata_path_thispack
  else:
      datastr = str('%s:%s'%(ordata_path_thispack, openrave_data_path))
      os.environ['OPENRAVE_DATA'] = datastr

#set database file to be in this folder only
relative_ordatabase = '/database'
ordatabase_path_thispack = curr_path + relative_ordatabase
os.environ['OPENRAVE_DATABASE'] = ordatabase_path_thispack

#get rid of warnings
openravepy.RaveInitialize(True, openravepy.DebugLevel.Fatal)
openravepy.misc.InitOpenRAVELogging()



class RoboHandler:
  def __init__(self):
    self.openrave_init()
    self.problem_init()

    #order grasps based on your own scoring metric
    self.order_grasps()

    #order grasps with noise
    self.order_grasps_noisy()


  # the usual initialization for openrave
  def openrave_init(self):
    self.env = openravepy.Environment()
    self.env.SetViewer('qtcoin')
    self.env.GetViewer().SetName('HW1 Viewer')
    self.env.Load('models/%s.env.xml' %PACKAGE_NAME)
    # time.sleep(3) # wait for viewer to initialize. May be helpful to uncomment
    self.robot = self.env.GetRobots()[0]
    self.manip = self.robot.GetActiveManipulator()
    self.end_effector = self.manip.GetEndEffector()

  # problem specific initialization - load target and grasp module
  def problem_init(self):
    self.target_kinbody = self.env.ReadKinBodyURI('models/objects/champagne.iv')
    #self.target_kinbody = self.env.ReadKinBodyURI('models/objects/winegoblet.iv')
    #self.target_kinbody = self.env.ReadKinBodyURI('models/objects/black_plastic_mug.iv')

    #change the location so it's not under the robot
    T = self.target_kinbody.GetTransform()
    T[0:3,3] += np.array([0.5, 0.5, 0.5])
    self.target_kinbody.SetTransform(T)
    self.env.AddKinBody(self.target_kinbody)

    # create a grasping module
    self.gmodel = openravepy.databases.grasping.GraspingModel(self.robot, self.target_kinbody)
    
    # if you want to set options, e.g. friction
    options = openravepy.options
    options.friction = 0.1
    if not self.gmodel.load():
      self.gmodel.autogenerate(options)

    self.graspindices = self.gmodel.graspindices
    self.grasps = self.gmodel.grasps

  
  # order the grasps - call eval grasp on each, set the 'performance' index, and sort
  def order_grasps(self):
    self.grasps_ordered = self.grasps.copy() #you should change the order of self.grasps_ordered
    for grasp in self.grasps_ordered:
      grasp[self.graspindices.get('performance')] = self.eval_grasp(grasp)
    
    # sort!
    order = np.argsort(self.grasps_ordered[:,self.graspindices.get('performance')[0]])
    order = order[::-1]
    self.grasps_ordered = self.grasps_ordered[order]

  
  # order the grasps - but instead of evaluating the grasp, evaluate random perturbations of the grasp 
  def order_grasps_noisy(self):
    orig_grasps = self.grasps_ordered.copy() #you should change the order of self.grasps_ordered_noisy
    #orig_grasps = orig_grasps2[0:10]
    print orig_grasps
    rand_graspindices=self.graspindices
    for grasp in orig_grasps:
      grasp_score=0
      total_distance=0
      for j in range(10):
        rand_grasp=self.sample_random_grasp(grasp)
        dist = np.linalg.norm(grasp[self.graspindices['igrasppos']]-rand_grasp[self.graspindices['igrasppos']])
        dist = dist + np.linalg.norm(grasp[self.graspindices['igraspdir']]-rand_grasp[self.graspindices['igraspdir']]) 
        dist = dist + np.linalg.norm(grasp[self.graspindices['igrasproll']]-rand_grasp[self.graspindices['igrasproll']]) 
        grasp_score=grasp_score + (self.eval_grasp(rand_grasp)/dist)
        total_distance=total_distance+(1/dist)
      grasp[rand_graspindices.get('performance')]=grasp_score/total_distance
    order = np.argsort(orig_grasps[:,rand_graspindices.get('performance')[0]])
    order = order[::-1]
    self.grasps_ordered_noisy = orig_grasps[order]
    #TODO set the score with your evaluation function (over random samples) and sort


  # function to evaluate grasps
  # returns a score, which is some metric of the grasp
  # higher score should be a better grasp
  def eval_grasp(self, grasp):
    with self.robot:
      #contacts is a 2d array, where contacts[i,0-2] are the positions of contact i and contacts[i,3-5] is the direction
      try:
        contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=False)

        obj_position = self.gmodel.target.GetTransform()[0:3,3]   
        
        num_contacts = len(contacts)
        # for each contact
        G = np.zeros([6, num_contacts]) #the wrench matrix

        for idx, c in enumerate(contacts):
          pos = c[0:3] - obj_position
          # print pos
          dir = -c[3:] #this is already a unit vector
          
          #TODO fill G
          G[0:3,idx] = dir.T
          G[3:6,idx] = np.cross(pos,dir).T
        
        #TODO use G to compute scrores as discussed in class
        U, s, V = np.linalg.svd(G, full_matrices=True)
        
        # print U.shape, s.shape, V.shape
        # Metric 1 minimum singular value
        if s.all() >= 0:
          m1 = np.amin(s)
        else:
          m1 = 0

        # Metric 2: volume of the ellipsoid  
        if np.linalg.det(np.dot(G,G.T)) >= 0:
          m2 = np.sqrt(np.linalg.det(np.dot(G,G.T)))
        else:
          m2 = 0;
        
        #Metric 3: Isotropy
        sigma_min = np.amin(s)
        sigma_max = np.amax(s)

        if sigma_max > 0:
          m3 = sigma_min / sigma_max
        else:
          m3 = 0

        # print U.shape, s.shape, V.shape
        #Need to come up with weights for each of the metric for evaluation function
        # print 'm1: ' + repr(m1) + '\nm2: ' + repr(m2) + '\nm3: ' + repr(m3)
        # rationale, m1 and m3 are highly correlated so I bring them to about the same order of magnitude
        # m2, is very small and boosted to about the same order of magnitude as well
        if np.linalg.matrix_rank(G) == 6:
          return 100*m1+50000*m2+1000*m3 
        else:
          return 0


      except openravepy.planning_error,e:
        #you get here if there is a failure in planning
        #example: if the hand is already intersecting the object at the initial position/orientation
        return  0.00 # TODO you may want to change this
      
      #heres an interface in case you want to manipulate things more specifically
      #NOTE for this assignment, your solutions cannot make use of graspingnoise
#      self.robot.SetTransform(np.eye(4)) # have to reset transform in order to remove randomness
#      self.robot.SetDOFValues(grasp[self.graspindices.get('igrasppreshape')], self.manip.GetGripperIndices())
#      self.robot.SetActiveDOFs(self.manip.GetGripperIndices(), self.robot.DOFAffine.X + self.robot.DOFAffine.Y + self.robot.DOFAffine.Z)
#      self.gmodel.grasper = openravepy.interfaces.Grasper(self.robot, friction=self.gmodel.grasper.friction, avoidlinks=[], plannername=None)
#      contacts, finalconfig, mindist, volume = self.gmodel.grasper.Grasp( \
#            direction             = grasp[self.graspindices.get('igraspdir')], \
#            roll                  = grasp[self.graspindices.get('igrasproll')], \
#            position              = grasp[self.graspindices.get('igrasppos')], \
#            standoff              = grasp[self.graspindices.get('igraspstandoff')], \
#            manipulatordirection  = grasp[self.graspindices.get('imanipulatordirection')], \
#            target                = self.target_kinbody, \
#            graspingnoise         = 0.0, \
#            forceclosure          = True, \
#            execute               = False, \
#            outputfinal           = True, \
#            translationstepmult   = None, \
#            finestep              = None )



  # given grasp_in, create a new grasp which is altered randomly
  # you can see the current position and direction of the grasp by:
  # grasp[self.graspindices.get('igrasppos')]
  # grasp[self.graspindices.get('igraspdir')]
  def sample_random_grasp(self, grasp_in):
    grasp = grasp_in.copy()

    number_to_change=np.random.randint(0,7,1)
    print number_to_change
    noise=np.zeros((1,3),dtype=float)
    if number_to_change < 3:
      noise[0,number_to_change]=(np.random.normal(0,0.01,1))
      pos_orig = grasp[self.graspindices['igrasppos']]
      grasp[self.graspindices['igrasppos']]= pos_orig+noise
      print pos_orig
      print 'rand pos'+str(grasp[self.graspindices['igrasppos']])
    elif number_to_change < 6:
      dir_orig = grasp[self.graspindices['igraspdir']]
      noise[0,number_to_change-3]=(np.random.normal(0.05,0.01,1))
      grasp[self.graspindices['igraspdir']]=dir_orig+noise
      print dir_orig
      print 'rand dir'+str(grasp[self.graspindices['igraspdir']])
    else:
      RAND_ANGLE_SIGMA = np.pi/24 #TODO you may want to change this
      roll_orig = grasp[self.graspindices['igrasproll']]
      grasp[self.graspindices['igrasproll']] = roll_orig+(np.random.uniform(-1*RAND_ANGLE_SIGMA,RAND_ANGLE_SIGMA,1))
      print roll_orig
      print 'rand roll'+str(grasp[self.graspindices['igrasproll']])

    #TODO set the direction and roll to be random
    return grasp


  #displays the grasp
  def show_grasp(self, grasp, delay=1.5):
    with openravepy.RobotStateSaver(self.gmodel.robot):
      with self.gmodel.GripperVisibility(self.gmodel.manip):
        time.sleep(0.1) # let viewer update?
        try:
          with self.env:
            contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=True)
            #if mindist == 0:
            #  print 'grasp is not in force closure!'
            contactgraph = self.gmodel.drawContacts(contacts) if len(contacts) > 0 else None
            self.gmodel.robot.GetController().Reset(0)
            self.gmodel.robot.SetDOFValues(finalconfig[0])
            self.gmodel.robot.SetTransform(finalconfig[1])
            self.env.UpdatePublishedBodies()
            time.sleep(delay)
        except openravepy.planning_error,e:
          print 'bad grasp!',e

if __name__ == '__main__':
  robo = RoboHandler()

  import IPython
  IPython.embed()

  #time.sleep(10000) #to keep the openrave window open

  
