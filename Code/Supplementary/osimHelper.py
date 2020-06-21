# -*- coding: utf-8 -*-
"""
Created on Fri Jun  5 21:02:40 2020

Author:
    Aaron Fox
    Centre for Sport Research
    Deakin University
    
Code includes a series of covenience functions for calls in the main code.
Functions defined here include:
    
    addCoordinateActuator   adds a coordinate actuator to the specified coordinate
                            with the specified parameters
    
    addMarkerEndPoints      adds marker end point goals relevant to the specified
                            task name to a Moco Problem

"""

# %% Import packages

import opensim as osim
import math
import pandas as pd

# %% addCoordinateActuator

def addCoordinateActuator(modelObject = None, coordinate = None, optForce = 1.0,
                          controlLevel = [float('inf'),float('-inf')], appendStr = '_torque'):
    
    # Convenience function for adding a coordinate actuator to model
    #
    # Input:    modelObject - Opensim model object to add actuator to
    #           coordinate - string of coordinate name for actuator
    #           optForce - value for actuators optimal force
    #           controlLevel - [x,y] values for max and min control
    #           appendStr - string to append to coordinate name in setting actuator name
    
    #Check for model object
    if modelObject is None:
        raise ValueError('A model object must be included')
    
    #Check for coordinate
    if coordinate is None:
        raise ValueError('A coordinate string must be specified')
        
    #Create actuator
    actu = osim.CoordinateActuator()
    
    #Set name
    actu.setName(coordinate+appendStr)

    #Set coordinate for actuator
    actu.setCoordinate(modelObject.getCoordinateSet().get(coordinate))
    
    #Set optimal force
    actu.setOptimalForce(optForce)
    
    #Set control levels    
    actu.setMaxControl(controlLevel[0])
    actu.setMinControl(controlLevel[1])
    
    #Add actuator to models forceset
    modelObject.updForceSet().append(actu)

# %% addMarkerEndPoints

def addMarkerEndPoints(taskName = None, mocoProblem = None, modelObject = None):
        
    # Convenience function for adding a coordinate actuator to model
    #
    # Input:    taskName - string of relevant task name options
    #           mocoProblem - MocoProblem object to add goals to
    #           modelObject - Opensim model object to add actuator to
    
    #Check for appropriate inputs
    if taskName is None or mocoProblem is None or modelObject is None:
        raise ValueError('All three input arguments are needed for addMarkerEndPoints function')
    
    if taskName == 'ConcentricUpwardReach105':
        
        #Get the desired end point of the movement. This will be at a point 15
        #degrees above the shoulder at a distance of 200% of forearm length.
        #(note there is no prescribed distance in the Vidt paper)

        #Get the position of the shoulder joint centre. Note that the 1 corresponds
        #to the humphant_offset frame. This command also transforms it to the
        #ground frame.
        modelObject_state = modelObject.initSystem()
        SJC_ground = modelObject.getJointSet().get('shoulder0').get_frames(1).getPositionInGround(modelObject_state)

        #Calculate the distance of the forearm (i.e. between the elbow and wrist
        #joint centre).

        #Get the position of the joint centres. Joint 1 corresponds to ulna offset
        #frame for elbow and joint 0 the radius offset for the radius hand joint
        EJC_ground = modelObject.getJointSet().get('elbow').get_frames(1).getPositionInGround(modelObject_state)
        WJC_ground = modelObject.getJointSet().get('radius_hand_r').get_frames(0).getPositionInGround(modelObject_state)

        #Calculate the distance between the joint centres
        elbow = [EJC_ground.get(0),EJC_ground.get(1),EJC_ground.get(2)]
        wrist = [WJC_ground.get(0),WJC_ground.get(1),WJC_ground.get(2)]
        FA_length = (((wrist[0]-elbow[0])**2) + ((wrist[1]-elbow[1])**2) + ((wrist[2]-elbow[2])**2))**0.5

        #Calculate the position 200% forearm length in front of the shoulder. In
        #front is represented by positive X
        inFrontPoint = [SJC_ground.get(0)+(FA_length*2),SJC_ground.get(1),SJC_ground.get(2)]

        #Calculate how far above this point is needed to generate a 15 degree angle
        #above the level of the shoulder joint. Calculate this using a 2D triangle
        #encompassing the X and Y axis

        #Calculate horizontal distance from shoulder to in front point
        Xdist = (FA_length*2) - SJC_ground.get(0)
        #Set angle to calculate height with
        theta = math.radians(15)
        #Calculate height of triangle
        Ydist = math.tan(theta) * Xdist

        #Prescribe upward reach point
        upwardReachPoint = [inFrontPoint[0],inFrontPoint[1]+Ydist,inFrontPoint[2]]

        #Create a marker end point cost for the reach position. Need to use the
        #markers on both sides of the wrist and the top of the hand to ensure that
        #the hand is placed level and palmar side down at the end - as such, need
        #to create markers end points for each of these.

        #Identify the distance between the two wrist markers
        RS = modelObject.getMarkerSet().get('RS').getLocationInGround(modelObject_state)
        US = modelObject.getMarkerSet().get('US').getLocationInGround(modelObject_state)
        RS = [RS.get(0),RS.get(1),RS.get(2)]
        US = [US.get(0),US.get(1),US.get(2)]
        wristWidth = (((RS[0]-US[0])**2) + ((RS[1]-US[1])**2) + ((RS[2]-US[2])**2))**0.5

        #Add and subtract half of the wrist distance from the original marker end
        #point along the Z-axis to get the proposed end points for the markers. It
        #is positive Z in the ground frame for the ulna marker and negative Z for
        #the radius marker
        US_endLoc = osim.Vec3(upwardReachPoint[0],upwardReachPoint[1],upwardReachPoint[2]+(wristWidth/2))
        RS_endLoc = osim.Vec3(upwardReachPoint[0],upwardReachPoint[1],upwardReachPoint[2]-(wristWidth/2))

        #Measure the distance from the wrist joint centre to the wri_out marker for
        #prescribing where the hand needs to go.
        wri_out = modelObject.getMarkerSet().get('wri_out').getLocationInGround(modelObject_state)
        wri_out = [wri_out.get(0),wri_out.get(1),wri_out.get(2)]
        wrist = [WJC_ground.get(0),WJC_ground.get(1),WJC_ground.get(2)]
        wristHeight = (((wri_out[0]-wrist[0])**2) + ((wri_out[1]-wrist[1])**2) + ((wri_out[2]-wrist[2])**2))**0.5

        #Add the wirst height amount along the y-axis from the proposed reach point
        #to get the point where the wri_out marker needs to go
        W_endLoc = osim.Vec3(upwardReachPoint[0],upwardReachPoint[1]+wristHeight,upwardReachPoint[2])

        #Add the end point costs equally weighted to contribute 50# to the problem
        endPointCost1 = osim.MocoMarkerFinalGoal('RS_endPoint',5)
        endPointCost1.setPointName('/markerset/RS')
        endPointCost1.setReferenceLocation(RS_endLoc)
        endPointCost2 = osim.MocoMarkerFinalGoal('US_endPoint',5)
        endPointCost2.setPointName('/markerset/US')
        endPointCost2.setReferenceLocation(US_endLoc)
        endPointCost3 = osim.MocoMarkerFinalGoal('W_endPoint',5)
        endPointCost3.setPointName('/markerset/wri_out')
        endPointCost3.setReferenceLocation(W_endLoc)

        #Add the end point cost along with an effort cost.
        mocoProblem.addGoal(endPointCost1)
        mocoProblem.addGoal(endPointCost2)
        mocoProblem.addGoal(endPointCost3)
        
    # elif taskName is 'OtherTasks...'
        
# %% addTaskBounds

def addTaskBounds(taskName = None, mocoProblem = None, modelObject = None, 
                  taskBoundsElv = None, taskBoundsRot = None, taskBoundsAng = None):
        
    # Convenience function for adding a coordinate actuator to model
    #
    # Input:    taskName - string of relevant task name options
    #           mocoProblem - MocoProblem object to add bounds to
    #           modelObject - Opensim model object to add actuator to
    #           taskBoundsElv - pandas dataframe containing shoulder elevation angle task bounds
    #           taskBoundsRot - pandas dataframe containing shoulder rotation task bounds
    #           taskBoundsAng - pandas dataframe containing elevation angle task bounds
    
    #Check for appropriate inputs
    if taskName is None or mocoProblem is None or modelObject is None \
        or taskBoundsElv is None or taskBoundsRot is None or taskBoundsAng is None:
        raise ValueError('All six input arguments are needed for addTaskBounds function')
        
    #Set the relevant task bounds
    
    if taskName == 'ConcentricUpwardReach105':
        
        #Set task name from dataframe
        dfTaskName = 'UpwardReach105'
            
        #Shoulder elevation
                
        #Set state name
        stateName = '/jointset/'+modelObject.getCoordinateSet().get('shoulder_elv').getJoint().getName()+'/'+modelObject.getCoordinateSet().get('shoulder_elv').getName()+'/value'
        #Set overall task bounds
        taskBounds = [math.radians(taskBoundsElv.loc[[dfTaskName],'Min'][0]),
                      math.radians(taskBoundsElv.loc[[dfTaskName],'Max'][0])]
        #Set bounds for end point
        endBounds = [math.radians(taskBoundsElv.loc[[dfTaskName],'ConcentricLowerBound'][0]),
                     math.radians(taskBoundsElv.loc[[dfTaskName],'ConcentricUpperBound'][0])]
        #Set bounds in problem        
        mocoProblem.setStateInfo(stateName,taskBounds,math.radians(0),endBounds)
        
        #Shoulder rotation
                
        #Set state name
        stateName = '/jointset/'+modelObject.getCoordinateSet().get('shoulder_rot').getJoint().getName()+'/'+modelObject.getCoordinateSet().get('shoulder_rot').getName()+'/value'
        #Set overall task bounds
        taskBounds = [math.radians(taskBoundsRot.loc[[dfTaskName],'Min'][0]),
                      math.radians(taskBoundsRot.loc[[dfTaskName],'Max'][0])]
        #Set bounds for end point
        endBounds = [math.radians(taskBoundsRot.loc[[dfTaskName],'ConcentricLowerBound'][0]),
                     math.radians(taskBoundsRot.loc[[dfTaskName],'ConcentricUpperBound'][0])]
        #Set bounds in problem        
        mocoProblem.setStateInfo(stateName,taskBounds,math.radians(0),endBounds)
        
        #Elevation angle
                
        #Set state name
        stateName = '/jointset/'+modelObject.getCoordinateSet().get('elv_angle').getJoint().getName()+'/'+modelObject.getCoordinateSet().get('elv_angle').getName()+'/value'
        #Set overall task bounds
        taskBounds = [math.radians(taskBoundsAng.loc[[dfTaskName],'Min'][0]),
                      math.radians(taskBoundsAng.loc[[dfTaskName],'Max'][0])]
        #Set bounds for end point
        endBounds = [math.radians(taskBoundsAng.loc[[dfTaskName],'ConcentricLowerBound'][0]),
                     math.radians(taskBoundsAng.loc[[dfTaskName],'ConcentricUpperBound'][0])]
        #Set bounds in problem        
        mocoProblem.setStateInfo(stateName,taskBounds,math.radians(0),endBounds)
        
        #Elbow flexion
        
        #Set state name
        stateName = '/jointset/'+modelObject.getCoordinateSet().get('elbow_flexion').getJoint().getName()+'/'+modelObject.getCoordinateSet().get('elbow_flexion').getName()+'/value'
        #Set minimum based on min joint coordinate limit
        mn = modelObject.getCoordinateSet().get('elbow_flexion').getRangeMin()
        #Set maximum based on coordinate limit or 90 degrees for reaching tasks
        if 'Reach' in taskName:
            #Limit to 90 degrees
            mx = math.radians(90)
        else:
            #Set to joint limit
            mx = modelObject.getCoordinateSet().get('elbow_flexion').getRangeMax()
        #Set overall task bounds
        taskBounds = [mn,mx]
        #Set bounds in problem        
        mocoProblem.setStateInfo(stateName,taskBounds,math.radians(0),[])
        
        #Forearm
        
        #Set state name
        stateName = '/jointset/'+modelObject.getCoordinateSet().get('pro_sup').getJoint().getName()+'/'+modelObject.getCoordinateSet().get('pro_sup').getName()+'/value'
        #Set minimum based on coordinate limit or -10 degrees for reaching tasks
        #This limits over supination in the reaching
        if 'Reach' in taskName:
            #Limit to 90 degrees
            mn = math.radians(-10)
        else:
            #Set to joint limit
            mn = modelObject.getCoordinateSet().get('elbow_flexion').getRangeMin()
        #Set minimum based on min joint coordinate limit
        mx = modelObject.getCoordinateSet().get('elbow_flexion').getRangeMax()
        #Set overall task bounds
        taskBounds = [mn,mx]
        #Set bounds in problem        
        mocoProblem.setStateInfo(stateName,taskBounds,math.radians(0),[])
            
    # elif taskName is 'OtherTasks...'
    
    #Set velocity bounds for all model coordinates to start and end at rest
    mocoProblem.setStateInfoPattern('/jointset/.*/speed',[-50.0,50.0],0.0,0.0)
    
    #Set muscle activation bounds for activation to start at min value
    mocoProblem.setStateInfoPattern('/forceset/.*/activation',[0.01,1.0],0.01,[])

# %% fixGuessFile

def fixGuessFile(guessFile = None, mocoSolver = None):
        
    # Convenience function for fixing a guess file that contains NaN's in it 
    # Some solution files seem to generate nan's in the last row
    # of the slack variables which generates a Casadi error when
    # attempting to use as a guess. To resolve this, a random
    # guess can be created (which seems to set the slacks to
    # zero), and be filled with the relevant data states,
    # controls, multipliers) from the existing solution.
    #
    # Input:    guessFile - string to path of guess file
    # 
    # Note: this function will only work if the guess file matches the inputs
    # to the solver exactly. If they don't match things won't work well...
    
    ##### TODO: this currently works for baseline sims, but may need some options if re-using in other processes...
    
    #Check for appropriate inputs
    if guessFile is None or mocoSolver is None:
        raise ValueError('A guess file and linked Moco Solver are needed in fixGuessFile')
        
    #Grab the file as a Moco tracjectory
    mocoTraj = osim.MocoTrajectory(guessFile)

    #Create random guess using the current solver
    randTraj = mocoSolver.createGuess()
    
    #Resample the randomly created guess if it doesn't match the guess file
    if randTraj.getNumTimes() != mocoTraj.getNumTimes():
        randTraj.resampleWithNumTimes(mocoTraj.getNumTimes())
    
    #Set time for random guess to original trajectory
    randTraj.setTime(mocoTraj.getTime())

    #Refresh states in random guess from the moco trajectory
    randTraj.setStatesTrajectory(mocoTraj.exportToStatesTable())

    #Refresh controls from moco trajectory
    controlNames = list(mocoTraj.getControlNames())
    #Reserve and torque actuators in this study are in the forceset, but were
    #components in the existing solution. Therefore, the control name has the 
    #forceset string in the current guess. This needs to be accounted for in
    #creating the new guess
    #Loop through and set the controls in the guess
    for cc in range(0,len(controlNames)-1):
        if '/forceset' not in controlNames[cc]:
            randTraj.setControl('/forceset'+controlNames[cc],mocoTraj.getControlMat(controlNames[cc]))
        else:
            randTraj.setControl(controlNames[cc],mocoTraj.getControlMat(controlNames[cc]))

    #Refresh multipliers from moco trajectory
    multiplierNames = list(mocoTraj.getMultiplierNames())
    for cc in range(0,len(multiplierNames)-1):
        randTraj.setMultiplier(multiplierNames[cc],mocoTraj.getMultiplierMat(multiplierNames[cc]))
        
    #Set the guess in the input solver
    mocoSolver.setGuess(randTraj)
    
    
# %%
    #...add function here...
    
    
