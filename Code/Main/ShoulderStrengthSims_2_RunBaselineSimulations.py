# -*- coding: utf-8 -*-
"""
Created on Fri Jun  5 16:16:04 2020

Author:
    Aaron Fox
    Centre for Sport Research
    Deakin University
    
This code runs the movement simulations with the baseline model. The simulation
results from the baseline model will serve as starting guesses for the same movement
tasks using the altered muscle strength models.

The mesh intervals used in the following simulations are based on the grid refinement
approach used in an existing paper simulating the same movements:
    
    Fox AS, Gill SD, Bonacci J, Page RS (2020). Simulating the impact of glenohumeral
    capsulorrhaphy on movement kinematics and muscle function in activities of
    daily living. bioRxiv, doi: https://doi.org/10.1101/2020.06.02.130880

Movement task options that can be simulated are currently:
    - 'ConcentricUpwardReach105'
        
"""

# %% Import packages

import opensim as osim
import os
import math

# %% Folder set up

#Set main directory
mainPath = os.getcwd()

#Set task name to be simulated
print('Select task to simulate:')
print('[1] Concetric Upward Reach 105')
taskNo = input('Enter number selection: ')
taskNo = int(taskNo)
if taskNo == 1:
    print('Concentric upward reach 105 task selected.')
    taskName = 'ConcentricUpwardReach105'
    meshInterval = 50
else:
    raise ValueError('No tasks match the input number')
    
#Create space to store results
resultsPath = mainPath+'\\..\\..\\SimulationResults'
os.chdir(resultsPath)
if not os.path.isdir(taskName): #check if directory already exists
    os.mkdir(taskName)

#Set task results directory
taskPath = resultsPath+'\\'+taskName

# %% Model set up

#Navigate to model directory
modelPath = mainPath+'\\..\\..\\ModelFiles'
os.chdir(modelPath)

#Add geometry directory
osim.ModelVisualizer.addDirToGeometrySearchPaths(modelPath+'\\Geometry')

# %% Simulation set up

#Create the Moco study
study = osim.MocoStudy()

#Initialise the problem
problem = study.updProblem()

#Load the baseline model
osimModel = osim.Model(modelPath+'\\BaselineModel.osim')

#Lock the thorax joints of the model to make this a shoulder only movement
osimModel.updCoordinateSet().get('thorax_tilt').set_locked(True)
osimModel.updCoordinateSet().get('thorax_list').set_locked(True)
osimModel.updCoordinateSet().get('thorax_rotation').set_locked(True)
osimModel.updCoordinateSet().get('thorax_tx').set_locked(True)
osimModel.updCoordinateSet().get('thorax_ty').set_locked(True)
osimModel.updCoordinateSet().get('thorax_tz').set_locked(True)

#Add a 1kg mass for the reaching tasks
if 'Reach' in taskName:
    #Get hand mass and calculate added 1kg value
    newHandMass = osimModel.getBodySet().get('hand_r').getMass() + 1
    #Set new hand mass
    osimModel.getBodySet().get('hand_r').setMass(newHandMass)

#Add relevant torque actuators to each degree of freedom
# addCoordinateActuator(osimModel,'elv_angle',1.0,[1,-1],'_reserve')

##### TODO: add this to an osimHelperFunction script instead of being here...

# Define function to use for adding coordinate actuators
def addCoordinateActuator(modelObject = None, coordinate = None, optForce = 1.0,
                          controlLevel = [math.inf,math.inf*-1], appendStr = '_torque'):
    
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
        
    


# %% ----- End of ShoulderStrengthSims_2_RunBaselineSimulations.py ----- %% #
