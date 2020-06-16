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
import pandas as pd

os.chdir('..\\Supplementary')
import osimHelper

os.chdir('..\\Main')

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
    meshInterval = 100
else:
    raise ValueError('No tasks match the input number')
    
#Create space to store results
resultsPath = mainPath+'\\..\\..\\SimulationResults'
os.chdir(resultsPath)
if not os.path.isdir(taskName): #check if directory already exists
    os.mkdir(taskName)

#Set task results directory
taskPath = resultsPath+'\\'+taskName

#Load task bounds

#Navigate to supporting data directory
os.chdir('..\\SupportingData')

#Load in dataframes
taskBoundsElv = pd.read_csv('shoulder_elv_bounds.csv', index_col = 'Task')
taskBoundsRot = pd.read_csv('shoulder_rot_bounds.csv', index_col = 'Task')
taskBoundsAng = pd.read_csv('elv_angle_bounds.csv', index_col = 'Task')

# %% Model set up

#Navigate to model directory
modelPath = mainPath+'\\..\\..\\ModelFiles'
os.chdir(modelPath)

#Add geometry directory
osim.ModelVisualizer.addDirToGeometrySearchPaths(modelPath+'\\Geometry')

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
#Loop through and add coordinate actuators
#Don't add anything for the thorax
for cc in range(0,osimModel.updCoordinateSet().getSize()):    
    #Get current coordinate name
    coordName = osimModel.updCoordinateSet().get(cc).getName()
    #Conditional statements for adding actuators
    if coordName == 'elbow_flexion':
        #Add an idealised torque actuator
        osimHelper.addCoordinateActuator(osimModel,coordName,75.0,
                                         [float('inf'),float('-inf')],'_torque')
    elif coordName == 'pro_sup':
        #Add an idealised torque actuator
        osimHelper.addCoordinateActuator(osimModel,coordName,30.0,
                                         [float('inf'),float('-inf')],'_torque')
    elif coordName == 'elv_angle':
        #Add a reserve torque actuator
        osimHelper.addCoordinateActuator(osimModel,coordName,1.0,
                                         [float('inf'),float('-inf')],'_reserve')
    elif coordName == 'shoulder_elv' or coordName == 'shoulder_rot':
        #Add a reserve torque actuator
        osimHelper.addCoordinateActuator(osimModel,coordName,1.0,
                                         [1.0,-1.0],'_reserve')

#Finalise model
osimModel.finalizeFromProperties()

#Something weird going on later if I try and call the model processor directly
#from the model object here (says the model has no subcomponents?). A workaround
#is to process print out the model and call it to the processor via its filename,
#and then get a variable to call the processed model.

#Print out the edited model
osimModel.printToXML('simModel.osim')

#Set up a model processor to configure model
modelProcessor = osim.ModelProcessor('simModel.osim')

#Convert model muscles to DeGrooteFregly type
modelProcessor.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())

#Append settings for muscle models
#Set to ignore tendon compliance
modelProcessor.append(osim.ModOpIgnoreTendonCompliance())
#Set to ignore conservative passive fiber forces
modelProcessor.append(osim.ModOpIgnorePassiveFiberForcesDGF())
#Set the fiber damping low to limit non-conservative passive forces
#This may also serve to limit the negative muscle forces that can happen
modelProcessor.append(osim.ModOpFiberDampingDGF(1e-05))
#Scale active force width of muscles
modelProcessor.append(osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))

#Process and get a variable to call the model
simModel = modelProcessor.process()

#Clean up the printed out model file from the directory
os.remove('simModel.osim')

# %% Simulation set up

#Create the Moco study
study = osim.MocoStudy()

#Initialise the problem
problem = study.updProblem()

#Set the model processor in the problem
problem.setModel(simModel)

#Navigate to guess directory
os.chdir('..\\GuessFiles')

#Set guess path
guessPath = os.getcwd()

#Set time bounds on the problem
##### NOTE: first iteration with end time bounds seemed to try and solve to the
##### the final end time bound (i.e. 1.0) rather than as fast as possible. Test
##### and see whether removing these fixes it.
# Set to a percentage of the final guess time to end
timeLB = 0.9*osim.Storage(taskName+'_StartingGuess.sto').getLastTime()
timeUB = 1.1*osim.Storage(taskName+'_StartingGuess.sto').getLastTime()
problem.setTimeBounds(osim.MocoInitialBounds(0.0),
                      osim.MocoFinalBounds(timeLB,timeUB))
# problem.setTimeBounds(0,[])

#Define marker end point goals
osimHelper.addMarkerEndPoints(taskName,problem,simModel)

#Define kinematic task bounds
osimHelper.addTaskBounds(taskName,problem,simModel,taskBoundsElv,taskBoundsRot,taskBoundsAng)

#Add a control cost to the problem
problem.addGoal(osim.MocoControlGoal('effort',1))

#Add a final time goal to the problem
problem.addGoal(osim.MocoFinalTimeGoal('time',1))

#Configure the solver
solver = study.initCasADiSolver()
solver.set_num_mesh_intervals(meshInterval)
solver.set_verbosity(2)
solver.set_optim_solver('ipopt')
solver.set_optim_convergence_tolerance(1e-3)
solver.set_optim_constraint_tolerance(1e-3)

#Set the guess to an existing solution for this task from existing work
#Note that this will be slightly off given the previous study included forces
#that approximated ligaments and joint capsule passive resistance, but should 
#still work as an OK start.

#Set guess in solver
#Note an accesory function is used here as some solution files seem to generate
#NaN's in the last row of the slack variables, which generates a Casadi error
#when attempting to use as a guess.
osimHelper.fixGuessFile(guessPath+'\\'+taskName+'_StartingGuess.sto',solver)

### nan's getting created in guess!!! happening in original created guess though
### has something to do with mesh interval I think

# %% Solve!

#Navigate to task results directory
os.chdir(taskPath)

#Set study name
study.setName('BaselineSim_'+taskName+'_'+str(meshInterval*2+1)+'nodes')

#Print setup file to directory
study.printToXML('BaselineSim_'+taskName+'_'+str(meshInterval*2+1)+'nodes.omoco')

#Run optimisation
baselineSolution = study.solve()


# if os.getenv('OPENSIM_USE_VISUALIZER') != '0':
#     study.visualize(baselineSolution)

# %% Re-run with JRF goals...

# %% ----- End of ShoulderStrengthSims_2_RunBaselineSimulations.py ----- %% #
