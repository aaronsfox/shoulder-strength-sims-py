# -*- coding: utf-8 -*-
"""
Created on Fri Jun  5 15:28:42 2020

Author:
    Aaron Fox
    Centre for Sport Research
    Deakin University
    
This code generates the models with adapated strength levels. Currently it generates
models with reduced (to 80% and 90%) and increased (to 110% and 120%) strength.

Not all model muscles strength are altered, with the following changed:
    - TRP1 and TRP2 together
    - TRP3 and TRP4 together
    - RMJ1, RMJ2 and RMN together
    - SRA1, SRA2 and SRA3 together
    - DELT1
    - DELT2
    - DELT3
    - SUPSP
    - INFSP and TMIN together
    - SUBSC
    - TMAJ
    - PECM1, PECM2 and PECM3 together
    - LAT
    - CORB
    
"""

# %% Import packages

import opensim as osim
import os

# %% Create models

#Set main directory
mainPath = os.getcwd()

#Create path to baseline model

#Add path to model directory
modelPath = mainPath+'\..\..\ModelFiles'
#Add geometry to visualiser
osim.ModelVisualizer.addDirToGeometrySearchPaths(modelPath+'\Geometry')
#Set the baseline model filename
baseModelFileName = modelPath+'\BaselineModel.osim'

#Set scale factors to weaken muscles by
scaleFactors = [0.8,0.9,1.1,1.2]

#Create a list of muscle(s) to alter
muscToAlter = [['TRP1','TRP2'],
    ['TRP3','TRP4'],
    ['SRA1','SRA2','SRA3'],
    ['DELT1'],
    ['DELT2'],
    ['DELT3'],
    ['SUPSP'],
    ['INFSP','TMIN'],
    ['SUBSC'],
    ['TMAJ'],
    ['PECM1','PECM2','PECM3'],
    ['LAT'],
    ['CORB']]

#Loop through muscle groups and adjust by the scale factors listed

#Loop through muscle groups
for mm in range(0,len(muscToAlter)):
    
    #Create a string label for the current muscle group
    s = '_'
    modelLabel = s.join(muscToAlter[mm])
    
    #Loop through scale factors
    for ss in range(0,len(scaleFactors)):
        
        #Create a version of the baseline model to edit
        osimModel = osim.Model(baseModelFileName)
        
        #Loop through muscles within the list
        for gg in range(0,len(muscToAlter[mm])):
            
            #Get string for current muscle
            currMusc = muscToAlter[mm][gg]
            
            #Get the current muscles strength
            currMuscStrength = osimModel.getMuscles().get(currMusc).get_max_isometric_force()
            
            #Scale to its new force generating capacity
            newMuscStrength = currMuscStrength * scaleFactors[ss]
            
            #Set the new strength in the model
            osimModel.getMuscles().get(currMusc).set_max_isometric_force(newMuscStrength)
            
        #Save new model to the output directory
        osimModel.setName(modelLabel+'_strength'+str(int(scaleFactors[ss]*100)))
        osimModel.finalizeConnections()
        osimModel.printToXML(modelPath+'\\'+modelLabel+'_strength'+str(int(scaleFactors[ss]*100))+'.osim')
        
# %% ----- End of ShoulderStrengthSims_1_GenerateModels.py ----- %% #