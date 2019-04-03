# -*- coding: utf-8 -*-
"""
Spyder Editor

"""
import time
import vrep
import sys
import math
import random
import matplotlib.pyplot as plt

jointhandles = []
fl = []
genome = []
cgenome = []
best_genome = []
fitness_track = []
generation = 0
cur_fitness = 0
best_fitness = 0
iteration = 0
genome = 0
x = 0
y = 0
z = 0
clientID=0
    
def initialize():
        global clientID

        vrep.simxFinish(-1)
        clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)#vrep connection

         # Connect to V-REP
        if clientID!=-1:
            print ('Connected to remote API server')
        else: 
            print('not')
            sys.exit('not connect')
    
def initgenome():
        gen = [] #32 values + 8 values
        for i in range(0,16):
#           
        			gen.append(random.randint(-70,70)) #Speed
        for i in range(4,8):
                        gen[i]=random.randint(0,120)
        for i in range(12,16):
                        gen[i]=random.randint(0,120) #Angle
        for i in range(16,32):
#            
        			gen.append(random.randint(-70,70))	
        return gen

def mutate(gen):
    index = random.randint(0,16)#returns a random index value
    if (index == 16) :
        index = 15
    if ((index <8 )and(index>=4)) or(index<16 and index>=12): #Angle
        gen[index] = float(format((((((random.randint(0,120)) - -0) * (1.2 - 0)) / (120 - 0)) + -0.0),'.2f'))
        gen[index+16] =float(format((((((random.randint(0,120)) - -0) * (1.2 - 0)) / (120 - 0)) + -0.0),'.2f'))
        return gen
    else:#Speed
        gen[index] = float(format((((((random.randint(-70,70)) - -70) * (0.7 - -0.7)) / (70 - -70)) + -0.7),'.2f'))
        gen[index+16] = float(format((((((random.randint(-70,70)) - -70) * (0.7 - -0.7)) / (70 - -70)) + -0.7),'.2f'))
        return gen
    



def fitness(Data): #euclidean distance [origin = 0]
        return math.sqrt(math.pow(Data[0],2)+math.pow(Data[1],2)+math.pow(Data[2],2))
   

def convert(genome):
    i=0
    while i < 32:
        a=((((genome[i] - -70) * (0.7 - -0.7)) / (70 - -70)) + -0.7) #Normalization
        cgenome.append(float(format(a, '.2f')))
        i=i+1
    return cgenome
        



        
        
        
initialize()
print(clientID)
generation =1
genome=initgenome() #initialization
cgenome=convert(genome)    #normalization
best_genome=cgenome[:]
for j in range (0,5000):
        print("Generation:")
        print(generation)
        cur_genome=mutate(best_genome[:])#Mutation
        print("Genome:")
        print(cur_genome)
        vrep.simxLoadModel(clientID,"C:\quad0_1.ttm",0,vrep.simx_opmode_blocking) #simulation loading
        time.sleep(1)
        errorcode,model_handle=vrep.simxGetObjectHandle(clientID,"quadruped",vrep.simx_opmode_blocking)
        i=0
        while i < 8: #8 joints reduced to 4
            name="quad_joint"+str(i+1)
            jointhandles.append(vrep.simxGetObjectHandle(clientID,name,vrep.simx_opmode_blocking)[1])
            i=i+1   
        
        errorcode,quad=vrep.simxGetObjectHandle(clientID,'quad_body',vrep.simx_opmode_blocking)
        k=0
        
        
            
        while k <=10:
                for i in range(0,4):
                    vrep.simxSetJointTargetVelocity(clientID,jointhandles[i],cur_genome[i+16],vrep.simx_opmode_streaming)
                    vrep.simxSetJointTargetPosition(clientID,jointhandles[i],cur_genome[i],vrep.simx_opmode_streaming) 
                time.sleep(1)
                for i in range(4,8):
                    vrep.simxSetJointTargetVelocity(clientID,jointhandles[i],cur_genome[i+16],vrep.simx_opmode_streaming)
                    vrep.simxSetJointTargetPosition(clientID,jointhandles[i],cur_genome[i],vrep.simx_opmode_streaming) 

                time.sleep(1)     
                for i in range(0,4):
                    vrep.simxSetJointTargetVelocity(clientID,jointhandles[i],cur_genome[i+24],vrep.simx_opmode_streaming)
                    vrep.simxSetJointTargetPosition(clientID,jointhandles[i],cur_genome[i+8],vrep.simx_opmode_streaming)
                time.sleep(1)
                for i in range(4,8):
                    vrep.simxSetJointTargetVelocity(clientID,jointhandles[i],cur_genome[i+24],vrep.simx_opmode_streaming)
                    vrep.simxSetJointTargetPosition(clientID,jointhandles[i],cur_genome[i+8],vrep.simx_opmode_streaming)

                time.sleep(1)
                k=k+1
        returnCode,Data=vrep.simxGetObjectPosition(clientID,model_handle,-1,vrep.simx_opmode_blocking) #return code - 0,1, data - x,y,z
        print("Position:")
        print(Data)
        cur_fitness=fitness(Data) #fitness calculator
        print("Fitness:")
        print(cur_fitness)
        fitness_track.append(cur_fitness) #comparision
#        for m in range (0,len(fitness_track)):
#            fl.append(m)
#        plt.plot(i,fitness_track)
#        plt.show()
        if cur_fitness>best_fitness:
            best_genome=cur_genome[:]
            best_fitness=cur_fitness
            
      
        vrep.simxRemoveModel(clientID,model_handle,vrep.simx_opmode_blocking)
        jointhandles=[]
        generation=generation+1

