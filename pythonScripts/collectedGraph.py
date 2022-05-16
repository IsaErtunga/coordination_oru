import json
from time import time
import matplotlib.pyplot as plt
import csv
import numpy as np
import json

plt.rcParams["figure.figsize"] = [7.50, 3.50]
plt.rcParams["figure.autolayout"] = True
path = "/home/parallels/Projects/coordination_oru/testResults/experiments.csv"

 # 1. 302: 163.5, 301: 180.3, Messages: 2859, ore: 696, dist: 25954.5
 # 2. 302: 149.6, 301: 186.5, Messages: 2739, ore: 672, dist: 24457.2
 # 3. 302: 117.0, 301: 136.8, Messages: 2726, ore: 528, dist: 20739.0

 # 

def readExperimentFile():
    oreStates = {"301": [180.9, 162.5, 151.0], "302": [168.5, 170.6, 172.0]}
    messages = [1641, 2203, 2828]
    distances = [13705, 12777, 12211]
    collectedOre = [480, 448, 456]


    plotOreState(oreStates=oreStates)
    plotMessages(messages=messages, distances=distances)
    # plotWaitingTimes(times=times)
    plotCollectedOre(collectedOre=collectedOre)
    plotDistances(distances=distances)

            

# Plots ore levels for storage agent
def plotOreState(oreStates):
    for plot in oreStates.keys():
        plt.figure()
        plt.plot([1, 2, 3], oreStates[plot], label="Ore state")
        plt.legend(loc="upper left")
        plt.title('Storage Agent: ' + plot)
        plt.xlabel('Time')
        plt.ylabel('Ore')
    

# Plot message count
def plotMessages(messages, distances):

    plt.figure()
    plt.step([1, 2, 3], messages, label="Messages")

    plt.legend(loc="upper left")
    plt.title('Message Counter')
    plt.xlabel('Time')
    plt.ylabel('Message')

# Plot message count
def plotWaitingTimes(times):
    waitingTimes = {
        "auctionToTask" : [[], [], 1],
        "idleUponExecution": [[], [], 1],
        "congestion": [[], [], 1]
    }

    for measurment in times:
        time = float(measurment[4])
        waitValue = float(measurment[5])
        type = measurment[3]

        waitingTimes[type][0].append(time)
        if (len(waitingTimes[type][1]) > 0): 
            waitingTimes[type][1].append(waitingTimes[type][1][-1] + waitValue)
        else:
            waitingTimes[type][1].append(waitValue)
        waitingTimes[type][2] += 1

    plt.figure()
    for measurment in waitingTimes.keys():
        plt.step(waitingTimes[measurment][0], waitingTimes[measurment][1], label=measurment)
        plt.title('Waiting Times: ' + measurment)    

    plt.title('Waiting Times')
    plt.legend(loc="upper left")
    plt.xlabel('Measurment')
    plt.ylabel('Wait Time')

def plotCollectedOre(collectedOre):
    plt.figure()
    plt.step([1, 2, 3], collectedOre, label="Collected Ore")

    plt.legend(loc="upper left")
    plt.title('Ore')
    plt.xlabel('Tests')
    plt.ylabel('Ore')

def plotDistances(distances):
    plt.figure()
    plt.step([1, 2, 3], distances, label="Distance")

    plt.legend(loc="upper left")
    plt.title('Distance')
    plt.xlabel('Tests')
    plt.ylabel('Distance')

readExperimentFile()
plt.show()