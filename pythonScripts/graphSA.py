from cProfile import label
import json
from time import time
import matplotlib.pyplot as plt
import csv
import numpy as np
import json

plt.rcParams["figure.figsize"] = [7.50, 3.50]
plt.rcParams["figure.autolayout"] = True
path = "/home/parallels/Projects/coordination_oru/testResults/experiments"


def readExperimentFile():
    oreStates = []
    messages = []
    times = []
    
    with open(path,'r') as csvfile:
        measure = csv.reader(csvfile, delimiter = ',')
        for experiment in measure:
            if (experiment[0] == "1"):
                if ("ORESTATE" in experiment):
                    oreStates.append(experiment)
                elif ("MESSAGE" in experiment):
                    messages.append(experiment)
                elif ("TIME" in experiment):
                    times.append(experiment)
    
    plotOreState(oreStates=oreStates)
    plotMessages(messages=messages)
    plotWaitingTimes(times=times)

            

# Plots ore levels for storage agent
def plotOreState(oreStates):
    oreStateById = {}
    for oreState in oreStates:
        if (oreState[1] not in oreStateById.keys()):
            oreStateById[oreState[1]] = [list(), list()]
        else:
            robotOreState = oreStateById.get(oreState[1])
            robotOreState[0].append(float(oreState[3]))
            robotOreState[1].append(float(oreState[4]))
    
    for plot in oreStateById.keys():
        plt.figure()
        plt.step(oreStateById[plot][0], oreStateById[plot][1], label="Ore state")
        plt.legend(loc="upper left")
        plt.title('Storage Agent: ' + plot)
        plt.xlabel('Time')
        plt.ylabel('Ore')
  

# Plot message count
def plotMessages(messages):
    messageTypes = {
        "total": [[],[], 1],
        "hello-world": [[],[], 1],
        "echo": [[],[], 1],
        "cnp-service": [[],[], 1],
        "offer": [[],[], 1],
        "accept": [[],[], 1],
        "decline": [[],[], 1],
        "inform": [[],[], 1],
    }
    
    for message in messages:
        time = float(message[2])

        messageTypes["total"][0].append(time)
        messageTypes["total"][1].append(messageTypes["total"][2])
        messageTypes["total"][2] += 1

        messageTypes[message[3]][0].append(time)
        messageTypes[message[3]][1].append(messageTypes[message[3]][2])
        messageTypes[message[3]][2] += 1


    plt.figure()
    plt.step(messageTypes["total"][0], messageTypes["total"][1], label="Messages")
    plt.step(messageTypes["hello-world"][0], messageTypes["hello-world"][1], label="hello-world")
    plt.step(messageTypes["echo"][0], messageTypes["echo"][1], label="echo")
    plt.step(messageTypes["cnp-service"][0], messageTypes["cnp-service"][1], label="cnp-service")
    plt.step(messageTypes["offer"][0], messageTypes["offer"][1], label="offer")
    plt.step(messageTypes["accept"][0], messageTypes["accept"][1], label="accept")
    plt.step(messageTypes["decline"][0], messageTypes["decline"][1], label="decline")
    plt.step(messageTypes["inform"][0], messageTypes["inform"][1], label="inform")
    plt.legend(loc="upper left")
    plt.title('Message Counter')
    plt.xlabel('Time')
    plt.ylabel('Message')

# Plot message count
def plotWaitingTimes(times):
    waitingTimes = {
        "pathCalculation": [[], [], 1],
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



    # print(json.dumps(waitingTimes,sort_keys=True, indent=2))
    
    plt.figure()
    for measurment in waitingTimes.keys():
        plt.step(waitingTimes[measurment][0], waitingTimes[measurment][1], label=measurment)
        plt.title('Waiting Times: ' + measurment)    


    plt.title('Waiting Times')
    
    plt.legend(loc="upper left")
    plt.xlabel('Measurment')
    plt.ylabel('Wait Time')


readExperimentFile()
plt.show()