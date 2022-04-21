import matplotlib.pyplot as plt
import csv
import os

plt.rcParams["figure.figsize"] = [7.50, 3.50]
plt.rcParams["figure.autolayout"] = True
dirPath = "/home/parallels/Projects/coordination_oru/testResults/testRun/"

# Plots ore levels for storage agent
def plotSAState():
    robotIDs = ["301", "302"]
    
    for id in robotIDs:
        path = dirPath + "OreState" + id + ".csv"
        times = []
        actualOre = []
        oreState = []

        with open(path,'r') as csvfile:
            plots = csv.reader(csvfile, delimiter = ',')
            for row in plots:
                times.append(float(row[0]))
                actualOre.append(float(row[1]))
                oreState.append(float(row[2]))

        plt.figure()
        plt.plot(times, actualOre, label="Actual Ore")
        plt.plot(times, oreState, label="Ore state")
        plt.legend(loc="upper left")
        plt.title('Storage Agent: ' + id)
        plt.xlabel('Time')
        plt.ylabel('Ore')
        path = ""
  
# Plot message count
def plotMessageAmount():
    path = dirPath + "Messages.csv"
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
    
    with open(path,'r') as csvfile:
        plots = csv.reader(csvfile, delimiter = ',')
        for row in plots:
            time = float(row[0])

            messageTypes["total"][0].append(time)
            messageTypes["total"][1].append(messageTypes["total"][2])
            messageTypes["total"][2] += 1

            messageTypes[row[1]][0].append(time)
            messageTypes[row[1]][1].append(messageTypes[row[1]][2])
            messageTypes[row[1]][2] += 1

        plt.figure()
        plt.plot(messageTypes["total"][0], messageTypes["total"][1], label="Messages")
        plt.plot(messageTypes["hello-world"][0], messageTypes["hello-world"][1], label="hello-world")
        plt.plot(messageTypes["echo"][0], messageTypes["echo"][1], label="echo")
        plt.plot(messageTypes["cnp-service"][0], messageTypes["cnp-service"][1], label="cnp-service")
        plt.plot(messageTypes["offer"][0], messageTypes["offer"][1], label="offer")
        plt.plot(messageTypes["accept"][0], messageTypes["accept"][1], label="accept")
        plt.plot(messageTypes["decline"][0], messageTypes["decline"][1], label="decline")
        plt.plot(messageTypes["inform"][0], messageTypes["inform"][1], label="inform")
        plt.legend(loc="upper left")
        plt.title('Message Counter')
        plt.xlabel('Time')
        plt.ylabel('Message')

# Plot message count
def plotWaitingTimes():
    times = []
    waitingTimes = []
    robotIDs = ["9401"]
    path = dirPath + "WaitingTimes" + robotIDs[0] + ".csv"
    with open(path,'r') as csvfile:
        plots = csv.reader(csvfile, delimiter = ',')
        for row in plots:
            times.append(float(row[0]))
            waitingTimes.append(float(row[1]))

        plt.figure()
        plt.plot(times, waitingTimes, label="Wait Time")
        plt.legend(loc="upper left")
        plt.title('Waiting Times')
        plt.xlabel('Time')
        plt.ylabel('Wait Time')


plotSAState()
plotMessageAmount()
plotWaitingTimes()
plt.show()