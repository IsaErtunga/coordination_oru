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
    times = []
    messageCounter = []
    with open(path,'r') as csvfile:
        plots = csv.reader(csvfile, delimiter = ',')
        for row in plots:
            times.append(float(row[0]))
            messageCounter.append(float(row[1]))

        for i in range(1, len(messageCounter)):
            messageCounter[i] += messageCounter[i-1]

        plt.figure()
        plt.plot(times, messageCounter, label="Messages")
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

def plotMessagesWithType():
    labels = ['G1', 'G2', 'G3', 'G4', 'G5']
    men_means = [20, 35, 30, 35, 27]
    women_means = [25, 32, 34, 20, 25]
    width = 0.35       # the width of the bars: can also be len(x) sequence

    plt.figure()
    fig, ax = plt.subplots()

    ax.bar(labels, men_means, width, label='Men')
    ax.bar(labels, women_means, width, bottom=men_means,
        label='Women')

    ax.set_ylabel('Scores')
    ax.set_title('Scores by group and gender')
    ax.legend()

# plotSAState()
# plotMessageAmount()
# plotWaitingTimes()
plotMessagesWithType()
plt.show()
