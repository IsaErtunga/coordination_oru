import matplotlib.pyplot as plt
import csv

plt.rcParams["figure.figsize"] = [7.50, 3.50]
plt.rcParams["figure.autolayout"] = True

# Plots ore levels for storage agent
def plotSAState():
    robotIDs = ["301", "302"]
    dirPath = "/home/parallels/testRun/"
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
    path = "/home/parallels/testRun/Messages.csv"
    times = []
    messageCounter = []
    with open(path,'r') as csvfile:
        plots = csv.reader(csvfile, delimiter = ',')
        for row in plots:
            times.append(float(row[0]))
            messageCounter.append(float(row[1]))

        for i in range(0, len(messageCounter)):
            messageCounter[i] += i

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
    path = "/home/parallels/testRun/WaitingTimes" + robotIDs[0] + ".csv"
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
