import matplotlib.pyplot as plt
import csv

plt.rcParams["figure.figsize"] = [7.50, 3.50]
plt.rcParams["figure.autolayout"] = True

filename = "StorageAgentState.csv"
path = "/home/parallels/" + filename
  
x = []
y = []
  
with open(path,'r') as csvfile:
    plots = csv.reader(csvfile, delimiter = ',')
      
    for row in plots:
        x.append(float(row[0]))
        y.append(float(row[1]))


plt.plot(sorted(x), sorted(y))
plt.title('StorageAgent (ore pass) state')
plt.xlabel('Time')
plt.ylabel('Ore')
plt.show()