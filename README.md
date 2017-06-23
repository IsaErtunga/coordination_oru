# coordination_oru
This package provides an online coordination method for multiple robots. It is based on the Meta-CSP Framework library available at <a href="http://metacsp.org">metacsp.org</a>.

## Overview
The coordination method is based on the trajectory envelope representation provided by the Meta-CSP framework. A trajecotry envelope is a set of spatio-temporal constraints on a robot's trajectory. The method works as follows:

** For each pair of trajecotry envelopes (of two distinct robots)
*** Bla bla
** bla bla

## Installation
To install, clone this repository and compile the source code with gradle (redistributable included):

```
$ git clone https://github.com/FedericoPecora/coordination_oru.git
$ cd coordination_oru
$ ./gradlew install
```

## Running an example
To run an example, issue the following command from the source code root directory:
```
$ ./gradlew run
```
The example continuously posts missions for three robots to reach locations along intersecting paths. The paths are stored in files provided in the ```paths``` directory. The poses of locations and pointers to relevant path files between locations are stored in the self-explanatory ```paths/test_poses_and_path_data.txt``` file.

![alt text](images/coord.png "Coordination GUI")

A gray arrow between robot A and robot B indicates that robot A will yield to robot B. These precedences are computed on the fly based on the current positions of robots and the intersecting areas of their trajectory envelopes (critical sections). Priorities are computed every time a new mission is added. If multiple missions are added in batch, yielding behavior follows a fixed priority which can be specified programmatically. Driving robots always have priority over robots whose missions have been newly computed.

