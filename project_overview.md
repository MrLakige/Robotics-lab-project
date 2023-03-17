
# Project n. 2
In a room, there are four objects located at approximately known positions. Specifically, each
object can be anywhere within a circular area with a known radius and a known centre. The
objects belong to different classes, each with known geometry (they are Megablocks) placed
on top of a stand. A mobile robot equipped with a 3D visual sensor moves in this room to
search for the objects.
To this end,
1. it moves to the different areas
2. it locates exactly the object and classifies it.
Then a human operator loads the object on the robot which manoeuvres to reach the
unloading station, where it has to park in an assigned configuration presenting the loaded
object to a UR5 robotic arm. The UR5 picks up the object from the mobile robot, uses the
information from the classification made previously, and places the object in a basket
according to its class. A second 3D sensor is used to estimate the pose of the object.
The project is organised as a sequence of assignments of increasing complexity. The
minimum requirement to pass the exam is that at least assignments 1 and 2 are completed.
Each assignment is associated with some performance indicators that are used in the
grading of the project.

## Assignment n. 1
The mobile robot has to visit all four different areas. For each area, it has to exactly
localise the object and identify its class. The object can be anywhere within the circular area,
but it is positioned in a natural configuration (base on the ground),
KPI-1-1 Total time to complete the mission
KPI-1-2 Accuracy in localising the objects
KP-1-3 Classification error rate

## Assignment n. 2
The mobile robot has to visit all four different areas. For each area, it has to exactly
localise the object and identify its class. The object can be anywhere within the circular area,
but it is positioned in an arbitrary position (i.e., on the base, on one side, on the top).
The arm picks the object from the robot, receives its class information, and stores it into the
right basket.
KPI in assignment 1
KI-2-1 Time required to the arm to pick up the object and store it

## Assignment n. 3
The mobile robot has to visit all four different areas. For each area, it has to exactly
localise the object and identify its class. The object can be anywhere within the circular area,
but is positioned in an arbitrary position (i.e., on the base, on one side, on the top). The
mobile robot moves to the fixed station and parks in the prescribed position.
The arm picks the object from the robot, receives its class information and repeats the
classification in order to avoid errors, and stores it in the right basket.
KPI in assignments 1 and 2
KPI-3-1 Total time to complete the four missions

# Delivery rules
The project is developed in groups. The typical group size consists of three-four members.
We can also accept groups with a smaller number of members. The group is supposed to
work in perfect cooperation and the workload is required to be fairly distributed. The specific
contribution of each member will be exposed during the project discussion.
The delivery phase is as follows:
1. The project will have to be tested in the laboratory with the Teaching Assistant at
most 5 least five days before the exam date. During the tests, small videos can be
shot and used for the presentation.
2. Each group will have to deliver the package containing the full code (with doxygen
documentation and a readme for use) plus a 5-6 pages report describing
    a. the technique used for perception
    b. the technique used for robot motion
    c. the technique used for high-level planning
    d. A table with the KPI measured on Gazebo
3. The delivery deadline is three days before the exam presentation
4. On the day of the exam, the students will give a 20 minutes presentation highlighting
the contribution of each member. A discussion will follow in which all members are
supposed to answer questions on the entire project (regardless of her/his specific
assignment within the group).
5. If allowed by the time, the group could also be asked to perform a small demo
session. Otherwise, we will rely on the clip shot before the exam.
