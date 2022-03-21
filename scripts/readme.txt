This Readme Gives information on the code structure and how to implement this code for performing the required simulations?

Code Structure :

    1. The Launch contains the launch files, in which the ardrone_simulator.lauch launches the required nodes to perform the simulation.
    2. The Lab_interface node that runs indoor_robotics_lab_interface.py, which has been provided as the attitude controller for the ARDrone.
    3. The Ros_interface node is the key node which has been edited to encompass the operations.
        a. The node subscribes to the Desired positions node(covered Below) and the Vicon data node.
        b. The node publishes to the Topics \cmd_vel_RHC, desired_positions, current_positions and the Positional errors.
        c. The node creates a Position Controller object - which is being initiated using the Position Controller Class in position_controller.py.
        d. The node takes the data from \desired_positions and uses it to set the desired position at that time instance for the ARDrone.
        e. The __main__ is called to generate the node

    4. The Desired_positions node is the node which has been edited to publish the desired positions.
        a. The node publishes the desired positions at a fixed rate of onboard loop frequency.
        b. The node has two trajectories defined in it, one for the straight line trajectory and another for the circle trajectory.
        
        Working : The node generates a string containing all the desired positions, which is then published to the desired_positions topic.
        The string is taken by the ros_interface.py node and is then converted to an array of floats.
        The array is then used to set the desired position of the ARDrone based on the actual time of the simulation in the update function in ros_interface.py.

Usage :
    1. Replace the Scripts and Launch Folders. These are the only folders having edited files in them.
    2. Specifically, position_controller, ros_interface and desired_positions have been edited in the Scripts folders
    3. The ardrone_simulator.launch file has been edited to include the above required nodes.