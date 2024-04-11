ROS2 bag
=====

.. _ros2_bag:

What is it?
------------

- This is a command-line tool in ROS2 used for recording and playing back data from ROS2 topics.
- It accumulates the data passed on any number of topics and saves it in a database.
- The data can then be replayed to reproduce the results of tests and experiments.

Do not forget 
-------------
Remember to have your environment properly setup. Perform the following, if the ROS2 package cannot be found when executing it:

.. code-block:: console

   source install/setup.sh

See this example: 

- Look at the error of not being able to find the desired package. This is because the workspace was not configured correctly.

.. image:: images/ErrorNotSourcing.png
   :alt: Error message of not sourcing workspace correctly.

- Source the appropriate setup script, that is, run ``source install/setup.sh``, to correctly configure the environment.

.. image:: images/SourcingWorkspace.png
   :alt: Correctly sourcing the workspace.

Notice that the sourcing is performed inside the workspace folder. 

ROS2 bag record
-------------

Start launching the ``turtlesim_node`` along with the ``turtle_teleop_key``. `Open two terminals`_ and execute in each of them:

.. _open a new terminal: https://alex-readthedocs-test.readthedocs.io/en/latest/Installation.html#opening-a-new-terminal

.. code-block:: console

   ros2 run turtlesim turtlesim_node

   ros2 run turtlesim turtle_teleop_key

`In a new terminal`_ create a new folder to store the ros2 bag file inside ``ros2_ws`` workspace, and get into that folder. For this, run:

.. _open a new terminal: https://alex-readthedocs-test.readthedocs.io/en/latest/Installation.html#opening-a-new-terminal

.. code-block:: console

   mkdir bag_files
   cd bag_files

Now, to record the data published to a specific topic use this command syntax:

.. code-block:: console

   ros2 bag record <topic_name>

Run this example:

.. code-block:: console

   ros2 bag record /turtle1/cmd_vel

The following will be displayed in the terminal:

.. code-block:: console

   [INFO] [1712867867.590658178] [rosbag2_recorder]: Press SPACE for pausing/resuming
   [INFO] [1712867867.621929975] [rosbag2_storage]: Opened database 'rosbag2_2024_04_11-20_37_47/rosbag2_2024_04_11-20_37_47_0.db3' for READ_WRITE.
   [INFO] [1712867867.624862506] [rosbag2_recorder]: Listening for topics...
   [INFO] [1712867867.624890008] [rosbag2_recorder]: Event publisher thread: Starting
   [INFO] [1712867867.629127497] [rosbag2_recorder]: Subscribed to topic '/turtle1/cmd_vel'
   [INFO] [1712867867.629331906] [rosbag2_recorder]: Recording...
   [INFO] [1712867867.629486213] [rosbag2_recorder]: All requested topics are subscribed. Stopping discovery...

Now ros2 bag is recording the data published on the ``/turtle1/cmd_vel`` topic. Return to the ``turtle_teleop_key`` terminal and move the turtle around again. 

Now, stop the recording (pressing Ctrl+C) and see that a new ``.yaml`` file was created. ``ls`` into the ``ros2_ws/bag_files`` folder and some file like this should be generated:

.. code-block:: console

   rosbag2_2024_04_11-20_37_47

ROS2 bag info
-------------

To see details about the recording, run a command following this structre:

.. code-block:: console

   ros2 bag info <bag_file_name>

Apply this command into the recently generated ros2 bag file:

.. code-block:: console

   ros2 bag info rosbag2_2024_04_11-20_37_47

Somthing like the following should be the otuput of this:

.. code-block:: console

   Files:             subset.db3
   Bag size:          228.5 KiB
   Storage id:        sqlite3
   Duration:          48.47s
   Start:             Oct 11 2019 06:09:09.12 (1570799349.12)
   End                Oct 11 2019 06:09:57.60 (1570799397.60)
   Messages:          3013
   Topic information: Topic: /turtle1/cmd_vel | Type: geometry_msgs/msg/Twist | Count: 9 | Serialization Format: cdr
                  Topic: /turtle1/pose | Type: turtlesim/msg/Pose | Count: 3004 | Serialization Format: cdr
   
ROS2 bag play
-------------

To replay the recorded ROS2 bag file, first stop the execution of ``turtlesim_node`` and ``turtle_teleop_key`` nodes and follow this command structre:

.. code-block:: console

   ros2 bag play <bag_file_name>

Apply this command into the generated ROS2 bag file, but first exeucte again the ``turtlesim_node``:

.. code-block:: console

   ros2 bag play rosbag2_2024_04_11-20_37_47

The turtle will follow the same path that was recorded before (though not 100% exactly; turtlesim is sensitive to small changes in the system's timing).



