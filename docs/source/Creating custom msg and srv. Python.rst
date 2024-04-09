Creating custom msg and srv. Python
==========================

.. _custom msg and srv python:


In previous sections, predefined messages and service types were used. Recall the String message type in :ref:`the publisher and subscriber example<Writting publisher and subscriber nodes. Python>` or the AddTwoInts service in the :ref:`service and client examples<Writting service and client. Python>`. These types of interfaces already existed and were ready to be used. In this section, custom messages and services types will be created and applied into program examples under the python programming language.


Setup for working with custom msg and srv
------------------------

Make sure to be in a brand new terminal window and no ROS commands are currently running. 

Create a new package. This package should be contained in the ``ros2_ws`` workspace, within its ``/src`` folder. The name provided to this new package will be ``tutorial_interfaces``. For more reference on package creation consult: :ref:`pacakge creation<conf_env/Creating a package>` or :ref:`pacakge creation2<Configuring environment/Creating a package>` or :ref:`pacakge creation3<_conf_env/Creating a package>`


.. code-block:: console

   ros2 pkg create --build-type ament_cmake --license Apache-2.0 tutorial_interfaces

Notice that the package created, is a CMake package. This is where the custom messages and services will be stored, but these can be used in any kind of packages, python or C++ packages.

Next, create the folder: ``msg`` and ``srv`` inside ``ros2_ws/src/tutorial_interfaces``. This is where messages and services types will be stored respectively.

Custom definitions
------------------------

In this part, the actual creation of the custom message and service types will be created. 

Message definition
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Inside ``tutorial_interfaces/msg`` create a new file named ``Sphere.msg``. Edit the content of ``Sphere.msg`` to include:

.. code-block:: console

   geometry_msgs/Point center
   float64 radius

This custom message uses a message from another message package (``geometry_msgs/Point`` in this case).


Service definition
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Inside ``tutorial_interfaces/srv`` create a new file named ``AddThreeInts.srv``. Edit the content of ``AddThreeInts.srv`` to include:

.. code-block:: console

   int64 a
   int64 b
   int64 c
   ---
   int64 sum

This custom service requests three integers named ``a``, ``b``, and ``c``, and responds with an integer called ``sum``.

Edditing the CMakeLists.txt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To convert the defined interfaces into language-specific code (like C++ and Python) so that they can be used in those languages, add the following lines to ``CMakeLists.txt``:

.. code-block:: console

   find_package(geometry_msgs REQUIRED)
   find_package(rosidl_default_generators REQUIRED)

   rosidl_generate_interfaces(${PROJECT_NAME}
   "msg/Sphere.msg"
   "srv/AddThreeInts.srv"
   DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
   )

The ``find_package()`` commands make the compiler look for the required packages. In this case, ``geometry_msgs`` and ``rosidl_default_generators`` are the required packages.

The ``rosidl_generate_interfaces()`` command line, actually generates the code for the custom message and service interfaces. It takes as arguments: The name of the project, the path to the custom message and service files and necessary package dependencies.

The ``CMakeLists.txt`` file should look similar to:

.. image:: images/CMakelistsCustomMsgSrv.png
   :alt: CMakeLists.txt for custom msg and srv.


Editting the pacakge.xml file
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The following should be added to ``tutorial_interfaces/package.xml``:

.. code-block:: console

   <depend>geometry_msgs</depend>
   <buildtool_depend>rosidl_default_generators</buildtool_depend>
   <exec_depend>rosidl_default_runtime</exec_depend>
   <member_of_group>rosidl_interface_packages</member_of_group>

- Because the interfaces rely on ``rosidl_default_generators`` for generating language-specific code, you need to declare a build tool dependency on it.
- ``rosidl_default_runtime`` is a runtime or execution-stage dependency, needed to be able to use the interfaces later.
- The ``rosidl_interface_packages`` is the name of the dependency group that the ``tutorial_interfaces package``, should be associated with, declared using the ``<member_of_group>`` tag.

The ``pacakge.xml`` file should look similar to:

.. image:: images/packageXmlForCustomMsgSrv.png
   :alt: package.xml to build the custom msg and srv.

Build and test
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

:ref:`Open a brand new terminal<installation/Running a docker container>`, make sure that no other ROS2 command is currently running, navigate to the workspace directory and execute:

.. code-block:: console

   colcon build --packages-select tutorial_interfaces

Now, source the setup file:

.. code-block:: console
   
   source install/setup.bash

For more reference on sourcing the setup file, see :ref:`sourcing the setup file<conf_env/Source the setup file>` .

Next, to check that the custom message is correctly created, run:

.. code-block:: console
   
   ros2 interface show tutorial_interfaces/msg/Sphere

The otuput should be: 

.. code-block:: console
   
   geometry_msgs/Point center
         float64 x
         float64 y
         float64 z
   float64 radius

And to test the service, run:

.. code-block:: console

   ros2 interface show tutorial_interfaces/srv/AddThreeInts

Should output the following:

.. code-block:: console

   int64 a
   int64 b
   int64 c
   ---
   int64 sum

Testing the Sphere custom msg
^^^^^^
Make sure to be in a brand new terminal window and no ROS commands are currently running. 

Create a new python package,  this package should be contained in the ``ros2_ws`` workspace, within its ``/src`` folder. The name provided to this new package will be ``testing_interfaces_python``. For more reference on package creation consult: :ref:`pacakge creation<conf_env/Creating a package>` or :ref:`pacakge creation2<Configuring environment/Creating a package>` or :ref:`pacakge creation3<_conf_env/Creating a package>`

.. code-block:: console

   ros2 pkg create --build-type ament_python --license Apache-2.0 testing_interfaces_python --dependencies rclpy tutorial_interfaces

The ``--dependencies`` argument will automatically add the necessary dependency lines to ``package.xml``. In this case, ``tutorial_interfaces`` is the package that includes the ``Sphere.msg`` file that is needed for this test.

Next, inside ``testing_interfaces_python/testing_interfaces_python`` create a python script, name it ``sphere_publisher.py``. 

Copy this content into the new python script. 

.. code-block:: console

   import rclpy
   from rclpy.node import Node

   from tutorial_interfaces.msg import Sphere                                      # Change

   class SpherePublisher(Node):                                                    # Change

      def __init__(self):
         super().__init__('sphere_publisher')                                    # Change
         self.publisher_ = self.create_publisher(Sphere, 'sphere_topic', 10)     # Change
         timer_period = 0.5  # seconds
         self.timer_ = self.create_timer(timer_period, self.timer_callback)
         self.count_ = 0.0

      def timer_callback(self):
         msg = Sphere()                                                          # Change
         msg.center.x = self.count_                                              # Change    
         msg.center.y = 1.0                                                      # Change
         msg.center.z = 2.0                                                      # Change    
         msg.radius = 10.0                                                       # Change
         self.publisher_.publish(msg)                                            # Change
         self.get_logger().info('Publishing sphere params (x, y, z, radius):' +  # Change
                                 'x=%s, y=%s, z=%s, radius=%s' % 
                                 (msg.center.x, msg.center.y, msg.center.z, msg.radius))
         self.count_ += 1.0    


   def main(args=None):
      rclpy.init(args=args)

      sphere_publisher = SpherePublisher()

      rclpy.spin(sphere_publisher)

      # Destroy the node explicitly
      # (optional - otherwise it will be done automatically
      # when the garbage collector destroys the node object)
      sphere_publisher.destroy_node()
      rclpy.shutdown()


   if __name__ == '__main__':
      main()

Notice that this code is very similar to the publisher script that was studied  :ref:`previously<Writting publisher and subscriber nodes. Python/Publisher node in python>`.

Check the important changes in this script.

.. code-block:: console

   from tutorial_interfaces.msg import Sphere                                      # Change
   ...
   self.publisher_ = self.create_publisher(Sphere, 'sphere_topic', 10)     # Change
   ...
   def timer_callback(self):
         msg = Sphere()                                                          # Change
         msg.center.x = self.count_                                              # Change    
         msg.center.y = 1.0                                                      # Change
         msg.center.z = 2.0                                                      # Change    
         msg.radius = 10.0                                                       # Change
         self.publisher_.publish(msg)


- It is important to correctly import the required libraries.
- The publisher node will now publish different type of messages and will also publish to a different topic. The topic name could have stayed the same, but it is better to name the topics accordingly.
- Finally, the callback function, instead of directly publishing a string message, it is necessary to fill every parameter that is needed for the new message type. 

Next, create another node a listener node for this publisher. Inside ``testing_interfaces_python/testing_interfaces_python`` create a python script, name it ``sphere_listener.py``. 

Copy this content into the new python script. 

.. code-block:: console

   import rclpy
   from rclpy.node import Node

   from tutorial_interfaces.msg import Sphere                                              # Change

   class SphereListener(Node):

      def __init__(self):
         super().__init__('sphere_listener')                                             # Change
         self.subscription_ = self.create_subscription(                                  # Change
               Sphere,
               'sphere_topic',
               self.listener_callback,
               10)
         self.subscription_  # prevent unused variable warning

      def listener_callback(self, msg):
         self.get_logger().info('I heard (x, y, z, radius):'+                            # Change
                                 'x=%s, y=%s, z=%s, radius=%s' %
                                 (msg.center.x, msg.center.y, msg.center.z, msg.radius))


   def main(args=None):
      rclpy.init(args=args)

      sphere_listener = SphereListener()

      rclpy.spin(sphere_listener)

      # Destroy the node explicitly
      # (optional - otherwise it will be done automatically
      # when the garbage collector destroys the node object)
      sphere_listener.destroy_node()
      rclpy.shutdown()


   if __name__ == '__main__':
      main()

The code is very similar to the listener script that was studied  :ref:`previously<Writting publisher and subscriber nodes. Python/Subscriber node in python>`.

Again, the relevant changes here, have to do with dealing with the appropriate topic name and message type. 

Once, these two python scripts are ready, it is necessary to add the required dependencies in the ``package.xml`` file, which was already added when creating this package. See that in the ``package.xml`` file it is present the tag ``package.xml``: ``<depend>tutorial_interfaces</depend>``.

Next, add the entry points in the ``setup.py`` file:

.. code-block:: console

   entry_points={
         'console_scripts': [
               'sphere_publisher = testing_interfaces_python.sphere_publisher:main',
               'sphere_listener = testing_interfaces_python.sphere_listener:main'
         ],
      }

Build the package with either of these commands:

.. code-block:: console

   colcon build --symlink-install
   colcon build --packages-select testing_interfaces_python

Source the setup file:

.. code-block:: console
   
   source install/setup.bash

And run the ``sphere_publisher`` node that was recently created. 

.. code-block:: console
   
   ros2 run testing_interfaces_python sphere_publisher

The result should be like the following:

.. code-block:: console
   
   [INFO] [1712658428.246483307] [sphere_publisher]: Publishing sphere params (x, y, z, radius):x=0.0, y=1.0, z=2.0, radius=10.0
   [INFO] [1712658428.603038612] [sphere_publisher]: Publishing sphere params (x, y, z, radius):x=1.0, y=1.0, z=2.0, radius=10.0
   [INFO] [1712658429.101586253] [sphere_publisher]: Publishing sphere params (x, y, z, radius):x=2.0, y=1.0, z=2.0, radius=10.0
   ...

`Open a new terminal`_ and execute the ``sphere_listener`` node:

.. _open a new terminal: https://alex-readthedocs-test.readthedocs.io/en/latest/Installation.html#opening-a-new-terminal

.. code-block:: console
   
   ros2 run testing_interfaces_python sphere_listener


The expected result is:

.. code-block:: console
   
   [INFO] [1712658569.240308588] [sphere_listener]: I heard (x, y, z, radius):x=282.0, y=1.0, z=2.0, radius=10.0
   [INFO] [1712658569.597305674] [sphere_listener]: I heard (x, y, z, radius):x=283.0, y=1.0, z=2.0, radius=10.0
   [INFO] [1712658570.098490216] [sphere_listener]: I heard (x, y, z, radius):x=284.0, y=1.0, z=2.0, radius=10.0
   ...

Finally, it can also be checked the echo of the messages arriving to the desired topic. `Open a new terminal`_ and execute:

.. _open a new terminal: https://alex-readthedocs-test.readthedocs.io/en/latest/Installation.html#opening-a-new-terminal

.. code-block:: console
   
   ros2 topic echo /sphere_topic

The expected result is:

.. code-block:: console
   
   x: 484.0
   y: 1.0
   z: 2.0
   radius: 10.0
   ---
   center:
   x: 485.0
   y: 1.0
   z: 2.0
   radius: 10.0
   ---
   ...

At this point, it can be seen that the custom message ``Sphere.msg`` that was created is being used successfully.

Testing the AddThreeInts custom srv 
^^^^^^
