Creating custom msg and srv. C++
==========================

.. _custom msg and srv cpp:


In previous sections, predefined messages and service types were used. Recall the String message type in :ref:`the publisher and subscriber example<Writting publisher and subscriber nodes. Python>` or the AddTwoInts service in the :ref:`service and client examples<Writting service and client. Python>`. These types of interfaces already existed and were ready to be used. In this section, custom messages and services types will be created and applied into program examples under the C++ programming language.


Setup for working with custom msg and srv
------------------------

In this :ref:`previous section<Creating custom msg and srv. Python/Setup for working with custom msg and srv>` it was already created a custom msg and srv: the ``Sphere.msg`` msg and the ``AddThreeInts.srv`` srv, that belong to the ``tutorial_interfaces`` package. These two custom interfaces will be used along this section of the course. 

Testing the Sphere custom msg in a C++ package
-----------------------
Make sure to be in a brand new terminal window and no ROS command is currently running. 

Create a new python package,  this package should be contained in the ``ros2_ws`` workspace, within its ``/src`` folder. The name provided to this new package will be ``testing_interfaces_cpp``. For more reference on package creation consult: :ref:`pacakge creation<conf_env/Creating a package>` or :ref:`pacakge creation2<Configuring environment/Creating a package>` or :ref:`pacakge creation3<_conf_env/Creating a package>`

.. code-block:: console

   ros2 pkg create --build-type ament_cmake --license Apache-2.0 testing_interfaces_cpp --dependencies rclcpp tutorial_interfaces

The ``--dependencies`` argument will automatically add the necessary dependency lines to ``package.xml`` file. In this case, ``tutorial_interfaces`` is the package that includes the ``Sphere.msg`` file that is needed for this test.

The code
~~~~~~~~~~~~~~~~

Next, inside ``testing_interfaces_cpp/src`` create a C++ script, name it ``sphere_publisher.cpp``. 

Copy this content into the new python script. 

.. code-block:: console

   #include <chrono>
   #include <memory>

   #include "rclcpp/rclcpp.hpp"
   #include "tutorial_interfaces/msg/Sphere.hpp"                                            // CHANGE

   using namespace std::chrono_literals;

   class SpherePublisher : public rclcpp::Node
   {
      public:
      SpherePublisher()
      : Node("sphere_publisher"), count_(0)
      {
         publisher_ = this->create_publisher<tutorial_interfaces::msg::Sphere>("sphere_topic", 10);  // CHANGE
         timer_ = this->create_wall_timer(
            500ms, std::bind(&SpherePublisher::timer_callback, this));
      }

      private:
      void timer_callback()
      {
         auto message = tutorial_interfaces::msg::Sphere();                                   // CHANGE
         message.center.x = this->count_;
         message.center.y = 1.0; 
         message.center.z = 2.0;
         message.radius = 10.0;

         RCLCPP_INFO_STREAM(this->get_logger(), "Publishing sphere params " \
         "(x, y, z, radius): x = " << message.center.x << ", y = " \
            << message.center.y << ", z = " << message.center.z << \
            ", radius = " << message.radius);    // CHANGE
         publisher_->publish(message);
         this->count_++;
      }

      rclcpp::TimerBase::SharedPtr timer_;
      rclcpp::Publisher<tutorial_interfaces::msg::Sphere>::SharedPtr publisher_;             // CHANGE
      size_t count_;
   };

   int main(int argc, char * argv[])
   {
      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<SpherePublisher>());
      rclcpp::shutdown();
      return 0;
   }

Notice that this code is very similar to the publisher script that was studied  :ref:`previously<Writting publisher and subscriber nodes. C++/Publisher node in C++>`.

Check the important changes in this script.

.. code-block:: console

   #include "tutorial_interfaces/msg/Sphere.hpp"                                      
   ...
   publisher_ = this->create_publisher<tutorial_interfaces::msg::Sphere>("sphere_topic", 10);     
   ...
   void timer_callback()
   {
      auto message = tutorial_interfaces::msg::Sphere();                                  
      message.center.x = this->count_;
      message.center.y = 1.0; 
      message.center.z = 2.0;
      message.radius = 10.0;

      RCLCPP_INFO_STREAM(this->get_logger(), "Publishing sphere params " \
      "(x, y, z, radius): x = " << message.center.x << ", y = " \
         << message.center.y << ", z = " << message.center.z << \
         ", radius = " << message.radius);    // CHANGE
      publisher_->publish(message);
      this->count_++;
   }


- It is important to correctly import the required libraries. Importing the custom message definition of ``Sphere``.
- The publisher node will now publish different type of messages and will also publish to a different topic. The topic name could have stayed the same, but it is better to name the topics accordingly.
- Finally, the callback function, instead of directly publishing a string message, it is necessary to fill every parameter that is needed for the new message type. 

Next, create another node, a listener node for this publisher. Inside ``testing_interfaces_cpp/src`` create a C++ script, name it ``sphere_listener.cpp``. 

Copy this content into the new C++ script. 

.. code-block:: console

   #include <functional>
   #include <memory>

   #include "rclcpp/rclcpp.hpp"
   #include "tutorial_interfaces/msg/Sphere.hpp"                                       // CHANGE

   using std::placeholders::_1;

   class SphereListener : public rclcpp::Node
   {
      public:
      SphereListener()
      : Node("sphere_listener")
      {
         subscription_ = this->create_subscription<tutorial_interfaces::msg::Sphere>(    // CHANGE
            "sphere_topic", 10, std::bind(&SphereListener::topic_callback, this, _1));
      }

      private:
      void topic_callback(const tutorial_interfaces::msg::Sphere & msg) const  // CHANGE
      {
         RCLCPP_INFO_STREAM(this->get_logger(), "I heard" \
         ": x = " << msg.center.x << ", y = " \
            << msg.center.y << ", z = " << msg.center.z << \
            ", radius = " << msg.radius);    // CHANGE
      }
      rclcpp::Subscription<tutorial_interfaces::msg::Sphere>::SharedPtr subscription_;  // CHANGE
   };

   int main(int argc, char * argv[])
   {
      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<SphereListener>());
      rclcpp::shutdown();
      return 0;
   }

The code is very similar to the listener script that was studied  :ref:`previously<Writting publisher and subscriber nodes. C++/Subscriber node in C++>`.

Again, the relevant changes here, have to do with dealing with the appropriate topic name and message type. 

Dependencies and CMakeLists
~~~~~~~~~~~~~~~~

Once, these two C++ scripts are ready, it is necessary to add the required dependencies in the ``package.xml`` file, which was already added when creating this package. See that in the ``package.xml`` file it is present the tags: ``<depend>rclcpp</depend>`` and ``<depend>tutorial_interfaces</depend>``.

Next, add the following in the ``CMakeLists.txt`` file:

.. code-block:: console

   add_executable(sphere_publisher src/sphere_publisher.cpp)
   ament_target_dependencies(sphere_publisher rclcpp tutorial_interfaces)    

   add_executable(sphere_listener src/sphere_listener.cpp)
   ament_target_dependencies(sphere_listener rclcpp tutorial_interfaces)    

   install(TARGETS
   sphere_publisher
   sphere_listener
   DESTINATION lib/${PROJECT_NAME})


Build and run the custom msg
~~~~~~~~~~~~~~~~

Build the package with either of these commands:

.. code-block:: console

   colcon build
   colcon build --packages-select testing_interfaces_cpp

Source the setup file:

.. code-block:: console
   
   source install/setup.bash

And run the ``sphere_publisher`` node that was recently created. 

.. code-block:: console
   
   ros2 run testing_interfaces_cpp sphere_publisher

The result should be like the following:

.. code-block:: console
   
   [INFO] [1712745603.801777360] [sphere_publisher]: Publishing sphere params (x, y, z, radius): x = 0, y = 1, z = 2, radius = 10
   [INFO] [1712745604.301748381] [sphere_publisher]: Publishing sphere params (x, y, z, radius): x = 1, y = 1, z = 2, radius = 10
   [INFO] [1712745604.801799750] [sphere_publisher]: Publishing sphere params (x, y, z, radius): x = 2, y = 1, z = 2, radius = 10
   ...

`Open a new terminal`_ and execute the ``sphere_listener`` node:

.. _open a new terminal: https://alex-readthedocs-test.readthedocs.io/en/latest/Installation.html#opening-a-new-terminal

.. code-block:: console
   
   ros2 run testing_interfaces_cpp sphere_listener

The expected result is:

.. code-block:: console
   
   [INFO] [1712745636.802284213] [sphere_listener]: I heard: x = 66, y = 1, z = 2, radius = 10
   [INFO] [1712745637.302150919] [sphere_listener]: I heard: x = 67, y = 1, z = 2, radius = 10
   [INFO] [1712745637.802143924] [sphere_listener]: I heard: x = 68, y = 1, z = 2, radius = 10
   ...

Finally, it can also be checked the echo of the messages arriving to the desired topic. `Open a new terminal`_ and execute:

.. _open a new terminal: https://alex-readthedocs-test.readthedocs.io/en/latest/Installation.html#opening-a-new-terminal

.. code-block:: console
   
   ros2 topic echo /sphere_topic

The expected result is:

.. code-block:: console
   
   center:
      x: 132.0
      y: 1.0
      z: 2.0
   radius: 10.0
   ---
   center:
      x: 133.0
      y: 1.0
      z: 2.0
   radius: 10.0
   ---
   ...

At this point, it can be seen that the custom message ``Sphere.msg`` that was created is being used successfully.

Testing the AddThreeInts custom srv in a python package
-----------------------

This example will be worked in the ``testing_interfaces_cpp`` package.

Make sure to be in a brand new terminal window and no ROS commands are currently running.

The code
~~~~~~~~~~~~~~~~

Inside ``testing_interfaces_cpp/src`` create a C++ script, name it ``add_service_node.cpp``. 

Copy this content into the new python script. 

.. code-block:: console

   #include "rclcpp/rclcpp.hpp"
   #include "tutorial_interfaces/srv/add_three_ints.hpp"                                        

   #include <memory>

   void add(const std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Request> request,     
            std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Response>       response)  
   {
      response->sum = request->a + request->b + request->c;                                      
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld" " c: %ld",  
                     request->a, request->b, request->c);                                         
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
   }

   int main(int argc, char **argv)
   {
      rclcpp::init(argc, argv);

      std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_server");   

      rclcpp::Service<tutorial_interfaces::srv::AddThreeInts>::SharedPtr service =               
         node->create_service<tutorial_interfaces::srv::AddThreeInts>("add_three_ints",  &add);   

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add three ints.");                     

      rclcpp::spin(node);
      rclcpp::shutdown();
   }

Notice that this code is very similar to the service script that was studied  :ref:`previously<Writting service and client. C++/Writting the service node. C++>`.

Check the important changes in this script.

.. code-block:: console

   #include "tutorial_interfaces/srv/add_three_ints.hpp"  
   ...
   rclcpp::Service<tutorial_interfaces::srv::AddThreeInts>::SharedPtr service =               
         node->create_service<tutorial_interfaces::srv::AddThreeInts>("add_three_ints",  &add);
   ...
   void add(const std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Request> request,     
            std::shared_ptr<tutorial_interfaces::srv::AddThreeInts::Response>       response)  
   {
      response->sum = request->a + request->b + request->c;                                      
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld" " c: %ld",  
                     request->a, request->b, request->c);                                         
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
   }

- It is important to correctly import the required service. In this case notice that ``add_three_ints.hpp`` is being imported when the actual created service was named ``AddThreeInts.srv``. If ``#include "tutorial_interfaces/srv/AddThreeInts.hpp"``  were to be imported, a compilation error would have arisen stating:

.. code-block:: console
   
   fatal error: tutorial_interfaces/srv/AddThreeInts.hpp: No such file or directory

This happens because in ROS 2, the naming convention for service files (.srv) is usually converted to snake_case when generating corresponding C++ code. So, a service file named ``AddThreeInts.srv``, when generating C++ code, it will typically be converted to ``add_three_ints.hpp``.
- The service node will now be of type ``AddThreeInts``, and the service name is also modified to be ``add_three_ints``. The service name could have stayed the same, but it is better to name the services accordingly.
- Finally, the callback function, instead of summing two values it will summ the three parameters in the request section of the service. 

Next, create a client node for this service. Inside ``testing_interfaces_cpp/src`` create a C++ script, name it ``add_client_node.cpp``. 

Copy this content into the new python script. 

.. code-block:: console

   #include "rclcpp/rclcpp.hpp"
   #include "tutorial_interfaces/srv/add_three_ints.hpp"                                       // CHANGE

   #include <chrono>
   #include <cstdlib>
   #include <memory>

   using namespace std::chrono_literals;

   int main(int argc, char **argv)
   {
      rclcpp::init(argc, argv);

      if (argc != 4) { // CHANGE
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_three_ints_client X Y Z");      // CHANGE
            return 1;
      }

      std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_client");  // CHANGE
      rclcpp::Client<tutorial_interfaces::srv::AddThreeInts>::SharedPtr client =                // CHANGE
         node->create_client<tutorial_interfaces::srv::AddThreeInts>("add_three_ints");          // CHANGE

      auto request = std::make_shared<tutorial_interfaces::srv::AddThreeInts::Request>();       // CHANGE
      request->a = atoll(argv[1]);
      request->b = atoll(argv[2]);
      request->c = atoll(argv[3]);                                                              // CHANGE

      while (!client->wait_for_service(1s)) {
         if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
         }
         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }

      auto result = client->async_send_request(request);
      // Wait for the result.
      if (rclcpp::spin_until_future_complete(node, result) ==
         rclcpp::FutureReturnCode::SUCCESS)
      {
         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
      } else {
         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_three_ints");    // CHANGE
      }

      rclcpp::shutdown();
      return 0;
   }

The code is very similar to the client node that was studied  :ref:`previously<Writting service and client. C++/Client node in C++>`.

Again, the relevant changes here, have to do with dealing with the appropriate import of the required library, the service name and service type. 

Dependencies and CMakeLists file
~~~~~~~~~~~~~~~~

Once, these two C++ scripts are ready, it is necessary to add the required dependencies in the ``package.xml`` file, which was already added when creating this package. See that in the ``package.xml`` file it is present the tags: ``<depend>rclcpp</depend>`` and ``<depend>tutorial_interfaces</depend>``.

Next, add the following to the ``CMakeLists.txt`` file:

.. code-block:: console

   ...
   add_executable(add_service_node src/add_service_node.cpp)
   ament_target_dependencies(add_service_node rclcpp tutorial_interfaces) 

   add_executable(add_client_node src/add_client_node.cpp)
   ament_target_dependencies(add_client_node rclcpp tutorial_interfaces) 
   ...
   install(TARGETS
      ...
      add_service_node
      add_client_node
      DESTINATION lib/${PROJECT_NAME})

Considering the changes for the custom msg as well, the final ``CMakeLists.txt`` file should look like this:

.. code-block:: console
   cmake_minimum_required(VERSION 3.8)
   project(testing_interfaces_cpp)

   if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      add_compile_options(-Wall -Wextra -Wpedantic)
   endif()

   # find dependencies
   find_package(ament_cmake REQUIRED)
   find_package(rclcpp REQUIRED)
   find_package(tutorial_interfaces REQUIRED)

   add_executable(sphere_publisher src/sphere_publisher.cpp)
   ament_target_dependencies(sphere_publisher rclcpp tutorial_interfaces)    

   add_executable(sphere_listener src/sphere_listener.cpp)
   ament_target_dependencies(sphere_listener rclcpp tutorial_interfaces)    

   add_executable(add_service_node src/add_service_node.cpp)
   ament_target_dependencies(add_service_node rclcpp tutorial_interfaces) 

   add_executable(add_client_node src/add_client_node.cpp)
   ament_target_dependencies(add_client_node rclcpp tutorial_interfaces) 

   install(TARGETS
      sphere_publisher
      sphere_listener
      add_service_node
      add_client_node
      DESTINATION lib/${PROJECT_NAME})

   if(BUILD_TESTING)
      find_package(ament_lint_auto REQUIRED)
      # the following line skips the linter which checks for copyrights
      # comment the line when a copyright and license is added to all source files
      set(ament_cmake_copyright_FOUND TRUE)
      # the following line skips cpplint (only works in a git repo)
      # comment the line when this package is in a git repo and when
      # a copyright and license is added to all source files
      set(ament_cmake_cpplint_FOUND TRUE)
      ament_lint_auto_find_test_dependencies()
   endif()

   ament_package()

Build and run the custom srv
~~~~~~~~~~~~~~~~

Build the package with either of these commands:

.. code-block:: console

   colcon build
   colcon build --packages-select testing_interfaces_cpp

Source the setup file:

.. code-block:: console
   
   source install/setup.bash

And run the ``add_service_node`` node that was recently created. 

.. code-block:: console
   
   ros2 run testing_interfaces_cpp add_service_node

As a result, this will be shown in the terminal, meaning that the service is ready to be consumed. 

.. code-block:: console

   [INFO] [1712746785.956178405] [rclcpp]: Ready to add three ints.

`Open a new terminal`_ and execute the ``add_client_node`` node:

.. _open a new terminal: https://alex-readthedocs-test.readthedocs.io/en/latest/Installation.html#opening-a-new-terminal

.. code-block:: console
   
   ros2 run testing_interfaces_cpp add_client_node 8 9 5

The expected result is:

.. code-block:: console
   
   [INFO] [1712746812.713518561] [rclcpp]: Sum: 22

Finally, the ``add_three_ints service`` can also be called from the terminal directly, without the necessity of coding a client node. `Open a new terminal`_ and execute:

.. _open a new terminal: https://alex-readthedocs-test.readthedocs.io/en/latest/Installation.html#opening-a-new-terminal

.. code-block:: console
   
   ros2 service call /add_three_ints tutorial_interfaces/srv/AddThreeInts "{a: 1, b: 2, c: 9}"

The expected result is:

.. code-block:: console
   
   requester: making request: tutorial_interfaces.srv.AddThreeInts_Request(a=1, b=2, c=9)

   response:
   tutorial_interfaces.srv.AddThreeInts_Response(sum=12)

At this point, it can be seen that the custom service ``AddThreeInts.srv`` that was created is being used successfully.