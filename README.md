# arceya_os


###  Setup Docker Container and build arceya_os on your PC

__Extract the docker zip file provided__
> Change the following line from `docker-compose.yaml` to your specified directory where arceya_os is cloned/installed
- `- /home/archit/Documents/Projects/arceya/arceya_os:/arceya_os`

__Move inside that directory and execute following commands in the terminal__
> `docker build -t arceya/dev_noetic:base .`

> `docker-compose up -d`

> `xhost local:root`

> `docker exec -it focal_arceya-dev-noetic_1 bash`


### Setup ROS environment

__Execute these commands once you have entered inside docker container (i.e., executed abover four commands)__
> `source /ros_entrypoint.sh `

> `cd /arceya_os/`

__Clean if any prior build files present__

    rm -r .catkin_tools/
    rm -r build/
    rm -r devel/
    rm -r logs/


### Build catkin workspace

> `catkin build`

__source once the build is completed successfully__

> `source /arceya_os/devel/setup.bash`


### Enter docker container in another terminal

__Use this command to enter docker-container from another terminal or the same one__

`docker exec -it focal_arceya-dev-noetic_1 bash`


### Common Launch File to run the software

__There are different input arguments while launching this file__


`roslaunch arc_ms arceya_os.launch mode:=sw gazebo:=false usb_cam:=true`

> __`mode:=sw`__  
> `Options: sw/hw`
> `Default value: sw`

> __`gazebo:=false`__  
> `Options: false/true`
> `Default value: false`

> __`usb_cam:=false`__  
> `Options: false/true`
> `Default value: false`


### -----------------------------------------------------------------------------------------------------------------


### Run Onboard

__[HW/SW]: Run rosbridge websocket server to subscribe and publish ros topics/msgs/services over websocket service__
> `roslaunch rosbridge_server rosbridge_websocket.launch`


__[HW/SW]: Run arc_ms launch file to verify auth status using arc_auth service__
> `roslaunch arc_ms arc_ms.launch`


__[SW/OPTIONAL]: Launch mecanum robot in Gazebo simulator__
> `roslaunch mecanum_drive gazebo.launch`


__[HW/SW]: Launch differential drive inverse kinematics controller from cmd vel controller package__
> `roslaunch cmd_vel_controller dd_controller.launch`


__[HW/SW]: Run usb web camera node from usb-cam package to stream it on webapp__
> `rosrun usb_cam usb_cam_node _pixel_format:=yuyv`


__[HW]: Run rosserial node to enable arduino to subscribe respective nodes and operate the robot__
> `rosrun rosserial_python serial_node.py /dev/ttyACM0`


### Run http server and web page on same device

__[HW/SW]: Move to web directory in arceya_os to host the server__
> `cd /arceya_os/src/web/user_dev/`

__[HW/SW]: Serving HTTP on 0.0.0.0 port 8000 (http://0.0.0.0:8000/)__
> `python3 -m http.server`

__[HW/SW]: Open your browser and enter this link__
> http://localhost:8000/