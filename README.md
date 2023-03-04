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


### Run simulator

__Launch mecanum robot in Gazebo simulator__
> `roslaunch mecanum_drive gazebo.launch`

__Run rosbridge websocket server to subscribe and publish ros topics/msgs/services over websocket service__
> `roslaunch rosbridge_server rosbridge_websocket.launch`


### Run http server and web page on same device

__Move to web directory in arceya_os to host the server__
> `cd /arceya_os/src/web/user_dev/`

__Serving HTTP on 0.0.0.0 port 8000 (http://0.0.0.0:8000/)__
> `python3 -m http.server`

__Open your browser and enter this link__
> http://localhost:8000/