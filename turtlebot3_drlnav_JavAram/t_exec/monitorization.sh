while true; do
    # /usr/bin/python3 /opt/ros/humble/bin/ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py world_path:=/ho
    ps -p 1225 -o pid,etime,time,cmd >> registro_proceso_launch_irobot_gazebo.log
    echo "----" >> registro_proceso_1102.log
    
    # /usr/bin/python3 /opt/ros/humble/bin/ros2 run turtlebot3_drl environment
    ps -p 1881 -o pid,etime,time,cmd >> registro_proceso_env.log
    echo "----" >> registro_proceso_1631.log
    
    # /usr/bin/python3 /opt/ros/humble/bin/ros2 run turtlebot3_drl train_agent dqn
    ps -p 1917 -o pid,etime,time,cmd >> registro_proceso_dqn.log
    echo "----" >> registro_proceso_1672.log
    
    # /usr/bin/python3 /opt/ros/humble/bin/ros2 run turtlebot3_drl gazebo_goals
    ps -p 1976 -o pid,etime,time,cmd >> registro_proceso_gazebo_goals.log
    echo "----" >> registro_proceso_1732.log
    
    sleep 30
done
