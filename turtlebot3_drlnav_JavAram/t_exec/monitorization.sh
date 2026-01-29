while true; do
    # /usr/bin/python3 /opt/ros/humble/bin/ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py world_path:=/ho
    ps -p 1102 -o pid,etime,time,cmd >> registro_proceso_1102.log
    echo "----" >> registro_proceso_1102.log
    
    # /usr/bin/python3 /opt/ros/humble/bin/ros2 run turtlebot3_drl environment
    ps -p 1631 -o pid,etime,time,cmd >> registro_proceso_1631.log
    echo "----" >> registro_proceso_1631.log
    
    # /usr/bin/python3 /opt/ros/humble/bin/ros2 run turtlebot3_drl train_agent ddpg
    ps -p 1672 -o pid,etime,time,cmd >> registro_proceso_1672.log
    echo "----" >> registro_proceso_1672.log
    
    # /usr/bin/python3 /opt/ros/humble/bin/ros2 run turtlebot3_drl gazebo_goals
    ps -p 1732 -o pid,etime,time,cmd >> registro_proceso_1732.log
    echo "----" >> registro_proceso_1732.log
    
    sleep 30
done
