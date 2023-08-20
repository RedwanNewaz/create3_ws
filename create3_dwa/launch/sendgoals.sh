

# Switch on the user's choice
case $1 in
    1)
        echo "You chose 1."
        ros2 topic pub --once /ac31/goal_pose geometry_msgs/msg/PoseStamped '{pose:{position:{x: -2.0, y: -2.0, z: 0.0 } } }'
        ros2 topic pub --once /ac32/goal_pose geometry_msgs/msg/PoseStamped '{pose:{position:{x: 2.0, y: 2.0, z: 0.0 } } }'
        ;;
    2)
        echo "You chose 2."
        ros2 topic pub --once /ac32/goal_pose geometry_msgs/msg/PoseStamped '{pose:{position:{x: -2.0, y: -2.0, z: 0.0 } } }'
        ros2 topic pub --once /ac31/goal_pose geometry_msgs/msg/PoseStamped '{pose:{position:{x: 2.0, y: 2.0, z: 0.0 } } }'
        ;;
    3)
        echo "You chose 3."
        ;;
    *)
        echo "Invalid choice."
        ;;
esac