- roslaunch kortex_driver kortex_driver.launch ip_address:=192.168.1.10 start_rviz:=false start_moveit:=false gripper:=robotiq_2f_85
- roslaunch asa_ros asa_ros.launch anchor_id:=<anchor_id>
- roslaunch ros_tcp_endpoint endpoint.launch
- roslaunch kinova_positional_control full_mapping.launch anchor_id:=<anchor_id>

To locate anchor:
- roslaunch holo_project base_live 
