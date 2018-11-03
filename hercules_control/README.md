	<node name="controller_spawner" pkg="controller_manager" type="spawner"
	    args="hercules_joint_publisher hercules_velocity_controller --shutdown-timeout 1" />
  


        
    <rosparam command="load" file="$(find hercules_control)/config/control.yaml" />