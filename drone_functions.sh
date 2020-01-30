function connect(){
	#source ~/DRDO/run_all_pixhawk.sh
	ssh  root@uav1 "source jetson_px4.sh" & 
	ssh  root@uav2 "source jetson_px4.sh" &
	ssh  root@uav3 "source jetson_px4.sh" 

}
#/home/manik/Downloads/px4_clone/src

function arm(){
	#source ~/DRDO/arm_drones.sh		#complete
	rosservice call /uav1/mavros/cmd/arming "value: true" &
	rosservice call /uav2/mavros/cmd/arming "value: true" &
	rosservice call /uav3/mavros/cmd/arming "value: true"
}

function takeoff(){
	#source ~/DRDO/mission.sh		#complete
	rosservice call /uav1/mavros/set_mode "custom_mode: 'AUTO.MISSION'" &
	rosservice call /uav2/mavros/set_mode "custom_mode: 'AUTO.MISSION'" &
	rosservice call /uav3/mavros/set_mode "custom_mode: 'AUTO.MISSION'" 

}

function return_home(){
	#source ~/DRDO/mission.sh		#complete
	rosservice call /uav1/mavros/set_mode "custom_mode: 'AUTO.RTL'" &
	rosservice call /uav2/mavros/set_mode "custom_mode: 'AUTO.RTL'" &
	rosservice call /uav3/mavros/set_mode "custom_mode: 'AUTO.RTL'" 

}

function disarm(){
	#source ~/DRDO/disarm_drones.sh		#complete
	rosservice call /uav1/mavros/cmd/arming "value: false" &
	rosservice call /uav2/mavros/cmd/arming "value: false" &
	rosservice call /uav3/mavros/cmd/arming "value: false"
}

function detect_object(){
	ssh aero@uav1 "source detect_object.sh" &
	ssh aero@uav2 "source detect_object.sh" &
	ssh aero@uav3 "source detect_object.sh"
}

function cluster_gps(){
	source ~/DRDO/change_mode.sh
}

function plot_on_map(){
	source ~/DRDO/change_mode.sh
}

function land(){
	#source ~/DRDO/land.sh		#complete
	rosservice call /uav1/mavros/set_mode "custom_mode: 'AUTO.LAND'" &
	rosservice call /uav2/mavros/set_mode "custom_mode: 'AUTO.LAND'" &
	rosservice call /uav3/mavros/set_mode "custom_mode: 'AUTO.LAND'"
}
