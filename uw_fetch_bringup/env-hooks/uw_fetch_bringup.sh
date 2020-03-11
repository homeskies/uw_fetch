fetch_viz() {
    rosrun rviz rviz -d $(rospack find uw_fetch_bringup)/config/fetch.rviz
}

fetch_key_teleop() {
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py $@
}

make_map() {
   rosrun hector_mapping hector_mapping _map_size:=2048 _map_resolution:=0.05 _pub_map_odom_transform:=true _scan_topic:=/base_scan _use_tf_scan_transformation:=false _map_update_angle_thresh:=2.0 _map_update_distance_thresh:=0.10 _scan_subscriber_queue_size:=1 _update_factor_free:=0.39 _update_factor_occupied:=0.85 _base_frame:=base_link _map_frame:=map
}