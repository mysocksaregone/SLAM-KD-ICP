"""
Launch file for the ICP localizer.

WHAT TO WRITE
    Copy launch/test_pf.py and change only the localizer Node:
        executable='pf.py'   ->  executable='icp_localizer.py'
        name='my_pf'         ->  name='icp_localizer'
    Keep the map_server + lifecycle_manager nodes and the use_sim_time arg
    (bag playback uses --clock, so sim time must be true).

    Optional: also launch rviz2 with rviz/icp.rviz once you've saved that
    config (Node(package='rviz2', executable='rviz2', arguments=['-d', path])).

USAGE
    ros2 launch robot_localization icp_localizer.py map_yaml:=<abs path>/maps/mac_1st_floor_final.yaml
"""
