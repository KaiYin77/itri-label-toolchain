roslaunch perception-offline-test-label-data.launch bags:=2022-08-17-16-10-32_13.bag
python ros_image_saver.py --bag decode_filter_2022-08-17-16-10-32_13.bag --topic /lucid_cameras_x00/gige_30_f_hdr/h265/rgb8 --save_dir /sdc/ros/src/itri/itri-label-toolchain/src/data/raw/camera
rosrun pcl_ros bag_to_pcd /sdc/ros/src/itri/itri-label-toolchain/src/data/raw/2022-08-17-16-10-32_13.bag /velodyne/top_vls128 /sdc/ros/src/itri/itri-label-toolchain/src/data/raw/lidar
