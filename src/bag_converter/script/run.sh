roslaunch perception-offline-test-label-data.launch bags:=/itri-label-toolchain/src/data/raw/2022-08-17-16-10-32_13.bag
python ros_image_saver.py --topic /lucid_cameras_x00/gige_30_f_hdr/h265/rgb8 --save_dir /itri-label-toolchain/src/data/raw
python ros_image_saver.py --topic /lucid_cameras_x00/gige_60_f_hdr/h265/rgb8 --save_dir /itri-label-toolchain/src/data/raw
rosrun pcl_ros bag_to_pcd /itri-label-toolchain/src/data/2022-08-17-16-10-32_13.bag /velodyne/top_vls128 /itri-label-toolchain/src/data/raw/lidar
