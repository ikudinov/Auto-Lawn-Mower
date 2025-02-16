rm -rf ../build/
rm -rf ../install/

docker run -it \
  -v "${PWD}/../":/opt/auto-lawn-mower \
  -v /dev/ttyS0:/dev/ttyS0 \
  -p 4040:4040 \
  -p 4041:4041 \
  ros2-lawn-mower \
  bash -c "colcon build && source install/setup.bash && ros2 launch lawn_mower_web_controller web_controller.launch.py"