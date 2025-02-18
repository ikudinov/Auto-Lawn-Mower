# sudo rm -rf ../build/
# sudo rm -rf ../install/
sudo systemctl stop serial-getty@ttyS0.service

docker run -it \
  -v "${PWD}/../":/opt/auto-lawn-mower \
  --device=/dev/ttyS0 \
  -p 80:80 \
  -p 4041:4041 \
  ros2-lawn-mower \
  bash -c "source install/setup.bash && ros2 launch lawn_mower_web_controller web_controller.launch.py"