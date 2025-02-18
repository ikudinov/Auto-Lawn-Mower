sudo rm -rf ../build/
sudo rm -rf ../install/

docker run -it \
  -v "${PWD}/../":/opt/auto-lawn-mower \
  ros2-lawn-mower \
  bash -c "colcon build"