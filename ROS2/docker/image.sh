docker rmi -f $(docker images | grep 'ros2-lawn-mower')

docker build -t ros2-lawn-mower .