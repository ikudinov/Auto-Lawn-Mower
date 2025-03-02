# остановка сервиса, занимающего USART
echo "orangepi" | sudo -S systemctl stop serial-getty@ttyS0.service

# старт контейнера с роботом
cd ~/Auto-Lawn-Mower/ROS2

while true; do
  docker run -it \
    -v ./:/opt/auto-lawn-mower \
    --device=/dev/ttyS0 \
    -d \
    -p 80:80 \
    -p 4041:4041 \
    ros2-lawn-mower \
    bash -c "source install/setup.bash && ros2 launch lawn_mower_web_controller web_controller.launch.py" \
    && break
done

# ожидание старта контейнера
while ! nc -z localhost 80; do
  sleep 1
done

# браузер показывает интерфейс робота
chromium http://localhost/ --start-fullscreen

