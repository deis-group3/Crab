yes | sudo ufw enable
sudo ufw deny 5000
sudo ufw disable
cd ../../../../
source mp-venv/bin/activate
python3 ros2_ws/src/crab/crab/main_gps.py