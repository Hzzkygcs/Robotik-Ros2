echo hi
source ~/.profile
eval "$(cat ~/.bashrc | tail -n +10)"


cd "$TUGAS"
colcon build --symlink-install
