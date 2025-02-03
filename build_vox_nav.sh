# build casadi
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DWITH_IPOPT=true --packages-select casadi
# build ACADO
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DACADOS_WITH_QPOASES=ON --packages-select ACADO
sudo cp install/casadi/lib/libcasadi.so* /usr/local/lib/
# build vox_nav except vox_nav_control and vox_nav_misc
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-skip-regex archive --packages-skip vox_nav_control vox_nav_misc
source install/setup.bash
source build/ACADO/acado_env.sh
# build vox_nav_control
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DWITH_IPOPT=true --packages-select vox_nav_control