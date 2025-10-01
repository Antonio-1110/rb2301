# RB2301 CA1 - Obstacle Avoidance
## 1. Extract compressed rb2301 folder to desired directory (e.g. ~/Documents)


## 2. Change directory to extracted folder then colcon build
```
cd ~/Documents/rb2301	
colcon build --symlink-install
```


## 3. Start the gazebo sim in a terminal 
```
./gz_ca1.sh
```

   If unable to run, you may need to assign permisions to run .sh files, if you have never done so before. Do this with:
```
chmod +x *.sh
```
   After that, run the gz_ca1.sh script as usual. A randomly generated world will be loaded with the robot and coke cans as obstacles. Modify can spawning properties under _obstacle_generator.py_ in the rb2301_ca1 package.

   
## 4. In another terminal, run the obstacle avoidance script.
```
./ca1.sh
```
The default behaviour is to move forward and print the LiDAR scan ranges. 
Add your code in the relevant portion within _obstacle_avoidance.py_ in the rb2301_ca1 package.
