#!/bin/bash
for number in {-5..5}
do
echo "$number "
rosservice call Arm/add_point_to_path "x: $(($number*5)) 
y: 25.0 
z: 15.0"
done
