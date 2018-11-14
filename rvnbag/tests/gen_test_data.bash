#! /bin/bash
# simple script to generate test data for rvnabg_play.py testing
 
roscore > /dev/null&
rospid=$!

echo "running ros core.. waiting for full come-up"

# wait for roscore to come up
until rostopic list > /dev/null; do sleep 0.25; done
echo "roscore has launched"

echo "publishing..."
rostopic pub -r 10 "/test/float64" std_msgs/Float64 '{data: 42.0}'  > /dev/null &
rospubpid=$!

echo "recording..."
rosbag record -a -l 10 "/testfloat64" -O "tests/testdata/float64.bag" & #> /dev/null &
rosbagpid=$!

sleep 10.0 # Hard code guess
echo "...done!"

echo "killing $rospubpid, $rosbagpid, $rospid"

kill -SIGINT $rospubpid
kill -9 $rosbagpid
kill -9 $rospid

