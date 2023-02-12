#include <iostream>
#include <math.h>
#include <cmath>

#include <robot.h>
#include <envir.h>
#include <sensor.h>
#include <sensor_range.h>
#include <sensor_bearing.h>

using namespace std;
using namespace arpro;

int main(int argc, char **argv)
{

  // default environment with moving target
  Environment envir;
  // sensors gets measurements from this environment
  Sensor::setEnvironment(envir);

  // init robot at (0,0,0)
  Robot robot("R2D2", 0,0,0);
  robot.initWheel(0.07,0.3,10.);
  envir.addRobot(robot);
 Pose p{0.1,0,0};

  RangeSensor Sensor1(robot,p.x,p.y,p.theta);
  Pose p2{0,0.05,0};

   //RangeSensor Sensor2(robot,p2.x,p2.y,p2.theta);


   Robot robot1("Ak111", 1, 1,0);
   robot1.initWheel(0.05, 0.3, 10.0);
   envir.addRobot(robot1);
   BearingSensor b_sensor(robot1,p.x,p.y,p.theta);
  // simulate 100 sec
  while(envir.time() < 100)
     {
    cout << "---------------------" << endl;

    // update target position
    envir.updateTarget();

    // try to follow target
    robot.goTo(envir.target());
    //robot1.moveWithSensor(Twist(0.4,0,0));
     robot1.goTo(robot.pose());
  }

  // plot trajectory
  envir.plot();

}
