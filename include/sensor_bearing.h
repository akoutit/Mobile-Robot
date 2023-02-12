#ifndef SENSOR_BEARING_H
#define SENSOR_BEARING_H

#include <string>
#include <math.h>
#include <cmath>
#include <envir.h>
#include <robot.h>
#include <sensor.h>

using std::endl; using std::cout;

namespace arpro
{

//3.2, Q1/
class BearingSensor : public Sensor
{
public :
    BearingSensor(Robot &_robot, double _x , double _y , double _theta ):
        Sensor( _robot , _x , _y , _theta ){ // call the Sensor constructor
        // the BearingSensor constructor does nothing
        }

    virtual void correctTwist(Twist &_t) {
        //3.2, Q4/
        //Data: Current twist T , current bearing s, gain g
        //Result: Modified twist T
        double g = 0.5;
        _t.w = _t.w - g*s_;
    }

    // update from current sensor pose
    virtual void update(const Pose &_p) {
        // look for first other robot
        for (auto other : envir_ -> robots_){
            if (other != robot_)
            {
                double xr = other->pose().x;
                double yr = other->pose().y;
                // compute angle between sensor and detected robot
                double alpha = atan2(yr-_p.y, xr-_p.x) - _p.theta;


                break;
            }
            double alpha = 0;
        }
    }
};

}

#endif // SENSOR_BEARING_H
