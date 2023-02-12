#ifndef SENSOR_RANGE_H
#define SENSOR_RANGE_H

#include <string>
#include <envir.h>
#include <robot.h>
#include <sensor.h>

using std::endl; using std::cout;

namespace arpro
{

//3.1, Q1/
class RangeSensor : public Sensor
{
public :
    RangeSensor(Robot &_robot, double _x , double _y , double _theta ):
        Sensor( _robot , _x , _y , _theta ){ // call the Sensor constructor
        // the RangeSensor constructor does nothing
        }

    virtual void correctTwist(Twist &_t) {
        //3.1, Q5/
        //Data: Current twist T , current range s, gain g, minimum range s m
        //Result: Modified twist T

        double g = .1;
        double sm = 0.1;
        if (_t.vx > g*(s_-sm))
            _t.vx = g*(s_-sm);
    }

    // update from current sensor pose
    virtual void update(const Pose &_p) {
        //3.1, Q4/
        // Compute the distance to the nearest wall in the direction of the x-axis
        Pose p1 , p2 ;

        // Initialisation of the distance : we take a value too high so that it will be rewritten in the next for loop
        double dist = 20;

        for (int i=0; i < envir_ -> walls.size() ; ++i) // Loop through all the walls
        {
            // [p1p2] is a segment of wall
            p1 = envir_ -> walls[i];
            p2 = envir_ -> walls[(i+1)% envir_ -> walls.size()];

            double x = _p.x;
            double y = _p.y;
            double theta = _p.theta;
            double x1 = p1.x; double y1 = p1.y;
            double x2 = p2.x; double y2 = p2.y;

            // Compute the distance between the sensor and the wall along x-axis
            double num = x1*y2 - x1*y - x2*y1 + x2*y + x*y1 - x*y2;
            double denom = x1*sin(theta) - x2*sin(theta) - y1*cos(theta) + y2*cos(theta);

            // If denom is 0, the wall and the sensor orientation are parallel, so no need to consider dist
            // If the distance is positive (the wall is in front of the sensor) and is the minimum, keep it
            if ((denom != 0) && (num/denom > 0) && (num/denom < dist))
                dist = num/denom; // Rewrite dist to keep the minimum
        }

        // Updates the attribute s_ of the sensor with the distance to the nearest wall
        s_ = dist;

        // Print the distance
        //std::cout << "Distance to the nearest wall: " << dist << std::endl;
    }
};

}

#endif // SENSOR_RANGE_H
