#include <iostream>
#include <math.h>
#include <robot.h>
#include <sensor.h>

using namespace arpro;
using namespace std;

Environment* Sensor::envir_ = nullptr;

Robot::Robot(string _name, double _x, double _y, double _theta)
{
    pose_.x = _x;
    pose_.y = _y;
    pose_.theta = _theta;

    name_ = _name;

    // init position history
    x_history_.push_back(_x);
    y_history_.push_back(_y);
}


void Robot::moveXYT(double _vx, double _vy, double _omega)
{
    // update position
    pose_.x += _vx*dt_;
    pose_.y += _vy*dt_;
    pose_.theta += _omega*dt_;

    // store position history
    x_history_.push_back(pose_.x);
    y_history_.push_back(pose_.y);
}
void Robot::initWheel(double r, double b, double wmax)
{
    //r wheel radius, d distance to the base
    _r = r;
    _b = b;
    _wmax = wmax;
    wheels_init_=1;
}



void Robot::rotateWheels(double _left, double _right)
{
    // to fill up after defining an initWheel method

    if (wheels_init_ == 1){

        //2.3, Q2/
        double a = max(std::abs(_left)/_wmax, std::abs(_right)/_wmax);
            if (a <= 1)
                a=1;
            else
                ;//cout << "Entered angular velocity higher than the limit, the velocities have been scaled" << endl;
        double wl = _left/a;
        double wr = _right/a;

        //2.2, Q6/
        double v= _r*(wl+wr)/2;
        double omega= _r*(wl-wr)/(2*_b);

        double vx = v*cos(pose_.theta);
        double vy = v*sin(pose_.theta);
        moveXYT(vx, vy, omega);
    } else { //2.2, Q7/
        cout << "You forgot to initialize the wheels radius and base distance !" << endl;
    }





}


// move robot with linear and angular velocities
void Robot::moveVW(double _v, double _omega)
{
    // to fill up
    //2.2, Q3/
        //double vx = _v*cos(pose_.theta);
        //double vy = _v*sin(pose_.theta);
        //moveXYT(vx, vy, _omega);

        //2.3, Q3/
        double wl = (_v + _b*_omega)/_r;
        double wr = (_v - _b*_omega)/_r;
        rotateWheels(wl,wr);

}




// try to go to a given x-y position
void Robot::goTo(const Pose &_p)
{
    // error in robot frame
    Pose error = _p.transformInverse(pose_);

    // try to do a straight line with sensor constraints
    moveWithSensor(Twist(error.x, error.y, 0));
}


void Robot::moveWithSensor(Twist _twist)
{
    // to fill up, sensor measurement and twist checking

    for (auto sensor : sensors_){
        sensor->correctRobotTwist(_twist);
        sensor->updateFromRobotPose(pose());}


    // uses X-Y motion (perfect but impossible in practice)
    //moveXYT(_twist.vx, _twist.vy,_twist.w);

    // to fill up, use V-W motion when defined

    double alpha = 20;
    double v_ = _twist.vx;
    double omega_ = alpha*_twist.vy + _twist.w;
    moveVW(v_,omega_);
}


void Robot::printPosition()
{
    cout << "Current position: " << pose_.x << ", " << pose_.y << endl;
}
