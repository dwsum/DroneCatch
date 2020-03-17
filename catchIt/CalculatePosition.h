//
// Created by li292 on 3/2/20.
//we could make this class static instead so that it doesn't have to be instantiated....
//

#ifndef DRONECATCH_CALCULATEPOSITION_H
#define DRONECATCH_CALCULATEPOSITION_H

#include <cmath>


class CalculatePosition {

private:
    double initialXVelocity;        //want in meters/second--should be constant!
    double initialYVelocity;        //want in meters/second--should be constant! to small for air resistance
    double initialZVelocity;        //want in meters/second
    double distanceToBall;          //in meters
    double droneHeight;             //height drone is at?

public:
    CalculatePosition() {

    }

    CalculatePosition(double initialX, double initialY, double initialZ, double distanceToBall, double droneHeight) {
        initialXVelocity = initialX;
        initialYVelocity = initialY;
        initialZVelocity = initialZ;
        this->distanceToBall = distanceToBall;
        this->droneHeight = droneHeight;
    }

    double getTime(double landingAltitude) {
        double timeOne;
        double timeTwo;
        double initialVzSquared = pow(initialZVelocity, 2);
        double altDiff = (distanceToBall + droneHeight) - landingAltitude;
        double radical = sqrt(initialVzSquared + (4*4.9*altDiff)); //these numbers are from the quadric equation using the projectile motion formula
        timeOne = (-initialZVelocity + radical) / 9.8;
        timeTwo = (-initialZVelocity - radical) / 9.8;
        if(timeOne > timeTwo)
            return timeOne;
        return timeTwo;
    }

    double getFinalX(double time) {
        return (initialXVelocity * time);
    }

    double getFinalY(double time) {
        return (initialYVelocity * time);
    }

};


#endif //DRONECATCH_CALCULATEPOSITION_H
