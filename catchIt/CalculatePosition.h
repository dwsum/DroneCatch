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
    double initialZHeight;          //in meters...Drew stored the initial Z Height here though.
    double droneHeight;             //height drone is at?
    double initialXPosition;        //meters
    double initialYPosition;        //meters

public:
    CalculatePosition() {

    }

    CalculatePosition(double initialX, double initialY, double initialZ, double distanceToBall, double droneHeight, double xPosition, double yPosition) {
        initialXVelocity = initialX;
        initialYVelocity = initialY;
        initialZVelocity = initialZ;
        this->initialZHeight = distanceToBall;
        this->droneHeight = droneHeight;
        initialXPosition = xPosition;
        initialYPosition = yPosition;
    }

    double getTime(double landingAltitude) {
//        double timeOne;
//        double timeTwo;
//        double initialVzSquared = pow(initialZVelocity, 2);
//        double altDiff = (initialZHeight + droneHeight) - landingAltitude;
//        double radical = sqrt(initialVzSquared + (4*4.9*altDiff)); //these numbers are from the quadric equation using the projectile motion formula
//        timeOne = (-initialZVelocity + radical) / 9.8;
//        timeTwo = (-initialZVelocity - radical) / 9.8;
//        if(timeOne > timeTwo)
//            return timeOne;
//        return timeTwo;

//        double finalTime = (-initialZVelocity + sqrt(2*(-initialZHeight)*(-9.81)+pow(initialZVelocity, 2)))/-9.81;
//        double finalTime2 = (-initialZVelocity - sqrt(2*(-initialZHeight)*(-9.81)+pow(initialZVelocity, 2)))/-9.81;
//
        double finalTime = (initialZVelocity + sqrt(2*(-initialZHeight)*(-9.81)+pow(initialZVelocity, 2)))/-9.81;
        double finalTime2 = (initialZVelocity - sqrt(2*(-initialZHeight)*(-9.81)+pow(initialZVelocity, 2)))/-9.81;

        std::cout << "THE TIMES" << finalTime << " " << finalTime2 << std::endl;
        return finalTime2;
    }

    double getFinalX(double time) {
        //we need some initial condition in here. I think this is assuming that it moves from the position of the drone
        std::cout << "NOTE:: initial xPosition " << initialXPosition << std::endl;
        return (initialXPosition + initialXVelocity * time);
//        return (initialXVelocity * time);
    }

    double getFinalY(double time) {
        //we need some initial condition in here. I think this is assuming it moves from the drone.
        std::cout << "initial YPosition " << initialYPosition << std::endl;
        return (initialYPosition + initialYVelocity * time);
        //return (initialYVelocity * time);
    }

};


#endif //DRONECATCH_CALCULATEPOSITION_H
