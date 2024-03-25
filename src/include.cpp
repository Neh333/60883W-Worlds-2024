#include "include.hpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"

//define controller
pros::Controller controller(CONTROLLER_MASTER);

//Drive train objects 
pros::Motor frontleft (18, true);
pros::Motor midleft   (19, true);
pros::Motor backleft  (20, true);

pros::Motor frontright(13, false);
pros::Motor midright  (12, false);
pros::Motor backright (11, false);

pros::MotorGroup leftMotors ({frontleft,midleft,backleft});
pros::MotorGroup rightMotors ({frontright, midright, backright});

//Define V5 sensorss
pros::Imu imu(17);

//Motors
pros::Motor intake(6);
pros::Motor hang(4,true);

//rotation sensor 
pros::Rotation hangRot(9,true);

//ADI Digital Out Devices 
pros::ADIDigitalOut backWings('G');
pros::ADIDigitalOut frontWings('A');
