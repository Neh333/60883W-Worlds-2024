#include "include.hpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"

//define controller
pros::Controller controller(CONTROLLER_MASTER);

//Deefine motors
pros::Motor frontright(20 ,false);
pros::Motor midright(10, false);
pros::Motor backright(8, true);

pros::Motor frontleft(12, true);
pros::Motor midleft(1, true);
pros::Motor backleft(2, false);

pros::Motor puncher(5, false);

pros::Motor intake(7, true);

//Define V5 sensorss
pros::Imu imu(17);

pros::Rotation puncherR(3);
pros::Vision vison(14);

//pistions
pros::ADIDigitalOut wingPis('H');
pros::ADIDigitalOut intakePis('G');
pros::ADIDigitalOut hangPis('F');
