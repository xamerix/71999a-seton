#include "main.h"
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
int autonSelected = 0;
/*
void right_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		autonSelected++;
	}
	else {
		autonSelected = autonSelected;
	}
}
void left_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		autonSelected--;
	}
	else{
		autonSelected=autonSelected;
	}
}
*/
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		autonSelected++;
	} else {
		autonSelected = autonSelected;
	}
	if (autonSelected > 4) {
		autonSelected = 0;
	}
}
int trackWidth = 13;
MotorGroup leftMotors({leftFront,leftBackTop,leftBackBottom});
MotorGroup rightMotors({rightFront,rightBackTop,rightBackBottom});
MotorGroup cata({cataMotor1, cataMotor2});
lemlib::Drivetrain drivetrain {
    &leftMotors, // left drivetrain motors
    &rightMotors, // right drivetrain motors
    13, // track width
    3.25, // wheel diameter
    360, // wheel rpm
    8
};
int horizOffset = -2;
int vertOffset = 0;
lemlib::TrackingWheel horiz_tracking(&horizWheel, 2.75, horizOffset, 1);
lemlib::TrackingWheel vert_tracking(&vertWheel, 2.75, vertOffset, 1);
Imu inertial(11);
lemlib::OdomSensors sensors {
    &vert_tracking, // vertical tracking wheel 1
    nullptr, // vertical tracking wheel 2
    &horiz_tracking, // horizontal tracking wheel 1
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &inertial // inertial sensor
};
// forward/backward PID
lemlib::ControllerSettings linearController {
    8, // kP
    30, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    5 // slew rate
};
 
// turning PID
lemlib::ControllerSettings angularController {
    4, // kP
    40, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    0 // slew rate
};
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

/*
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */


 /*
bool fireCata = false;
//double norm_power = 0;
double PID_power = 0;
double currentCataPos = 0;
double targetValue = 55;
double cataerror = 0;
double catapreverror = 0;
double cataOutput;
double cataderivative = 0;
double rotation_value = 0;
//double totalError;
//double last_derivatives[5] = {0}; // array to store last 5 derivative values

/// Cata PID Tuning Values
const double CataKp = 0.1; // Tune this
const double CataKd = 0.1; // Tune this
const double feedforward = 127; // tune this
//const double filter_gain = 0.0; // tune this
double cataTime = 0;
void InitializeCata(){
  cataRotate.reset_position();
  cataRotate.reverse();
  cataRotate.set_data_rate(10);
  cata.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
}
*/
/*
bool fireCata = false;
double norm_power = 0;
double PID_power = 0;
double rotation_value = 0;
double error = 0;
double preverror = 0;
double Output;
double derivative;


/// Cata PID Tuning Values
double CataKp = 3.14; // Tune this
double CataKd = 3.14; // Tune this
double feedforward = 5; // tune this


void firePult() {
  fireCata = true;
  pros::delay(200);
  fireCata = false;
  }

void InitializeCata(){
  cataRotate.reset_position();
  cataRotate.reverse();
  cataRotate.set_data_rate(10);
  cata.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
}
  
  double PIDCalculate(){

  rotation_value = (double(cataRotate.get_angle()) / 100);

  // Calculate the error between the current position and desired position
  error = (55.00 - rotation_value);

  // Apply Kp to the motor power
  PID_power = ((error * CataKp) + (CataKd + derivative));

  // Limit the motor power to prevent damage to the catapult
  PID_power = std::min(std::max(PID_power, 127.0), 86.0);
}

void DerivativeCalculate(void*){
  derivative = error - preverror;
  pros::delay(10);
}

void SystemControl(void*){
  Output = PIDCalculate();
  Output += feedforward;
  preverror = error;
  pros::delay(10);
}

void CataControl(void*) {
  if(fireCata == true){
   cata.move(127); 
  }
  else if (error >= 0){
   cata.move(Output);
  }
  else {
   cata.move(0);
   cata.brake();
  }
}
/*
void CataControl() {
  while(true){
    rotation_value = (double(cataRotate.get_angle())/100);
	  cataerror = targetValue - rotation_value;
	  cataderivative = cataerror - catapreverror;
	  PID_power = (cataerror * CataKp) + (cataderivative * CataKd);
    //PID_power = std::clamp(PID_power, -126,126);
    catapreverror = cataerror;
    if(int(cataerror) < 0){fireCata = false;}
    delay(20);
  }
}
void CataControlMatchLoad() {
  while(true){
    rotation_value = (double(cataRotate.get_angle())/100);
	  cataerror = targetValue*46 - rotation_value;
	  cataderivative = cataerror - catapreverror;
	  PID_power = (cataerror * CataKp) + (cataderivative * CataKd);
   // PID_power = std::clamp(PID_power, -126,126);
    catapreverror = cataerror;
    if(int(cataerror) < 0){fireCata = false;}
    delay(20);
  }
}


bool enablecataPID;
bool enablematchloading;
void firePult(void*p) {
  while (enablecataPID) {
    if (fireCata) {cata = 127;}
    else if(!enablematchloading) {CataControl();}
    if (enablematchloading) { cata = 127;}
    else if (enablematchloading) {CataControlMatchLoad();}
    }
  }
*/
/*
int cataAngle;
bool fireCatabool;
void firePult() {
  fireCatabool = true;
}
void fireCata() {
     double cataAngle = double(cataRotate.get_position())/100;
     cata.move(100);
     //pros::lcd::print(0, "Cata Rotation angle: %f", cataRotate.get_angle());
     delay(400);
     if (double(cataRotate.get_position())/100 < 310) {
       cata.set_brake_modes(E_MOTOR_BRAKE_HOLD);
       cata.brake();
    
   }
}

/*
void fireCata(/*void*p) {
   cataAngle = double((cataRotate.get_angle)/100);
   cata.move(127);
   //pros::lcd::print(0, "Cata Rotation angle: %f", cataRotate.get_angle());
   delay(400);
   if (double(cataRotate.get_angle())/100 < 310) {
     cata.set_brake_modes(E_MOTOR_BRAKE_HOLD);
     cata.brake();
     delay(100);
     fireCatabool = false;
  
  }
  }
*/
/*
bool fireCata = false;
double norm_power = 0;
double PID_power = 0;
double rotation_value = 0;
double error = 0;
double preverror = 0;
double Output;
double derivative = 0;
double totalError;
double last_derivatives[5] = {0}; // array to store last 5 derivative values

/// Cata PID Tuning Values
const double CataKp = 0.1; // Tune this
const double CataKd = 0.1; // Tune this
const double feedforward = 127; // tune this
const double filter_gain = 0.1; // tune this

void firePult() {
  fireCata = true;
  pros::delay(200);
  fireCata = false;
}

void InitializeCata(){
  cataRotate.reset_position();
  cataRotate.reverse();
  cataRotate.set_data_rate(10);
  cata.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
}
void CataControl() {
  while(true){

    rotation_value = (double(cataRotate.get_angle()) / 100);

    // Calculate the error between the current position and desired position
    error = (90.00 - rotation_value);

    // Update derivative
    double raw_derivative = error - preverror;
    last_derivatives[0] = raw_derivative;
    for (int i = 1; i < 5; i++) {
      last_derivatives[i] = last_derivatives[i-1] * filter_gain + last_derivatives[i] * (1 - filter_gain);
    }
    derivative = last_derivatives[4];
    // Apply Kp to the motor power
    PID_power = ((error * CataKp) + (CataKd * derivative));

    // Add feedforward term to PID output
    Output = PID_power + feedforward;

    // Save current error for next loop iteration
    preverror = error;

    if(int(error) > 90){
      error = 90;
    }

    if(fireCata == true){
      cata.move(127); 
    }
    else if (int(error) > 0){
      cata.move(Output);
    }
    else {
      cata.move(0);
      cata.brake();
    }

    pros::delay(5);
  }
}
*/
/*
bool matchBool;
void matchloading(/*void*p) {
  if (matchBool) {
  cata.move_relative(55*46,100);
  delay(29000);
  firePult();
  }
}
*/
int stateOP = 0;
int pressed;
void CataControl(void*p) {
  while (true) {
  if (stateOP == 0) {
        if (!cataLimit.get_value() && pressed == 1) {
          cata.move(127);
        }
        if (cataLimit.get_value() && pressed == 1) {
          cata.brake();
         }
        }
  
  if (stateOP == 1) {
    if(cataLimit.get_value()) {
        cata.move(127);
        delay(400);
        pressed = 1;
        stateOP = 0;
        }
  }
  else {
    stateOP = 0;
  }
delay(20);
}
}
bool matchloading;
void fireCata(bool matchloading) {
  if (!matchloading) {
    stateOP = 1;
  }
  else {
    for (int i = 0; i < 47; i++) {
      stateOP = 1;
    }
  }
}

ASSET(superpath_txt); // "." replaced with "_". File name is path.txt
/*
follow(const asset& path, int timeout, float lookahead, bool async = false, bool forwards = true,
float maxSpeed = 127, bool log = false);
moveTo(float x, float y, float theta, int timeout, bool async = false, bool forwards = true,
float chasePower = 0, float lead = 0.6, float maxSpeed = 127, bool log = false);
*/
void nearSideElims() { //0
  chassis.setPose(-33.139, 58.358,0);
/*
  chassis.setPose(87.046,144.039,0);
  chassis.moveTo(87.046, -130, 0, 4000,false,true,8,0.6,50,false);*/
  //chassis.follow(therightone_txt,4000,15,false,true,50,false);
	chassis.follow(superpath_txt, 15, 12000, true);
}
void farSideElims() { //1

}
void nearSideWP() { //2

}
void farSideWP() { //3

}
void skills() { //4

}

void initialize() {
	chassis.calibrate();
  wings.set_value(false);
  clamper.set_value(false);
  cata.set_brake_modes(E_MOTOR_BRAKE_HOLD);
  pros::Task cataCtrl(CataControl);
  stateOP = 0;
  //pros::Task loadCtrl(matchloading);
  cataRotate.reset_position();
  //cata.move_relative(240*4.5,100);
  /*
  pros::Task derivative(DerivativeCalculate);
  pros::Task systemCntrl(SystemControl);
  pros::Task cataCntrl(CataControl);*/
	pros::lcd::initialize();
	pros::Task screenTask([&]() {
        lemlib::Pose pose(0, 0, 0);
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
  //pros::Task cataTask(firePult);  
 // pros::Task matchloadTask(matchLoading);
	pros::lcd::set_text(1, "Hello PROS User!");
	pros::lcd::register_btn1_cb(on_center_button);
	

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  nearSideElims();
  /*
	if (autonSelected = 0) {
		nearSideElims();
	}
	if (autonSelected = 1) {
		farSideElims();
	}
	if (autonSelected = 2) {
		nearSideWP();
	} 
	if (autonSelected = 3) {
		farSideWP();
	}
	if (autonSelected = 4) {
		skills();
	}
  */
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
bool clampState = false;
bool wingState = false;
void opcontrol() {
  stateOP = 0;
  pressed = 1;
	pros::Controller master(pros::E_CONTROLLER_MASTER);
  leftMotors.set_brake_modes(E_MOTOR_BRAKE_COAST);
  rightMotors.set_brake_modes(E_MOTOR_BRAKE_COAST);
	while (true) {
    pros::lcd::print(4, "Cata Rotation angle: %f", double(cataRotate.get_position())/100);
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);

		leftMotors = left;
		rightMotors = right;
		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_R2)) {
      clampState = !clampState;
			clamper.set_value(clampState);
		}
		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)) {
      wingState = !wingState;
      wings.set_value(wingState);
    }
    if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_L2)) {
      pros::lcd::print(0, "Cata Rotation angle: %f", cataRotate.get_angle());
      fireCata(false);
    }
    else if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_X)) {
      pros::lcd::print(0, "Cata Rotation angle: %f", cataRotate.get_angle());
      fireCata(true);
    }
    if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_A)) {
      blocker.set_value(true);
    }
		pros::delay(20);
	}
}
