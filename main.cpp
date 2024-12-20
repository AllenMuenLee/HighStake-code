#include "main.h"
#include "bits/stdc++.h"
#include "VOSS/api.hpp"
#include "replay.h"


auto ec = voss::controller::ExitConditions::new_conditions()
              .add_settle(400, 0.5, 400)
              .add_tolerance(1.5, 2.0, 100)
              .add_timeout(22500)
              .add_thru_smoothness(20)
              .build();

auto odom = voss::localizer::TrackingWheelLocalizerBuilder::new_builder()
                .with_right_motor(16)
                .with_left_motor(-11)
                .with_track_width(13.5)
                .with_left_right_tpi(18.43)
                .with_imu(4)
                .build();

auto pid = voss::controller::PIDControllerBuilder::new_builder(odom)
               .with_linear_constants(20, 0.02, 169)
               .with_angular_constants(250, 0.05, 2435)
               .with_min_error(30)
               .with_min_vel_for_thru(70)
               .build();

auto swing = voss::controller::SwingControllerBuilder::new_builder(odom)
                .with_angular_constants(20, 0.05, 2435)
                .build();
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
    odom->begin_localization(); //calibrate and begin localizing
    pros::screen::set_pen(pros::Color::red);
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

#define LEFT_MOTORS {-11, -14, -15}                                                                                                                                                                                                  
#define RIGHT_MOTORS {16, 18, 19}

pros::Motor Intake(9, pros::v5::MotorGear::blue);
pros::MotorGroup wall({3, -5}, pros::v5::MotorGear::green);
//pros::MotorGroup Left({-11, -12, -13}, pros::v5::MotorGear::blue);
//pros::MotorGroup Right({1, 2, 3}, pros::v5::MotorGear::blue);
pros::Optical optical(7);
pros::ADIDigitalOut piston ('h');
pros::ADIDigitalOut sweeper ('g');

bool piston_bool = true;
bool sweeper_bool = false;
bool rev = false;
bool in_intake = false;
bool intake_block = false;
long long deg = 0;
int last_deg = 0;
int iter = 0;
int wall_count = 0;
int update_pos = 1890;
std::vector<int> pos = {0, 502, 1233};
std::vector<int> wall_pos = {0, 270, 1800, 2200};
std::queue<int> s;
std::queue<int> stop_pos;

auto chassis = voss::chassis::DiffChassis(LEFT_MOTORS, RIGHT_MOTORS, pid, ec, 8, pros::E_MOTOR_BRAKE_BRAKE);
pros::Controller master(pros::E_CONTROLLER_MASTER);

//we recommend using the pid controller as default controller

void intake_contr(){
    if (!intake_block){
        if (!master.get_digital(DIGITAL_L1) && master.get_digital(DIGITAL_R1)){
            Intake.move_voltage(10000);
        } else if (!master.get_digital(DIGITAL_L1) && master.get_digital(DIGITAL_R2)){
            Intake.move_voltage(-10000);
        } else {
            Intake.move_velocity(0);
        }
    }
}

void hang(){
    if (master.get_digital_new_press(DIGITAL_Y)){
        wall_count = 3;
    }
}

void wall_contr(){
    int prev = wall_count;
    if (master.get_digital_new_press(DIGITAL_L1) && wall_count < 2){
        if (wall_count == 2){
            intake_block = true;
            Intake.move_voltage(-7000);
        }
        pros::delay(200);
        wall_count++;
    } else if (master.get_digital_new_press(DIGITAL_L2) && wall_count > 0){
        wall_count--;
    }  
    hang();
    if (prev != wall_count){
        wall.move_absolute(wall_pos[wall_count], 12000);
    }
    if (wall.get_actual_velocity() < 10 && (wall.get_position() < 1830 && wall.get_position() > 1770)){
        intake_block = false;
        wall_count = 1;
        wall.move_absolute(wall_pos[wall_count], 12000);
    }
}

void fixer(){
    if (master.get_digital_new_press(DIGITAL_A)){
        piston_bool = !piston_bool;
        pros::delay(200);
    }
}

void sweep(){
    if (master.get_digital_new_press(DIGITAL_B)){
        sweeper_bool = !sweeper_bool;
        pros::delay(200);
    }
}

void funcs(){
    intake_contr();
    fixer();
    wall_contr();
    sweep();

    piston.set_value(!piston_bool);
    sweeper.set_value(sweeper_bool);
}

void filt(){
    if (!in_intake && (optical.get_hue() < 50 && optical.get_hue() > 0)){
        in_intake = true;
    }
    if (in_intake && !(optical.get_hue() < 50 && optical.get_hue() > 0)){
        s.push(Intake.get_position());
        in_intake = false;
    }
    if (!s.empty()){
        int p = std::upper_bound(pos.begin(), pos.end(), s.front()) - pos.begin();
        if (p != stop_pos.front()){
            stop_pos.push((pos[(p+1) % pos.size()]));
        }
        s.pop();
    }
    if (!stop_pos.empty()){
        intake_block = true;
        Intake.move_absolute(stop_pos.front(), 12000);
        pros::delay(100);
        stop_pos.pop();
        intake_block = false;
    }
}

void update(){
    if (Intake.get_position() >= update_pos){
        Intake.set_zero_position(update_pos);
    } else if (Intake.get_position() <= update_pos * -1){
        Intake.set_zero_position(update_pos * -1);
    }
}

void autonomous() {
    chassis.move(-53.5, 50, voss::Flags::REVERSE);
    chassis.turn(-90);
    chassis.move(-12, 60, voss::Flags::REVERSE);
    Intake.move_voltage(12000);
    pros::delay(500);
    chassis.move(28);
    chassis.turn(125, 60);
    chassis.move(-94, 70, voss::Flags::REVERSE);
    piston.set_value(true);
    pros::delay(150);
    chassis.turn(0, 70);
    chassis.move(80, 70);
    chassis.turn(-105, 70);
    chassis.move(25, 70);
    chassis.turn(180, 70);
    chassis.move(40, 70);
    chassis.turn(-170, 70);
    chassis.move(70, 70);
}

void opcontrol() {
    wall.tare_position();

    /*while(Intake.get_position() >= update_pos){
        Intake.set_zero_position(update_pos);
    }*/
	
	while (true) {
        funcs();
        /*update();
        filt();*/
		
		int master_y = master.get_analog(ANALOG_LEFT_Y);
		int master_x = master.get_analog(ANALOG_RIGHT_X);

        if (!rev){
            chassis.arcade(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_X));
        }/* else {
            chassis.arcade(master_y, master_x);
        }*/
	}
}