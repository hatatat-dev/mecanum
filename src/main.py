
import math

from vex import *

DEAD_JOYSTICK = 5

JOYSTICK_CENTER = 20

ROTATE_MOVEMENT = 50

MOTOR_SCALE = 0.8

# Brain should be defined by default
brain=Brain()

# Robot configuration code
controller = Controller(PRIMARY)

RED_CARTRIDGE = GearSetting.RATIO_36_1
GREEN_CARTRIDGE = GearSetting.RATIO_18_1
BLUE_CARTRIDGE = GearSetting.RATIO_6_1

motor_left_front = Motor(Ports.PORT11, BLUE_CARTRIDGE, False)
motor_left_back = Motor(Ports.PORT20, BLUE_CARTRIDGE, False)

motor_right_front = Motor(Ports.PORT1, BLUE_CARTRIDGE, True)
motor_right_back = Motor(Ports.PORT10, BLUE_CARTRIDGE, True)

def zero_to_three_sixty(angle: float) -> float:
    """Clip angle to [0, 360) range."""
    while angle < 0:
        angle += 360
    
    while angle >= 360:
        angle -= 360
    
    return angle

def compute_joystick_angle(x: float, y: float) -> float:
    """Compute angle for the joystick, in degrees."""
    if x >= 0:
        return zero_to_three_sixty(math.degrees(math.atan2(y, x)))
    else:
        return 180 - math.degrees(math.atan2(y, -x))
    
def compute_joystick_distance(x: float, y: float) -> float:
    """Compute distance for the joystick."""
    return max(abs(x), abs(y))

def compute_movement_left_front(angle: float, distance: float) -> float:
    """Compute movement for left front and right back Mecanum wheels."""
    return math.sin(math.radians(angle + 45)) * distance

def compute_movement_right_front(angle: float, distance: float) -> float:
    """Compute movement for right front and left back Mecanum wheels."""
    return math.sin(math.radians(angle - 45)) * distance

def spin_motor(motor: Motor, movement: float):
    if movement == 0:
        motor.stop()
    elif movement < 0:
        motor.set_velocity(min(-movement, 100) * MOTOR_SCALE, VelocityUnits.PERCENT)
        motor.spin(REVERSE)
    else:
        motor.set_velocity(min(movement, 100) * MOTOR_SCALE, VelocityUnits.PERCENT)
        motor.spin(FORWARD)


def controller_function():
    global controller, motor_left_front, motor_left_back, motor_right_front, motor_right_back

    while True:
        left_joystick_x = controller.axis4.position()
        left_joystick_y = controller.axis3.position()

        right_joystick_x = controller.axis1.position()
        right_joystick_y = controller.axis2.position()

        left_joystick_angle = compute_joystick_angle(left_joystick_x, left_joystick_y)
        left_joystick_distance = compute_joystick_distance(left_joystick_x, left_joystick_y)

        right_joystick_angle = compute_joystick_angle(right_joystick_x, right_joystick_y)
        right_joystick_distance = compute_joystick_distance(right_joystick_x, right_joystick_y)

        if left_joystick_distance < DEAD_JOYSTICK:
            movement_left_front = 0
            movement_right_front = 0
        else:
            movement_left_front = compute_movement_left_front(left_joystick_angle, left_joystick_distance)
            movement_right_front = compute_movement_right_front(left_joystick_angle, left_joystick_distance)

        if -JOYSTICK_CENTER < right_joystick_x < JOYSTICK_CENTER:
            rotate_left_front = 0
            rotate_left_back = 0
            rotate_right_front = 0
            rotate_right_back = 0
        elif right_joystick_x > 0:
            if -JOYSTICK_CENTER < right_joystick_y < JOYSTICK_CENTER:
                rotate_left_front = ROTATE_MOVEMENT
                rotate_left_back = ROTATE_MOVEMENT
                rotate_right_front = -ROTATE_MOVEMENT
                rotate_right_back = -ROTATE_MOVEMENT
            elif right_joystick_y > 0:
                rotate_left_front = ROTATE_MOVEMENT
                rotate_left_back = ROTATE_MOVEMENT
                rotate_right_front = 0
                rotate_right_back = 0
            else:
                rotate_left_front = ROTATE_MOVEMENT
                rotate_left_back = 0
                rotate_right_front = -ROTATE_MOVEMENT
                rotate_right_back = 0
        else:
            if -JOYSTICK_CENTER < right_joystick_y < JOYSTICK_CENTER:
                rotate_left_front = -ROTATE_MOVEMENT
                rotate_left_back = -ROTATE_MOVEMENT
                rotate_right_front = ROTATE_MOVEMENT
                rotate_right_back = ROTATE_MOVEMENT
            elif right_joystick_y > 0:
                rotate_left_front = 0
                rotate_left_back = 0
                rotate_right_front = ROTATE_MOVEMENT
                rotate_right_back = ROTATE_MOVEMENT
            else:
                rotate_left_front = 0
                rotate_left_back = -ROTATE_MOVEMENT
                rotate_right_front = 0
                rotate_right_back = ROTATE_MOVEMENT
            

        spin_motor(motor_left_front, movement_left_front + rotate_left_front)
        spin_motor(motor_left_back, movement_right_front + rotate_left_back)

        spin_motor(motor_right_front, movement_right_front + rotate_right_front)
        spin_motor(motor_right_back, movement_left_front + rotate_right_back)

        brain.screen.clear_screen()
        brain.screen.set_cursor(1, 1)
        brain.screen.print("left_joystick_x=%d" % left_joystick_x)
        brain.screen.next_row()
        brain.screen.print("left_joystick_y=%d" % left_joystick_y)
        brain.screen.next_row()
        brain.screen.print("left_joystick_angle=%f" % left_joystick_angle)
        brain.screen.next_row()
        brain.screen.print("left_joystick_distance=%f" % left_joystick_distance)
        brain.screen.next_row()
        brain.screen.print("movement_left_front=%f" % movement_left_front)
        brain.screen.next_row()
        brain.screen.print("movement_right_front=%f" % movement_right_front)
        brain.screen.next_row()
        brain.screen.print("right_joystick_x=%d" % right_joystick_x)
        brain.screen.next_row()
        brain.screen.print("right_joystick_y=%d" % right_joystick_x)

        # wait before repeating the process
        wait(20, MSEC)

# create a function for handling the starting and stopping of all autonomous tasks
def vexcode_auton_function():
    # wait for the driver control period to end
    while( competition.is_autonomous() and competition.is_enabled() ):
        # wait 10 milliseconds before checking again
        wait( 10, MSEC )

def vexcode_driver_function():
    controller_task = Thread(controller_function)

    # wait for the driver control period to end
    while( competition.is_driver_control() and competition.is_enabled() ):
        # wait 10 milliseconds before checking again
        wait( 10, MSEC )

    # Stop the driver control tasks
    controller_task.stop()

# register the competition functions
# competition = Competition( vexcode_driver_function, vexcode_auton_function )

controller_thread = Thread(controller_function)