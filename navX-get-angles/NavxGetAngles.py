from navx import AHRS

def get_yaw_angle():
    try:
        # Create an instance of the AHRS class
        navx = AHRS.create_spi()
        # Get the current yaw angle
        yaw = navx.getAngle()
        # Print the yaw angle
        print("Yaw angle: ", yaw)
    except Exception as e:
        print(e)
        
def get_pitch_angle():
    try:
        # Create an instance of the AHRS class
        navx = AHRS.create_spi()
        # Get the current pitch angle
        pitch = navx.getPitch()
        # Print the pitch angle
        print("Pitch angle: ", pitch)
    except Exception as e:
        print(e)

def get_roll_angle():
    try:
        # Create an instance of the AHRS class
        navx = AHRS.create_spi()
        # Get the current roll angle
        roll = navx.getRoll()
        # Print the roll angle
        print("Roll angle: ", roll)
    except Exception as e:
        print(e)
