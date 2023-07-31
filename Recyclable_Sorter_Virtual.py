ip_address = 'localhost' # Enter your IP Address here
project_identifier = 'P3B' # Enter the project identifier i.e. P3A or P3B

# SERVO TABLE CONFIGURATION
short_tower_angle = 315 # enter the value in degrees for the identification tower 
tall_tower_angle = 90 # enter the value in degrees for the classification tower
drop_tube_angle = 180 # enter the value in degrees for the drop tube. clockwise rotation from zero degrees

# BIN CONFIGURATION
# Configuration for the colors for the bins and the lines leading to those bins.
# Note: The line leading up to the bin will be the same color as the bin 

bin1_offset = 0.15 # offset in meters
bin1_color = [1,0,0] # e.g. [1,0,0] for red
bin1_metallic = False

bin2_offset = 0.15
bin2_color = [0,1,0]
bin2_metallic = False

bin3_offset = 0.15
bin3_color = [0,0,1]
bin3_metallic = False

bin4_offset = 0.15
bin4_color = [0,1,1]
bin4_metallic = False
#--------------------------------------------------------------------------------
import sys
sys.path.append('../')
from Common.simulation_project_library import *

hardware = False
if project_identifier == 'P3A':
    table_configuration = [short_tower_angle,tall_tower_angle,drop_tube_angle]
    configuration_information = [table_configuration, None] # Configuring just the table
    QLabs = configure_environment(project_identifier, ip_address, hardware,configuration_information).QLabs
    table = servo_table(ip_address,QLabs,table_configuration,hardware)
    arm = qarm(project_identifier,ip_address,QLabs,hardware)
else:
    table_configuration = [short_tower_angle,tall_tower_angle,drop_tube_angle]
    bin_configuration = [[bin1_offset,bin2_offset,bin3_offset,bin4_offset],[bin1_color,bin2_color,bin3_color,bin4_color],[bin1_metallic,bin2_metallic, bin3_metallic,bin4_metallic]]
    configuration_information = [table_configuration, bin_configuration]
    QLabs = configure_environment(project_identifier, ip_address, hardware,configuration_information).QLabs
    table = servo_table(ip_address,QLabs,table_configuration,hardware)
    arm = qarm(project_identifier,ip_address,QLabs,hardware)
    bot = qbot(0.1,ip_address,QLabs,project_identifier,hardware)

#NOTE! Please ensure all settings in Quanser related to shadow and reflections are turned down to minimum
#Ensure FPS setting is turned up so that it is running between 50-60 FPS (We run it at 120FPS on VMWare and get ~55-60FPS
#Physics in Quanser are directly tied to framerate so if it is not running at an ideal frame rate bottles may fall off or Q-Bot may leave line

def drop_off(): #function which empties the hopper
    bot.activate_linear_actuator()
    bot.rotate_hopper(10)
    time.sleep(1)
    bot.rotate_hopper(20)
    time.sleep(1)
    bot.rotate_hopper(40)
    time.sleep(1)
    bot.rotate_hopper(70)
    time.sleep(1)
    bot.rotate_hopper(90)
    time.sleep(1)
    bot.rotate_hopper(0)
    print("Dropoff completed")

def get_to_bin(): #function which moves in front of bin after colour detection

    if bin_location == "Bin01":  
        bot.forward_distance(0.20)
        time.sleep(0.5)
        bot.rotate(-30)
        time.sleep(0.5) #moves bot around curve to bin if on edge
        bot.forward_distance(0.1)
    elif bin_location == "Bin03":
        bot.forward_distance(0.25)
        time.sleep(0.5)
        bot.rotate(-15)
        time.sleep(0.5) #moves bot around curve to bin if on edge
        bot.forward_distance(0.1)
    elif bin_location == "Bin04":
        bot.forward_distance(0.2) #moves bot straight in front of bin 
    else: 
         bot.forward_distance(0.3) #moves bot straight in front of bin otherwise
         bot.rotate(-5)
    print("We have arrived at the bin")
         
def container_position(): #loads hopper with containers by Q-Arm 
    arm.move_arm(0.658, 0.0, 0.283)
    time.sleep(1)
    arm.control_gripper(40)
    time.sleep(1)
    arm.move_arm(0.015, -0.289, 0.679)
    time.sleep(1)
    arm.move_arm(0.018, -0.495, 0.508)
    time.sleep(1)
    arm.control_gripper(-40)
    time.sleep(1)
    arm.home()

def moveit(): #function which follows line
    line_location = bot.line_following_sensors() #reads the value of the current location on the line

    if line_location[1] == 1: #if right sensor detects line (i.e. on straight path)
        bot.set_wheel_speed([0.1, 0.1])

    elif line_location[1] != 1: #turns when right sensor loses line
        bot.set_wheel_speed([0.1, 0.2])

def dispense_container(): #randomly dispenses container and returns its properties
    container_ID = random.randint(1,6) #randomly picks a container identity
    attributes = table.dispense_container(container_ID, True)

    return attributes

def load_container():

    global bin_location, binID

    checker = True #controls the iterations of while loop

    if len(container_list) >= 1: #checks for container left on servo table
        print("something is on the table :)")
        count = 1
        print("Attributes of the container are ", container_list)
        bin_location = container_list[0][2]
        total_mass = container_list[0][1]
        container_position()
        print("The total mass on the Q-Bot is ", total_mass)
        time.sleep(2)

        del container_list[0] #delete container attributes once moved off of table
        time.sleep(2)

        while count < 3 and checker == True: #keep dispensing containers while conditions aren't met
            attributes = dispense_container()
            container_list.append(attributes) #add leftover container attributes to list
            binID = attributes[2]
            mass = attributes[1]
            time.sleep(1)
            print("Attributes of the container are ", attributes)
            total_mass += mass
           

            if binID == bin_location and total_mass < 90: #check if container is appropriate to be added
                container_position()
                count += 1
                del container_list[0] #delete attributes from list
                print("The total mass on the Q-Bot is ", total_mass)
                

            else:
                checker = False #end loop if conditions aren't met
                print("ready to move")

        
        

    else: #if there is no container on servo table
        attributes = dispense_container()
        print("Attributes of the container are ", attributes)
        time.sleep(1)
        bin_location = attributes[2]
        total_mass = attributes[1]
        container_position()
        print("The total mass on the Q-Bot is ", total_mass)
        count = 1
        time.sleep(1)

        while count < 3 and checker == True: #Meets the first constraint
            attributes = dispense_container()
            print("Attributes of the container are ", attributes)
            container_list.append(attributes)
            binID = attributes[2]
            mass = attributes[1]
            time.sleep(1)
            total_mass += mass
            

            if binID == bin_location and total_mass < 90: #meets the second and third constraint and the container is added to the hopper
                container_position()
                count += 1
                print("The total mass on the Q-Bot is ", total_mass)
                del container_list[0] #delete attributes from list

            else: 
                checker = False
                print("Ready to move")

def transfer_container(binID):
    bot.activate_line_following_sensor()
    bot.activate_color_sensor()
    colour_reading = bot.read_color_sensor()[0] #reads the current color calues

    if bin_location == "Bin01": #convert binID to RGB values
        binID = [1, 0, 0]

    elif bin_location == "Bin02":
        binID = [0, 1, 0]

    elif bin_location == "Bin03":
        binID = [0, 0, 1]

    elif bin_location == "Bin04":
        binID = [0, 1, 1]

    while binID != colour_reading: #move along line until colour is sensed
        colour_reading = bot.read_color_sensor()[0]
        moveit()      

    bot.stop()
    bot.deactivate_line_following_sensor()
    bot.deactivate_color_sensor()
    get_to_bin()
    drop_off()

def get_position(): #detect position of Q-Bot
    x = bot.position()[0]
    y = bot.position()[1]

    if 1.5>x>1.4 and 0.02>y>-0.02: #range of values corresponding to home position

        return False

    else:

        return True
    
    
def bot_home(): 
    bot.activate_line_following_sensor()
    bot.activate_color_sensor()

    while get_position(): #follow line while position is not home
        colour_reading = bot.read_color_sensor()[0] #reads the current color calues
        line_location = bot.line_following_sensors() #reads the value of the current location on the line
        moveit()

    if not get_position(): #stop once home
        bot.stop()
        bot.deactivate_line_following_sensor()
        bot.deactivate_color_sensor()
    print("Welcome back home Mr. Bot")

container_list = [] #list of containers on table with attributes

while True:   
    load_container()
    transfer_container(binID)
    bot_dot_go_home_please()

