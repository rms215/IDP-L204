from controller import Robot, Motor
import numpy as np

TIME_STEP = 64
MAX_SPEED = 10

class Mybot():
		def __init__(self):
        # create a robot
        robot = Robot()

        # get devices
        self.us_right = robot.getDevice("us_right")
        self.us_left = robot.getDevice("us_left")
        self.ir = robot.getDevice("ir_right")
        self.lightSensor = robot.getDevice("TEPT4400")
        self.motor_left = robot.getDevice("Wheel_L")
        self.motor_right = robot.getDevice("Wheel_R")
        self.arm_left = robot.getDevice("Arm_L")
        self.arm_right = robot.getDevice("Arm_R")
        self.compass = robot.getCompass("compass")
    
    def move(self, speed):
        self.motor_left.setPosition(float('inf'))
        self.motor_right.setPosition(float('inf'))
        self.motor_left.setVelocity(speed * MAX_SPEED)
        self.motor_right.setVelocity(speed * MAX_SPEED)
    
    def open_arms(self):
        self.arm_left.setPosition(1)
        self.arm_right.setPosition(-1)
          
    #Rowan time
    def close_arms(self):
        self.arm_left.setPosition(0) #those arms do be looking kinda closed though
        self.arm_right.setPosition(0)   
    
    #Rowan time
    def getSensorValues(self):
				sensorValues = [[0,0],[0,0]]
        for i in range (2):
        		sensorValues[i] = ds[i].getValue()/5700 ####NOT sure on this line
        
        lookUpTable = ds[2].getLookupTable() ##Or this one
        lookUpTableSize = ds[2].getLookupTableSize() ###Or this one. WHat is ds?
    		
        distanceActual = 0
        
        
        
      #are you kidding me this deleted everything I was sat for 45 minutes doing -Liza
      #Yea so I was working on getting sensor lookup tables and values, cool, gonna go to supo and then get back to whatever state this is in
    
    # add other robot movement

    # sensor and bearing functions
    # 
    # In your code in cpp you passed the get bearing in degrees function
    # as an argument since it was declared later and you couldn't use it
    # here but in python this is unnecessary

    def getRawSensorValues(self):
        ir = self.ir.getValue()
        us_r_value = self.us_right.getValue()
        us_l_value = self.us_left.getValue()
        sensor_values = [ir,us_r_value,us_l_value]
        return sensor_values
    
    def 

    def getBearingInDegrees(self):
        north = compass.getValues()						
        rad = np.arctan(north[0]/north[2])
        bearing = rad/np.pi*180.0
        if bearing<0:
						bearing += 360
        return bearing
    
    #Rowan wrote this function
    #Function to rotate to a given bearing
    #Queries: I'm rusty on classes- is self.motor_left etc correct or is it just motor_left? I think I have done it right
    def rotateTheta(self, theta, inital_bearing):
				self.motor_left.setVelocity(0.5*MAX_SPEED)                                 #Set velocity of each motor to 0.5*max speed or -0.5*max speed
				self.motor_right.setVelocity(-0.5*MAX_SPEED) 
    		while True:                                                                #While true loop
        		bearing = getBearingInDegrees()                                        #Get bearing of block from where I am 
        		if ((bearing - initial_bearing) >= 0:                                  #If i'm not pointing at block, angle to rotated is different
								angle_rotated = bearing - inital bearing
            else: 
            		angle_rotated = bearing - inital bearing + 360
            if angle_rotated > theta:                                              #Woo the angle has been found
            		self.motor_left.setVelocity(0)
                self.motor_right.setVelocity(0)
                break
     
    #Rowan Time
    #Not sure the difference between this and rotate theta. Asked cindy though ;)
    #Again, self.motor unsure
		def rotateUntilBearing(self, target_bearing, initial_bearing):
    		angle_rotated = 0
        if target_bearing > initial_bearing:
        		self.left_motor.setVelocity(0.1*MAX_SPEED)
            self.right_motor.setVelocity(-0.1*MAX_SPEED)
            while True:
            		bearing = getBearingInDegrees()
                if bearing > target_bearing:
            				self.move(0)  
                    break	
                    
				if target_bearing < initial_bearing:
        		self.left_motor.setVelocity(-0.1*MAX_SPEED)
            self.right_motor.setVelocity(0.1*MAX_SPEED)  
            while True:
            		bearing = getBearingInDegrees()
                if bearing < target_bearing:
            				self.move(0)  
                    break	                
        
            
            
    		
# create the Robot instance.
robot = Mybot()

robot.move_forward(.5)

i = 0
fin1 = 0

while robot.step(TIME_STEP) != -1: #@Cindy can you possibly do this? I don't follow what the code is doing
		Values = robot.getSensorValues()
		sensorValuesScan.push_back(Values) ####I DON'T KNOW WHAT THIS DOES OR WHAT PYTHON EQUIVALENT IS  
    
    if i ==0:
    		currentBearing = robot.get_bearing_in_degrees()
        robot.rotate_theta(355,currentBearing)
      
  	blockBearings = robot.get_block_bearings()
    rotate_until_bearing(blockBearing[0],robot.get_bearing_in_degrees)
