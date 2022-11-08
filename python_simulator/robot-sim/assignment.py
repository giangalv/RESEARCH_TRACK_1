from __future__ import print_function

import time
from sr.robot import *

a_th = 2.0
""" float: Threshold for the control of the orientation"""

d_SILVER = 0.4
""" float: Threshold for the control of the linear distance between robot and silver token"""

d_GOLD = 0.6
""" float: Threshold for realese the Silver token from the gold token"""

d_BLOCK = 0.8
""" float: Threshold for check the color of the token"""

NO_BLOCK = 'no-close'
""" string: for know there is a token but not near"""

listcode_token_SILVER = list()
""" lists for the silver code token"""

listcode_token_GOLD = list()
""" lists for the gold code token"""

NUMBERS_TOKEN = 6
"""Number of differents tokens to take and move"""

R = Robot()
""" instance of the class Robot"""


def drive(speed):
    """
    Function for setting a linear velocity
    
    Arg: speed (int): the speed of the wheels
    """
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = speed
    
  
def turn_dx(speed):
    """
    Function for setting an angular velocity
    
    Arg: speed (int): the speed of the wheels
    """
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = -speed

    
def turn_sx(speed):
    """
    Function for setting an angular velocity
    
    Arg: speed (int): the speed of the wheels
    """
    R.motors[0].m0.power = -speed
    R.motors[0].m1.power = speed
    
    
def stop_motor():
	"""
	Function for stopping the motors
	"""
	R.motors[0].m0.power = 0
	R.motors[0].m1.power = 0
	

def info_token(color):
    """
    Function to find the closest color token 
    
    Args: color (str): color to find

    Returns:
	dist (float): distance of the closest token (-1 if no token is detected)
	rot_y (float): angle between the robot and the token (-1 if no token is detected)
	code (int): indintification code of the token choose
    """
    dist=100
    
    for token in R.see():
    	if (token.dist < dist and token.info.marker_type == color):
        	dist=token.dist
		rot_y=token.rot_y
	   	code = token.info.code
    if (dist == 100):
        return -1, -1, 0
    else:
        return dist, rot_y, code
        
        
def info_color():
	"""
	Function to know the color token in the reach of the robot
	
	"""
	dist = 100
	for token in R.see():
		if(token.dist < dist and token.dist > d_BLOCK):
			color = token.info.marker_type
			dist = token.dist
	# control for know if the robot can see or not something
	if (dist == 100): 
		return NO_BLOCK
	else:	
		return color
        

def drive_to_token(rot_y):
	"""
	Function to orentation the robot perpendicolar to the token with a +- 0.1 tolerance
	
	Arg: rot_y (float): rotation to do for beeing perpendicolar with a token
	"""
	if(rot_y >= -a_th and rot_y <= a_th):
		stop_motor()
		drive(40)
	elif(rot_y > a_th + 5):
		turn_dx(18)	
	elif(rot_y < -5 - a_th):
		turn_sx(18)
	elif(rot_y >= a_th and rot_y <= 5 + a_th):
		turn_dx(4)
	elif(rot_y <= -a_th and rot_y >= -a_th - 5):
		turn_sx(4)
	
		
def search_new_block(color,tint):
	"""
	Function to find a new token not inside the list GOLD and SILVER
	
	Args: color (str): to find a list
		  tint (str): similar to color but to find a effective token
	
	Return:
	code of the new token
	distance of the new token
	"""
	listcode_new = list()
	# 0 means that the robot isn't seeing blocks
	listcode_new.append(0)
	dist_block = 100
	# the robot does 360 grades more or less
	for t in range(83): 
		dist,rot,code = info_token(color)
		color_block = info_color()
		s = check_code(listcode_token_SILVER,code)
		g = check_code(listcode_token_GOLD,code)
		# check if the block it was used
		if(str(color_block) ==  'silver-token' and s == -1) or (str(color_block) ==  'gold-token' and g == -1):
			pass
		else:
			if(str(color_block) == tint and s == 1):
				i = check_code(listcode_new,code)
				if(i == 1):
					listcode_new.append(code)
					prompt_code = str(code)
					print("New block " + prompt_code + "-SILVER")
					if(dist < dist_block):
						new_code = code
						dist_block = dist
			if(str(color_block) == tint and g == 1):
				i = check_code(listcode_new,code)
				if(i == 1):
					listcode_new.append(code)
					prompt_code = str(code)
					print("New block " + prompt_code + "-GOLD")
					if(dist < dist_block):
						new_code = code
						dist_block = dist
		# move
		turn_dx(15)
		time.sleep(0.1)
		stop_motor()
	stop_motor()
	return new_code, dist_block
	
	
def find_block_saw(code,color,speed):
	"""
	Function to find a token 
	
	Args: code (int): numer of token
	      color (str): color to find
	      speed (int): the speed of the wheels	      
	"""
	dist, rot_y, code_see = info_token(color)
	while(code_see != code):
		turn_dx(speed)
		dist, rot_y, code_see = info_token(color)
	stop_motor()
			
		
def check_code(listcode, code):
	"""
	Function to check if the token was taken before or not
	
	Args: listcode (list): list with the code of the tokens
	      code (int): numer of token
		
	Return:
	-1 if the token was used
	 1 if the token is new
	"""
	for x in listcode:
		if(x == code):
			return -1
	return 1
	
	
def main():	
	t = 0
	# 0 means that the robot isn't seeing blocks
	listcode_token_SILVER.append(0)
	listcode_token_GOLD.append(0)
	high_speed = 20
	low_speed = 8
	while(t < NUMBERS_TOKEN):
		print("TOKEN:",t+1)
		print("I'm searching SILVER token:")
		code, dist = search_new_block(MARKER_TOKEN_SILVER,'silver-token')
		print("I'm going to take",code)
		find_block_saw(code, MARKER_TOKEN_SILVER,high_speed)		
		while(dist > d_SILVER):
			dist, rot_y, code_seeing = info_token(MARKER_TOKEN_SILVER)
			if(code_seeing == code):
				drive_to_token(rot_y)
			else:
				print("TOKEN LOST")
				drive(2)
				time.sleep(1)
				stop_motor()
				find_block_saw(code, MARKER_TOKEN_SILVER,low_speed)					
		stop_motor()
		time.sleep(1)
		R.grab()
		print("GRAB IT")
		# Add code to the list SILVER
		listcode_token_SILVER.append(code)
		print("I'm searchin a GOLD token:")
		code, dist = search_new_block(MARKER_TOKEN_GOLD,'gold-token')
		print("I'm going to",code)
		find_block_saw(code,MARKER_TOKEN_GOLD,high_speed)		
		while(dist > d_GOLD):
			dist, rot_y, code_seeing = info_token(MARKER_TOKEN_GOLD)
			if(code_seeing == code):
				drive_to_token(rot_y)
			else:
				print("TOKEN LOST")
				drive(2)
				time.sleep(1)
				stop_motor()
				find_block_saw(code, MARKER_TOKEN_GOLD,low_speed)	
		stop_motor()
		time.sleep(1)
		R.release()
		print("RELEASE IT")
		# Add code to the list GOLD
		listcode_token_GOLD.append(code)
		t = t + 1
		# back a litle bit
		drive(-20)
		time.sleep(1)
		stop_motor()	
	print("I finished, I have orginized " + str(NUMBERS_TOKEN) + " tokens.")	
	exit()
	
	
main()	
