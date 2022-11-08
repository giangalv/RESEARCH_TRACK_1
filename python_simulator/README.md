Python Robotics Simulator
================================

This is a simple, portable robot simulator developed by [Student Robotics](https://studentrobotics.org).
Some of the arenas and the exercises have been modified for the Research Track I course

Installing and running
----------------------

The simulator requires a Python 2.7 installation, the [pygame](http://pygame.org/) library, [PyPyBox2D](https://pypi.python.org/pypi/pypybox2d/2.1-r331), and [PyYAML](https://pypi.python.org/pypi/PyYAML/).

Pygame, unfortunately, can be tricky (though [not impossible](http://askubuntu.com/q/312767)) to install in virtual environments. If you are using `pip`, you might try `pip install hg+https://bitbucket.org/pygame/pygame`, or you could use your operating system's package manager. Windows users could use [Portable Python](http://portablepython.com/). PyPyBox2D and PyYAML are more forgiving, and should install just fine using `pip` or `easy_install`.

## Troubleshooting

When running `python run.py <file>`, you may be presented with an error: `ImportError: No module named 'robot'`. This may be due to a conflict between sr.tools and sr.robot. To resolve, symlink simulator/sr/robot to the location of sr.tools.

On Ubuntu, this can be accomplished by:
* Find the location of srtools: `pip show sr.tools`
* Get the location. In my case this was `/usr/local/lib/python2.7/dist-packages`
* Create symlink: `ln -s path/to/simulator/sr/robot /usr/local/lib/python2.7/dist-packages/sr/`

First Assignment
=================

At the end, the goal of this first assignment is to have silver and golden boxes distributed in pairs.

I wrote a program where the robot searchs and grabs the nearby silver block and then the robot goes to realise it near the adjacent golden block.

# The constants #

* a_th, threshold for the orientation.
* d_SLVER, distance where the robot can grab the silver token.
* d_GOLD, distance where the robot can realise the gold token close to the silver token.
* d_BLOCK, distance to do not see all token silver if the robot grabbed a token.
* NO_BLOCK, message to said that the robot is not seeing tokens.
* listcode_token_SILVER, list for codes' of silver tokens.
* listcode_token_GOLD, list for codes' of golden tokens.
* NUMBERS_TOKEN, number of couple of tokens to take (silver + gold).

# The functions #

### drive(speed) ###

This function actives two motors (m0 and m1) with the same rotation and same speed.

```python
R.motors[0].m0.power = speed
R.motors[0].m1.power = speed
```

### turn_dx(speed) ###

This function is used to turn on the right, the two motors have a opposite speed.

```python
R.motors[0].m0.power = speed
R.motors[0].m1.power = -speed
```

### turn_sx(speed) ###

This function is opposed than turn_sx.

```python
R.motors[0].m0.power = -speed
R.motors[0].m1.power = speed
```

### stop_motor() ###

This function is used to stop the motors.

```python
R.motors[0].m0.power = 0
R.motors[0].m1.power = 0
```

### info_token(color) ###

This function is used to know three values.
The distance (dist) between robot and token.
The rotation (rot_y) respect the axis perpendicular with the front robot and the token wich the robot is seeing (it is a angolar value).
And the code of this block.

The function has an argument.
The color is used to find only one kind of token. 

If the robot does not seeing anything the return values is -1,-1,0.

```python
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
```

### info_color() ###

This function is used to know wich color th robot is seeing. 

If it is seeing token, the function return its color.

If it is not seeing token, return a message inside NO_BLOCK.

```python
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
```

### drive_to_token(rot_y) ###

This function is used to drive the robot in the directions' token.

The principal thing is if the robot is perpendicular more or less with the token, it has to go linear.

If the robot loses that perpendicolar condiction it starts to move, slowly than before, on the right or on the left, it is depend by the rot_y (the angle between the perpendicular axis with the robot and the token).

```python
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
```

### search_new_block(color,tint) ###

This function in used to search a token wich it was not taken before.

The robot computes 82 controlls if it sees or not a token and turns around itself.

It checks if the token was not used and then add its code in the robots' list and check if the distance between the robot and the token is the shortest, add the code to the new_code to return that value after the 82 controlls.

Color and tint are two string variables and they are used to enter in the right color (silver or gold).

```python
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
```

### find_block_saw(code,color,speed) ###

This function searchs a token with the code and color and the robot turns until it is seeing the right token and then it stops to move.

The code and the color are used to find the right token and the speed is the rapidity to turn.

```python
	dist, rot_y, code_see = info_token(color)
	while(code_see != code):
		turn_dx(speed)
		dist, rot_y, code_see = info_token(color)
	stop_motor()
```

### check_code(listcode, code) ###

This function checks if there is the code inside a listcode and return two differents number if there is or not the code.

```python
	for x in listcode:
		if(x == code):
			return -1
	return 1
```

### main() ###

This program finds 6 silver and 6 golden tokens and move a robot to the close different token.

Firstly, robot starts to search the nearby silver token, then it goes to grab it and adds its code in the listSilver.

Secondly, the robot searches the adjacent gold token and after find it, the robot goes close by it to realese the silver token and adds the golds' code in the ListGold.

Thirdly, the robot repeats this two points for the NUMBERS_TOKEN, how i explained before in this case NUMBERS_TOKEN is SIX.

At the end, the robot exits by the program. 

```python
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
```

Robot API
---------

The API for controlling a simulated robot is designed to be as similar as possible to the [SR API][sr-api].

### Motors ###

The simulated robot has two motors configured for skid steering, connected to a two-output [Motor Board](https://studentrobotics.org/docs/kit/motor_board). The left motor is connected to output `0` and the right motor to output `1`.

The Motor Board API is identical to [that of the SR API](https://studentrobotics.org/docs/programming/sr/motors/), except that motor boards cannot be addressed by serial number. So, to turn on the spot at one quarter of full power, one might write the following:

```python
R.motors[0].m0.power = 25
R.motors[0].m1.power = -25
```

### The Grabber ###

The robot is equipped with a grabber, capable of picking up a token which is in front of the robot and within 0.4 metres of the robot's centre. To pick up a token, call the `R.grab` method:

```python
success = R.grab()
```

The `R.grab` function returns `True` if a token was successfully picked up, or `False` otherwise. If the robot is already holding a token, it will throw an `AlreadyHoldingSomethingException`.

To drop the token, call the `R.release` method.

Cable-tie flails are not implemented.

### Vision ###

To help the robot find tokens and navigate, each token has markers stuck to it, as does each wall. The `R.see` method returns a list of all the markers the robot can see, as `Marker` objects. The robot can only see markers which it is facing towards.

Each `Marker` object has the following attributes:

* `info`: a `MarkerInfo` object describing the marker itself. Has the following attributes:
  * `code`: the numeric code of the marker.
  * `marker_type`: the type of object the marker is attached to (either `MARKER_TOKEN_GOLD`, `MARKER_TOKEN_SILVER` or `MARKER_ARENA`).
  * `offset`: offset of the numeric code of the marker from the lowest numbered marker of its type. For example, token number 3 has the code 43, but offset 3.
  * `size`: the size that the marker would be in the real game, for compatibility with the SR API.
* `centre`: the location of the marker in polar coordinates, as a `PolarCoord` object. Has the following attributes:
  * `length`: the distance from the centre of the robot to the object (in metres).
  * `rot_y`: rotation about the Y axis in degrees.
* `dist`: an alias for `centre.length`
* `res`: the value of the `res` parameter of `R.see`, for compatibility with the SR API.
* `rot_y`: an alias for `centre.rot_y`
* `timestamp`: the time at which the marker was seen (when `R.see` was called).

[sr-api]: https://studentrobotics.org/docs/programming/sr/
