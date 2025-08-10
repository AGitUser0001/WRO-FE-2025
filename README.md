# WRO-FE-2025
Our repository for the 2025 World Robot Olympiad - Future Engineers.

Team: 

This repository contains information about our robot and the building process. 


## Team Members
### Lucas Zheng






**Age:** 14

**Achievements:**

### Walter Wu






**Age:** 14

**Achievements:**
- 31st in WRO 2023 International RoboMission Elementary
### Eric Huang






**Age:** 15

**Achievements:**

## Our coach

### Rice Rao
<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="https://explorer-robotics.com/images/users/CoachRice.png" width="100%"> 
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Achievements:</h3>
      <ul>
        <li>Head coach of Robotics Competitions including FLL (First LEGO League) Tournament and WRO (World Robotics Olympiad). Led teams in winning multiple national, international robotics and programming awards.</li>
        <li>Over 20 years of IT industry experience as software engineer working internationally.</li>
        <li>MSc in Eletrical & Computer Engineering from University of Alberta.</li>
        <li>BSc in Mathematics from Peking University.</li>
      </ul>
    </td>
  </tr>
</table>

## Group photo





## Video of Lucas explaining our 3D printings

## The Open Challenge
The robot must complete three laps on the track after being placed randomly inside the walls of the track. The robot must have one button pressed to turn on the robot and one button pressed to run the program. After this, **no further interactions** with the robot are allowed.
## The Obstacle Challenge
The robot must complete three laps on the track with randomly arranged traffic signs which are green and red. The robot must go around the **right** side of a **red** pillar, and the **left** side of a **green** pillar. 
## Photos of Our Completed Robot:

## Video of Our Robot
## Breakdown Of Individual Responsibilities
## Challenges We Faced 
### Short Circuit
After we soldered the battery connector to the ESC and the extension board, we were ready to flip the switch. Unfortunately, we switched the positive and negative wires, resulting our extension board to be burned. This taught us to be more cautious before closing the circuit and helped us avoid similar situations along the way.
### Servo Motor
We realized that we could not fit the battery and the servo motor under our 3D printed plate. We decided to raise the 3D printed plate to make more space. 
A new problem came up. It was hard to fully secure the servo motor in place. 

## Mobility Management

### Our Motor
<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./images/FuritekMotor.webp" width="100%"> 
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <p>Voltage: 10V</p>
      <p>Gear Ratio: 1:2</p>
      <p>Speed: 3450 rpm</p>
      <p>Weight: 17.5g</p>
    </td>
  </tr>
</table>

### Our Servo Motor
<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./images/servo motor.png" width="100%"> 
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <p>Operating Voltage Range: 4.8V ~ 6.0V</p>
      <p>Speed: 2.0 ~ 1.7</p>
      <p>Maximum Torque Range kg. / cm.: 1.3 ~ 1.6</p>
      <p>Weight: 9.5g</p>
    </td>
  </tr>
</table>

## Camera
<table>
  <tr>
    <td width="50%" style="text-align: left;">
      <img src="./images/pi cam.png" width="100%"> 
    </td>
    <td width="50%" style="text-align: left; vertical-align: top;">
      <h3>Specifications:</h3>
      <p>Sensor Size: 1/4-inch optical format (approx. 3.674 mm × 2.760 mm)</p>
      <p>Sensor Resolution: 3280 × 2464 pixels</p>
      <p>Pixel Size: 1.12 µm × 1.12 µm</p>
      <p>Lens Mount Type: M12 (S mount)</p>
      <p>Autofocus support: None</p>
    </td>
  </tr>
</table>

## Obstacle Management
## Circuit Diagram
![Main](/circuits/wro_schematic_circuit_schem.png)


## Total Price of the Robot
| Component  | Cost |
| ------------- | ------------- |
| Motor  | $110.51  |
| Servo Motor  | $24.99  |
| Camera  | $16.87  |
| Raspberry Pi 5  | $113.95  |
| 3D Printings  | $8  |
| Raspberry Pi Expansion Board   | $54.98  |
| Metal Chassis  | $86.38  |
| Battery  | $29.99  |
| Wires  | $26.75  |
| Wheels  | $9.13  |
| Switch  | $1.03  |
| Velcro and Tape  | $21.30  |
| Total Price Before Tax   | $503.88  |
| Total Price   | $569.38  |
## How we Built the Robot
The construction of our robot will be split up into 8 steps. Step zero being to first print our 3d prints which can be found in our 3d prints folder, step one being to modify the chassis and soldering, step two being to next gather all the required parts and components, step 3 is to attach the 3d prints, step four is to start the build by attaching the movement parts, step five is to wire everything together, and finally step six is to add the final parts, step seven to finish the build with final touches, and step eight to import the code and run it.

***Step Zero:***
This step is purely dedicated to acquiring the 3d prints, you will need to find the prints in the 3d prints folder on our repository, next download them and print them out, if you are unsure on how to you can always use external help such as OpenAI like ChatGPT or Youtube.

***First step:***
This step is to alter your chassis to become fit for our build with a series of modifications. These being first to take out the long beam on the top which is connected to the front and rear gear boxes as it is in the way, and then secondly to take out the servo housing.

**1.**

<img src="/images/chassis%20beam.png" width="700" height="7000">

**2.**

<img src="/images/servo%20housing.png" width="500" height="500">


Aswell this step includes soldering a wire and switch. You will need to solder a switch to a wire as well as making it have one input and two outputs, the input will be from the battery and the outputs will be into the expansion board and into the ESC for the BLDC motor.

***Second step:***
You need to gather all the parts and components for our robot, those which are listed before when we calculated the total cost of our robot, these components are a BLDC motor, a servo motor, a camera, Rasberry Pi 5, all of our 3d prints, the expansion board, our chassis, battery, wires,  switch, Velcro, tape, and electrical tape.

***Third step:***
Attaching the 3d prints, the baseplate is to be screwed in on top of the chassis with screws and extra long screw extensions, and the back part which should slide into the very back of the chassis along with the stopper that you put in through the top of the of the bottom of the backplate, there is a hole made for it.

**Baseplate**

<img src="/images/baseplateAndScrews.png" width="400" height="400">

**Back piece**

<img src="/images/BackPiece.png" width="400" height="400">


***Fourth step:***
This step will be to attach all the parts that are involved with moving these being the BLDC motor, the servo motor, Raspberry pi 5, and the expansion board.

**Part 1.**
Attaching the servo and BLDC motors. Attach the servo motor with Velcro, and the BLDC motor with the pregiven mount that came along with the chassis.

<img src="/images/ServoAnDBLDC.png" width="400" height="400">

**Part 2.**
Attaching the Raspberry Pi 5 and the expansion board. So both of these circuits will be screwed into place with screw extenders on each of the four corners in a relatively systematic fashion as each corner for their relative circuit will be the same. The images following will have the circuit encircled and each smaller screw circled also.

**Raspberry Pi 5**

<img src="/images/raspi.png" width="400" height="400">

**Expansion Board**

<img src="/images/expansionboard.png" width="400" height="400">

***Fifth Step:***
This step will be to wire everything together; the servo motor goes to the pwm control on the expansion board and the BLDC motor will be connected to an esc which will be Velcroed to the side of the baseplate. This esc will then connect to the switch, the expansion board, and the battery. Next you will need two USB cables, one for data, one for power. The data cable will be USB C to USB A, and the power cable will be USB C to USB C. Next just attach those cables. Finally, as well add a strip of Velcro on the underside of the baseplate. This will be used to attach the battery.

***Sixth Step:***
Step six, we will be adding the battery and the camera, the camera will be attached to the camera port on the raspberry pi 5 board and each battery that you decide to use will have a strip of Velcro on its back. After you attach the battery, check all of the connections first before you plug it in. Safety first!!!

***Seventh Step:***
For step seven you will need to tape the camera cable aswell as the camera in place, attach the servo to the servo arm on the chassis, and fix the cables in place with tape or if you have them, zip ties.

***Eight Step:***
Finally step eight is to just import our code and run it!





## Potential Future Improvements
