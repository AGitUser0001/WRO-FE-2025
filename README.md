# WRO-FE-2025
Our repository for the 2025 World Robot Olympiad - Future Engineers.

Team Name: Team Cucumber

This repository contains information about our robot and the building process. 

## Links
- [Building Instructions](/Building-Instructions.md)
- [Software Documentation](/Software-Documentation.md)
- [Open Challenge Documentation](/Open-Challenge.md)
- [Obstacle Challenge Documentation](/Obstacle-Challenge.md)

## Table of Contents
* [The Team](#team)
* [The Challenge](#challenge)
* [The Robot](#robot-image)
* [Performance Video](#video)
* [Mobility Management](#mobility-management)
* [Circuit Diagram](#circuit-diagram)
* [Obstacle Management](#obstacle-management)
* [Total Cost](#cost-report)

## Team Members <a class="anchor" id="team"></a>

### Lucas Zheng






**Age:** 14

**Introduction:**

### Walter Wu






**Age:** 14

**Introduction:**
Hi, my name is Walter and I'm from Canada, this is my second WRO season, two years ago I participated in WRO Robo mission and ended up on the international stage in Panama. This year since I was old enough for future engineers I decided to sign up for a completly different challenge, I am currently 14 years old and I have many interests from coding to a variety of video games and sports, I will be attending All saints catholic secondary school for their AMP program.
### Eric Huang






**Age:** 15

**Introduction:**
Hello, I'm Eric, and I have already participated in two WRO competitions so far. Although I have not entered the international stage in any of them, I believe these competitions helped me better problem solve and gave me something to do. This is also why I am participating in Future Engineers this year. My interests are chess, badminton, and swimming. 
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
        <li>MSc in Electrical & Computer Engineering from University of Alberta.</li>
        <li>BSc in Mathematics from Peking University.</li>
      </ul>
    </td>
  </tr>
</table>

## Group photo
<img src="/images/IMG_5017.jpeg" width="300" height="400">

## Video of Lucas explaining our 3D printings

## The Open Challenge <a class="anchor" id="challenge"></a>
The robot must complete three laps on the track after being placed randomly inside the walls of the track. The robot must have one button pressed to turn on the robot and one button pressed to run the program. After this, **no further interactions** with the robot are allowed.
## The Obstacle Challenge
The robot must complete three laps on the track with randomly arranged traffic signs which are green and red. The robot must go around the **right** side of a **red** pillar, and the **left** side of a **green** pillar. 

Learn about the challenges as well as the rules [here](https://wro-association.org/wp-content/uploads/WRO-2024-Future-Engineers-Self-Driving-Cars-General-Rules.pdf)

## Photos of Our Completed Robot: <a class="anchor" id="robot-image"></a>

| <img src="./images/robotfront.jpg" width="60%" height="10%" /> | <img src="./images/robotback.jpg" width="55%" height="50%" /> | 
| :--: | :--: | 
| *Front* | *Back* |
| <img src="./images/robotleft.jpg" width="60%" height="50%" /> | <img src="./images/robotright.jpg" width="55%" height="50%" /> | 
| *Left* | *Right* |
| <img src="./images/robottop.jpg" width="60%" height="50%" /> | <img src="./images/robotbottom.jpg" width="55%" height="50%" /> | 
| *Top* | *Bottom* |

## Our video of the open challenge on [Youtube](https://youtu.be/EdWDk1boRc8)  <a class="anchor" id="video"></a>

## First design
<img src="/images/first1.png" width="400" height="400"> <img src="/images/first2.png" width="400" height="400">

Above are some pictures of our first design for our robot. As you can see there are many flaws and as well as components missing. Although these designs not quite what we needed, they helped us find inspiration for our eventual final design. In fact, we decided to keep a lot of ideas from our earlier generations to incorporate into our robot. One of such is that we have the Raspberry Pi mounted on the back instead of the usual where it is found on top the chassis. Our servo was mounted using tape at first, but we realized that it wasn't secure enough. We switched to Velcro, but we realized that the servo motor still wiggled around a little. We switched to a type of Velcro that snaps in place, but it was still moving. We never had much problem securing our motor, using a piece of metal that came with the metal chassis to secure it. Finally, our 3d printed mount was not stable enough and the plastic was not thick enough.



## Challenges We Faced 
### Short Circuit
After we soldered the battery connector to the ESC and the extension board, we were ready to flip the switch. Unfortunately, we switched the positive and negative wires, resulting our extension board to be burned. This taught us to be more cautious before closing the circuit and helped us avoid similar situations along the way.
### Servo Motor
We realized that we could not fit the battery and the servo motor under our 3D printed plate. We decided to raise the 3D printed plate to make more space. 
A new problem came up. It was hard to fully secure the servo motor in place. 
### Processing Too Slow
We overcame the problem of our program running too slowly by switching to a combination of multiprocessing and threading. Initially, our single-threaded approach caused delays, especially when handling tasks like image processing, motor control, and sensor feedback all at once. To solve this, we implemented multiprocessing to separate heavy tasks, such as camera frame processing, into their own processes, allowing them to run in parallel without blocking the main loop. At the same time, we used threading for lighter background tasks like logging data or checking sensors continuously. This made our program significantly faster and more responsive, allowing our robot to perform in real time without lag or missed inputs.
### LIDAR Was Unreliable
At first, we used our mounted camera to follow walls. This worked out great for the open challenge and seemed to work well for our obstacle challenge at first, but we soon found some cases in which the robot would completely go the wrong way with our camera wall following and pillar detection. Take this case for example.

<img src="/images/ObstacleChallengeProblem.png" width="400" height="500">

The robot sees this red pillar and tries to turn left, but as soon as it stops seeing it, the robot turns right. We thought using LIDAR for wall following would be a good solution since it can "see" very far. However, LIDAR was very unreliable, especially when the robot was in motion. This led to the robot wobbling around and sometimes just crashing into a wall. We went back to using our camera for wall following. 
## Final Design

<img src="/images/IMG_5054.jpeg" width="400" height="500">

Our final design has many improvements and additions than the previous versions. Our robot includes sensors, those being the lidar and a camera, motors; being the servo and BLDC motors, a raspberry pi 5 and expansion board, as well as wheels and a battery. Our robot has been made to be compact so that we can be efficient with our limited space on the 1/28 scale chassis. We raised the baseplate to make space for the battery and the servo motor. To secure the servo motor, we used tape with Velcro, finding that this was better than just tape or just Velcro. We also added triangular supports to the mount so that it can hold the extra weight of the LIDAR. We made the plastic of the mount thicker as well to support the camera, Raspberry Pi, and the LIDAR. We changed the pin so that it was more stable and better connected the mount to the metal chassis.

<img src="/images/IMG_5055.jpeg" width="200" height="200"> <img src="/images/IMG_5056.jpeg" width="200" height="200">

## Mobility Management <a class="anchor" id="mobility-management"></a>
Our mobility is made possible by a few different parts, these being the code that gets run through our raspberry pi and expansion board, the actual moving made possible through the BLDC motor and ESC, the steering which is controlled by our servo motor and can well, steer the front wheels, and our wheels. Firstly, our code is tuned so that the servo motor can steer precisely where we want it to, and so that the DC motor can move exactly as fast as we want it to. The next parts being our actual BLDC moto and ESC. The ESC is wired into our expansion board so that we can utilize PWM (pulse width modulation) to control how fast our BLDC motor is and our BLDC motor runs into a differential gear at the back of our car as we decided to use RWD instead of FWD or AWD because well, the servo is controlling the front wheels and AWD is banned. 
Next is our servo motor. Our servo motor is wired directly into the PWM pins on our expansion board and the servo arm is attached to our front wheels, the servo motor required a lot of fine tuning as the servo_straight variable was hard to decide. Lastly our wheels went through many variations before finally settling on the one we are using currently. Before as the tires were too small we would add tire grips to increase the circumference of the tires as well as the grip as sometimes they would slip, but those tire grips would often fall off decreasing the consistency of our robot, so after careful consideration we landed on the current version of wheels which worked really well during testing.

### Our Motor
Our DC brushless motor drives the back wheels thanks to the differential gear which ensures that the car can drive straight. Our motor is secured using a metal part of the chassis that is secured with screws. We chose motor because it is small, reliable, and the gear ratio matched with the chassis gears.
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

## Obstacle Management <a class="anchor" id="obstacle-mangement"></a>
## Power and Sense Management
## Circuit Diagram <a class="anchor" id="circuit-diagram"></a>
<img src="/images/wroschematic.png" width="1000" height="1000">


## Total Price of the Robot <a class="anchor" id="cost report"></a>
| Component  | Cost |
| ------------- | ------------- |
| Motor  | $110.51 |
| Servo Motor  | $24.99  |
| Camera  | $16.87  |
| Raspberry Pi 5  | $113.95  |
| 3D Printings  | $8  |
| Raspberry Pi Expansion Board   | $54.98  |
| Metal Chassis  | $86.38  |
| Battery  | $42.78  |
| Wires  | $26.75  |
| Wheels  | $9.13  |
| Switch  | $1.03  |
| Velcro and Tape  | $21.30  |
| LiDAR sensor  | $89.99  |
| Total Price Before Tax   | $606.66  |
| Total Price   | $685.53  |

## Potential Future Improvements
