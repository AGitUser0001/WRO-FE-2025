# WRO-FE-2025
Our repository for the 2025 World Robot Olympiad - Future Engineers.

Team Name: Team Cucumber

This repository contains information about our robot and the building process. 

## Links
- [Building Instructions](/Building-Instructions.md)

## Team Members
### Lucas Zheng






**Age:** 14

**Introduction:**

### Walter Wu






**Age:** 14

**Introduction:**
Hi, my name is Walter and I'm from Canada, this is my second WRO season, two years ago I participated in WRO robomission and ended up on the international stage in Panama. This year since I was old enough for future engineers I decided to sign up for a comepletly different challenge, I am currently 14 years old and I have many interests from coding to a variety of video games and sports, I will be attending All saints catholic secondary school for their AMP program.
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
        <li>MSc in Eletrical & Computer Engineering from University of Alberta.</li>
        <li>BSc in Mathematics from Peking University.</li>
      </ul>
    </td>
  </tr>
</table>

## Group photo
<img src="/images/IMG_5017.jpeg" width="300" height="400">

## Video of Lucas explaining our 3D printings

## The Open Challenge
The robot must complete three laps on the track after being placed randomly inside the walls of the track. The robot must have one button pressed to turn on the robot and one button pressed to run the program. After this, **no further interactions** with the robot are allowed.
## The Obstacle Challenge
The robot must complete three laps on the track with randomly arranged traffic signs which are green and red. The robot must go around the **right** side of a **red** pillar, and the **left** side of a **green** pillar. 

Learn about the challenges as well as the rules [here](https://wro-association.org/wp-content/uploads/WRO-2024-Future-Engineers-Self-Driving-Cars-General-Rules.pdf)

## Photos of Our Completed Robot:

| <img src="./images/robotfront.jpg" width="60%" height="10%" /> | <img src="./images/robotback.jpg" width="55%" height="50%" /> | 
| :--: | :--: | 
| *Front* | *Back* |
| <img src="./images/robotleft.jpg" width="60%" height="50%" /> | <img src="./images/robotright.jpg" width="55%" height="50%" /> | 
| *Left* | *Right* |
| <img src="./images/robottop.jpg" width="60%" height="50%" /> | <img src="./images/robotbottom.jpg" width="55%" height="50%" /> | 
| *Top* | *Bottom* |

## Our video of the open chllenge on [Youtube](https://youtu.be/EdWDk1boRc8) 

## First design
<img src="/images/first1.png" width="400" height="400"> <img src="/images/first2.png" width="400" height="400">

Above are some pictures of our first design for our robot. As you can see there are many flaws and as well as components missing. Although these designs not quite what we needed, they helped us find inspiration for our eventual final design. In fact, we decided to keep a lot of ideas from our earlier generations to incorporate into our robot. One of such is that we have the Raspberry Pi mounted on the back instead of the usual where it is found on top the chassis.


## Challenges We Faced 
### Short Circuit
After we soldered the battery connector to the ESC and the extension board, we were ready to flip the switch. Unfortunately, we switched the positive and negative wires, resulting our extension board to be burned. This taught us to be more cautious before closing the circuit and helped us avoid similar situations along the way.
### Servo Motor
We realized that we could not fit the battery and the servo motor under our 3D printed plate. We decided to raise the 3D printed plate to make more space. 
A new problem came up. It was hard to fully secure the servo motor in place. 
### Processing Too Slow
We overcame the problem of our program running too slowly by switching to a combination of multiprocessing and threading. Initially, our single-threaded approach caused delays, especially when handling tasks like image processing, motor control, and sensor feedback all at once. To solve this, we implemented multiprocessing to separate heavy tasks, such as camera frame processing, into their own processes, allowing them to run in parallel without blocking the main loop. At the same time, we used threading for lighter background tasks like logging data or checking sensors continuously. This made our program significantly faster and more responsive, allowing our robot to perform in real time without lag or missed inputs.
## Mobility Management
Our motor powers the two back wheels. We removed the differential gear in the front of the car to increase the steering angle. Our mobility management is mainly made up of our motor, servo motor, wheels, and our differential gear at the back of the car.
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
| LiDAR sensor  | $----  |
| Total Price Before Tax   | $503.88  |
| Total Price   | $569.38  |

## Potential Future Improvements
