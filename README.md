# PiDrone
Pi powered drone with pico flight controller and pi camera for fpv flying.

Note: This is the first itteration, so is not a perfect product and there are many improvements to be made.

**Parts list:**

These are the exact parts I used in my build. There are many other combinations of parts which would also work. I used parts commonly used on 5 inch freestyle fpv drones.

Motors: Axis x Supafly x Sync – Bando 2207.5 Motor - 1860kv
Battery: DOGCOM 1300mAh 150C 4S 14.8V UCELL series
ESC: SpeedyBee BLS 60A 30×30 4-in-1 ESC
Propellors: HQProp MCK Prop 5.1 Inch (2CW+2CCW) – Poly Carbonate
IMU: MPU6050
Camera: Raspberry Pi camera v2.1 
Camera-VTX interface: Raspberry Pi zero 2W (outputs composite video to VTX)
VTX: SpeedyBee TX800
Flight controller: Raspberry Pi Pico 2
Receiver: Flysky FS-iA6B
Transmitter: Flysky FS-i6
BEC: I used an old ESC with built in BEC, but any DC-DC 5v regulator will work

**Drone design:**

The first iteration was fully 3d printed, with a fully enclosed design. I designed the arms like I would a cantilever, since the arms are supporting a point force at their ends. The honeycomb lid allowed cooling, and the enclosure had holes for embedded XT-60 and MR-30 connections, giving the drone a clean look. I designed for all the electrical components to be rigidly mounted inside, since unwanted movement could make flight more difficult.

After testing, I found that the 3D printed frame was quite brittle, and would often break when the drone crashed. On top of this, I was limited by the build area of the 3D printer, and consequently the motors had to be quite close together. Instead, I CNC routed a new frame from 4mm carbon fibre with an increased wheelbase. I used the Carveco software to generate toolpaths, and then cut out the frame on the maker lab’s workbee CNC. 2 hours later I had a nice frame, which I assembled with the redesigned plastics. 

I have attached all files needed for either the fully 3D printed version, or the 3D printed and carbon fibre (reccomended) version for anyone who wants to build this.

![image3](https://github.com/user-attachments/assets/e9986c82-5181-49ca-b7da-8cd1b995be3f)

**Electrical design:**

The pico 2 gets gyrometer data from the MPU6050 using the I2C protocol, and controller inputs from the receiver using IBUS protocol. Initially I was using a receiver which outputted 5 separate PWM signals, for each of the controls: throttle, yaw, pitch, roll, and an additional arming channel. However, reading each of these signals in turn took far too long, and severely limited the loop frequency of the Pico flight controller. IBUS reduced the number of wires, and sped up the loop around 10x. The pico performs PID calculations based on the drones current situation (gyrometer) and desired situation (receiver), then outputs the calculated 50 Hz PWM signals to a 4 in 1 ESC. The duty cycle of these signals range from 1 to 2 ms, with 1ms corresponding to 0% throttle, and 2ms 100%. The ESC then deals with generating the correct signal to spin up the brushless motors.

In addition to this, the drone contains a Pi camera and Pi zero 2W. The zero outputs composite video to an analog VTX which transmits a 5.8 GHz radio wave. This can be received by analog fpv goggles and allows the flyer to see what the drone sees in real time - first person view flying.

![image4](https://github.com/user-attachments/assets/1be1e011-c1ed-46c4-8537-f77f6632d0ad)

**Programming:**

Largely based on Tim Hanewich’s Scout flight controller code. Implements a basic ‘rate’ mode flight controller, where the user inputs desired angular velocities with the controller, and the flight controller adjusts throttles to bring the drone to the desired angular velocities. Uses 3 PID controllers for yaw, pitch and roll. PID controllers take in the error (difference between actual and deserted state), and output a PID value, just the sum of a value proportional to error, a value proportional to integral of error and a value proportional to rate of change of error. The different proportions of P, I and D and controlled by 3 PID gains, which must be tuned to make the drone fly. The flight controller then takes its 3 PID values, (for yaw, roll, pitch) and performs throttle mixing: adding and subtracting these PID values to calculate what throttle it should add to each motor to make the drones state match the desired state. For example, t1 (motor 1 throttle) = desired_throttle + pid_pitch + pid_roll - pid_yaw. Desired_throttle is simply based on the user's throttle input, which sets the drone's speed. Then to make the drone spin in the desired axis, the PID values are added. The combination of + and - is just due to the motors position: for example, if you desire the drone to roll clockwise (when looking at the drone from behind), you would add throttle (+pid_roll) to the two left motors, and subtract throttle from the two right motors. 

I have attached the flight controller programme and also the 2 libaries needed. You will need to tune the PID gains based on your build since they are dependent on mass, size, shape etc.
