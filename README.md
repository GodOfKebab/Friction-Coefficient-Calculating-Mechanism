## A Mechanism That Calculates The Coefficients Of Friction Between Two Surfaces(Yes, both static and kinetic coefficients!)  
  
Honestly, I couldn't find this mechanism a better name. So we will call it the "friction mechanism" throughout the 
readme. Here are some photos 
  

<p align="center">
  <img src="https://github.com/GodOfKebab/Friction-Coefficient-Calculating-Mechanism/blob/master/Media/1.jpg" width=30% />
  <img src="https://github.com/GodOfKebab/Friction-Coefficient-Calculating-Mechanism/blob/master/Media/2.jpg" width=30% /> 
  <img src="https://github.com/GodOfKebab/Friction-Coefficient-Calculating-Mechanism/blob/master/Media/3.jpg" width=30% />
</p>

<p align="center">
  <img src="https://github.com/GodOfKebab/Friction-Coefficient-Calculating-Mechanism/blob/master/Media/4.jpg" width=30% />
  <img src="https://github.com/GodOfKebab/Friction-Coefficient-Calculating-Mechanism/blob/master/Media/5.jpg" width=30% /> 
  <img src="https://github.com/GodOfKebab/Friction-Coefficient-Calculating-Mechanism/blob/master/Media/6.jpg" width=30% />
</p>



## Physics Behind This Project
  
  
<img src="https://github.com/GodOfKebab/Friction-Coefficient-Calculating-Mechanism/blob/master/Media/Hand_Drawing.jpg" width=100% />

The θ is the angle between the inclined surface and the horizontal surface when the object starts to slide.

## How It Works 

This mechanism calculates the coefficients of friction using an inclined surface. It does it by increasing the incline of the 
plane. This is done by a 30 kg-cm torqued motor with a 3D printed 7:24 gears. By using the potentiometer on the other side, 
the program is also aware of the angle with respect to the horizontal surface. 

According to the calculations above, the coefficient of **static** friction is the tangent of the angle measured by the 
inclined surface when the object starts sliding. Therefore to accurately stop and determine this angle, a **Raspberry Pi** and its 
**camera module** is used along with **OpenCV**.

When the program is started with the [mode] set as *static*, the program starts showing the camera feed. From this window,
the user needs to press 's' to select the object to detect a slip. If the selection is done poorly, it can be reset by 
pressing 'r'. 

After this, the user needs to **set the motor speed** to increase the slope. In our mechanism, 45% power is the minimum 
amount of power required to increase the slope. Therefore most of our tests were done around this power level. When the 
program detects a slid after a sufficient increase in slope, the speed is set to 0 and the mechanism calculates and shows 
the **µs** on the screen. 

The **coefficient of kinetic friction, µk,**, on the other hand, is calculated in a rather more complicated way. Because according
to the physics of the mechanism above, the µk also requires the **average acceleration** of the object over the course of slide 
down a 40 cm plane. So just like the µs calculation, the mechanism is leveraged until the object starts to slide. However,
to calculate acceleration, an **ultrasonic distance measurement sensor HC-SR04** is used. With this sensor, the position of 
the sliding object is recorded with time stamps. It records until the button connected to Arduino is pressed. Then, the program graphs this data. By fitting the data 
with a second degree polynomial and taking its **first and second derivatives**, the velocity and acceleration graphs are 
generated. Since the acceleration graph is constant, it is used as the magnitude of acceleration used in calculations. 
Finally, with all the data collected(angle and average acceleration), program calculates and shows 
the equations used in graphs and the **µk**.


## Usage  
  
You can download this repository and run the code in your system by the following commands.
  
```
git clone https://github.com/GodOfKebab/Friction-Coefficient-Calculating-Mechanism
cd Friction-Coefficient-Calculating-Mechanism/Codes
python3 main.py [mode]
```

There is only one python program named "main.py". However, to run it, you also need to specify the mode: "static" or "kinetic".
This mode determines which coefficient of friction you want to find out. 


## List of Parts

* Raspberry Pi 3B with Pi Camera V2
* 12 V 10 RPM Motor with a maximum torque of 30 kg-cm [[link]](https://www.robotistan.com/12v-42mm-10rpm-reduktorlu-dc-motor)
* L298N Motor Driver Board
* LM2596 Step-Down Voltage Regulator for Raspberry Pi's 5V Power Requirements
* 12V 3A Power Adapter
* HC-SR04
* Arduino Uno for analog potentiometer and button state reading
* Total 6 kg weights to keep the mechanism in balance
* Lots of 3D Printing Material(PLA)


## License
[MIT](https://choosealicense.com/licenses/mit/)
