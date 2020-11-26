# LidarBot - Experiments in using Lidar for mobile robot navigation

## Hardware

ydlidar X4
 https://www.amazon.com/gp/product/B07DBYHJVQ/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1
 https://www.ydlidar.com/Public/upload/files/2020-04-13/YDLIDAR%20X4%20Datasheet.pdf

Motorized Base
 https://www.amazon.com/pop-mart-Laboratory-Programming-Recognition/dp/B07X2HQ23D/

I purchased a version of this platform which does not appear to be available anymore.
My version cost $200.00 and was only the platform, motors, wheels, battery and motor controller.


## Software

### Motor Controller
 https://github.com/egregori3/miiboo_driver

This is my version of the driver. There is a link to the original ROS version in the comments.


### Lidar
 https://github.com/egregori3/lidar

The original Lidar driver used CMake without a cross compile script. I did not change much
of the lidar driver code, just created a new Makefile and flattened the build. 
I also added a static library build option.


### lidarbot
 https://github.com/egregori3/LidarBot

 This is a c++ class that sits on top of the motor driver and lidar driver and presents a simple
 API to the application/algorithm layer.

 lidarbot.cc and lidarbot.h are the class.
 The API provides services fro reading the lidar and moving the robot.

* algorithm1.cc is an experimental algorithm for guiding the robot through doorways.
* manual.cc is an application that just turns the robot and reports sensor data.
   Use ctrl-c to stop the robot.
* lidartest.cc is used to visualize the lidar data and save the data to a file.


![Software Stack](https://github.com/egregori3/LidarBot/blob/master/LidarBot1.png)


## Style Guide

https://google.github.io/styleguide/cppguide.html

**Tabs Versus Spaces:** Use only spaces, and indent 2 spaces at a time (https://www.youtube.com/watch?v=SsoOG6ZeyUI).

**File Names:** Filenames should be all lowercase and can include underscores (_) or dashes (-). Follow the convention that your project uses. If there is no consistent local pattern to follow, prefer "_".

**Extensions:**  source.cc,  header.h

**Local variables:** Place a function's variables in the narrowest scope possible, and initialize variables in the declaration.

**Function length:** Prefer small and focused functions.

**Efficiency:** Use ++i as opposed to i++.  (https://medium.com/better-programming/stop-using-i-in-your-loops-1f906520d548)

**Integers:** <cstdint> defines types like int16_t, uint32_t, int64_t, etc. You should always use those in preference to short, unsigned long long and the like, when you need a guarantee on the size of an integer. Of the C integer types, only int should be used.

**Variable Names:** The names of variables (including function parameters) and data members are all lowercase, with underscores between words. Data members of classes (but not structs) additionally have trailing underscores. For instance: a_local_variable, a_struct_data_member,a_class_data_member_.

**Function Names:** Ordinarily, functions should start with a capital letter and have a capital letter for each new word.

**Comments:**

Comments are absolutely vital to keeping our code readable. The following rules describe what you should comment and where. But remember: while comments are very important, the best code is self-documenting. Giving sensible names to types and variables is much better than using obscure names that you must then explain through comments.

When writing your comments, write for your audience: the next contributor who will need to understand your code. Be generous — the next one may be you!

Almost every function declaration should have comments immediately preceding it that describe what the function does and how to use it.

In your implementation you should have comments in tricky, non-obvious, interesting, or important parts of your code.

**Function Calls:** If the arguments do not all fit on one line, they should be broken up onto multiple lines, with each subsequent line aligned with the first argument. Do not add spaces after the open paren or before the close paren:

**Use of spaces:**

if (condition) {               // Proper use of spaces
x = *p;
p = &x;
x = r.y;
x = r->y;
// Assignment operators always have spaces around them.
x = 0;

// Other binary operators usually have spaces around them, but it's
// OK to remove spaces around factors.  Parentheses should have no
// internal padding.
v = w * x + y / z;
v = w*x + y/z;
v = w * (x + z);

// No spaces separating unary operators and their arguments.
x = -5;
++x;
if (x && !y)i

**Return Values:** Do not needlessly surround the return expression with parentheses.
