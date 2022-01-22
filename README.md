# ME495 Sensing, Navigation and Machine Learning For Robotics
* Marshall Johnson
* Winter 2022
# Package List
This repository consists of several ROS packages
- <PACKAGE1> - <one sentence description>

# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- rigid2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality

   One possible design is using a function within a struct or class. Another possible design would be creating a normalize class or struct and using a function within that. A third possible design would be using a function outside of both struct and class. 

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
   Adding a function to an existing class would be simple and keep the complexity down. However, it could make things confusing by adding another item to a complex class. Making a class for only the normalize function could add complexity that's not needed. However, the plus side is that it would be private to the class. Using a function outside of class and struct would reduce the confusion, but it would not be tied to anything and would be public to other users.

   - Which of the methods would you implement and why?

   A function within a struct --- the struct format would help keep things organized and being able to call variables from this struct is convenient. The function would have access to the struct variables allowing for a simple elegant solution.

2. What is the difference between a class and a struct in C++?

A struct and a class are very similar in C++. The primary difference between the two is that everything in a struct is public by default, whereas private variables can be created in a class. 


3. Why is Vector2D a struct and Transform2DClass (refer to at least 2 specific C++ core guidelines in your answer)?

Vector2D is a struct because the members can vary independently (C++ core guideline C.2). Transform2D 


4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?




5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

   

# Sample Run of frame_main
```
<Put the output of a representative run here>