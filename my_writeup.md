## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./imgs/project2_kuka_arm_1.png
[image2]: ./imgs/project2_kuka_arm_2.png
[image3]: ./imgs/project2_kuka_arm_3.png
[image4]: ./imgs/fk.png
[image5]: ./imgs/l21-l-inverse-kinematics-new-design-fixed.png


---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Most of the code is copied from Demo Walk through placed on your site.

## Kinematic Analysis


#### 1. Using the DH parameter table, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0  | 0      | 0     | 0     | q1
1  | - pi/2 | 0.35  | 0.75  | -pi/2 + q2
2  | 0      | 1.25  | 0     | q3
3  | - pi/2 | -0.054| 0     | q4
4  |   pi/2 | 0     | 1.5   | q5
5  | - pi/2 | 0     | 0     | q6
6  | 0      | 0     | 0.303 |  0


#### 2. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Translation matrices for each joint:
At first we have to define function which will help to calculate matrices

```
def TF_Matrix(alpha, a, d, q):
  TF = Matrix([
    [cos(q), 		-sin(q), 		0, 		a],
    [sin(q)*cos(alpha), 	cos(q)*cos(alpha), 	-sin(alpha), 	-sin(alpha)*d],
    [sin(q)* sin(alpha), 	cos(q)*sin(alpha), 	cos(alpha), 	cos(alpha)*d],
    [0,			0,			0,		1]])
   return TF
```

The individual transoframtion matrices are using arguments from DH table above

```
  T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
  T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
  T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
  T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
  T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
  T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
  T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)
```


## Inverse Kinematic

![alt text][image5]

Below is the code I used to calculate Inverse Kinematic

```
  theta1 = atan2(WC[1], WC[0])

  side_a = 1.501
  side_b = sqrt(pow(sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35, 2)+ pow((WC[2] - 0.75), 2))

  side_c = 1.25

  angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
  angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))

  theta2 = pi/2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35) 

  theta3 = pi/2 - (angle_b + 0.036) # 0.036 accounts for sag in link4 of -0.054m
  
  R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
  R0_3 = R0_3.evalf(subs={q1: theta1, q2:theta2, q3: theta3})

  R3_6 = R0_3.transpose() * ROT_EE

  theta4 = atan2(R3_6[2,2], -R3_6[0,2])
  theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
  theta6 = atan2(-R3_6[1,1], R3_6[1,0])
``` 


Below are pictures of successed picked and dropped cylinders, some runs had troubles and didnt succed every time but 7 out of 10 times was very rare, but sometimes happend

![alt text][image1]

![alt text][image2]

![alt text][image3]


