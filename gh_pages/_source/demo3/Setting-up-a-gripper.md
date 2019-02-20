3 Setting up a Gripper

For this we will use the Schunk WSG 050 gripper.
We will use the ipa325_wsg50 driver

Step 1: Clone the driver into your workspace with 

git clone https://github.com/ipa320/ipa325_wsg50.git


Step 2: Edit necessary parts (see git history)
Here I will list example changes but let the student make them.
* CMake
* package.xml
* bringup robot.launch
* main pick and place node
