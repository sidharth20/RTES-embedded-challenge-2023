# RTES-embedded-challenge-2023
## This project was a part of the course ECE-GY-6483 - Real Time Embedded Systems(RTES).

**Technical Aspects:**
* **Board used:** STM32F429 Discovery
* **IDE:** PlatformIO on VSCode
* **Coding Language:** C
* **Processor:** ARM 

**Objectives:**
* Use the data collected from a single gyroscope that is present on the board to record a hand movement sequence as a means to generally “unlock” a resource.
* Recorded sequence must be saved on the microcontroller, the correct gesture is automatically recorded.
* User then must replicate the key sequence within sufficient tolerances to unlock
the resource. (*User must press detect pattern in the serial monitor*)
* A successful unlock must be indicated by a visual indication, such as showing the board is locked or unlocked on the LCD display.

**Logic Used:**
* Using the gyroscope, I collected RAW data of the change in radians.
* Each movement will have 3 coordinates (i.e. aX, aY, aZ) 
* I am recording 1 gesture movement.
* Then normalizing the data based on linear algebra using L2 norm.
* And using this normalized data to make decision whether the entered gesture matches the saved gesture and how close the match is.

**Video Demonstration:** [Link](https://drive.google.com/file/d/1lBpOHOMhmpoazfBHMAqMsBt1aU_QnAA6/view?usp=share_link)

**Group Members**
*Sidharth Shyamsukha(ss14885@nyu.edu)
*Shvejan Mutheboyina(ssm10076@nyu.edu)
