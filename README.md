# MEOWTOR

<img width="1080" height="540" alt="METOR" src="https://github.com/user-attachments/assets/4a203d8a-39df-4669-aa41-93dcbc078392" />

## Power Management
Our vehicle uses a 3.7V LiPo battery as its main power source and a Mabuchi DC motor for driving. The electrical system also supplies the Raspberry Pi Pico 2, ESP32-CAM, steering servo, and sensors. All components share a common ground.
A regulated 5V supply is planned for the ESP32-CAM and steering servo. Since the battery voltage is below 5V, this supply requires a suitable step-up converter. The Pico 2 and sensors will be connected according to their required input voltages. The converter output and all power connections will be verified before the electronics are connected.
