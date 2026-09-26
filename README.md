# MEOWTOR

<img width="1080" height="540" alt="METOR" src="https://github.com/user-attachments/assets/4a203d8a-39df-4669-aa41-93dcbc078392" />

## Power Management
Our vehicle uses a 3.7V LiPo battery as its main power source and a Mabuchi DC motor for driving. The electrical system also supplies the Raspberry Pi Pico 2, ESP32-CAM, steering servo, and sensors. All components share a common ground.
A regulated 5V supply is planned for the ESP32-CAM and steering servo. Since the battery voltage is below 5V, this supply requires a suitable step-up converter. The Pico 2 and sensors will be connected according to their required input voltages. The converter output and all power connections will be verified before the electronics are connected.

 ## Electronics and Components

 The table below lists the main electronic components selected for MEOWTOR and the role of each component.

| Component | Role |
| --- | --- |
| Raspberry Pi Pico 2 | Main control microcontroller |
| ESP32-CAM | Captures images for visual sensing |
| DRV8833 motor driver | Controls the drive motor |
| Mabuchi DC motor | Drives the vehicle |
| Steering servo | Turns the steering mechanism |
| VL53L1X ToF sensor | Measures distance in front of the vehicle |
| Two VL53L0X ToF sensors | Measure distance on the left and right sides |
| MPU-6050 | Measures motion and rotation |
| 3.7V LiPo battery | Main power source |

## Team Members

| Member | Role |
| --- | --- |
| ENG.Hoor | Electronics & Project Documentation |
|ENG. Jana | Embedded Systems Programming |
| ENG.Raghad | Mechanical Design, 3D Printing & Assembly |

## Challenges and Engineering Solutions

### 1. Power Instability Under Multiple Loads

**Challenge:**

During the initial integration tests, the sensors, SG90 steering servo, and ESP32-CAM operated correctly when tested separately. However, when all components were connected and activated simultaneously, the system became unstable. Some sensors stopped responding, the ESP32-CAM operated inconsistently, and the steering servo did not move reliably.

**Engineering Analysis:**

The team evaluated the electrical system as a complete load rather than treating each component individually. The analysis indicated that the combined current demand exceeded the capacity of the original power supply. The SG90 servo produces short current peaks while moving, and the ESP32-CAM also requires a stable power supply during camera processing. When these components operated simultaneously, the resulting voltage drop affected the controllers and interrupted communication with the sensors.

To confirm the cause, the team compared the power supply’s rated current with the estimated current requirements of all connected components and monitored the system while activating several loads simultaneously.

**Engineering Solution:**

The original supply was replaced with a regulated power source capable of supporting the combined current demand with an appropriate safety margin. The power distribution was reorganized so that high-current components, particularly the steering servo, did not draw power through the controller’s low-current output pins. All controllers and components were connected to a common ground to maintain reliable signal references.

The team also verified that the voltage supplied to each component remained within its safe operating range. After these changes, the sensors, SG90 servo, Raspberry Pi Pico 2, and ESP32-CAM operated simultaneously without resets, communication failures, or inconsistent steering.

**Validation:**

The complete electrical system was tested repeatedly under simultaneous operation. The camera processed images while the servo moved and the distance sensors continuously collected measurements. The system remained stable throughout the test.

### 2. Mechanical Integration and Custom Metal Fabrication

**Challenge:**

A rigid metal component had to be adapted to fit the available space within the vehicle chassis. Its original dimensions were incompatible with the final assembly, and inaccurate modification could have affected the vehicle’s alignment, weight distribution, structural strength, and steering stability.

Initial attempts using the available hand tools did not produce a sufficiently accurate or safe cut. Continuing with the same method could have deformed the component, created uneven edges, weakened the material, or produced an unreliable mounting point.

**Engineering Analysis:**

The team examined the component’s structural purpose, mounting position, surrounding clearances, and required final dimensions. The analysis showed that forcing the cut with unsuitable tools could permanently deform the material and reduce the accuracy of the complete chassis.

The team therefore treated the component as a custom-fabricated structural part. Precise measurements were taken from the chassis, and the required cutting line, mounting position, and final dimensions were marked before fabrication.

**Engineering Solution:**

The team completed the dimensional planning, measurement reference, cutting template, and component design. Because suitable metal-cutting equipment was unavailable to the team, a specialized workshop operator performed only the machine-cutting operation using the team’s prepared measurements and marked cutting line.

After cutting, the edges were finished to remove sharp or uneven areas. The team then repeatedly test-fitted the component and inspected its alignment, rigidity, mounting stability, and compatibility with the surrounding mechanical and electronic parts.

This approach transformed the original metal component into a vehicle-specific structural part without requiring a complete chassis redesign. It preserved the useful strength of the material, reduced unnecessary replacement and waste, and allowed the mechanical design to proceed while maintaining accurate alignment.

**Validation:**

The customized component was installed and inspected under normal mechanical loading. Its mounting points remained stable, and no visible deformation, interference, or alignment problems were observed during steering and movement tests.

<img width="1600" height="1200" alt="non hoor" src="https://github.com/user-attachments/assets/daf19e1b-fbce-4673-864b-18fddd2c5198" />
<img width="1600" height="1200" alt="jana by hoor" src="https://github.com/user-attachments/assets/56a8feee-4388-4657-8250-73539bd0ea1a" />
