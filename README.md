Automated Live-Insect Dispenser for Studying Prey-Capture Behavior in Rodents 



Build and script files for an automated, 3D-printed dispenser designed to release individual live crickets during rodent prey-capture experiments. The dispenser was developed to reduce experimenter intervention during prey delivery and is controlled through an Arduino. 3D-printed parts are provided as STEP files, a complete Fusion build, and a complete PreForm print assembly. Arduino scripts are provided for calibrating the motor and operating the dispenser in manual trigger and timed trigger modes.



The design was inspired by the [Turrigiano Lab cricket dispenser](https://github.com/regisshan/CricketDispenser), with modifications to the rotating carousel, capping and release mechanisms, and servo control system. 



The dispenser uses an eight-bucket carousel driven by a Parallax Feedback 360 servo motor. Each bucket holds one live cricket, and the carousel rotates to release crickets individually into the arena. Assembly, calibration, and operation are described in the accompanying protocols.io protocol.



Components: 

* Eight-bucket rotating carousel (consists of hoop and bucket holder components; holds buckets upright until the release opening, where they rotate freely around a dowel pin to release prey)
* Hinged bucket caps (open upward as buckets approach the release opening)
* Servo encasing (houses and stabilizes the servo motor and helps attenuate motor noise)
* Axle (attaches to the servo wheel with screws; rotates the bucket holder and cap pieces)
* Parallax Feedback 360 servo motor
* Arduino and wiring 
* Mounting hardware (currently mounted using 80/20 aluminum extrusions)



Control: 

* Arduino controls rotation of the carousel following either a key-press trigger or a timed interval using positional feedback
* Optional TTL output for synchronizing dispensing events with behavioral recordings



Configurations

* Single dispenser 
* Dual dispenser (increases prey capacity and release coverage area)



Possible Modifications and Uses 

* The mounting components can be adjusted so long as the dispenser remains stable and aligned
* Dispenser can also be used to release food rewards at different locations within the arena









