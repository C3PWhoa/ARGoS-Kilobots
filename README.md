# Citation

In scientific manuscripts that are based on simulations offered by this pluging, please cite the paper:

C. Pinciroli, M.S. Talamali, A. Reina, J.A.R. Marshall and V.Trianni. Simulating Kilobots within ARGoS: models and experimental validation. In _Proceedings of 11th International Conference on Swarm Intelligence (ANTS)_, LNCS 11172: 176-187, Springer, Cham, 2018. doi: [10.1007/978-3-030-00533-7_14](https://doi.org/10.1007/978-3-030-00533-7_14)

# Compiling the code

Make sure you have ARGoS >= 3.0.0-beta52 installed!

Commands:
```shell
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ../src
make
sudo make install
```

## Lab 0
```shell
argos3 -c src/examples/experiments/kilobot_blinky.argos
```

## Lab 1.2
```shell
argos3 -c src/examples/experiments/kilobot_simple_movement.argos
```

## Lab 1.3
```shell
argos3 -c src/examples/experiments/kilobot_nonblocked_movement.argos
```

## Lab 2.1-2.2
```shell
argos3 -c src/examples/experiments/kilobot_speaker_listener.argos
```

## Lab 2.3-2.4
```shell
argos3 -c src/examples/experiments/kilobot_speaker_listener_mod.argos
```

## Lab 3
```shell
argos3 -c src/examples/experiments/kilobot_disperse.argos
```

## Lab 4
```shell
argos3 -c src/examples/experiments/kilobot_orbit.argos
```

## Lab 5
```shell
argos3 -c src/examples/experiments/kilobot_move_to_light.argos
```

## Lab 6
```shell
argos3 -c src/examples/experiments/kilobot_gradient_simple.argos
```

## Lab 7
```shell
argos3 -c src/examples/experiments/kilobot_sync.argos
```

## Lab 8
```shell
argos3 -c src/examples/experiments/kilobot_orbit_2.argos
```

## Lab 9
```shell
argos3 -c src/examples/experiments/stop_orbit.argos
```

## Lab 10
```shell
argos3 -c src/examples/experiments/kilobot_multi_orbit.argos
```

## Lab 11
```shell
argos3 -c src/examples/experiments/shape.argos
```

## Lab 12
```shell
argos3 -c src/examples/experiments/phototaxis.argos
```

## Lab 13
```shell
argos3 -c src/examples/experiments/big_form.argos
```

## Lab 14
```shell
argos3 -c src/examples/experiments/shape54.argos
```

# Differences between Kilombo and ARGoS

## Kilombo
  * Architecture
    * Single-thread, single process wrapper around kilolib.h
      * Robots must run the same behavior
      * Global variables cannot be used to contain state
  * Models
    * Only model offered is the Kilobot
    * Motion is kinematics with simple overlap resolution
      * Robots cannot push other objects
    * Communication neglects obstructions
    * Message drop has uniform probability

## ARGoS
  * Architecture
    * Multi-thread, multi-process architecture
    * Robots can run different behaviors
    * Global variables can be used to contain state
  * Models
    * Models of Kilobot, other robots, boxes, cylinders
      * Motion is full 2D dynamics
      * Robots can push other objects
    * Communication considers obstruction
    * Message drop considers local density
   
# New labs designed to understand a method of shape formation
   f. Turchet. As part of a swarm robotics course

   For the proper functioning of these labs, the position of the 'comm' entity has been modified in the file kilobot_entity.cpp. You may be required to make the modification to the original file yourself.

   SAnchor& cCommAnchor = m_pcEmbodiedEntity->AddAnchor("comm", CVector3(0.01, 0.0, KILOBOT_RAB_ELEVATION));

  ## Lab 8 : kilobot_orbit_2
     * Goal: to orbit one kilobot around three stationary kilobots.
       * Concept of minimum distance.
       * 3 states : 
         * ORBIT_TOOCLOSE
         * ORBIT_FORWARD
         * ORBIT_NORMAL

  ## Lab 9 : stop_orbit
     * Goal: Stop a robot when it reaches the desired relative position.
       * Concept of minimum distance.
       * Declaration of a distance array
       * 4 states : 
         * ORBIT_TOOCLOSE
         * ORBIT_FORWARD
         * ORBIT_NORMAL
         * ORBIT_STOP


   ## Lab 10 : kilobot_multi_orbit
     * Goal: Stop 3 robots when it reaches the desired relative position.
       * Concept of minimum distance.
       * Declaration of a distance array
       * 4 states : 
         * ORBIT_TOOCLOSE
         * ORBIT_FORWARD
         * ORBIT_NORMAL
         * ORBIT_STOP

   ## Lab 11 : shape 
     * Goal: Stop 3 robots when it reaches the desired relative position With the assignment of a new identifier. 
     * The arrival order of the robots is no longer relevant. See the XML file.
       * Concept of minimum distance.
       * Declaration of a distance array
       * Assignment of a new identifier.
       * 4 states : 
         * ORBIT_TOOCLOSE
         * ORBIT_FORWARD
         * ORBIT_NORMAL
         * ORBIT_STOP

   ## Lab 12 : phototaxis 
     * Goal: Stop 12 robots when it reaches the desired relative position With the assignment of a new identifier. 
     * The arrival order of the robots is no longer relevant.
     * The robots approach the origin of the shape through the phenomenon of phototaxis.
       * Concept of minimum distance.
       * Declaration of a distance array
       * Assignment of a new identifier.
       * 5 states : 
         * ORBIT_TOOCLOSE
         * ORBIT_FORWARD
         * ORBIT_NORMAL
         * ORBIT_STOP
         * MOVE_TO_LIGHT

   ## Lab 13 : big form - 22 kilobots 
     * Goal: Stop 22 robots when it reaches the desired relative position With the assignment of a new identifier. 
     * The arrival order of the robots is no longer relevant.
     * The robots approach the origin of the shape through the phenomenon of phototaxis.
       * Concept of minimum distance.
       * Declaration of a distance array
       * Assignment of a new identifier.
       * Divides the group of robots into two to avoid congestion.
       * Simplification of the method for calculating the number of robots present in the shape. This avoids errors.
       * 6 states : 
         * ORBIT_TOOCLOSE
         * ORBIT_FORWARD
         * ORBIT_NORMAL
         * ORBIT_STOP
         * MOVE_TO_LIGHT
         * WAIT_TOGO

   ## Lab 14 : shape54 - 54 kilobots 
     * Goal: Stop 54 robots when it reaches the desired relative position With the assignment of a new identifier. 
     * The arrival order of the robots is no longer relevant.
     * The robots approach the origin of the shape through the phenomenon of phototaxis.
       * Concept of minimum distance.
       * Declaration of a distance array
       * Assignment of a new identifier.
       * Divides the group of robots into 7 to avoid congestion.
       * Each group starts within a predefined time interval in the 'wait_togo()' function.
       * Simplification of the method for calculating the number of robots present in the shape. This avoids errors.
       * The definition of the shape is important. The construction order must have a certain logic.
       * 6 states : 
         * ORBIT_TOOCLOSE
         * ORBIT_FORWARD
         * ORBIT_NORMAL
         * ORBIT_STOP
         * MOVE_TO_LIGHT
         * WAIT_TOGO
