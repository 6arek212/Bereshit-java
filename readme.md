
# Bereshit Landing

In this project we simulate a successful landing with bereshit spacecraft actual settings

<img width="600" height="400" src="https://user-images.githubusercontent.com/10331972/228521956-ad967e3c-6475-4686-9ea7-b16fe0a81a49.png"/>

## About the simulation

- **PID** controller was used to control the engine thrust and vehicle 
angle, It's done by maintaining a specific vertical/horizontal speed and angle.
- The starting point mimcs bereshit actual landing settings
- Landing physics

### Starting Settings

<img width="600" height="400" src="https://user-images.githubusercontent.com/10331972/228526687-0e0682f3-efc2-4660-993b-51ebe20f7218.png"/>


## Besheshit Crash Landing Story



<img width="600" height="400" src="https://user-images.githubusercontent.com/10331972/228522415-fcd43bf2-dcb0-46ff-9372-8268b47ceb4d.jpeg"/>

- At the start of the voyage a problem with the star tracking system
was found to be corrupted which responsible for determining the ship angle
and the crew decided to use the acceleration sensor instead
- The low cost materials were damaged by the sun radiations which cause the ship system to restart
- Because of the low budget the ship had no redundant system
- While landing the acceleration sensor shutdown (IMU2) so the crew had to make a quick
decision to rely on IMU1 or to restart IMU2, And they choose to restart
- Activating IMU2 blocked data transmission from IMU1 because of some system logic,
Therefor the system did not receive any acceleration data for about a second and a system restart sequence began
- The restarting shutdown the main engine and the ship was in free fall till it crashes the Moon


## Besheshit Crash Landing Sequence

- IMU2 issue
- Activating IMU2
- Data transfer from IMU1 was blocked due to restart of IMU2
- System issues a restart
- The system waiting for the control commands to be uploaded
- meanwhile the main engine shutdown due to the restart
- Crashed :|

## Besheshit Crash Landing Cuses

- IMU2 issue
- low cost materials
- low experienced crew
- A rushed mission (more testing needed)
- System restarts
- Data transfer block
- Control commands were not hard coded in the system

More info: https://github.com/6arek212/Bereshit-java/blob/main/explanation.pdf

## Team Members

- Tarik Husin
- Reuet 
- Najeeb
