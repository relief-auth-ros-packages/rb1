This repository should be cloned into the home directory.

Directory `scripts` hosts the scripts that run and kill the reader's
application and the robot's bringup. Directory `configuration_files` hosts the
controller's (joystick) configuration that maps its buttons to keyboard keys,
so that a SSH connection to the robot is made redundant for execution and
teleoperation.

Once the robot's pc is up and running execute

- `L1+circle` to launch the robot (once executed, use `R1+right_pad` to rotate and `R1+left_pad` to translate)
- `L1+L2+circle` to kill the robot
- `L1+square` to launch the reader
- `L1+L2+square` to kill the reader

A useful mnemonic device is remembering that the robot has a circular shape,
while the antennas attached to the robot are of a square shape.

If keyboard execution is desired, use the following mapping:
- `L1 = Ctrl`
- `L2 = Alt`
- `square = 1`
- `circle = 2`

so that `Ctrl+1` launches the reader application, `Ctrl+Alt+1` kills it,
`Ctrl+2` launches the robot, and `Ctrl+Alt+2` kills it.

As regards the binding of the content of the `relief_support_files` directory to
the OS:

- Keyboard shortcuts should be mapped to their execution as per the instructions above).
- `antimicro_conf.gamecontroller.amgp` should be added to the startup applications
  of the robot's OS with the command field filled as follows:
  `screen -S relief_antimicro -d -m antimicro --hidden --profile /home/$USER/relief_support_files/configuration_files/antimicro_conf.gamecontroller.amgp`.
