Your path should include directories `application` and `octane_etk-5.12.0.240`.
Follow instructions from `https://support.impinj.com/hc/en-us/articles/202755288-Speedway-Revolution-Embedded-Development-Tools-and-Sample-Application-ETK-`

Execute the reader's acquisition code by issuing

```$ cd application; make clean; make x86; bin/speedwayr_x86 <reader-ip> <reader-mac-address-condensed>```

As regards `<reader-ip>`:

1. If you connect the reader **directly** to a pc, its ip will most probably be
169.254.1.1. If it's not in the range of 169.254.\*.\*, scan for it by issuing
```$ nmap -sP 169.254.0-255.255```

2. If you connect the reader to the router the pc is connected to, its ip
will reside in the range 192.168.X.X. You can then scan in this range to find
it by trial and error, or, if access to the router's interface is feasible,
the reader's ip can be obtained by accessing the list of connected clients.

In any case, you should be able to access the reader's webpage through its ip
(and in this way you will know that this is its real ip if you need to search
for it in 192.168.X.X).

Furthermore the RFID reader's clock and the computer's clock should be synchronised.
See file `TIME_SYNCHRONISATION_GUIDE.txt` under directory `application` for
instructions.


Subscribed topics: none

Params:

`robot_type`: `rb1` or `turtlebot`
`callback_frequency`: How many times in a second the rfid localisation algorithm
should run
`rfid_locations`: The absolute path of the file in which the output of the rfid
localisation algorithm is written
`tags_infos`: The absolute path of a supplementary file containing information
about the object each RFID tag tags
`robot_velocity`: The absolute path of the file in which the robot's velocity
is logged
`rfid_antennas_param_file`: A list of absolute paths to all the files that host
the poses of an antenna
`rfid_measurements_param_file`: A list of absolute paths to all the files that
host the raw RFID measurements of each RFID reader
