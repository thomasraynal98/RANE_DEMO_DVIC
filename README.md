## README DEMO DVIC

Here is the file gathering all the codes necessary for the permanent demo of the DVIC of summer 2021.

## LIEN UTILE
https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/#overview

http://libserial.sourceforge.net/doxygen/class_serial_port.html#a6d063c5984b774bd09b7df95192f1af0

https://libserial.readthedocs.io/en/latest/tutorial.html

## FEATURES AND BUGS
* [BUG]       : fix bug 2 ports deconnexion.
* [ADD]       : add all variable (voltage, etc...).
* [BUG]       : thread pass to stop when port is disconnect.
* [BUG]       : thread frequency problem TH3/TH5.
* [BUG]       : the cpu load is not very accurate.
* [ADD]       : create a .sh to chmod all component.
* [ADD]       : in make_weighted_map.cpp user input add lower case security for interest name.
* [BUG]       : resolve the data running problem for metadata callback
to add it to debug interface.
* [ADD]       : for the moment the "try_avoid" area is only use for
validation distance and not in A* algorythme.
* [ADD]       : automatic distance between KP en fonction of drift security distance.
* [ADD]       : take speed of robot in compte to create validation speed.
* [BUG]       : put some lock during distance_RKP and target_angle update.
* [BUG]       : bug of path planning when destination is very far.
* [ADD]       : add global variable for map precision.
* [ADD]       : smooth voltage sensor value.

## INSTALLATION INFORMATION
* sometime you need to chmod all ports for microcontrolers.

## WHEN YOU USE ./rane_software
* run this command "sudo ./rane_software".

## WHEN YOU USE ./make_weighted_map EXECUTABLE
* put the occupensy grid map.png in the folder "/data/map_brut".
* when you run "./make_weighted_map" add in argument the name of "map.occupensy". 
* see the process 1 and 2 processing map to protect robot navigation.
* at the end you need to select interest point in map & add is name in console & press 'entrer'.
* all the interest points and weigthed map will be save in "/data/map_weighted" and "/data/map_interest_point".
