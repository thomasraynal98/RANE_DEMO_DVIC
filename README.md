## README DEMO DVIC

Here is the file gathering all the codes necessary for the permanent demo of the DVIC of summer 2021.

## LIEN UTILE
https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/#overview

http://libserial.sourceforge.net/doxygen/class_serial_port.html#a6d063c5984b774bd09b7df95192f1af0

https://libserial.readthedocs.io/en/latest/tutorial.html

## FEATURES AND BUGS
* [BUG]       : Fix bug 2 ports deconnexion.
* [ADD]       : Add all variable (voltage, etc...).
* [BUG]       : thread pass to stop when port is disconnect.
* [BUG]       : thread frequency problem TH3/TH5.
* [BUG]       : the cpu load is not very accurate.
* [ADD]       : create a .sh to chmod all component.
* [ADD]       : in make_weighted_map.cpp add lower case security for interest name.

## INSTALLATION INFORMATION
* sometime you need to chmod all ports for microcontrolers.

## WHEN YOU USE ./make_weighted_map EXECUTABLE
* put the occupensy grid map.png in the folder "/data/map_brut".
* when you run "./make_weighted_map" add in argument the name of "map.occupensy". 
* see the process 1 and 2 processing map to protect robot navigation.
* at the end you need to select interest point in map & add is name in console & press 'entrer'.
* all the interest points and weigthed map will be save in "/data/map_weighted" and "/data/map_interest_point".
