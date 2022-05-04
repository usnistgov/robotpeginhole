#!/bin/bash

p=`pwd`


ans=$(yad --geometry=400x100 --title "Motoman Configuration" --form --text="Choose options" \
 --field="Gripper:CB" "Peter Schunk"\!"Robotiq C85"\!"Brian Fingers" \
 --field="Motoman Urdf:CB" Old\!New\
 --field="Finger Contact:CB" Yes\!No\!Old
 )
echo ans = $ans

if [ -z "$ans" ]
then
    notify-send "Configuration setup cancelled"
#    zenity --info --text="Configuration setup cancelled" 
    exit
fi

OIFS=$IFS
IFS='|'  
read -r GRIPPER URDF CONTACT <<< "$ans" # read field values and assign to variables 
#MULTILINE=$(echo -e $MULTILINE)                   # Convert '\n' in TXT field back into linefeeds 
echo -e "$GRIPPER\n$URDF\n$CONTACT"
IFS=$OIFS

peter=$(echo $GRIPPER | grep "Peter" | wc -l)
motorobotiq=$(echo $GRIPPER | grep "Robotiq C85" | wc -l)
motofingers=$(echo $GRIPPER | grep "Brian Fingers" | wc -l)
newurdf=$(echo $URDF | grep "New" | wc -l)
contact=$(echo $CONTACT | grep "Yes" | wc -l)



# Determine preprocessor defines
D=

if [ "$contact" = 1 ] ; then
D="$D -DCONTACT"
fi

if [ "$motorobotiq" = 1 ] ; then
D="$D -DMOTO_ROBOTIQ"
fi

if [ "$motofingers" = 1 ] ; then
D="$D -DMOTO_APRS_FINGERS"
fi

if [ $newurdf = 0 ] ; then
peter=1
echo "use peter new URDF model and schunk gripper"
fi

# Assume all gears for now


if [ "$peter" = 1 ] ; then
aprs=`pwd`/../motoman_aprs
sia=`pwd`/../motoman_sia20d
gcc -E $D -P -C  - < $aprs/motoman_aprs_master.sdf |  $p/removecomments.perl | sed '/^$/d'   >  $sia/sia20d.sdf
else
sia=`pwd`/../motoman_sia20d
gcc -E $D -P -C  - < $sia/sia20d-master.sdf |  $p/removecomments.perl | sed '/^$/d'   >  $sia/sia20d.sdf

fi



