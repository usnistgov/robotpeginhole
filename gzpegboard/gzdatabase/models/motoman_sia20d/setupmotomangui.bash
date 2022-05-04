#!/bin/bash

#      ^^ this has to be bash, not /bin/sh, for arrays to work

p=`pwd`

ans=$(zenity --width=600 --height=400  --list  --text "Motoman options?" --checklist  --column "Feature" --column "options" FALSE "Peter" FALSE "Moto Robotiq" FALSE "Moto Fingers" TRUE "Finger Contact"   --separator=":")


if [ "$?" != 0 ]
then
    notify-send "Configuration setup cancelled"
#    zenity --info --text="Configuration setup cancelled" 
    exit
fi

peter=$(echo $ans | grep "peter" | wc -l)
contact=$(echo $ans | grep "Finger Contact" | wc -l)
kludge=$(echo $ans | grep "Gripper Kludge" | wc -l)
motorobotiq=$(echo $ans | grep "Moto Robotiq" | wc -l)
motofingers=$(echo $ans | grep "Moto Fingers" | wc -l)

D=
if [ "$kludge" = 1 ] ; then
D="$D -DGRIPPER_KLUDGE"
fi

if [ "$contact" = 1 ] ; then
D="$D -DCONTACT"
fi

if [ "$motorobotiq" = 1 ] ; then
D="$D -DMOTO_ROBOTIQ"
fi
if [ "$motofingers" = 1 ] ; then
D="$D -DMOTO_APRS_FINGERS"
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




