#!/bin/sh
# convert a motor module into an axis module

#
errorAndHelp()
{
	echo >&2 "usage $0 <directory)"
	exit 1
}


if test -z "$1"; then
	errorAndHelp
fi

DIR="$1"
MOTORSTRINGS="asynMotorAxis.h|MOTOR|motorSupport.dbd|_LIBS.*=.*motor|/motor/|record.*motor"

files=$(egrep -l -R  $MOTORSTRINGS $DIR)
echo files=$files
if test -z "$files"; then
	exit
fi

for file in $files; do
	echo file=$file
	if test -w "$file"; then
		cp -v "$file" /tmp/$$.tmp &&
		sed </tmp/$$.tmp >"$file" \
				-e "s/asynMotorAxis.h/asynAxisAxis.h/g"	\
				-e "s/motorSupport.dbd/axisSupport.dbd/g"	\
				-e "s!/motor/!/axis/!g"	\
				-e "s/record\(.*\)motor/record\1axis/g"	\
				-e "s/LIBS\(.*=.*\)motor/LIBS\1axis/g"	\
				-e "s/asynMotorController.h/asynAxisController.h/g"	&&
		rm /tmp/$$.tmp
	fi
done
