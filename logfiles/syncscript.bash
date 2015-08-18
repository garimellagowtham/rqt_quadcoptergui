#!/bin/bash
#Get the path of the current folder from home
if [ -z "$2" ]
	then
		if [ -z "$1" ]
			then
				echo "Usage  syncscript.bash [USER_NAME_OPTIONAL] [HOST_NAME] [FILE_OR_DIR_OPTIONAL]"
				echo "If no username, assumes same username"
				echo "file/dir relative to current path without ./"
				echo "If no file/dir, syncs the current directory"
				exit
			else
				HOST_RSYNC=$1
				USER_RSYNC=$USER
		fi
	else
		HOST_RSYNC=$2
		USER_RSYNC=$1
fi
if [ -z "$3" ]
	then
		foo=${PWD#$HOME/}
		echo "Rsync running on $foo with User:$USER_RSYNC Host:$HOST_RSYNC"
		rsync  -arvz  -e ssh $USER_RSYNC@$HOST_RSYNC:$foo/ $PWD/
	else
		if [ -f "$3" ]
			then
				file=${PWD}/$3
				foo=${file#$HOME/}
				echo "Rsync running on $foo with User:$USER_RSYNC Host:$HOST_RSYNC"
				rsync  -arvz  -e ssh $USER_RSYNC@$HOST_RSYNC:$foo $file
			else
				if [ -d "$3" ]
					then
						dir=${PWD}/$3
						foo=${dir#$HOME/}
						echo "Rsync running on $foo with User:$USER_RSYNC Host:$HOST_RSYNC"
						rsync  -arvz  -e ssh $USER_RSYNC@$HOST_RSYNC:$foo/ $dir/
					else
						echo "File specified on arg 3 not a regular file or directory"
				fi
		fi
fi

unset HOST_RSYNC
unset USER_RSYNC
