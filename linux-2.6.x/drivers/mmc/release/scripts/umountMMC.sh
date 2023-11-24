#!/bin/sh
REPEAT=10
echo "umount MMC card start"

umount /mmc/mmca1

if [ $? -ne 0 ];then
    i=1
    while [ $i -le $REPEAT ]
    do
	if mount | grep /mmc/mmca1 >/dev/null
	then
		fuser -k /mmc/mmca1; 
		umount /mmc/mmca1
		ret=$?
		if [ $ret -eq 0 ]; then
			echo "umount MMC card successful ($i)!"
        		break
		fi
		i=$(($i + 1))
        else
		break
                echo "umount MMC card end"
		exit 0;
	fi
    done
else
    echo "umount MMC card successful end!"
    exit 0;
fi
echo "umount MMC card umount failed"
exit 1;
