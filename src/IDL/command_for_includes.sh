#!/bin/sh
cd M12Typelib/include;
ls > headerfiles.txt ;
sed -i -e 's/^/#include "/' headerfiles.txt;
sed -i -e 's/$/"/' headerfiles.txt;
sed -i '1d' headerfiles.txt
