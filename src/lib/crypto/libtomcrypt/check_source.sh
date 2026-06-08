#!/bin/bash

# output version
bash printinfo.sh

make clean > /dev/null

echo "checking..."
./helper.pl --check-source --check-makefiles --check-defines|| exit 1

exit 0

# ref:         HEAD -> main
# git commit:  684efeb5120df0df20fb56771cda3d310858777d
# commit time: 2026-06-07 22:27:16 +0800
