#!/bin/bash

CS_TARGET_FRAMEWORKS="net48 net462 net45 net35 netstandard2.0 netstandard1.3"

if [[ ! -f "../../../nuget_packages.config" ]]; then
    echo "[ERROR] You are not running this script from native_build/external/nuget_packages folder!"
    exit -1
fi

# Populate C-sharp packages
if [ ! -f "../nuget.exe" ]; then
    wget https://dist.nuget.org/win-x86-commandline/latest/nuget.exe -O ../nuget.exe
fi
cp ../../../nuget_packages.config ./packages.config
if [ -f '/usr/bin/mono' ]; then
    mono ../nuget.exe install packages.config -OutputDirectory .
else
    chmod +x ../nuget.exe
    ../nuget.exe install packages.config -OutputDirectory .
fi

# sort out all dll files under specified compile targets.
# enter directory
for d in */ ; do
    # if only one dll exists, extract it.
    dlls=$(find $d/lib -maxdepth 2 -type f -name "*.dll")
    if [[ `echo $dlls | wc --words` = "1" ]]; then
        mv $dlls .
        continue
    fi
    # otherwise, follow the given priority:
    # TODO: User can add more precedence below.
    for target in $CS_TARGET_FRAMEWORKS; do
        dlls=$(find $d/lib -maxdepth 2 -type f -name "*.dll" | grep -i "$target")
        if [[ `echo $dlls | wc --words` = "1" ]]; then
            mv $dlls .
            continue 2
        fi
    done
done
# the package folders are all useless now. Keep extracted dlls only.
rm -r ./*/
rm ./packages.config
