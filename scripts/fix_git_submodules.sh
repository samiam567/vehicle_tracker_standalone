#!/bin/bash

# set current directory to git top level directory
cd $(git rev-parse --show-toplevel)

cp .gitmodules .gitmodules.backup

git config -f .gitmodules --get-regexp '^submodule\..*\.path$' |
    while read path_key path
    do
        url_key=$(echo $path_key | sed 's/\.path/.url/');
        branch_key=$(echo $path_key | sed 's/\.path/.branch/');
        # If the url_key doesn't yet exist then backup up the existing
        # directory if necessary and add the submodule
        if [ ! $(git config --get "$url_key") ]; then
            # before proceeding, we need to make sure the folder with the same name does not exist,
            # either by backing up or removing the folder.
            if [ -d "$path" ] && [ "$(ls -A $path)" ] ; then
                mv "$path" "$path""_backup_""$(date +'%Y%m%d%H%M%S')";
                echo "[WARNING] ORIGINAL NON-EMPTY ${path} HAS BEEN BACKED UP AS ${path}_backup_$(date +'%Y%m%d%H%M%S')."
            elif [ -d "$path" ] && [ ! "$(ls -A $path)" ] ; then
                # this is an empty folder, remove it anyway.
                rm -rf $path
            fi;
            url=$(git config -f .gitmodules --get "$url_key");
            # If a branch is specified then use that one, otherwise
            # default to master
            branch=$(git config -f .gitmodules --get "$branch_key");
            if [ ! "$branch" ]; then branch="master"; fi;
            git submodule add -f -b "${branch##origin/}" "$url" "$path";
        fi;
    done;

# In case the submodule exists in .git/config but the url is out of date

git submodule sync;

# Now actually pull all the modules. I used to use this...
#
# git submodule update --init --remote --force --recursive
# ...but the submodules would check out in detached HEAD state and I
# didn't like that, so now I do this...

git submodule foreach --recursive 'git checkout $(git config -f $toplevel/.gitmodules submodule.$name.branch || echo master) && git submodule init && git submodule update --recursive';

rm .gitmodules
mv .gitmodules.backup .gitmodules
