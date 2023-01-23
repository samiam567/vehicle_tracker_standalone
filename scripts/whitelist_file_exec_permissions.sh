#!/bin/bash

# disable executable flag using a white-listing approach (only directory and the
# files with whitelisted extensions are applied the +x flag.)
# this is a whitelist approach.
GIT_ROOT=$(git rev-parse --show-toplevel)
find ${GIT_ROOT} -type f -print0 | xargs -0 chmod -x

# append the whitelist below.
declare -a whitelistedExts=(
    "*.sh"
    "*.bash"
    "*.py"
)

for ext in "${whitelistedExts[@]}"; do
    if [[ -n $(find ${GIT_ROOT} -type f -name ${ext}) ]]; then
        find ${GIT_ROOT} -type f -name ${ext} -print0 | xargs -0 chmod +x
    fi
done

# launch files do not need +x permission despite they have the .py extension.
find ${GIT_ROOT} -type f -name "*.launch.py" -print0 | xargs -0 chmod -x

# special cases are handled separately
cd .git/hooks
chmod +x post-checkout post-commit post-merge pre-push
cd ..

cd scripts
chmod +x ade_entrypoint
cd ..
