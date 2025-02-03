# Script to configure and build Stratux images with pi-gen
#!/bin/bash -e -x

arch=`uname -m`
echo "arch ${arch}"

# disable full image (stage 5), normal image (stage 4) and desktop image (stage 3)
touch pi-gen/stage3/SKIP pi-gen/stage4/SKIP pi-gen/stage5/SKIP

# and disable building the images based on the SKIPPED stages
touch pi-gen/stage3/SKIP_IMAGES pi-gen/stage4/SKIP_IMAGES pi-gen/stage5/SKIP_IMAGES

# copy the config file into place
cp config pi-gen/

# copy the stage2 10-stratux files into place
rsync -av --delete stage2/10-stratux/ pi-gen/stage2/10-stratux/

# clone the local git repository (and present branch) into pi-gen
# so the files are visible to docker
#
# NOTE: This means local file changes will NOT be reflected in the image build
local_git=`pwd`/../
(cd pi-gen && rm -rf stratux && git clone ${local_git} stratux)
(cd pi-gen/stratux && git submodule update --init --recursive)

# build via docker
#
# pass PRESERVE_CONTAINER=1 to keep the container in the case of error
# to enable debugging
(cd pi-gen && ./build-docker.sh)
