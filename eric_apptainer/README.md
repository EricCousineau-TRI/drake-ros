https://drakedevelopers.slack.com/archives/C2CP2E50A/p1663368591567899

https://gist.github.com/sloretz/074541edfe098c56ff42836118d94a8d

```sh
git clone https://github.com/sloretz/apptainer-ros

apptainer build --fakeroot repro.sif \
    ./apptainer-ros/jammy-ros-humble-desktop/Apptainer
apptainer exec \
    --fakeroot --nv --writable --containall \
    repro.sif

mkdir src
cd src
```
