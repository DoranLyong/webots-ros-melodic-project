XAUTH=/tmp/.docker.xauth

mkdir $XAUTH

if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    sudo chmod a+r $XAUTH
fi


sudo docker run -t -d \
    --name webots_melodic \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --runtime=nvidia \
    -v $HOME:/workspace \
    doranlyong/webots-ros-melodic:rviz-moveit-v1.0.4 \
    bash
