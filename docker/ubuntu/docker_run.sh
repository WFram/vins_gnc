WS=$1
docker run \
    --name \
    vins_test \
    -it \
    -v \
    /tmp/.X11-unix:/tmp/.X11-unix \
    -e \
    HOME=$WS \
    --net=host \
    --gpus \
    all \
    -e \
    DISPLAY=$DISPLAY \
    -w \
    $WS \
    -v \
    $HOME:$HOME \
    --device=/dev/dri:/dev/dri \
    -it \
    vins:main \
    bash