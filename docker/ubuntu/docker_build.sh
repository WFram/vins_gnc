docker image build \
    -t \
    vins:main \
    --build-arg \
    USER_ID=$(id -u) \
    --build-arg \
    GROUP_ID=$(id -g) \
    .