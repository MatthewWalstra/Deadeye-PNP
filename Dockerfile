# syntax=docker/dockerfile:1

FROM j3ff/deadeye-cpp-base
LABEL maintainer=“jeff@j3ff.io”

WORKDIR /app

COPY . /app
WORKDIR /app/build

RUN cmake \
    -D CMAKE_BUILD_TYPE=Debug \
    -D DEADEYE_BUILD_TESTS=ON \
    -D DEADEYE_UNIT_ID=C \
    -D DEADEYE_NT_SERVER=10.27.67.2 \
    -D DEADEYE_CAMERA0_PIPELINE=custom::CustomPipeline \
    /app \
    && make -j$(nproc)

EXPOSE 5805/tcp

CMD ["./custom_deadeyed"]
