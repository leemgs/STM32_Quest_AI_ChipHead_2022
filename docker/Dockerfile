# Download base image ubuntu 18.04
FROM ubuntu:18.04

# Build essentials
RUN apt -y update &&\
    apt -y upgrade &&\
    apt -y install git zip make build-essential tree software-properties-common &&\
    add-apt-repository ppa:deadsnakes/ppa &&\
    apt -y install python3.8

# Copy X-CUBE-AI-Linux  https://www.st.com/en/embedded-software/x-cube-ai.html#get-software
RUN mkdir /openmv_workspace && cd /openmv_workspace && \
    git clone --recursive https://github.com/openmv/openmv.git


ARG X_CUBE_AI_LINUX_PATH=en/stm32ai-linux-6.0.0.zip
ARG X_CUBE_AI_PACK_PATH=en/STMicroelectronics.X-CUBE-AI.6.0.0.pack
ARG MODEL_PATH=en/baseline_emotion_model.hdf5
ARG MODEL_TYPE=keras

COPY $X_CUBE_AI_LINUX_PATH /openmv_workspace/stm32ai-linux.zip
COPY $X_CUBE_AI_PACK_PATH /openmv_workspace/STMicroelectronics.X-CUBE-AI.pack

RUN cd /openmv_workspace  && \
    mv STMicroelectronics.X-CUBE-AI.pack STMicroelectronics.X-CUBE-AI.zip  && \
    unzip STMicroelectronics.X-CUBE-AI.zip -d X-CUBE-AI  && \
    unzip stm32ai-linux.zip -d X-CUBE-AI/Utilities

ENV PATH="/openmv_workspace/X-CUBE-AI/Utilities/linux:${PATH}"

RUN apt -y remove gcc-arm-none-eabi && \
    apt -y autoremove && \
    add-apt-repository ppa:team-gcc-arm-embedded/ppa && \
    apt -y update && \
    apt -y install gcc-arm-embedded && \
    ln -s /usr/bin/python3.8 /usr/bin/python

RUN cd /openmv_workspace/openmv && \
    # git checkout v3.9.4 && \
    cd src/stm32cubeai  && \
    mkdir -p AI/Inc AI/Lib data && \
    cp -r /openmv_workspace/X-CUBE-AI/Middlewares/ST/AI/Inc/* AI/Inc/ && \
    cp /openmv_workspace/X-CUBE-AI/Middlewares/ST/AI/Lib/GCC/STM32H7/NetworkRuntime*_CM7_GCC.a AI/Lib/NetworkRuntime_CM7_GCC.a

COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
