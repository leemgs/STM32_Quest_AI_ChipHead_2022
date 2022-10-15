#!/bin/sh

model_name=`basename $1`

cp /data/$model_name /openmv_workspace/openmv/src/stm32cubeai/

cd /openmv_workspace/openmv/src/stm32cubeai/  && \
    stm32ai generate -m $model_name --compression 8 -o data/


cd /openmv_workspace/openmv/src/micropython/mpy-cross &&\
    make && \
    cd /openmv_workspace/openmv/src/ && \
    sed -i"" "s/OMV_HEAP_SIZE           (240K)/OMV_HEAP_SIZE           (230K)/g" \
            /openmv_workspace/openmv/src/omv/boards/OPENMV4P/omv_boardconfig.h && \
    make clean && \
    make TARGET=OPENMV4P CUBEAI=1


mkdir -p /data/output/model/
cp /openmv_workspace/openmv/src/stm32cubeai/data/* /data/output/model/
cp /openmv_workspace/openmv/src/build/bin/firmware.bin /data/output/
