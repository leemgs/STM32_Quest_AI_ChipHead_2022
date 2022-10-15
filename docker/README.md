## Building CUBEAI

- Download X-CUBE_AI-Linux from https://www.st.com/en/embedded-software/x-cube-ai.html#get-software
- Build the firmware and network data using the Dockerfile.

```
docker build . -t cube_ai --build-arg X_CUBE_AI_LINUX_PATH=stm32ai-linux-6.0.0.zip \
    --build-arg X_CUBE_AI_PACK_PATH=STMicroelectronics.X-CUBE-AI.6.0.0.pack
# Ensure that you provide the correct paths to the downloaded X-CUBE-AI library.

```

  -
- Once you have built the cube_ai docker image, prepare to optimize the model you’re using for the STM32 microcontroller. Go to your workspace, create a directory and copy the model file. 

```
mkdir data && cp {path_to_model_file} data/
```

```
docker run -v {full_path_to_data_folder}:/data cube_ai data/{model_file}
```


## Reference
* https://www.cardinalpeak.com/blog/optimizing-edge-ml-ai-applications-using-stmicros-stm32cube-ai
