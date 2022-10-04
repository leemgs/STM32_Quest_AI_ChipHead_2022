# STM32_Quest_AI_ChipHead_2022
STM32_Quest_AI_ChipHead_2022


# Getting Started
* https://make.e4ds.com/contest/?ctidx=7 (STM32 Cube AI)

* Entire Schedule
```bash
QUEST 1, STM32Cube.AI 개발 환경 구축하기
2022. 8. 1 (월) ~ 2022. 8. 7 (일)

QUEST 2, Neural Network Model 생성 개발 환경 구축하기
2022. 8. 8 (월) ~ 2022. 8. 28 (일)

QUEST 3, B-L475E-IOT01A 보드에 바이너리 파일 라이팅하기
2022. 8. 29 (월) ~ 2022. 9. 11 (일)

QUEST 4, B-L475E-IOT01A 보드 가속도 센서 활용하여 AI 모델을 위한 Dataset 구성하기
2022. 9. 12 (월) ~ 2022. 9. 18 (일)

QUEST 5, 새로 생성한 학습된 Model을 STM32Cube.AI를 사용하여 프로젝트에 적용하기
2022. 9. 19 (월) ~ 2022. 9. 25 (일)

QUEST 6, STM32Cube.AI와 B-L475E-IOT01A 보드를 이용하여 본인만의 AI 프로젝트를 만들기
2022. 9. 26 (월) ~ 2022. 10. 10 (월)
```

* Corrected Entired Schedule
```bash

Quest4: IoT 보드로 센서 데이타 수집 및 데이타셋 만들기 (.csv)
--> 사용툴: IoT보드 (B-L475E-IOT01A), STBLESensor Mobile App

Quest2: 데이타 셋으로 Keras Model (예: har_ign.h5) 만들기  <====== (issue: Quest1의 예제는 cnn_gmp.h5)  
--> 사용툴: Anaconda, PyCharm IDE

Quest1: Keras Model의 그래프 분석 및 모델구조 조회하기 
--> 사용툴: STM32CubeMX, X-CUBE-AI software pack, FP-AI-SENSING1 function pack

Quest5: Keras Model로 전처리/후처리 소스 코드 생성하기 (예제: har_ign.*, har_ign_data.*)  
--> 사용툴: STM32CubeMX

Quest3: 생성한 소스 코드로 펌웨어 이미지 생성 및   IoT 보드에 이미지 플래싱하기  
--> 사용툴: IoT 보드 (B-L475E-IOT01A), STM32CubeIDE, STM32Programmer, STBLESensor Mobile App
```


##  파일 포맷별 차이점 비교: .ckpt vs .pb vs .h5
1. ckpt: Pytorch에 주로 사용되는 포맷이다. 그래프 (=모델구조)만 있는 .ckpt-meta 파일, 가중치(weight)만 있는 .ckpt-data 파일으로 구성된다.
2. pb: 그래프 (=모델구조)와 가중치(weight) 를 모두 가지고 있는 파일이다.
3. h5: Keras에서 주로 사용하는 포맷이다. 그래프 (=모델구조)와 가중치(weight)를 모두 가지고 있는 파일이다.



## Reference
* https://www.st.com/en/development-tools/stm32cubemx.html
* https://www.st.com/en/embedded-software/fp-ai-sensing1.html
* https://www.st.com/en/evaluation-tools/b-l475e-iot01a.html
* https://www.st.com/en/development-tools/stm32cubeprog.html
* https://www.st.com/en/embedded-software/x-cube-ai.html
* https://www.st.com/en/embedded-software/fp-ai-sensing1.html
* https://www.st.com/en/embedded-software/stblesensor.html
* https://www.st.com/resource/en/user_manual/um2153-discovery-kit-for-iot-node-multichannel-communication-with-stm32l4-stmicroelectronics.pdf
* https://wiki.st.com/stm32mcu/wiki/Category:STM32Cube.AI
* https://wiki.st.com/stm32mcu/wiki/AI:Introduction_to_Artificial_Intelligence_with_STM32
* https://community.st.com/s/topic/0TO0X0000003iUqWAI/stm32-machine-learning-ai
* https://www.anaconda.com
* https://www.jetbrains.com/pycharm
* https://www.tensorflow.org/tutorials
* https://github.com/STMicroelectronics/STBlueMS_Android


## Terminology
* AI: Artificial Intelligence
* HAR: Human Activity Recognition
* ASC: Acuastic Scene Classification 
* IGN: IGN Model Type
* GMP: IGN Model Type
* EWARM: Embedded Workbench for ARM
* IDE: Integrated Development Environment
* STM32: STMicroelectronics 32bit
* TBA: To Be Added
