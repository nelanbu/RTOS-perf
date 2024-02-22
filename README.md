# Comparative Study of FreeRTOS vs NuttX Performance

## Overview
This project investigates the performance differences between FreeRTOS and NuttX, two prominent open-source Real-Time Operating Systems (RTOS). It focuses on key performance metrics such as context switch time, semaphore passing latency, inter-task messaging latency, task activation from ISR latency, memory allocation latency, and memory footprint. Conducted as part of the Research Methodology and Scientific Writing course at KTH, this study aims to provide insights into selecting an appropriate RTOS for embedded systems projects.

## Key Findings
- **Context Switch Time:** FreeRTOS demonstrated lower latency compared to NuttX, making it more efficient in task management.
- **Semaphore Passing Latency:** NuttX showed almost 1.5 times shorter latency, indicating better performance in semaphore shuffling.
- **Inter-Task Messaging Latency:** FreeRTOS outperformed NuttX, suggesting a faster message transfer between tasks.
- **Task Activation from ISR Latency:** FreeRTOS had lower latency, indicating quicker response to interrupts.
- **Memory Allocation Latency & Footprint:** FreeRTOS exhibited both lower memory allocation time and a smaller memory footprint, suggesting higher efficiency in resource-constrained environments.

## Methodology
The study employs a range of tests conducted on the STM32 Nucleo-144 development board with an STM32F767ZI microcontroller. It measures the performance of both RTOS in terms of scheduling efficiency and resource utilization, providing a quantitative basis for comparing these systems.

## Conclusion
The analysis concludes that FreeRTOS generally offers better performance across most tested parameters, suggesting its suitability for general-purpose applications in embedded systems. However, NuttX's lower semaphore passing latency indicates its potential advantages in specific use cases.

## How to Run Tests
Instructions on replicating the tests are provided, detailing the hardware setup, software configuration, and execution steps necessary to measure the performance of FreeRTOS and NuttX according to the methodology described in this study.

## Contributing
Contributions to enhance the study or extend the tests to cover additional metrics are welcome. Please submit pull requests or open issues to discuss potential improvements.

## Acknowledgments
- Special thanks to my collaborator, Robert Hynasinski, for his invaluable insights and contributions throughout the duration of this project.
- KTH Royal Institute of Technology for academic support.
- The developers and contributors of FreeRTOS and NuttX for providing the open-source platforms for this study.
