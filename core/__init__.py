"""
Core — Drone Navigation and Mapping (3-Processor Architecture)

Physical drone has 3 processors connected via SPI/UART buses:

  STM32H7  (stm32h7/)  Flight controller — IMU, motors, GPS, battery
  STM32N6  (stm32n6/)  Neural processor  — LiDAR, camera, SLAM, search algorithm
  STM32WL  (stm32wl/)  Mesh radio        — gossip protocol, inter-drone comms

Inter-processor data channels defined in processor_bus.py.
drone_manager.py is the simulation orchestrator that spans all 3 processors.
"""
