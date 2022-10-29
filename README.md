<div align="center">
<img alt="Avionics" src="https://user-images.githubusercontent.com/78698227/197661461-2b9ddcb8-5559-4407-aab9-86414d6145f3.png" width="550"/>
</div>

## About
Discovery board support for [AvionicsSoftware](https://github.com/StudentOrganisationForAerospaceResearch/AvionicsSoftware)

## Setup
1. Initialize the AvionicsSoftware submodule by using the commands
```
git submodule init
git submodule update
```
2. Change **huart5** to **huart6** in main_avionics.hpp

## Relevant Pinout
DEBUG_UART_RX = PC6 </p>
DEBUG_UART_TX = PC7
