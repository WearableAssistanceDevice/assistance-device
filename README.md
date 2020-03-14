# assistance-device

## Setup
Download SEGGER Embedded Studio [here](https://www.segger.com/products/development-tools/embedded-studio/)

Download the nRF5 SDK [here](https://www.nordicsemi.com/Software-and-tools/Software/nRF5-SDK)

Clone this repo into the folder `${NRF_SDK_DIR}/projects/wearable/`,  where `NRF_SDK_DIR` is the nRF5 SDK folder.

## Usage
Pressing the button indicated by `ASSIST_REQ_KEY_EVENT` will initiaite the assistance request process. The program will write `true` to the assistance request characteristic and attempt to connect to a server with the name indicated by `SERVER_PERIPH_NAME`. Once connected, the server should fetch the value of the assistance request characteristic and write back `false` to acknowledge receipt. Once this has happened, the device will disconnect from the server.

The server software can be found here: https://github.com/WearableAssistanceDevice/assistance-request-server
