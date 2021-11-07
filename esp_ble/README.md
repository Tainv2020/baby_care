Folder
| esp-idf-v4.2.1 | gatt_client |

ESP-IDF Gatt Client
===================

1. Download ESP IDF: git clone -b v4.2.1 --recursive https://github.com/espressif/esp-idf.git esp-idf-v4.2.1
2. Download project: git clone https://gitlab.com/truonggiangbk/esp_ble.git
3. sudo apt-get install git wget flex bison gperf python3 python3-pip python3-setuptools cmake ninja-build ccache libffi-dev libssl-dev dfu-util
4. cd esp-idf-v4.2.1
5. ./install.sh
6. . ./export.sh
7. cd ../gatt_client
8. idf.py flash monitor
