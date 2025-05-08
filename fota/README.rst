# FOTA
This example uses the nRF peripheral_lbs sample (BLE LED Button Service) as a starting point to evaluate FOTA (Firmware Over The Air) updates.

# Main changes for (BLE) FOTA applications
Add these lines to your BLE app's prj.conf file (BLE needs to be enabled/configured):
CONFIG_BOOTLOADER_MCUBOOT=y
CONFIG_NCS_SAMPLE_MCUMGR_BT_OTA_DFU=y

# How to update your firmware over the air (BLE):
- Open your nRF Connect app on your smart device.
- Connect to your peripheral.
- Click on the DFU icon (near the top right).
- Select the app_update.bin file.
- Select mode.
- Click ok.