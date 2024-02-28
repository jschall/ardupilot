## Info
`libraries/AP_HAL_ChibiOS/hwdef/CubeOrange-KHA/default.parm` -- contains all default fixed wing parameters with NextVision Gimbal being the default payload except for:
- SYSID_THISMAV (should be manually set to correspond to the aircraft's tail #)
- BRD_SERIAL_NUM (same as SYSID_THISMAV)
- calibration values (should be stored [HERE](https://drive.google.com/drive/folders/1ScxtoawJhTXpWJX9DlWr1udHUKfenv-b?usp=drive_link))
  - accel
  - servo
  - compass
- payload specifics

`VTOL8S.parm` -- contains 8S VTOL params


[Instruction on how to convert between fixed wing and VTOL](https://krausaerospace.slab.com/posts/converting-k-1000-p-to-vtol-and-vice-versa-rs4zldgv)
