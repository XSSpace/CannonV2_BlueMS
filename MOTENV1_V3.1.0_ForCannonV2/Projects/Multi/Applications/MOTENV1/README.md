Bluetooth low energy and sensors in MOTENV1 
=======================================

The MOTENV1 expansion software package for STM32Cube lets you read and display real-time inertial (e.g. motion MEMS) and environmental (e.g. humidity, pressure, temperature) sensors  data  with a dedicated Android™/iOS™ application. The package includes:

*	MotionFX (INEMOEngine), which uses advanced algorithms to integrate outputs from multiple MEMS sensors in a "smart" way, independent of environmental conditions, to reach optimal performance.

*	MotionAR (iNEMOEngine) real-time activity-recognition algorithm

*	MotionCP (iNEMOEngine) carry-position recognition algorithm

*	MotionGR (iNEMOEngine) gesture recognition algorithm

*	MotionPM (iNEMOEngine) pedometer algorithm

*	MotionID (iNEMOEngine) real-time Motion Intensity recognition



This project uses the following STMicroelectronics elements:

 * X-NUCLEO-IDB04A1/X-NUCLEO-IDB05A1 Bluetooth Low energy expansion boards

 * X-NUCLEO-IKS01A1 Expansion board for four MEMS sensor devices:
       HTS221, LPS25H, LSM6DS0, LSM6DS3, LIS3MDL
	   
 * X-NUCLEO-IKS01A2 Expansion board for four MEMS sensor devices:
       HTS221, LPS22HB, LSM6DSL, LSM303AGR

 * NUCLEO-F401RE NUCLEO-L476RG NUCLEO-L053R8 Nucleo boards

 * STEVAL-STLCS01V1 (SensorTile) evaluation board that contains the following MEMS sensor devices:
      HTS221, LPS22HB, LSM303, LSM6DSM

This example must be used with the related BlueMS Android/iOS application available on Play/itune store, in order to read the sent information by Bluetooth Low Energy protocol:

[link to BlueMS Android](https://play.google.com/store/apps/details?id=com.st.bluems)

[link to BlueMS iOS](https://itunes.apple.com/it/app/st-bluems/id993670214)
