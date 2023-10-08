##########################################################
# Dev board used
##########################################################
    Nucleo-144 stm32f429zi
    https://botland.com.pl/stm32-nucleo/5527-stm32-nucleo-f429zi-stm32f429zit6-arm-cortex-m4-5904422364939.html

##########################################################
# GPIOs for motor pwm (TIM3)
##########################################################
    PA6 for CH1 (alias pwm1_dcmA1)
    PA7 for CH2 (alias pwm2_dcmA2)

    Timer for PWM motor control is tim3 (htim3) 
    (APB1@84MHz) on CH1 & CH2

    *   PSC set to 83 (84)
    *   ARR set to 999 (1000)
    *   Auto-reload preload: enabled
    *   Counter mode: up 
    
    *   Pwm freq = 1kHz

##########################################################
# GPIOs for motor encoder (TIM4)
##########################################################
    PD12 for CH1 (alias enc_A)
    PD13 for CH2 (alias enc_B)
    
    Timer for encoder is tim4 (htim4) 
    (APB1@84MHz) on CH1 & CH2
	
    For Encoder_mode : Encoder Mode TI1   => ARR set to 1103 (final CPR)
					   (= 23*48-1 = 1103 = gear_ration*base_CPR*1)
 	For Encoder_mode : Encoder Mode TI1&2 => ARR set to 2207 (final CPR)
 					   (= 23*48*2-1 = 2207 = gear_ration*base_CPR*2*1)

    Counter mode: up

##########################################################
# AS5600 magnetic encoder
##########################################################
    https://www.reddit.com/r/embedded/comments/sebcb5/c_driver_for_ams_as5600_magnetic_position_sensor/
    https://github.com/raulgotor/ams_as5600

    for arduino (c++)
    https://github.com/RobTillaart/AS5600/tree/master

    PB8 for i2c1 scl (alias I2C1_SCL) 
    PB9 for i2c1 sda (alias I2C1_SDA)
    
    I2C speed mode : Standard mode
    I2C clock speed : 100_000


##########################################################
# Board view
########################################################## 
    built in button: PC13

                            ________________________________________________________________ arduino shield 1 starts here (with grove connectors)
                                                                                    [ ][ ] |   PB8 I2C1_SCL
                                                                                    [ ][ ] |   PB9 I2C1_SDA
                            ---------------------------------------------------------------- arduino shield 2 starts here                                    
                            | [ ][ ]                                                [ ][ ] |
                            | [ ][ ]                                                [ ][ ] |
                            | [ ][ ]                                                [ ][ ] |
               3.3V         | [x][ ]                                                [ ][x] |   PA6 (pwm tim3 ch1)
                 5V         | [x][ ]                                                [ ][x] |   PA7 (pwm tim3 ch2)
                GND         | [x][ ]                                                [ ][ ] |
                            | [][]                                                  [ ][ ] |
                            | [][]                                                  [ ][ ] |
                            |                                                              |     
   PA3 (adc_pot / ADC3/3)   | [x][ ]                                                [ ][ ] |
                            | [ ][ ]                                                [ ][ ] |
                            | [ ][ ]                                                [ ][ ] |
                            | [ ][ ]                                                [ ][ ] |
                            | [ ][ ]                                                [ ][ ] |
                            | [ ][ ]                                                [ ][ ] |
                            | [ ][ ]                                                [ ][ ] |
                            | [ ][ ]                                                [ ][ ] |
                            ---------------------------------------------------------------- arduino shield 1&2 ends here
                            | [ ][ ]                                                [ ][ ] |
                            | [ ][ ]                         PD13 (enc tim4 ch2)    [x][ ] |
                            | [ ][ ]                         PD12 (enc tim4 ch1)    [x][ ] |
                            | [ ][ ]                                                [ ][ ] |
                            | [ ][ ]                                                [ ][ ] |
                            | [ ][ ]                                                [ ][ ] |
                            | [ ][ ]                                                [ ][ ] |
                            | [ ][ ]                                                [ ][ ] |
                            | [ ][ ]                                                [ ][ ] |
                            ________________________________________________________________


