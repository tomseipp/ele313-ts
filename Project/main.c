// TASK 2


// Adam was actually here 22/4/26
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include "leds.h"
#include "spi_comm.h"
#include "sensors/proximity.h"
#include "motors.h"
#include "epuck1x/uart/e_uart_char.h"
#include "stdio.h"
#include "serial_comm.h"
#include "selector.h"
#include "sensors/VL53L0X/VL53L0X.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

// Initialising stuff

int main(void){ 
    halInit();
    chSysInit();
    mpu_init();

    // Proximity
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    proximity_start(0);
    calibrate_ir();

	//TOF
	VL53L0X_start();

    //LED
    clear_leds();
    spi_comm_start();
	
    //Motors
    motors_init();

	//Bluetooth
	serial_start();

    // variable declarations
	int strongest;
	int strongest_ir;

	int minir = 20;
	int threshold = 1000;
	int selecta;
	int tof;
	int tof_thresh_base=100;
	int tof_thresh;

	while (1){ // infinite Main Loop!
		while (get_selector() < 11){ // when selector >=11 robot wont move, if it was over 11 wheelspeed would be too high
			
			strongest = 0;
			strongest_ir =0;

			//find max value
			for (i=0; i<8; i++){
				if (get_calibrated_prox(i)>=get_calibrated_prox(strongest)){
					strongest = i;
				}
			}

			strongest_ir = get_calibrated_prox(strongest);

			
			// if it can see the object
			if (strongest_ir>minir){
				// if it is close enough already, stop then turn
				if (strongest_ir>threshold){ // only runs when robot is super close, turns on spot then stops
					switch (strongest){
						// turn until maximum is at front
						case 0:// infront therefore do nothing
							left_motor_set_speed(0);
							right_motor_set_speed(0);
						break;
						case 1:
						case 2:
						case 3:
							while (strongest!=0 ||strongest!=7){ // runs while neither of the front sensors are strongest
								left_motor_set_speed(500); // on right therfore spin right
								right_motor_set_speed(-500);
								//find strongest sensor each loop of while
								for (i=0; i<8; i++){
									if (get_calibrated_prox(i)>get_calibrated_prox(strongest)){
									strongest = i;
									}
								}
							}
							left_motor_set_speed(0); // once object is in front, stop spinning
							right_motor_set_speed(0);
						break;
						case 4:
						case 5:
						case 6:
						//on left spin left
							while (strongest!=0 ||strongest!=7){
								left_motor_set_speed(-500);
								right_motor_set_speed(500);
								//find max value
								for (i=0; i<8; i++){
									if (get_calibrated_prox(i)>get_calibrated_prox(strongest)){
									strongest = i;
									}
								}
							}
							left_motor_set_speed(0); // once object is in front, stop spinning
							right_motor_set_speed(0);
						break;
						case 7: // infront therfore do nothing
							left_motor_set_speed(0);
							right_motor_set_speed(0);
						break;
					}
				
				}else{ // only runs when at middle distance (ir can see object but not touching)
					switch(strongest){
						case 0:
							left_motor_set_speed(700);
							right_motor_set_speed(700);
							break;
						case 1:
							left_motor_set_speed(700);
							right_motor_set_speed(500);
							break;
						case 2:
							left_motor_set_speed(600);
							right_motor_set_speed(100);
							break;
						case 3:
							left_motor_set_speed(500);
							right_motor_set_speed(-500);
							break;
						case 4:
							left_motor_set_speed(-500);
							right_motor_set_speed(500);
							break;
						case 5:
							left_motor_set_speed(100);
							right_motor_set_speed(600);
							break;
						case 6:
							left_motor_set_speed(500);
							right_motor_set_speed(700);
							break;
						case 7:
							left_motor_set_speed(700);
							right_motor_set_speed(700);
							break;
					}
				}
			} else{	// if ir cannot see it, revert to tof
				//else
				// find the object by spinning or whatever tof sensor

				// get tof
				//get selecta
				//while tof is greater than threshold, spin
				// once tof can see object, go forwards

				tof = VL53L0X_get_dist_mm();
				selecta = get_selector();
				tof_thresh = selecta * tof_thresh_base;

				if (tof>tof_thresh){
					left_motor_set_speed(-500);
					right_motor_set_speed(500);
					
				}else{
					left_motor_set_speed(1000);
					right_motor_set_speed(1000);
				}
			
			}

			
		} //end of selector loop
	} // end of infinite loop
	
   
} // end of main


#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
