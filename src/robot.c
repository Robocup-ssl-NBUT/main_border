 /********************************************************************************
 ********************************************************************************
 * È¨ÏÞ:	º¼ÖÝÄÏ½­»úÆ÷ÈË¹É·ÝÓÐÏÞ¹«Ë¾
 * ÎÄ¼þÃû:	  robot.c 		
 * ¹¦ÄÜÃèÊö:	
		  ¿ØÖÆÆ÷Ö÷³ÌÐò
 * °æ±¾ 	 ×÷Õß		Ê±¼ä		  ×´Ì¬
 * V1.0 	 shouxian	2015.9.10	´´½¨ÎÄ¼þ
 * V1.1      sunlifeng  2016.2.28   ÐÞ¸ÄÎÄ¼þ
 *****************************************************************************
 *****************************************************************************/
#include <string.h>
#include "math.h"
#include "arm_math.h"
#include "typedef.h"
#include "robot.h"
#include "cfg.h"
#include "action.h"
#include "timer.h"
#include "gpio.h"
#include "factory.h"
#include "param.h"
#include "pid.h"
#include "misc.h"
#include "motor.h"
#include "comm.h"
#include "usart_ble.h"


#include "usb_bsp.h"
#include "usbh_core.h"
#include "usbh_usr.h"
#include "usbh_hid_core.h"
#include "usbh_hid_joystick.h"
#include "usbd_cdc_vcp.h"
#include "MPU6050_driver.h"
#include "error_event.h"
#include "packet.h"
/*************** Globals ****************/
robot_t g_robot;

int wheel_reduction_ratio_x_set; /*¼õËÙ±È*/		
int wheel_reduction_ratio_yz_set; /*¼õËÙ±È*/  //¾ÉÂÖ×Ó¼õËÙ±ÈÎª70/22 Îª3.1818 ÍâÈ¦×ª1È¦£¬ÄÚÈ¦ÂëÅÌ×ª3.1818È¦
int max_shot_strength_set;
unsigned char startpoint;
unsigned char steplength;

volatile char g_do_set_receive_mode_flag = 0;
volatile char g_set_receive_mode_flag = 0;

int forcestopcounter=0;
unsigned char last_beep = 0;

u8 is_low_power_cnt = 0;

u8 g_pid_timer_cnt = 0;
timer_t power_mon_timer;
timer_t heart_led_timer;
timer_t rf_comm_tim;           //·¢Éä»úÍ¨ÐÅ³¬Ê±Ê±¼ä
timer_t identify_cpuid_tim;    //cpuidÈÏÖ¤³¬Ê±Ê±¼ä ÉèÖÃÎª10S
timer_t gyro_mon_timer;

timer_t shoot_interval_timer;

extern char eeprom_blue_init_flag;
extern char bluetooth_name[BLUETOOTH_MAX_NAME];
extern char bluetooth_code[BLUETOOTH_MAX_CODE];

extern void UNLOCK_FLASH_RDP(void);
extern u8  cpuid_data[12];
extern unsigned char identify_success;

extern packet_robot_t joystick_packet;

extern char gyro_control_flag;

/*******************************************************************************
* Function Name   : init_robot
* Description	  : ³õÊ¼»¯¿ØÖÆÆ÷Ïà¹ØµÄÒ»Ð©±äÁ¿
*					
* Input		      : None
* Output		  : None
* Return		  : ·µ»ØÖµ0.
*******************************************************************************/
int init_robot(void)
{
	param_t param;
	u8 mode;
	u8 freq;
	u8 num;
	u8 i;
	float angle;
	
  	float wheel_angle[ 4 ] = { 
		 D_WHEEL_ANGLE_FRONT,     //×óÇ°ÂÖ
		-D_WHEEL_ANGLE_FRONT,      //ÓÒÇ°ÂÖ
		-D_WHEEL_ANGLE_BACK_2013,   //ÓÒºóÂÖ
		 D_WHEEL_ANGLE_BACK_2013     //×óºóÂÖÂÖ
	};
		
	
	/* initial parameter from eeprom */
	load_param(&param);
	wheel_reduction_ratio_x_set = param.dat[12];
 	wheel_reduction_ratio_yz_set = param.dat[13];
    eeprom_blue_init_flag = param.dat[15];
	if(param.dat[14] <= MAX_SHOT_STRENGTH)
    {
    	max_shot_strength_set = param.dat[14];
    }
    else 
    {
    	max_shot_strength_set = MAX_SHOT_STRENGTH;
    }

	/* initial g_robot */
	read_dip_sw(&freq, &num, &mode);
	
	memset(&g_robot, 0, sizeof(g_robot));
	g_robot.num = num;
	g_robot.frq = freq;
	g_robot.mode = (mode_t)(mode & 0x7);
    mode = mode & 0x7;
	if(mode == 2) 
		g_robot.rf_mode = RF_BTE;
	else if(mode == 0 || mode == 5 || mode == 6)
		g_robot.rf_mode = RF_24L01;
	     else
	     {
	     }

	for(i = 0; i < CHANNEL_NUM; i++)
	{
		pid_init(&(g_robot.wheels[i].pid), MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD);
	}
	#if MPU6050_GYRO_USED
	gyro_pid_init(&gyro_pid, GYRO_PID_KP, GYRO_PID_KI, GYRO_PID_KD);
	#endif
	g_robot.dribbler = 0;

//	g_robot.kv2n = ( (float)wheel_reduction_ratio_x_set + (float)wheel_reduction_ratio_yz_set 
//		* 0.01f ) * ( 4 * ENCODER_COUNTS_PER_TURN_SET) / 2 / (float)PI / WHEEL_RADIUS;

	g_robot.kv2n = 74037;
	
	/* initial sin and cos table */
	for( i = 0; i < 2; i++ )
	{
	    angle = wheel_angle[ i ] / 180.0f * (float)PI;
	    g_robot.sin_angle[i] = sin( angle ) ;
	    g_robot.cos_angle[i] = cos( angle ) ;
	}
	
    for(i = 2; i < 4; i++)
    {
      	angle = wheel_angle[ i ] / 180.0f * (float)PI;
		    g_robot.sin_angle[i] = sin(angle) ;
      	g_robot.cos_angle[i] = cos(angle) ;
    }
    g_robot.firmware_version = software_verison;
	/* initial other */
	init_shooter();

	/* initial ir pwm */
	start_ir_pwm();

	return 0;
}

/*******************************************************************************
* Function Name   : int do_power_monitor(void)
* Description	  : µç³ØµçÑ¹¼à¿Ø
*					
* Input		      : None
* Output		  : None
* Return		  : ·µ»Ø1±íÊ¾µç³ØµçÑ¹½µµ½ãÐÖµ1 ·µ»Ø2±íÊ¾µç³ØµçÑ¹½µµ½ãÐÖµ2 ·µ»Ø0 µç³ØµçÑ¹Õý³£
*******************************************************************************/
int do_power_monitor(void)
{
	char retflag;
	u8 cap_v;
	
	/* check the cap voltage */
	cap_v = get_cap_v();

	DIS_INT();
	g_robot.cap_v_f = cap_v * CAP_V_ADC_GAIN;
	g_robot.cap_v = cap_v;
	EN_INT();

	retflag = is_power_low();
    if( retflag )
	{ 
		if(is_low_power_cnt < 5)
			is_low_power_cnt++;
		else
		{
			is_low_power_cnt = 5;
		
#ifdef ENABLE_BEEP
			BEEP_ON();
#endif
			return retflag;
		}
	}
	else
	{
        is_low_power_cnt = 0;		
		
#ifdef ENABLE_BEEP
		if(last_beep == 0) BEEP_OFF();
#endif
	}

	return 0;
}


/*******************************************************************************
* Function Name   : do_timer
* Description	  : ÖÐ¶ÏÖÜÆÚµ÷ÓÃ,pid¼ÆËã
*					
* Input		      : None
* Output		  : None
* Return		  : None.
*******************************************************************************/
void do_timer()
{
	/* do motor pid control */
	g_pid_timer_cnt++;
	if(g_pid_timer_cnt >= PID_COUNTER_OVERFLOW)
	{
		update_motor();
		g_pid_timer_cnt = 0;
	}
	
	if(g_do_set_receive_mode_flag)
	{
		g_set_receive_mode_flag++;
	}
}


/*******************************************************************************
* Function Name   : do_run
* Description	  : ½øÈëÖ÷Ñ­»·£¬¸ù¾ÝÑ¡Ôñ²¦ÂëÅÌµÄ²»Í¬£¬Ñ¡Ôñ²»Í¬µÄ¹¤×÷Ä£Ê½¡£
*Ö§³Ö±ÈÈüÄ£Ê½(·¢Éä»úÍ¨ÐÅPC),ÊÖ±úÄ£Ê½(ÓÎÏ·ÊÖ±ú²Ù¿Ø),À¶ÑÀÄ£Ê½(ÊÖ»ú²Ù¿Ø),USBÍ¨ÐÅÄ£Ê½(debugÄ£Ê½PC»úµÄ²ÎÊýÏÂ·¢),×Ô¼ìÄ£Ê½					
* Input		      : None
* Output		  : None
* Return		  : None.
*******************************************************************************/
void do_run(void)
{
	int start_beep_flag = 1;
	static int mode_count = 0;
	int i;
	
	/* at begining, check if do beep when start */
	if(start_beep_flag)
	{
		i = 0;
		if(g_robot.mode <= DEBUG_MODE)	
		{
			i = g_robot.mode + 1;		
		}
		
		while(i != 0)
		{
			BEEP_ON();
			wait_ms(200);
			BEEP_OFF();
			wait_ms(200);
			i--;
		}
		start_beep_flag = 0;
	}

	/* initial timer */
	power_mon_timer = get_one_timer(POWER_MON_TIME);
	heart_led_timer = get_one_timer(HEARTBEAT_TIME);
	rf_comm_tim = get_one_timer(COMM_TIMEOUT_TIME); //ÎÞÏßÍ¨ÐÅÄ£Ê½³¬Ê±Ê±¼ä
	shoot_interval_timer = get_one_timer(1);
	identify_cpuid_tim =get_one_timer(IDENTIFY_CPUID_TIMEOUT_TIME); //cpuidÈÏÖ¤³¬Ê±Ê±¼äÉèÖÃÎª10s 10sÈÏÖ¤²»³É¹¦ÔòÍ£Ö¹»úÆ÷ÈË
	gyro_mon_timer = get_one_timer(GYRO_MON_TIME);
	/* start motor */
	start_motor();
	
	while(1)
	{
   
	   	
		/* check battery voltage */
#ifdef ENABLE_POWERMON
        
		if(check_timer(power_mon_timer))
		{
			power_mon_timer = get_one_timer(POWER_MON_TIME);
			if(do_power_monitor() == 2)//µç³ØµçÑ¹½µµÍµ½Éè¶¨ãÐÖµ1 Ôò·äÃùÆ÷±¨¾¯ ½µµ½ãÐÖµ2ÔòÍ£Ö¹»úÆ÷ÈË
			{
				/* battery voltage is too low, I will stop the robot */
				forcestopcounter++;
				if(forcestopcounter >= LOW_POWER_TIME)
				{
					do_dribbler(0);
					do_move(0,0,0);
					do_shoot(0,0);
					do_chip(0,0);
					while(1) ; /* stop robot */
				}
			}
			else
			{
				if(forcestopcounter > 0) forcestopcounter--;
			}
		}
#endif

#ifdef ENABLE_HEARTBEAT
		/* heart led */
		if(check_timer(heart_led_timer))
		{
			//heart_beat();
			heart_led_timer = get_one_timer(HEARTBEAT_TIME);
		}
#endif
    
	
		/* do robot job */
		switch(g_robot.mode)
		{ 
		    case BlUETOOTH_MODE:
				mode_count++;
				LED_POWER_ON_HIGH();
				set_heart_led(0, 0, 100, 0x1f);//À¶É«
				
			case NORMAL_MODE :
			{
				if(mode_count == 1)//
				{
				   mode_count = 0;
				}
				else
				{  
				    //LED_POWER_ON_HIGH();
				    //set_heart_led(100, 0, 0, 0x1f);//ºìÉ«
				    LED_POWER_ON_LOW();//Ï¨ÃðÉ«²ÊµÆ
				}
				/* process message from comm device( rs232, wireless, etc. ) */
				do_comm();

			    /*Í¨Ñ¶³¬Ê±ÉèÖÃÎª500ms»òÕßý£Ê²Ã´¶¼²»×ö ÉÏÎ»»ú8msÏÂ·¢Ò»¸öÊý¾Ý°ü (¹²2°üÒ»°ü(25Byte)¸øÒ»¶Ó³µ£¬ÁíÍâÒ»°ü¸øÁíÒ»¶Ó³µ)*/
				if(check_timer(rf_comm_tim)) 
				{
					/* ·ÀÖ¹Í¨Ñ¶ÖÐ¶Ï£¬ÖÃÎ»ÉèÖÃÍ¨Ñ¶Îª½ÓÊÕÄ£¿é±êÖ¾Î» */
					g_do_set_receive_mode_flag = 1; 
					
					do_dribbler(0);
					do_move(0,0,0);
					do_shoot(0,0);
					do_chip(0,0);

					start_nRF24L01_RX();	
					rf_comm_tim = get_one_timer(COMM_TIMEOUT_TIME);
					identify_cpuid_tim = get_one_timer(IDENTIFY_CPUID_TIMEOUT_TIME);
				}
//				if(0)
				if(check_timer(identify_cpuid_tim))//Éí·ÝÊ¶±ð³¬Ê±»òÕßÉí·ÝÊ¶±ð²»Í¨ÔòÍ£Ö¹»úÆ÷ÈË
				{
				    /* ·ÀÖ¹Í¨Ñ¶ÖÐ¶Ï£¬ÖÃÎ»ÉèÖÃÍ¨Ñ¶Îª½ÓÊÕÄ£¿é±êÖ¾Î» */
					g_do_set_receive_mode_flag = 1; 
					
					do_dribbler(0);
					do_move(0,0,0);
					do_shoot(0,0);
					do_chip(0,0);

					start_nRF24L01_RX();
					identify_success = 0;
					identify_cpuid_tim = get_one_timer(IDENTIFY_CPUID_TIMEOUT_TIME);
				}
				
				if(g_do_set_receive_mode_flag)	//·¢ËÍÊý¾Ý°üºóÖÃ1µÈ´ýÊý¾Ý·¢ËÍ³öÈ¥ºó½«Ä£Ê½ÐÞ¸ÄÎª½ÓÊÕÄ£Ê½				
				{
					/* ½«Í¨Ñ¶ÉèÖÃÎª½ÓÊÕÄ£Ê½£¬²¢ÖÃÎ»¿É½ÓÊÜ±êÖ¾Î» */
					if(g_set_receive_mode_flag >= 3)
					{				
						start_nRF24L01_RX();
					
						g_set_receive_mode_flag = 0;
						g_do_set_receive_mode_flag = 0; 	//ÖÃÎ»¿É½ÓÊÕ±êÖ¾Î»
					}
				}

				break;
			}

			case JOYSTICK_MODE:
			{
				static int test_time = 0;
				LED_POWER_ON_HIGH();
				set_heart_led(0, 100, 0, 0x1f);//°?greenÉ«			
				test_time++;
				COMM_LED_ON();
				wait_ms(2000);
			  COMM_LED_OFF();	
//				set_test_shooter();
        do_dribbler(3);//ÉèÖÃ¿ØÖÆµµÎ»
				wait_ms(2000);
				do_dribbler(0);
				wait_ms(1000);
				do_shoot(20, 0);//Æ½Éä
		        wait_ms(2000);
				do_chip(0, 20); //ÌôÉä
                wait_ms(2000);				
                if(test_time == 1)
                {
									
								 do_acc_handle_move(0, 0,100);
								 wait_ms(2000);
									do_acc_handle_move(0, 0,-100);
									wait_ms(2000);
								do_acc_handle_move(0, 0,0);
								test_time = 0;
                }
//				else if(test_time == 2)
//					{
//					 do_acc_handle_move(0, 0,-100);
//				     wait_ms(2000);
//				     do_acc_handle_move(0, 0,0);
//					 test_time = 0;
//					}
//                   	
				break;
				
//			/*±±Í¨ÓÎÏ·ÊÖ±ú¿ØÖÆ*/
//#ifdef ENABLE_JOYSTICK_CONTROL
//				
//               
//				LED_POWER_ON_HIGH();
//				set_heart_led(0, 100, 0, 0x1f);//ÂÌÉ«
//				/* Host Task handler */
//				USBH_Process(&USB_OTG_Core_dev , &USB_Host);
//				if(HID_JOYSTICK_Data.packet_valid == 1)
//				{
//					HID_JOYSTICK_Data.packet_valid = 0;
//					do_joystick_packet((unsigned char*)(&HID_JOYSTICK_Data.key_button));
//					
//				}
//				
//				if(check_timer(gyro_mon_timer))
//				{  
//				   
//				   if(gyro_control_flag ==1)
//				   {
//				   	 Get_Motion_data(&acc_gyro_adc,&acc_gyro_actul);
//				     gyro_pid.feedback_speed = -acc_gyro_actul.GYRO_z;  //ÍÓÂÝÒÇZÖáÐý×ªË³Ê±ÕëÎª¸º£¬ÓëÐ¡³µË³Ê±ÕëÐý×ªÖµÎªÕýÏà·´ È¡¸ººÅ
//				   
//				     gyro_control_calcuate(&gyro_pid);
//				   }   
//				  
//		
//				   gyro_mon_timer = get_one_timer(GYRO_MON_TIME);
//				}
//             
//				break;
//#endif
//		
//                   	
//				break;
			}
			case SELFTEST_MODE: //×Ô¼ìÄ£Ê½
			{    
				static int test_time = 0;
				LED_POWER_ON_HIGH();
				set_heart_led(100, 100, 100, 0x1f);//°×É«			
				test_time++;
				COMM_LED_ON();
				wait_ms(2000);
			    COMM_LED_OFF();	
//				set_test_shooter();
				do_dribbler(1);
				wait_ms(5000);
			  do_dribbler(2);
				wait_ms(5000);
        do_dribbler(3);//ÉèÖÃ¿ØÖÆµµÎ»
				wait_ms(5000);
//        wait_ms(2000);
//				do_dribbler(0);
//				do_shoot(20, 0);//Æ½Éä
//		        wait_ms(2000);
//				do_chip(0, 20); //ÌôÉä
//                wait_ms(2000);
//				
//                if(test_time == 1)
//                {
//									
//								 do_acc_handle_move(0, 0,100);
//								 wait_ms(2000000);
//								 do_acc_handle_move(0, 0,0);
//                }
//				else if(test_time == 2)
//					{
//					 do_acc_handle_move(0, 0,-100);
//				     wait_ms(2000);
//				     do_acc_handle_move(0, 0,0);
//					 test_time = 0;
//					}
//                   	
				break;
				
			}
			case DEBUG_MODE: //PC»úÏÂ·¢²ÎÊýÄ£Ê½
			{
                LED_POWER_ON_HIGH();
                set_heart_led(100, 100, 0, 0x1f);//»ÆÉ«
                if(USB_Receive_ok == 1) //usb ½ÓÊÕµ½Êý¾Ý
                {
                   int i = 0;
				   int rtn = 0;
				  switch(USB_Receive_Buffer[0])
				  {
                   	 case SET_BLUETOOTH_NAME:   //ÉÏ´«1Byte
					 {
						for(i = 0; i < BLUETOOTH_MAX_NAME; i++)
						{
						  bluetooth_name[i]=USB_Receive_Buffer[i+1];
						}
                        
						 rtn = AT_Set_device_name_cmd(bluetooth_name);
						 if(rtn == 1) //ÉèÖÃOK
						 {
						   APP_Rx_Buffer[APP_Rx_ptr_in] = 1; //·µ»Ø1 Ö÷»úÊÕµ½1±íÊ¾ÉèÖÃ³É¹¦
						 }
						 else
						 {
						   APP_Rx_Buffer[APP_Rx_ptr_in] = 0; //·µ»Ø0±íÊ¾ Ö÷»úÉèÖÃ²ÎÊýÊ§°Ü
						 }
						  APP_Rx_ptr_in++;
						 
	                      // To avoid buffer overflow 
                         if(APP_Rx_ptr_in == APP_RX_DATA_SIZE)
                         {
                           APP_Rx_ptr_in = 0;
                         }  
	                        		
						break;
                   	 }
     /*	case CIRCLE_MODE: // zhuanquan 
			{
        static int test_time = 0;
				LED_POWER_ON_HIGH();
				set_heart_led(100, 0, 0, 0x1f);   //red     set_heart_led(100, 100, 100, 0x1f);//°×É«			set_heart_led(100, 100, 100, 0x1f)
				test_time++;
				COMM_LED_ON();
				wait_ms(2000);
			  COMM_LED_OFF();	
				         
				if(test_time == 1)
                {
									
								 do_acc_handle_move(0, 0,100);
								 wait_ms(5000);
									do_acc_handle_move(0, 0,-100);
									wait_ms(5000);
								do_acc_handle_move(0, 0,0);
								test_time = 0;
                }
				break;
}
					 case SET_BLUETOOTH_CODE: //ÉÏ´«1Byte
					 {
						
                         for(i = 0; i < BLUETOOTH_MAX_CODE; i++)
						 {
						   bluetooth_code[i]=USB_Receive_Buffer[i+1];
						 }
                        
						 rtn = AT_Set_device_name_code(bluetooth_code);
						 if(rtn == 1) //ÉèÖÃOK
						 {
						   APP_Rx_Buffer[APP_Rx_ptr_in] = 1; //·µ»Ø1 Ö÷»úÊÕµ½1±íÊ¾ÉèÖÃ³É¹¦
						 }
						 else
						 {
						   APP_Rx_Buffer[APP_Rx_ptr_in] = 0; //·µ»Ø0±íÊ¾ Ö÷»úÉèÖÃ²ÎÊýÊ§°Ü
						 }
						  APP_Rx_ptr_in++;
						 
	                      // To avoid buffer overflow 
                         if(APP_Rx_ptr_in == APP_RX_DATA_SIZE)
                         {
                           APP_Rx_ptr_in = 0;
                         }  
						

						break;
					 }
					 case SET_MOTOR_PID:
					 {
					 	break;
					 }
					 case GET_CONTROL_STATUS:  //»ñÈ¡¿ØÖÆÆ÷×´Ì¬ ÉÏ´«3Byte
					 {
                     
                         APP_Rx_Buffer[APP_Rx_ptr_in] = (g_robot.firmware_version >> 8)& 0xff; //
                         APP_Rx_ptr_in++;
                         APP_Rx_Buffer[APP_Rx_ptr_in] = g_robot.firmware_version & 0xff; //
                         APP_Rx_ptr_in++;
						 APP_Rx_Buffer[APP_Rx_ptr_in]  = error_flag.all;
                         APP_Rx_ptr_in++;
                         if(APP_Rx_ptr_in == APP_RX_DATA_SIZE)
                         {
                           APP_Rx_ptr_in = 0;
                         }        
						break;
					 }*/
					 case GET_CPU_ID:  //»ñÈ¡¿ØÖÆÆ÷CPUÎ¨Ò»ID ÉÏ´«12Byte
					 {  
                       
					     for(i = 0 ; i < 12; i ++)//µÍ×Ö½ÚÏÈ·¢ËÍ
					     {  
					         APP_Rx_Buffer[APP_Rx_ptr_in] = cpuid_data[i];
							 APP_Rx_ptr_in++;   
					     }
						
                         if(APP_Rx_ptr_in == APP_RX_DATA_SIZE)
                         {
                           APP_Rx_ptr_in = 0;
                         }        
						break;
					 }
					 case SET_UNLOCK_FALSH_RDPROTECTION:  //½â³ýflash¶Á±£»¤»úÖÆ ÉÏ´«1Byte
					 {

						  if(USB_Receive_Buffer[1] == 'F' ) //Í¨¹ýusbÏÂ·¢Êý¾Ý 0x05 0x46 Ôò¶Ôflash¶Á±£»¤½â³ý flash³ÌÐò²Á³ý ½÷É÷²Ù×÷
                          {
						      UNLOCK_FLASH_RDP();
						      APP_Rx_Buffer[APP_Rx_ptr_in] = 1; //·µ»Ø1 Ö÷»úÊÕµ½1±íÊ¾ÉèÖÃ³É¹¦
						      APP_Rx_ptr_in++;
                          }
						  if(APP_Rx_ptr_in == APP_RX_DATA_SIZE)
                          {
                              APP_Rx_ptr_in = 0;
                          }        
					 }
					 case GET_CAP_VOLATAGE:   //ÉÏ´«4Byte
					 {
					 	 short cap_v_int = 0; //Êµ¼ÊµçÑ¹À©´ó100±¶
						 short bat_v_int = 0;
						 cap_v_int = (short)(g_robot.cap_v_f * 100);
						 bat_v_int = (short)(g_robot.bat_v_f * 100);
					 	 APP_Rx_Buffer[APP_Rx_ptr_in] = (u8)(cap_v_int & 0xff);
						 APP_Rx_ptr_in++;
						 APP_Rx_Buffer[APP_Rx_ptr_in] = (u8)((cap_v_int >> 8) & 0xff);
						 APP_Rx_ptr_in++;
						 APP_Rx_Buffer[APP_Rx_ptr_in] = (u8)(bat_v_int & 0xff);
						 APP_Rx_ptr_in++;
						 APP_Rx_Buffer[APP_Rx_ptr_in] = (u8)((bat_v_int >> 8)& 0xff);
						 APP_Rx_ptr_in++;
						  if(APP_Rx_ptr_in == APP_RX_DATA_SIZE)
                          {
                           APP_Rx_ptr_in = 0;
                          }        
						break;
					 }
					 case SET_ROBOT_DO_SHOOT:
					 {
					 	  u8 strength = 0;
                          if(USB_Receive_Buffer[1] == 'S' ) // shootÍ¨¹ýusbÏÂ·¢Êý¾Ý 0x08 0x53 ÔòÖ´ÐÐÆ½Éä¶¯×÷ µÚ3¸öbyteÆ½ÉäÁ¦Á¿
                          {   
                              strength = USB_Receive_Buffer[2];
						      do_shoot(strength, 0);//Æ½Éä
						      APP_Rx_Buffer[APP_Rx_ptr_in] = 1; //·µ»Ø1 Ö÷»úÊÕµ½1±íÊ¾ÉèÖÃ³É¹¦
						      APP_Rx_ptr_in++;
                          }
						  else if(USB_Receive_Buffer[1] == 'C')//chip 0x08 0x43 Ö´ÐÐÌôÉä
						  	   {
						  	     strength =  USB_Receive_Buffer[2];
								 do_chip(0, strength);//ÌôÉä
						  	     APP_Rx_Buffer[APP_Rx_ptr_in] = 1; //·µ»Ø1 Ö÷»úÊÕµ½1±íÊ¾ÉèÖÃ³É¹¦
						         APP_Rx_ptr_in++;
						  	   }
						  if(APP_Rx_ptr_in == APP_RX_DATA_SIZE)
                          {
                           APP_Rx_ptr_in = 0;
                          }   
						 break;
					 }

					  default:
					  	break;
                   }



				   USB_Receive_ok = 0;
                }
				
				break;

			}
		  //case PID_TUNE_MODE:
			case PID_TUNE_MODE:
			{
				if(mode_count == 1)//
				{
				   mode_count = 0;
				}
				else
				{  
				    //LED_POWER_ON_HIGH();
				    //set_heart_led(100, 0, 0, 0x1f);//ºìÉ«
				    LED_POWER_ON_LOW();//Ï¨ÃðÉ«²ÊµÆ
				}
				/* process message from comm device( rs232, wireless, etc. ) */
				do_comm();

			    /*Í¨Ñ¶³¬Ê±ÉèÖÃÎª500ms»òÕßý£Ê²Ã´¶¼²»×ö ÉÏÎ»»ú8msÏÂ·¢Ò»¸öÊý¾Ý°ü (¹²2°üÒ»°ü(25Byte)¸øÒ»¶Ó³µ£¬ÁíÍâÒ»°ü¸øÁíÒ»¶Ó³µ)*/
				if(check_timer(rf_comm_tim)) 
				{
					/* ·ÀÖ¹Í¨Ñ¶ÖÐ¶Ï£¬ÖÃÎ»ÉèÖÃÍ¨Ñ¶Îª½ÓÊÕÄ£¿é±êÖ¾Î» */
					g_do_set_receive_mode_flag = 1; 
					
					do_dribbler(0);
					do_move(0,0,0);
					do_shoot(0,0);
					do_chip(0,0);

					start_nRF24L01_RX();	
					rf_comm_tim = get_one_timer(COMM_TIMEOUT_TIME);
					identify_cpuid_tim = get_one_timer(IDENTIFY_CPUID_TIMEOUT_TIME);
				}
				if(check_timer(identify_cpuid_tim))//Éí·ÝÊ¶±ð³¬Ê±»òÕßÉí·ÝÊ¶±ð²»Í¨ÔòÍ£Ö¹»úÆ÷ÈË
				{
				    /* ·ÀÖ¹Í¨Ñ¶ÖÐ¶Ï£¬ÖÃÎ»ÉèÖÃÍ¨Ñ¶Îª½ÓÊÕÄ£¿é±êÖ¾Î» */
					g_do_set_receive_mode_flag = 1; 
					
					do_dribbler(0);
					do_move(0,0,0);
					do_shoot(0,0);
					do_chip(0,0);

					start_nRF24L01_RX();
					identify_success = 0;
					identify_cpuid_tim = get_one_timer(IDENTIFY_CPUID_TIMEOUT_TIME);
				}
				
				if(g_do_set_receive_mode_flag)	//·¢ËÍÊý¾Ý°üºóÖÃ1µÈ´ýÊý¾Ý·¢ËÍ³öÈ¥ºó½«Ä£Ê½ÐÞ¸ÄÎª½ÓÊÕÄ£Ê½				
				{
					/* ½«Í¨Ñ¶ÉèÖÃÎª½ÓÊÕÄ£Ê½£¬²¢ÖÃÎ»¿É½ÓÊÜ±êÖ¾Î» */
					if(g_set_receive_mode_flag >= 3)
					{				
						start_nRF24L01_RX();
					
						g_set_receive_mode_flag = 0;
						g_do_set_receive_mode_flag = 0; 	//ÖÃÎ»¿É½ÓÊÕ±êÖ¾Î»
					}
				}
				break;
			}
					  //case PID_TUNE_MODE:
			case SHOOT_POWER_CURVE_TUNE_MODE:
			{
				if(mode_count == 1)//
				{
				   mode_count = 0;
				}
				else
				{  
				    //LED_POWER_ON_HIGH();
				    //set_heart_led(100, 0, 0, 0x1f);//ºìÉ«
				    LED_POWER_ON_LOW();//Ï¨ÃðÉ«²ÊµÆ
				}
				/* process message from comm device( rs232, wireless, etc. ) */
				do_comm();

			    /*Í¨Ñ¶³¬Ê±ÉèÖÃÎª500ms»òÕßý£Ê²Ã´¶¼²»×ö ÉÏÎ»»ú8msÏÂ·¢Ò»¸öÊý¾Ý°ü (¹²2°üÒ»°ü(25Byte)¸øÒ»¶Ó³µ£¬ÁíÍâÒ»°ü¸øÁíÒ»¶Ó³µ)*/
				if(check_timer(rf_comm_tim)) 
				{
					/* ·ÀÖ¹Í¨Ñ¶ÖÐ¶Ï£¬ÖÃÎ»ÉèÖÃÍ¨Ñ¶Îª½ÓÊÕÄ£¿é±êÖ¾Î» */
					g_do_set_receive_mode_flag = 1; 
					
					do_dribbler(0);
					do_move(0,0,0);
					do_shoot(0,0);
					do_chip(0,0);

					start_nRF24L01_RX();	
					rf_comm_tim = get_one_timer(COMM_TIMEOUT_TIME);
					identify_cpuid_tim = get_one_timer(IDENTIFY_CPUID_TIMEOUT_TIME);
				}
				
				if(g_do_set_receive_mode_flag)	//·¢ËÍÊý¾Ý°üºóÖÃ1µÈ´ýÊý¾Ý·¢ËÍ³öÈ¥ºó½«Ä£Ê½ÐÞ¸ÄÎª½ÓÊÕÄ£Ê½				
				{
					/* ½«Í¨Ñ¶ÉèÖÃÎª½ÓÊÕÄ£Ê½£¬²¢ÖÃÎ»¿É½ÓÊÜ±êÖ¾Î» */
					if(g_set_receive_mode_flag >= 3)
					{				
						start_nRF24L01_RX();
					
						g_set_receive_mode_flag = 0;
						g_do_set_receive_mode_flag = 0; 	//ÖÃÎ»¿É½ÓÊÕ±êÖ¾Î»
					}
				}
				break;
			}
				
			default: break;
		}

		/* ¶ÔÉäÃÅÍê³ÉÖ®ºóµÄÑÓÊ±½øÐÐ¼ÆÊ± */
#ifdef ENABLE_SHOOTER
        if(g_robot.mode != SELFTEST_MODE)
        {
           if(check_timer(shoot_interval_timer)) 
		   {
			 update_shooter();
			 shoot_interval_timer = get_one_timer(1);
		   }
        }
		
#endif




	}	
}


/*******************************************************************************
* Function Name   : SysTick_Handler
* Description	  : ÏµÍ³Ê±ÖÓÖÐ¶Ïº¯Êý
*					
* Input		      : None
* Output		  : None
* Return		  : None
*******************************************************************************/
void SysTick_Handler(void)
{
	/* not need disable int, because the same int of ARM cortex-M3 cannot
	   be nested */
    static int watch_dog_tick = 0;
	watch_dog_tick++;
	if(watch_dog_tick >=1500)
	{
	  feed_iwdog();
	  watch_dog_tick = 0;
	}
	update_sys_timer();
	do_timer();

}



