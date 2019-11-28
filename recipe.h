/*Command Op-codes*/
#define MOV (0x20)
#define WAIT (0x40)
#define LOOP (0x80)
#define END_LOOP (0xA0)
#define RECIPE_END (0x00)
#define SKIP_NEXT (0xE0) //Skips the next MOV Command. . .


/*MASKS*/
#define COMMAND_MASK 0xE0
#define VALUE_MASK 0x1F


/*Servo Positions*/
#define SERVO_POSITION_0 500
#define SERVO_POSITION_1 730
#define SERVO_POSITION_2 1100	
#define SERVO_POSITION_3 1380
#define SERVO_POSITION_4 1700
#define SERVO_POSITION_5 2000

/*States*/

enum s_servo_1{
   s1_pause,
	 s1_continue,
	 s1_positioning,
	 s1_wait,
	 s1_looping
};
enum s_servo_2{
   s2_pause,
	 s2_continue,
	 s2_positioning,
		s2_wait,
	 s2_looping
	
};


/*Function Prototypes*/

void servo1_positions(int s1);
void servo2_positions(int s2);
void process_servo1(char * str1);
void process_servo2(char * str2);
void reset_buffer(void);
void get_UI_cmd(void);
