/**************************************************
* CMPEN 473, Spring 2018, Penn State University
* 
* Homework 7 
* Revision V1.0
* On 3/16/2019
* By Boris Feng
* 
***************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include "import_registers.h"
#include "gpio.h"
#include "cm.h"
#include "pwm.h"
#include "spi.h"
#include "io_peripherals.h"
#include "enable_pwm_clock.h"
#include "LSM9DS1.h"
#include "wait_key.h"
#include "transact_SPI.h"
#include "raspicam_wrapper.h"
#include "wait_period.h"
#include "time_difference.h"
#include "pixel_format_RGB.h"

/* setting the RPi3 hardware PWM range */
#define PWM_RANGE 32

#define APB_CLOCK 250000000

#define DEN_PIN   18
#define CS_M_PIN  19
#define CS_AG_PIN 20

#define ROUND_DIVISION(x,y) (((x) + (y)/2)/(y))

struct done_flag_t
{
  pthread_mutex_t                 lock;                 /* used to lock the contents to prevent race conditions */
  bool                            done;                 /* set to true to indicate that it is time to shut down */
};


struct image_data_t
{
  pthread_mutex_t                 lock;                 /* used to lock the contents to prevent race conditions */
  bool                            data_ready;           /* set to true when data is valid */
  unsigned char *                 data;                 /* cast to struct pixel_format_RGB to access the pixel data, index via RGB_data[x+y*image_width], total array length is image_height*image_width */
  unsigned int                    image_height;         /* the height of the RGB data */
  unsigned int                    image_width;          /* the width of the RGB data */
};

struct camera_thread_parameter_t
{
  long                            last_execution_time;  /* the execution time consumed in the last period */
  struct image_data_t *           image_data;           /* output from ThreadCamera */
  struct done_flag_t *            done;                 /* input to ThreadCamera */
};

struct image_analysis_thread_parameter_t
{
  long                            last_execution_time;  /* the execution time consumed in the last period */
  struct image_data_t *           image_data;           /* input to ThreadImageAnalysis */
  struct dimming_command_t *      dimming_command;      /* output from ThreadImageAnalysis */
  struct done_flag_t *            done;                 /* input to ThreadImageAnalysis */
};

struct thread_parameter
{
  volatile struct gpio_register * gpio;
  volatile struct spi_register * spi;
};

int array[6] = {0, 0, 0, 0, 0, 0}; //w, s, b, a, d, i & j
int mode = 1;
int IMU_On = 0;
int count = 0;
int cam = 0;
int avg_pixel = 0;
int left_avg_pixel = 0;
int right_avg_pixel = 0;
int mid_avg_pixel = 0;
int left, right, middle, far_left, far_right;

int IMU_data[6000][9];
float temp[6000][9];
float min[9];
float max[9];
float avg_accel[3] = {0, 0, 0}; //x,y,z
float avg_speed;
float end = 0;

char map_display[20][20];

int picture[96][128];

int get_pressed_key(void) {
  struct termios  original_attributes;
  struct termios  modified_attributes;
  int             ch;

  tcgetattr( STDIN_FILENO, &original_attributes );
  modified_attributes = original_attributes;
  modified_attributes.c_lflag &= ~(ICANON | ECHO);
  modified_attributes.c_cc[VMIN] = 1;
  modified_attributes.c_cc[VTIME] = 0;
  tcsetattr( STDIN_FILENO, TCSANOW, &modified_attributes );

  ch = getchar();

  tcsetattr( STDIN_FILENO, TCSANOW, &original_attributes );

  return ch;
}

int searchArray(int elem, int check){
	if (array[elem] == check)
		return 1;
	else
		return 0;
}

int getSpeed(){
	return array[5];
}

void getMin() {
	for (int i = 0; i < 9; i++) {
		float small = temp[0][i];
		for (int j = 1; j < 6000; j++) {
			if (temp[j][i] < small)
				small = temp[j][i];
		}
		min[i] = small;		
	}	
}

void getMax() {
	for (int i = 0; i < 9; i++) {
		float large = temp[0][i];
		for (int j = 1; j < 6000; j++) {
			if (temp[j][i] > large)
				large = temp[j][i];
		}
		max[i] = large;		
	}	
}

void printArray() {
	for (int i = 0; i < 9; i++) {
		printf("%.2f ", min[i]);
	}
	printf("\n");
	for (int i = 0; i < 9; i++) {
		printf("%.2f ", max[i]);
	}
	printf("\n");
}

void clearArray() {
	for (int i = 0; i < 6000; i++) {
		for (int j = 0; j < 9; j++) {
			IMU_data[i][j] = 0;
			temp[i][j] = 0;
		}
	}
}

void normalize() {
	for (int i = 0; i < 6000; i++) {
		for (int j = 0; j < 9; j++) {
			IMU_data[i][j] = (int)(round((temp[i][j] - min[j]) * 9) / (max[j] - min[j]));
		}
	}
}
 
void genMap() {
	for (int i = 0; i < 20; i++) {
		for (int j = 0; j < 20; j++) {
			map_display[i][j] = '0';
		}
	}
	
	for (int i = 1; i < 19; i++) {
		for (int j = 1; j < 19; j++) {
			map_display[i][j] = ' ';
		}
	}		
}

void placeMap() {
	int x = 9;
	int y = 9;
	map_display[9][9] = '0'; //start
	int iteration = 0;
	float mag_x, mag_y; //N(0,0) W(+,0) S(+,-) E(0,-)
	int speed = 0;
	float test = 0;
	for (int i = 0; i < 300; i++) {
			for (int j = 0; j < 17; j++) {
					mag_x = mag_x + temp[j + (iteration * 16)][6];
					mag_y = mag_y + temp[j + (iteration * 16)][7];
					speed = speed + IMU_data[j+ (iteration * 16)][0];
					test = test + temp[j+ (iteration * 16)][0];
			}
			iteration++;
			mag_x = mag_x / 16;
			mag_y = mag_y /16;
			speed = round(speed / 16);
			test = test / 16;
			if (test == 0)
				return;
			if (mag_x < 0.09 && mag_y > -0.09) {
					//North
					x--;
					if (x < 0) {
						return;
					}
					map_display[x][y] = (char)speed;
			}
			if (mag_x > 0.09 && mag_y > -0.07) {
					//West
					y--;
					if (y < 0) {
						return;
					}
					map_display[x][y] = (char)speed;
			}
		    if (mag_x > 0.09 && mag_y < -0.09) {
					//South
					x++;
					if (x > 20) {
						return;
					}
					map_display[x][y] = (char)speed;
			}
			if (mag_x < 0.05 && mag_y < -0.09) {
					//East suppose to be north
					y++;
					if (y > 20) {
						return;
					}
					map_display[x][y] = (char)speed;
			}
	}
}

void *Accelerometer(void *arg) {
	
	struct thread_parameter *parameter = (struct thread_parameter *)arg;
	
	struct timespec           sleep_start;
    struct timespec           sleep_end;
    struct timespec           previous_reading_start;
    struct timespec           reading_start;
    struct timespec           reading_end;
    struct LSM9DS1_reading_t  linear_reading;
    struct LSM9DS1_reading_t  mag_reading;
    struct LSM9DS1_reading_t  gyro_reading;
    int                       sleep_duration;
    int                       pressed_key;

    clock_gettime( CLOCK_REALTIME, &sleep_start );
    clock_gettime( CLOCK_REALTIME, &reading_start );
    
	while(1) {
		while(IMU_On == 1 && searchArray(1,1) == 1 && count != 6000) {
			usleep(17000);
			clock_gettime( CLOCK_REALTIME, &sleep_end );
			
			previous_reading_start = reading_start;
			clock_gettime( CLOCK_REALTIME, &reading_start );
			read_accelerometer( parameter->spi, parameter->gpio, CS_AG_PIN, &linear_reading );
			read_magnetometer( parameter->spi, parameter->gpio, CS_M_PIN, &mag_reading );
			read_gyroscope( parameter->spi, parameter->gpio, CS_AG_PIN, &gyro_reading );
			clock_gettime( CLOCK_REALTIME, &reading_end );
			
			temp[count][0] = linear_reading.X;
			temp[count][1] = linear_reading.Y;
			temp[count][2] = linear_reading.Z;
			
			temp[count][3] = gyro_reading.X;
			temp[count][4] = gyro_reading.Y;
			temp[count][5] = gyro_reading.Z;
			
			temp[count][6] = mag_reading.X;
			temp[count][7] = mag_reading.Y;
			temp[count][8] = mag_reading.Z;
			
			end = end + 0.02;
			
			count++;
			
			if (count == 6000)
					printf("Stop Recording\n");
			
			/*
			printf( "Accel X=%.2f m/s^2\tY=%.2f m/s^2\tZ=%.2f m/s^2\n",
				linear_reading.X,
				linear_reading.Y,
				linear_reading.Z );
			printf( "Mag   X=%.2f gauss\tY=%.2f gauss\tZ=%.2f gauss\n",
				mag_reading.X,
				mag_reading.Y,
				mag_reading.Z );
			printf( "Gyro  X=%.2f dps  \tY=%.2f dps  \tZ=%.2f dps\n",
				gyro_reading.X,
				gyro_reading.Y,
				gyro_reading.Z );
			printf( "waitkey: %dus, reading: %dus, loop time: %dus\n",
				(sleep_end.tv_sec-sleep_start.tv_sec) * 1000000 +
				(sleep_end.tv_nsec-sleep_start.tv_nsec) / 1000,
				(reading_end.tv_sec-reading_start.tv_sec) * 1000000 +
				(reading_end.tv_nsec-reading_start.tv_nsec) / 1000,
				(sleep_end.tv_sec-previous_reading_start.tv_sec) * 1000000 +
				(sleep_end.tv_nsec-previous_reading_start.tv_nsec) / 1000);
			*/
			clock_gettime( CLOCK_REALTIME, &sleep_start );
#if 0
			/*
			* The accelerometer reading function is too greedy.
			* It will take as much time as it needs to get the next sample.
			* Why wait in a busy-wait loop when you can just wait in wait_key, a more productive and compassionate function to sleep in?
			* We are *not* going to hit 100Hz since the next sample is simply not ready that fast.
			*/
			sleep_duration = 16 - ((reading_end.tv_sec-reading_start.tv_sec) * 1000 + (reading_end.tv_nsec-reading_start.tv_nsec) / 1000000);
#else
			sleep_duration = 16;
#endif
		}
	}
}

void *ThreadCamera(void* arg) {
	struct raspicam_wrapper_handle *  camera_handle;  /* Camera handle */
	struct timespec                   timer_state;    /* used to keep the thread running at the correct rate */
	struct camera_thread_parameter_t *parameter = (struct camera_thread_parameter_t *)arg;
	struct timespec                   execution_start;/* used to determine the execution time of the loop */
	struct timespec                   execution_end;  /* used to determine the execution time of the loop */
	
    /* Open camera */
    printf("Opening Camera\n");
    while(1){
		while (cam == 1) {
			camera_handle = raspicam_wrapper_create();
			if (camera_handle != NULL) {
				if (raspicam_wrapper_open(camera_handle)) {  

					// allocate memory
					pthread_mutex_lock( &(parameter->image_data->lock) );
					parameter->image_data->data = (unsigned char *)malloc( raspicam_wrapper_getImageTypeSize( camera_handle, RASPICAM_WRAPPER_FORMAT_RGB ) );
					pthread_mutex_unlock( &(parameter->image_data->lock) );

					// set the start time of the time period
					wait_period_initialize( &timer_state );
					if (parameter->image_data->data != NULL)
					{
						pthread_mutex_lock( &(parameter->done->lock) );
						while (!(parameter->done->done))
						{
							pthread_mutex_unlock( &(parameter->done->lock) );

							// start measuring the execution time
							clock_gettime( CLOCK_REALTIME, &execution_start );

							// capture the image
							raspicam_wrapper_grab( camera_handle );
							printf("Take photo\n");

							// extract the image in rgb format
							pthread_mutex_lock( &(parameter->image_data->lock) );
							raspicam_wrapper_retrieve( camera_handle, parameter->image_data->data, RASPICAM_WRAPPER_FORMAT_IGNORE );
							parameter->image_data->image_height  = raspicam_wrapper_getHeight( camera_handle );
							parameter->image_data->image_width   = raspicam_wrapper_getWidth( camera_handle );
							parameter->image_data->data_ready    = true;
							pthread_mutex_unlock( &(parameter->image_data->lock) );

							// record the execution time
							clock_gettime( CLOCK_REALTIME, &execution_end );
							parameter->last_execution_time = time_difference_us( &execution_start, &execution_end );

							// wait for the next 100ms period
							wait_period( &timer_state, 100*1000 );

							pthread_mutex_lock( &(parameter->done->lock) );
						}
						pthread_mutex_unlock( &(parameter->done->lock) );
					}
					else
					{
					printf( "unable to allocate image data\n" );
					}
				}
				else
				{
				printf( "Error opening camera\n" );
				}
			}
			else
			{
			printf( "Unable to allocate camera handle\n" );
			}
		if (cam == 0)
			cam = 1;
		}
	}
}

void *ThreadImageAnalysis( void * arg )
{
  struct timespec                           timer_state;      /* used to keep the thread running at the correct rate */
  struct image_analysis_thread_parameter_t *parameter = (struct image_analysis_thread_parameter_t *)arg;
  unsigned long                             pixel_sum;        /* used to determine if the image is greater than half intensity */
  unsigned long                             pixel_index;      /* used to index through the image data */
  unsigned long                             pixel_count;      /* the total number of pixels */
  struct pixel_format_RGB *                 structured_data;  /* used to view the data in an organized manner */
  bool                                      threshold_reached;/* set to true when the pixel sum exceeds the target threshold */
  struct timespec                           execution_start;  /* used to determine the execution time of the loop */
  struct timespec                           execution_end;    /* used to determine the execution time of the loop */

  // set the start time of the time period
  wait_period_initialize( &timer_state );

  pthread_mutex_lock( &(parameter->done->lock) );
  while (!(parameter->done->done))
  {
    pthread_mutex_unlock( &(parameter->done->lock) );

    // start measuring the execution time
    clock_gettime( CLOCK_REALTIME, &execution_start );

    // process data, if available
    pthread_mutex_lock( &(parameter->image_data->lock) );
    if (parameter->image_data->data_ready)
    {
      structured_data = (struct pixel_format_RGB *)parameter->image_data->data;
      pixel_sum       = 0;  /* 1920*1080*3*255 < 2^32 */
      pixel_count     = parameter->image_data->image_height * parameter->image_data->image_width;
      for (pixel_index = 0; pixel_index < pixel_count; pixel_index++)
      {
        pixel_sum += structured_data->R + structured_data->G + structured_data->B;
      }

	  struct RGB_pixel {
		unsigned char R;
		unsigned char G;
		unsigned char B;
	  };
	  struct RGB_pixel* pixel;
	  unsigned int      pixel_count;
	  unsigned int      pixel_index;
	  unsigned char     pixel_value;

	  pixel = (struct RGB_pixel *)structured_data;
	  pixel_count = parameter->image_data->image_height * parameter->image_data->image_width;
	  for (pixel_index = 0; pixel_index < pixel_count; pixel_index++) {
		pixel_value = (((unsigned int)(pixel[pixel_index].R)) +
		((unsigned int)(pixel[pixel_index].G)) +
		((unsigned int)(pixel[pixel_index].B))) / 3; // do not worry about rounding
		pixel[pixel_index].R = pixel_value;
		pixel[pixel_index].G = pixel_value;
		pixel[pixel_index].B = pixel_value;
	  }
	  FILE * outFile = fopen( "raspicam_image_scale.ppm", "wb" );
	  unsigned int scaled_height;
	  unsigned int scaled_width;
	  unsigned int height_index;
	  unsigned int width_index;
	  unsigned int horizontal_reduction = 10;
	  unsigned int vertical_reduction = 10;
	  struct RGB_pixel* scaled_data;
								
	  scaled_data = pixel;
	  scaled_height = parameter->image_data->image_height/vertical_reduction;
	  scaled_width  = parameter->image_data->image_width/horizontal_reduction;
								
	  for (height_index = 0; height_index < scaled_height; height_index++) {
		for (width_index = 0; width_index < scaled_width; width_index++) {
			scaled_data[height_index*scaled_width + width_index] = pixel[height_index*scaled_width*vertical_reduction*horizontal_reduction + width_index*horizontal_reduction];
			//picture[height_index][width_index] = scaled_data[height_index*scaled_width + width_index].R;
			avg_pixel = avg_pixel + scaled_data[height_index*scaled_width + width_index].R;
		}
	  }
	  avg_pixel = avg_pixel/12288;
	  printf("avg: %d\n", avg_pixel);
								
	  for(int i = 1; i < 81; i++) {
		for(int j = 0; j < 16; j++) {
			left_avg_pixel = left_avg_pixel + scaled_data[(512*i)+j].R;
		}		
	  }
	  left_avg_pixel = left_avg_pixel/1280;
	  printf("left avg: %d\n", left_avg_pixel);
								
	  for(int i = 1; i < 81; i++) {
		for(int j = 0; j < 16; j++) {
			right_avg_pixel = right_avg_pixel + scaled_data[(1280*i)+j].R;
		}		
	  }
	  right_avg_pixel = right_avg_pixel/1280;
	  printf("right avg: %d\n", right_avg_pixel);
								
	  for(int i = 1; i < 17; i++) {
		for(int j = 0; j < 1; j++) {
			mid_avg_pixel = mid_avg_pixel + scaled_data[(1024*i)+j].R;
		}		
	  }
	  mid_avg_pixel= mid_avg_pixel/32;
	  printf("mid avg: %d\n", mid_avg_pixel);
								
	  left = (scaled_data[960].R + scaled_data[965].R + scaled_data[964].R)/3;
	  right = (scaled_data[1170].R + scaled_data[1169].R + scaled_data[1168].R)/3;
	  middle = scaled_data[64].R;
	  far_left = scaled_data[5].R;
	  far_right = scaled_data[123].R;
	  printf("middle: %d\n", middle);
	   	
      threshold_reached = (pixel_sum > (pixel_count*3*255/2));
    }
    else
    {
      threshold_reached = false;  // no data ready yet, wait for the next period
    }
    pthread_mutex_unlock( &(parameter->image_data->lock) );
   	
    // record the execution time
    clock_gettime( CLOCK_REALTIME, &execution_end );
    parameter->last_execution_time = time_difference_us( &execution_start, &execution_end );

    // wait for the next 100ms period
    wait_period( &timer_state, 100*1000 );

    pthread_mutex_lock( &(parameter->done->lock) );
    }
  pthread_mutex_unlock( &(parameter->done->lock) );

  return 0;
}

void *Actions() {
	int abs = 0;
	while (1) {
      switch (get_pressed_key())
      {
        case 'w':
          if (array[0] == 0)
          {
				printf("Move Foward\n");
				array[0] = 1;
				array[1] = 1;				
          }
          break;

        case 's':
          //if (array[1] == 1)
          //{
				printf("STOP\n");
				if (IMU_On == 1)
					printf("Stop Recording\n");
				array[1] = 0;
				array[0] = 0;
				array[2] = 0;
				IMU_On = 0;				 
          //}
          
          break;

		case 'b':
          if (array[2] == 0)
          {
				printf("Reverse\n");
				array[2] = 1;
				array[1] = 1;
          }
          break;

		case 'a':
          if (array[3] == 0)
          {
				printf("Turn Left\n");
				array[3] = 1;
          }
          break;
          
		case 'd':
          if (array[4] == 0)
          {
				printf("Turn Right\n");
				array[4] = 1;
          }
          break;        
      
		case 'i':
          if (array[5] != 15)
          {
				array[5]++;
				printf("current speed %d\n", array[5]+1);
				
				if (IMU_On == 0) {
					printf("Start Recording\n");
					count = 0;
					clearArray();
					genMap();
					avg_accel[0] = 0;				
					end = 0;				
					IMU_On = 1;
				}
          }
          break;
                  
		case 'j':
          if (array[5] > 0)
          {
				array[5]--;
				printf("current speed %d\n", array[5]+1);
          }
          break;
          
        case 'm':
		  switch (get_pressed_key()) {
			  case '1':
				mode = 1;
				cam = 0;
				printf("Mode 1\n");
				break;
			  case '2':
				mode = 2;
				cam = 1;
				printf("Mode 2\n");
				break;
		  }
		  break;
		  
		case 'p':
          if (array[1] == 0)
          {
				getMin();
				getMax();
				
				for (int i = 0; i < 6000; i++) {
					for (int j = 0; j < 9; j++) {
						IMU_data[i][j] = (int)(round((temp[i][j] - min[j]) * 9) / (max[j] - min[j]));
					}
				}
				
				printf("AC GY MG\n");
				for (int i = 0; i < 6000; i++) {
					//printf("Row %d: ", i);
					for (int j = 0; j < 9; j++) {
						printf("%d", IMU_data[i][j]);
					}
					printf("\n");
				}
				
				//printArray();
          }
          else
			printf("Please stop the car first\n");
          break;    
		  
		case 'n':
			if (array[1] == 0)
			{
				for (int i = 0; i < 6000; i++) {
					for (int j = 0; j < 3; j++) {
							abs = temp[i][j];
							if (abs < 0)
								abs = abs * -1;
							avg_accel[j] = avg_accel[j] + abs;
						}
					}			
				
				//avg_accel[0] = avg_accel[0] / 6000;
				//avg_accel[1] = avg_accel[1] / 6000;
				//avg_accel[2] = avg_accel[2] / 6000;
				//end = end * pow(10,-6);
				printf("Time: %.3f\n", end);
				//printf("Avg accel in X: %.2f\n", avg_accel[0]);
				//printf("Avg accel in Y: %.2f\n", avg_accel[1]);
				//printf("Avg accel in Z: %.2f\n", avg_accel[2]);
				avg_speed = avg_accel[0] / end;
				printf("Avg Speed: %.2f\n", avg_speed);
				//printf("Avg Speed in Y: %.2f\n", avg_accel[1] / end);
				//printf("Avg Speed in Z: %.2f\n", avg_accel[2] / end);
			
				printf("Distance: %.2f\n", avg_speed * end);
				//printf("Distance in Y: %.2f\n", 0.5 * avg_accel[1] * pow(end,2));
				//printf("Distance in Z: %.2f\n", 0.5 * avg_accel[2] * pow(end,2));
			}
			else
				printf("Please stop the car first\n");
          break;
          
          case 'k':
			if (array[1] == 0)
			{
				placeMap();
				for (int i = 0; i < 20; i++) {
					for (int j = 0; j < 20; j++) {
							printf("%c ", map_display[i][j]);
						}
					printf("\n");
					}			
				
			}
			else
				printf("Please stop the car first\n");
          break;	
		}
	}
}    

int main( void )
{
  volatile struct io_peripherals *io;
  pthread_t	action_handle;
  pthread_t	accel_handle;
  pthread_t thread_camera_handle;
  pthread_t thread_image_analysis_handle;
  
  struct thread_parameter accel_parameter;
  struct done_flag_t done = {PTHREAD_MUTEX_INITIALIZER, false};
  struct image_data_t image_data = {PTHREAD_MUTEX_INITIALIZER, false, NULL, 0U, 0U};
  struct camera_thread_parameter_t thread_camera_parameter;
  struct image_analysis_thread_parameter_t thread_image_analysis_parameter;

  
  io = import_registers();
  if (io != NULL)
  {
    /* print where the I/O memory was actually mapped to */
    printf( "mem at 0x%8.8X\n", (unsigned int)io );

    enable_pwm_clock( io );

    io->gpio.GPFSEL1.field.FSEL2 = GPFSEL_ALTERNATE_FUNCTION0;
    io->gpio.GPFSEL1.field.FSEL3 = GPFSEL_ALTERNATE_FUNCTION0;
    io->gpio.GPFSEL0.field.FSEL5 = GPFSEL_OUTPUT;
    io->gpio.GPFSEL0.field.FSEL6 = GPFSEL_OUTPUT;
    io->gpio.GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT;
    io->gpio.GPFSEL2.field.FSEL3 = GPFSEL_OUTPUT;
    
    io->gpio.GPFSEL0.field.FSEL9 = GPFSEL_ALTERNATE_FUNCTION0;
    io->gpio.GPFSEL1.field.FSEL0 = GPFSEL_ALTERNATE_FUNCTION0;
    io->gpio.GPFSEL1.field.FSEL1 = GPFSEL_ALTERNATE_FUNCTION0;
    io->gpio.GPFSEL1.field.FSEL8 = GPFSEL_OUTPUT;
    io->gpio.GPFSEL1.field.FSEL9 = GPFSEL_OUTPUT;
    io->gpio.GPFSEL2.field.FSEL0 = GPFSEL_OUTPUT;

	/* set initial output state */
    GPIO_SET(&(io->gpio), DEN_PIN);
    GPIO_SET(&(io->gpio), CS_M_PIN);
    GPIO_SET(&(io->gpio), CS_AG_PIN);
    usleep( 100000 );

    /* configure the PWM channels */
    io->pwm.RNG1 = PWM_RANGE;     /* the range value, 32 level steps */
    io->pwm.RNG2 = PWM_RANGE;     /* the range value, 32 level steps */
    io->pwm.CTL.field.MODE1 = 0;  /* PWM mode */
    io->pwm.CTL.field.MODE2 = 0;  /* PWM mode */
    io->pwm.CTL.field.RPTL1 = 1;  /* not using FIFO, but repeat the last byte anyway */
    io->pwm.CTL.field.RPTL2 = 1;  /* not using FIFO, but repeat the last byte anyway */
    io->pwm.CTL.field.SBIT1 = 0;  /* idle low */
    io->pwm.CTL.field.SBIT2 = 0;  /* idle low */
    io->pwm.CTL.field.POLA1 = 0;  /* non-inverted polarity */
    io->pwm.CTL.field.POLA2 = 0;  /* non-inverted polarity */
    io->pwm.CTL.field.USEF1 = 0;  /* do not use FIFO */
    io->pwm.CTL.field.USEF2 = 0;  /* do not use FIFO */
    io->pwm.CTL.field.MSEN1 = 1;  /* use M/S algorithm */
    io->pwm.CTL.field.MSEN2 = 1;  /* use M/S algorithm */
    io->pwm.CTL.field.CLRF1 = 1;  /* clear the FIFO, even though it is not used */
    io->pwm.CTL.field.PWEN1 = 1;  /* enable the PWM channel */
    io->pwm.CTL.field.PWEN2 = 1;  /* enable the PWM channel */

    /* set up the SPI parameters */
    io->spi.CLK.field.CDIV = ((ROUND_DIVISION(250000000,4000000))>>1)<<1; /* this number must be even, so shift the LSb into oblivion */
    io->spi.CS.field.CS       = 0;
    io->spi.CS.field.CPHA     = 1;  /* clock needs to idle high and clock in data on the rising edge */
    io->spi.CS.field.CPOL     = 1;
    io->spi.CS.field.CLEAR    = 0;
    io->spi.CS.field.CSPOL    = 0;
    io->spi.CS.field.TA       = 0;
    io->spi.CS.field.DMAEN    = 0;
    io->spi.CS.field.INTD     = 0;
    io->spi.CS.field.INTR     = 0;
    io->spi.CS.field.ADCS     = 0;
    io->spi.CS.field.REN      = 0;
    io->spi.CS.field.LEN      = 0;
	io->spi.CS.field.CSPOL0   = 0;
    io->spi.CS.field.CSPOL1   = 0;
    io->spi.CS.field.CSPOL2   = 0;
    io->spi.CS.field.DMA_LEN  = 0;
    io->spi.CS.field.LEN_LONG = 0;
    
    thread_camera_parameter.last_execution_time         = 0;
    thread_camera_parameter.image_data                  = &image_data;
    thread_camera_parameter.done                        = &done;
    thread_image_analysis_parameter.last_execution_time = 0;
    thread_image_analysis_parameter.image_data          = &image_data;
    thread_image_analysis_parameter.done                = &done;
	
	initialize_accelerometer_and_gyroscope( &(io->spi), &(io->gpio), CS_AG_PIN );
    initialize_magnetometer( &(io->spi), &(io->gpio), CS_M_PIN );
	
	accel_parameter.gpio = &(io->gpio);
	accel_parameter.spi = &(io->spi);
	genMap();
	
    printf("\n Hit ctl c to quit\n");
    printf("m1: manual			i: inc speed/start IMU\n");
    printf("m2: line trace			j: dec speed\n");
    printf("w: foward			p: print IMU data\n");			
    printf("a: turn left			n: print avg speed & distance\n");
    printf("s: stop				k: print map\n");
    printf("d: turn right\n");
    printf("b: reverse\n");
	
	pthread_create( &action_handle, 0, Actions, NULL);
	pthread_create( &accel_handle, 0, Accelerometer, (void *)&accel_parameter);
	pthread_create( &thread_camera_handle, 0, ThreadCamera, (void *)&thread_camera_parameter);
    pthread_create( &thread_image_analysis_handle, 0, ThreadImageAnalysis, (void *)&thread_image_analysis_parameter);
	
	GPIO_CLR( &(io->gpio), 5);
	GPIO_CLR( &(io->gpio), 6);
	GPIO_CLR( &(io->gpio), 22);
	GPIO_CLR( &(io->gpio), 23);	
	
	while (1) {
		GPIO_CLR( &(io->gpio), 5);
		GPIO_CLR( &(io->gpio), 6);
		GPIO_CLR( &(io->gpio), 22);
		GPIO_CLR( &(io->gpio), 23);
		array[1] = 0;
		array[0] = 0;
		array[2] = 0;  
		//MODE 1
		do {
			//foward
			while (searchArray(1,1) && searchArray(0,1)) {
				if (mode != 1) {
					GPIO_CLR( &(io->gpio), 5);
					GPIO_CLR( &(io->gpio), 6);
					GPIO_CLR( &(io->gpio), 22);
					GPIO_CLR( &(io->gpio), 23);
					array[1] = 0;
					break;
				}			
				GPIO_SET( &(io->gpio), 5);	//a1
				GPIO_CLR( &(io->gpio), 6);	//a2
				GPIO_SET( &(io->gpio), 22);
				GPIO_CLR( &(io->gpio), 23);
				io->pwm.DAT1 = 16 + getSpeed();  /*GPIO12 left motor*/
				io->pwm.DAT2 = 14.75 + getSpeed(); /*GPIO13 right motor*/
				usleep(10000);
				//turn left foward
				if (searchArray(3,1) == 1) {
					//for(int i = 0; i < 10; i ++) {				
						io->pwm.DAT1 = 2 + (getSpeed()/2);  /*GPIO12 left motor*/
						io->pwm.DAT2 = 14 + getSpeed(); /*GPIO13 right motor*/
						usleep(100000);
					//}
					array[3] = 0;
				}
				//turn right foward
				if (searchArray(4,1) == 1) {
					//for(int i = 0; i < 10; i ++) {
						io->pwm.DAT1 = 16 + getSpeed();  /*GPIO12 left motor*/
						io->pwm.DAT2 = 4 + (getSpeed()/2); /*GPIO13 right motor*/
						usleep(100000);
					//}
					array[4] = 0;
				}
				//going foward but reverse
				if (searchArray(2,1) == 1) {
					printf("Pause\n");
					GPIO_CLR( &(io->gpio), 5);
					GPIO_CLR( &(io->gpio), 6);
					GPIO_CLR( &(io->gpio), 22);
					GPIO_CLR( &(io->gpio), 23);
					usleep(500000);
					array[0] = 0;
					break;
				}
				//stop				
				if (searchArray(1,1) == 0) {
					GPIO_CLR( &(io->gpio), 5);
					GPIO_CLR( &(io->gpio), 6);
					GPIO_CLR( &(io->gpio), 22);
					GPIO_CLR( &(io->gpio), 23);
					array[1] = 0;
					break;
				}
			}
			if (mode != 1)
				break;
		
			//reverse
			while (searchArray(2,1) && searchArray(1,1)) {
				if (mode != 1) {
					GPIO_CLR( &(io->gpio), 5);
					GPIO_CLR( &(io->gpio), 6);
					GPIO_CLR( &(io->gpio), 22);
					GPIO_CLR( &(io->gpio), 23);
					break;
				}
				GPIO_CLR( &(io->gpio), 5);
				GPIO_SET( &(io->gpio), 6);
				GPIO_CLR( &(io->gpio), 22);
				GPIO_SET( &(io->gpio), 23);
				io->pwm.DAT1 = 16 + getSpeed();  /*GPIO12 left motor*/
				io->pwm.DAT2 = 14.75 + getSpeed(); /*GPIO13 right motor*/
				usleep(10000);
				//turn left reverse
				if (searchArray(3,1) == 1) {
					for(int i = 0; i < 100; i ++) {
						io->pwm.DAT1 = 10 + getSpeed();  /*GPIO12 left motor*/
						io->pwm.DAT2 = 14 + getSpeed(); /*GPIO13 right motor*/
						usleep(10000);
					}
					array[3] = 0;
				}
				//turn right reverse
				if (searchArray(4,1) == 1) {
					for(int i = 0; i < 100; i ++) {
						io->pwm.DAT1 = 16 + getSpeed();  /*GPIO12 left motor*/
						io->pwm.DAT2 = 12 + getSpeed(); /*GPIO13 right motor*/
						usleep(10000);
					}
					array[4] = 0;
				}
				//going reverse but foward
				if (searchArray(0,1) == 1) {
					printf("Pause\n");
					GPIO_CLR( &(io->gpio), 5);
					GPIO_CLR( &(io->gpio), 6);
					GPIO_CLR( &(io->gpio), 22);
					GPIO_CLR( &(io->gpio), 23);
					usleep(500000);
					array[2] = 0;
					break;
				}
				//stop			
				if (searchArray(1,1) == 0) {
					GPIO_CLR( &(io->gpio), 5);
					GPIO_CLR( &(io->gpio), 6);
					GPIO_CLR( &(io->gpio), 22);
					GPIO_CLR( &(io->gpio), 23);
					break;
				}
			if (mode != 1)
				break;
			}	
		} while (mode == 1);
		
		GPIO_CLR( &(io->gpio), 5);
		GPIO_CLR( &(io->gpio), 6);
		GPIO_CLR( &(io->gpio), 22);
		GPIO_CLR( &(io->gpio), 23);
		array[1] = 0;
		array[0] = 0;
		array[2] = 0;  

		//MODE 2
		//white = 0
		do {
			GPIO_SET( &(io->gpio), 5);
			GPIO_CLR( &(io->gpio), 6);
			GPIO_SET( &(io->gpio), 22);
			GPIO_CLR( &(io->gpio), 23);
			io->pwm.DAT1 = 10 + getSpeed();  /*GPIO12 left motor*/
			io->pwm.DAT2 = 9 + getSpeed(); /*GPIO13 right motor*/
			usleep(500);
//64,65

			//turn left		
			if (left_avg_pixel > left) {
				printf("Turn Left\n");		
				io->pwm.DAT1 = 7;  //GPIO12 left motor
				io->pwm.DAT2 = 10 + (getSpeed() * 2); //GPIO13 right motor
				usleep(5000);
			}
			if (left_avg_pixel > far_left) {
				printf("Turn far left\n");
				io->pwm.DAT1 = 2;  //GPIO12 left motor
				io->pwm.DAT2 = 10 + (getSpeed() * 2); //GPIO13 right motor
				usleep(2000);
			}
			
			//turn right
			if (right_avg_pixel > right) {
				printf("Turn Right\n");
				io->pwm.DAT1 = 11 + (getSpeed() * 2);  //GPIO12 left motor
				io->pwm.DAT2 = 7; //GPIO13 right motor
				usleep(5000);
			}
			if (right_avg_pixel > far_right) {
				printf("Turn far Right\n");
				io->pwm.DAT1 = 11 + (getSpeed() * 2);  //GPIO12 left motor
				io->pwm.DAT2 = 2; //GPIO13 right motor
				usleep(2000);
			}			 
			/*
			//Fail Safe
			while (avg_pixel > 20) {
				GPIO_CLR( &(io->gpio), 5);
				GPIO_CLR( &(io->gpio), 6);
				GPIO_CLR( &(io->gpio), 22);
				GPIO_CLR( &(io->gpio), 23);	
			}
			*/ 	
			//avg_pixel = 0;
			//left_avg_pixel = 0;
			//right_avg_pixel = 0;
			
		} while (mode == 2);
	}
		
  }
  else
  {
    ; /* warning message already issued */
  }

  return 0;
}
