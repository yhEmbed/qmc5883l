/**
 *
 * @file      main.c
 * @brief     main source file
 * @version   1.0.0
 * @author    Chen Fan
 * @date      2022-07-5
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2021/03/20  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */
#include <stdio.h>
#include <stdlib.h>
#include "qmc5883.h"
#include <unistd.h>
#include <getopt.h>
#include <dirent.h>
#include <sys/stat.h>
#include <string.h>

/* Define ----------------------------------------------------------------------------------*/
/* Macro -----------------------------------------------------------------------------------*/
/* Typedef ---------------------------------------------------------------------------------*/
typedef struct {
    short mag[3];
}MagnetData_t;

MagnetData_t algo_imu;


#define  APP_TOOL_DIR "/tmp/apptool/"
static void usage()
{
	printf("\n");
	printf("Usage:\n");
	printf("Examples:\n");
	printf("  qmc5883 -h\n");
	printf("  qmc5883 -s 1 -t 500 \n");
	printf("  qmc5883 -c 1 -n 20 -t 500 \n");
	printf("  qmc5883 -p 1  -t 500 \n");
	printf("Options:\n");
	printf("  -s single read mode \n");
	printf("  -c mutiple read mode \n");
	printf("  -p permanent read mode\n");
	printf("\n");
	exit(0);
}

static void dump_to_file(const char* file_path, const void* data, int len)
{
	FILE* fp = fopen(file_path, "w");
	if (fp == NULL)
	{
	    printf("open file %s error\n", file_path);
	    return;
	}
	fwrite(data, len, 1, fp);
	fclose(fp);
	//printf("save to file %s success\n", file_path);
}

/**
 * @brief     main function
 * @param[in] argc is arg numbers
 * @param[in] **argv is the arg address
 * @return    status code
 *             - 0 success
 * @note      none
 */
//int main(int argc, char **argv)
int main(int argc, char* argv[])
{
    uint8_t res;
	int c,i;
	int conti_act=0;
	int conti_num=0;
	int single_act=0;
	int interval_time=500;
	int permanent=0;
	int verbose=0;
	int heading ;
	char str[100];
	char filename[100];
	char path[100];
	if(argc < 2) {
		usage();
		exit(0);
	}
	
	while((c = getopt(argc,argv,"s:c:n:t:p:hv")) != -1){
		switch(c){
		case 's':
			single_act = atoi(optarg);
			break;
		case 'c':
			conti_act = atoi(optarg);
			break;
		case 'n':	
			conti_num =	atoi(optarg);		
			break;
		case 't':	
			interval_time =	atoi(optarg);		
			break;
		case 'p':	
			permanent =	atoi(optarg);		
			break;
		case 'v':	
			verbose =	1;		
			break;
		case 'h':
			usage();
			break;
		default:
			usage();
			break;
		}
	}
	
	if(qmc5883_init())return -1;
	
	qmc5883_setSamplingRate(50);
	
	if(access(APP_TOOL_DIR, F_OK) != 0 ) {
        printf("to create apptool dir: %s\n", APP_TOOL_DIR);
        if((mkdir(APP_TOOL_DIR, 0777)) < 0)
        {
            printf("mkdir %s failed\n", APP_TOOL_DIR);
            //return -1;
        }
    }
	
	
#if 1
	while(1)
	{
		if (QMC5883_Calculate_Flag > 1)
		{
			QMC5883L_GetData(algo_imu.mag);
			Calculate_QMC5883();
			// usleep(interval_time);
		}

		
		if(QMC5883_Calculate_Flag == 1)
		{
			QMC5883L_GetData(algo_imu.mag);
			QMC5883L_ConvertrawData();
			usleep(interval_time*1000);
		}
	}

		
#else
	while(1)
	{	
		heading = qmc5883_readHeading();
		if(heading==0) {
			/* Still calibrating, so measure but don't print */
		} else {
		
			break;
		}
	}
	sprintf(path,"%s%s",APP_TOOL_DIR,"qmc5883");
	
	if(single_act){
		heading = qmc5883_readHeading();
		sprintf(str,"%d",heading);
		
		dump_to_file(path,str,strlen(str));
		if(verbose)	printf("heading=%d\n",heading);
	}else if(conti_num){

	  for(i=0;i<conti_num;i++){
		heading = qmc5883_readHeading();
		sprintf(str,"%d",heading);
		dump_to_file(path,str,strlen(str));
		if(verbose)	printf("heading=%d\n",heading);
	  	usleep(interval_time*1000);
	  }

	}else if(permanent){

		for(;;){
			heading = qmc5883_readHeading();
			sprintf(str,"%d",heading);
			dump_to_file(path,str,strlen(str));
			if(verbose)	printf("heading=%d\n",heading);
	  		usleep(interval_time*1000);
		}
	}
#endif

    return 0;
}
