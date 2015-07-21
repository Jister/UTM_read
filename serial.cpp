#include     <stdio.h>      /*标准输入输出定义*/
#include     <stdlib.h>     /*标准函数库定义*/
#include     <unistd.h>     /*Unix 标准函数定义*/
#include     <sys/types.h>  
#include     <sys/stat.h>   
#include     <fcntl.h>      /*文件控制定义*/
#include     <termios.h>    /*PPSIX 终端控制定义*/
#include     <errno.h>      /*错误号定义*/
#include     <string.h>


#define BAUDRATE B19200 ///Baud rate : 19200  
#define DEVICE "/dev/ttyACM0"  
#define SERIAL_PORT "/dev/ttyAMA0"
#define MAX_DISTANCE 30000
#define DANGER_DISTANCE 1600
#define WARNING_DISTANCE 2000
                            
int read_fd,serial_fd;
char databuf[4096];
int frame_len;
bool frame_start;
char distancebuf[3243];
int distance[1081];
char parse_head[2]={'G','D'}; 
char parse_end[2]={'\n','\n'};

char send_buf[50];

int set_serial(int fd,int nSpeed, int nBits, char nEvent, int nStop)  
{  
		struct termios newtio,oldtio;  
		if  ( tcgetattr( fd,&oldtio)  !=  0) {   
		    perror("SetupSerial 1");  
			return -1;  
		}  
		bzero( &newtio, sizeof( newtio ) );  
		newtio.c_cflag  |=  CLOCAL | CREAD;  
		newtio.c_cflag &= ~CSIZE;  

		switch( nBits )  
		{  
			case 7:  
				newtio.c_cflag |= CS7;  
				break;  
			case 8:  
				newtio.c_cflag |= CS8;  
				break;  
		}

		switch( nEvent )  
		{  
			case 'O':  
				newtio.c_cflag |= PARENB;  
				newtio.c_cflag |= PARODD;  
				newtio.c_iflag |= (INPCK | ISTRIP);  
				break;  
			case 'E':   
				newtio.c_iflag |= (INPCK | ISTRIP);  
				newtio.c_cflag |= PARENB;  
				newtio.c_cflag &= ~PARODD;  
		    	break;
			case 'N':    																									
				newtio.c_cflag &= ~PARENB;  
				break;  
		}  
		
		switch( nSpeed )  
		{  
			case 2400:  
				cfsetispeed(&newtio, B2400);  
				cfsetospeed(&newtio, B2400);  
				break;  
			case 4800:  
				cfsetispeed(&newtio, B4800);  
				cfsetospeed(&newtio, B4800);  
				break;
			case 9600:  
				cfsetispeed(&newtio, B9600);  
				cfsetospeed(&newtio, B9600);  
				break;  
			case 19200:  
				cfsetispeed(&newtio, B19200);  
				cfsetospeed(&newtio, B19200);  
				break; 
			case 115200:  
				cfsetispeed(&newtio, B115200);  
				cfsetospeed(&newtio, B115200);  
				break;  
			case 460800:  
				cfsetispeed(&newtio, B460800);  
				cfsetospeed(&newtio, B460800);  
				break;  
			default:  
				cfsetispeed(&newtio, B9600);  
				cfsetospeed(&newtio, B9600);  
				break;  
		}  
		
		if( nStop == 1 )  
		{
			newtio.c_cflag &=  ~CSTOPB;  
		}
		else if ( nStop == 2 )
		{  
			newtio.c_cflag |=  CSTOPB;  
		} 
		
		newtio.c_cc[VTIME]  = 100;//重要  
		newtio.c_cc[VMIN] = 0;//返回的最小值  重要  
		tcflush(fd,TCIFLUSH);  
		if((tcsetattr(fd,TCSANOW,&newtio))!=0)  
		{  
			perror("com set error");  
			return -1;  
		}  
		return 0;  
}  

//设备使能
void switch_on()
{
		int ret;
		char write_buf[20];

		memset(write_buf,0,20);   
		sprintf(write_buf, "BM\n");
		ret = write(read_fd,write_buf,3);
		if(-1 == ret)  
		{
			perror("Switch_on Write Data Error!\n");  
			exit(1); 
		}  
}

void switch_off()
{
		int ret;
		char write_buf[20];
		
		memset(write_buf,0,20);   
		sprintf(write_buf, "QT\n");
		ret = write(read_fd,write_buf,3);
		if(-1 == ret)  
		{
			perror("Switch_off Write Data Error!\n");  
			exit(1); 
		}  
}

//获得一次数据
void get_data_by_GD(int inStart, int inEnd, int inClusterCnt)
{
		int ret;
		char write_buf[20];

		memset(write_buf,0,20);   
		sprintf(write_buf, "GD%04d%04d%02d\n", inStart, inEnd, inClusterCnt);
		ret = write(read_fd,write_buf,13);
		if(-1 == ret)  
		{
			perror("Get Data Write Data Error!\n");  
			exit(1); 
		}
}  

//获得设备信息
void get_data_by_VV()
{
		int ret;
		char write_buf[20];

		memset(write_buf,0,20);   
		sprintf(write_buf, "VV\n");
		ret = write(read_fd,write_buf,3);
		if(-1 == ret)  
		{
			perror("Get Data Write Data Error!\n");  
			exit(1); 
		}else
		    printf("OK\n");
}  

void read_data()
{
		int ret;

		memset(databuf,0,4096);   
		ret = read(read_fd, databuf, 4096);
		if(-1 == ret)  
		{
			perror("Read Data Error!\n");  
			exit(1); 
												}
}

long urg_decode(const char *data, int data_byte)
{	
		long value = 0;
		for (int i = 0; i < data_byte; ++i)
		{
			value <<= 6;
			value &= ~0x3f;
			value |= data[i] - 0x30;
		}
		return value;
}

void parse(char *data)
{
		int nData;
		int nBlock;
		int nLeft;
		int nstart;
		int distance_num;
		char* pDataBuf;

		frame_len = 0;
		frame_start = false;
		memset(distance,0,1081);   

		for(int i=0; i<4096; i++)
		{
			if(frame_start ==false)
			{
				if( (data[i]=='G') && (data[i+1]=='D') )
				{
					frame_start = true;
					nstart = i;
					frame_len++;
				}
			}
			else
			{
				frame_len++;
				if( (data[i]==parse_end[0]) && (data[i+1]==parse_end[1]) )
				{
					frame_len++;;
					break;
				}
			}
		}

		nData = frame_len - 23/*cmd*/ - 1/*最后一个LF*/;
		nBlock = nData/66;
		nLeft = nData%66 - 2/*sum+LF*/;

		pDataBuf = data + nstart +23 ;
		//data block
		for (int i=0;i<nBlock;i++)
		{
			memcpy((distancebuf+i*64),pDataBuf,64);
			pDataBuf += 66;
		}
		//left
		if (nLeft>0)
		{
			memcpy((distancebuf+nBlock*64),pDataBuf,nLeft);
		}

		distance_num = (nBlock*64+nLeft)/3;
		pDataBuf = distancebuf;

		for (int i=0;i<distance_num;i++)
		{
			int temp;
			temp = urg_decode(pDataBuf,3);
			if (temp < 10)
			{
				distance[i] = 30000;
			}
			else
			  distance[i] = temp;
			pDataBuf += 3;
		}
}

void record()
{
		int ret,fd;
		char record_buf[11891];
		char *p;

		fd = open("/home/chenjie/test"  ,O_RDWR);
		ret = write(fd,databuf,4096);
		if (ret == -1)  
		{
			perror("write Error!\n");  
			exit(1);  
		}
		close(fd);
		
		p = record_buf;
		for(int i=0;i<1081;i++)
		{
			sprintf(p,"%04d:%05d\n",i,distance[i]);
			p+=11;
		}
		fd = open("/home/chenjie/test1"  ,O_RDWR);
		ret = write(fd,record_buf,11891);
		if (ret == -1)  
		{
			perror("write Error!\n");  
			exit(1);  
		}
		close(fd);
}

void send(char*data, int data_byte)
{
		int ret;

		ret = write(serial_fd,data,data_byte);
		if (ret == -1)  
		{
			perror("write Error!\n");  
			exit(1);  
		}
}

void init()
{
	    int ret;
		read_fd = open(DEVICE, O_RDWR);//打开串口  
		if (read_fd == -1)
	    {
			perror("Open Error!\n");  
			exit(1);    
		}  
		ret = set_serial(read_fd,BAUDRATE, 8, 'N', 1);//设置串口属性  
		if (ret == -1)  
		{
			perror("Set Error!\n");  
			exit(1);  
		}

		serial_fd = open(SERIAL_PORT, O_RDWR);//打开串口  
		if (serial_fd == -1)
		{
			perror("Open Error!\n");  
			exit(1);	
		}  
		ret = set_serial(serial_fd,115200, 8, 'N', 1);//设置串口属性  
		if (ret == -1)  
		{
			perror("Set Error!\n");  
			exit(1);  
		}
}

void calculate()
{
		int min_distance;
		int angle;

		min_distance = distance[0];
		angle = 0;
		memset(send_buf,0,50);
		for(int i=1;i<1081;i++)
		{
			if(distance[i]<min_distance)
			{
				min_distance = distance[i];
				angle = i*0.25;
			}
		}
		sprintf(send_buf,"M%03d%05d\n" , angle , min_distance);
}

int main(void)  
{  
		    int ret;  

			init();
			switch_on();

			while(1)
			{
				get_data_by_GD(0,1080,1);
				usleep(75000);
				read_data();
				parse(databuf);
				calculate();

				ret = write(serial_fd,send_buf,10);
				if(ret == -1)
				{
					perror("write error!\n");
					exit(1);
				}
				printf("send OK\n");
				for(int i=0;i<10;i++)
				{
					printf("%c",send_buf[i]);
				}
			}
			close(serial_fd);  
			return 0;  
}  

