#ifndef _SERIAL_H
#define _SERIAL_H

#include<stdio.h>  
#include<stdlib.h>  
#include<unistd.h>  
#include<sys/types.h>  
#include<sys/stat.h>  
#include<fcntl.h>  
#include<termios.h>  
#include<errno.h>  

int speed_arr[] = { B1000000 ,B38400, B19200, B9600, B4800, B2400, B1200, B300,B38400, B19200, B9600, B4800, B2400, B1200, B300, };  
int name_arr[] = {1000000, 38400,  19200,  9600,  4800,  2400,  1200,  300, 38400, 19200,  9600, 4800, 2400, 1200,  300, };  
 

class Serial 
{
	private:
	    int m_speed,m_databits,m_parity,m_stopbits;
	    int m_fd;
     
    public:
  	    Serial();
  	    int Open(const char* path);
  	    int Set(int speed,int databits,int parity,int stopbits);
  	    int Write(unsigned char *buf,int len);
  	    int Read(unsigned char *buf,int len);
  	    
    public:
	    unsigned long int IOTimeOut;
	    
};

#endif
