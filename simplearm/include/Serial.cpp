#include "Serial.h"

Serial::Serial(){
	m_speed = 1000000;
	m_databits = 8;
	m_parity = 'N';
	m_stopbits = 1;
	m_fd = -1;
}

int Serial::Open(const char* path){
    m_fd = open( path , O_RDWR | O_NDELAY);  
	if(m_fd == -1)  
    {  
       perror("serialport error\n");  
       return -1;
    }        
    else  
    {  
       //printf("%s",ttyname(m_fd)); 
       //printf(" opened\n"); 
       tcflush(m_fd, TCIOFLUSH); 
       return 1;
    }  
}

int Serial::Set(int speed,int databits,int parity,int stopbits){
	  if(!m_fd) return -1;
	  
	  m_speed = speed;
	  m_databits = databits;
	  m_parity = parity;
	  m_stopbits = stopbits;
	  
      int i;   
      int status;   
      struct termios   Opt;  
      tcgetattr(m_fd, &Opt);   
      
      Opt.c_cflag &= ~CSIZE;   
      switch (databits)   
      {     
        case 7:       
            Opt.c_cflag |= CS7;   
            break;  
        case 8:       
            Opt.c_cflag |= CS8;  
            break;     
        default:      
            fprintf(stderr,"Unsupported data size\n"); return (false);    
      }  
      
      switch (parity)   
      {     
            case 'n':  
            case 'N':      
                Opt.c_cflag &= ~PARENB;   /* Clear parity enable */  
                Opt.c_iflag &= ~INPCK;     /* Enable parity checking */   
                break;    
            case 'o':     
            case 'O':       
                Opt.c_cflag |= (PARODD | PARENB);   
                Opt.c_iflag |= INPCK;             /* Disnable parity checking */   
                break;    
            case 'e':    
            case 'E':     
                Opt.c_cflag |= PARENB;     /* Enable parity */      
                Opt.c_cflag &= ~PARODD;      
                Opt.c_iflag |= INPCK;       /* Disnable parity checking */  
                break;  
            case 'S':   
            case 's':  /*as no parity*/     
                Opt.c_cflag &= ~PARENB;  
                Opt.c_cflag &= ~CSTOPB;break;    
            default:     
                fprintf(stderr,"Unsupported parity\n");      
                return (false);    
       }    
          
       switch (stopbits)  
       {     
            case 1:      
                Opt.c_cflag &= ~CSTOPB;    
                break;    
            case 2:      
                Opt.c_cflag |= CSTOPB;    
               break;  
            default:      
                 fprintf(stderr,"Unsupported stop bits\n");    
                 return (false);   
       }   
       
       if (parity != 'n')     
         Opt.c_iflag |= INPCK;   
       tcflush(m_fd,TCIFLUSH);  
       Opt.c_cc[VTIME] = 150;   
       Opt.c_cc[VMIN] = 0; /* Update the options and do it NOW */  
       if (tcsetattr(m_fd,TCSANOW,&Opt) != 0)     
       {   
           perror("SetupSerial 3");     
           return (false);    
       }   

      for ( i= 0;  i < (int)(sizeof(speed_arr) / sizeof(int));  i++) {   
        if  (m_speed == name_arr[i]) {       
          tcflush(m_fd, TCIOFLUSH);       
          cfsetispeed(&Opt, speed_arr[i]);    
          cfsetospeed(&Opt, speed_arr[i]);     
          status = tcsetattr(m_fd, TCSANOW, &Opt);    
          if  (status != 0) {          
            perror("tcsetattr fd1");    
            return -1;       
          }      
          tcflush(m_fd,TCIOFLUSH);     
        }    
      }  	
      
 
      return (true);    
	
}

int Serial::Write(unsigned char *buf,int len){
    if(!m_fd) return false;
    return write(m_fd,buf,len);  
}

int Serial::Read(unsigned char *buf,int len){
    if(!m_fd) return false;
    return read(m_fd,buf,len);  
}
