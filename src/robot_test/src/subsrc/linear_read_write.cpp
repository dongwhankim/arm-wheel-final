// C library headers
#include <stdio.h>
#include <string.h>
#include <iostream>
//#include <string>
#include <cmath>
#include <stdlib.h>
#include <pthread.h>
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function0
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <linear_read_write.h>
//unsigned char select[12];
//unsigned char select[] ;

//#define PERIOD_NS 1000000  // 1Khz
//#define SEC_IN_NSEC 1000000000


Clinear::Clinear()
{		  
  _linear_serial_port = 0;
  _count = 0;
  _encoder_count = 0;
  _t = 0;
  _temp_present_position = 0;
  _plus_minus_state = 0;
  _encoder_plus_minus_state = 1;
  _tmpbuf = 1000000;
  _tmpbuf_old = 1000000;
  _read_state = 0;
  for(int i = 0; i< 255; i++)
      {
        _read_buf[i] = '\0';
      }
}
Clinear::~Clinear()
{
}

void Clinear::homing()
{  
  write(_linear_serial_port, _homing, sizeof(_homing));
}

void Clinear::read_encoder()
{ 
  //std::cout<<"read_buf : "<<_read_buf;
  if(((_read_buf[0] >= '0' && _read_buf[0] <= '9') || _read_buf[0] <= '-') && (int)_read_buf[0] != 10)
    {
      _read_state = 1; 
      //std::cout<<"read[0] : "<<(int)_read_buf[0] << std::endl;   
    }
  else
    {
      _read_state = 0;
    }
    //std::cout<<_read_state <<std::endl;
  if(_read_state == 1)
    {

    _tmpbuf = 0;
    _encoder_count = 0;
    // rkqt cpzm
    for(int i = 0 ; i<255 ; i++)
    {
      if(_read_buf[i] == '-')
      {
        _encoder_plus_minus_state = 0; // minus
      }
      else
      {
        _encoder_plus_minus_state = 1; // plus
      }
      if(_read_buf[i] >= '0' && _read_buf[i] <= '9')
      {
      _encoder_count = i;
      //_tmpbuf = _tmpbuf + char2int(_read_buf[i]) * pow(10,i);
      //std::cout<<"i = " << i << "tmpbuf = " <<_tmpbuf<<std::endl;
      }
    }

    if(_encoder_plus_minus_state == 1)
    {
      for(int i = 0; i<_encoder_count+1 ; i++)
      {
        _tmpbuf = _tmpbuf + char2int(_read_buf[i]) * pow(10, _encoder_count-i);
      }
    }
  
    else if (_encoder_plus_minus_state == 0)
    {
      for(int i = 1; i<_encoder_count+1 ; i++)
      {
        _tmpbuf = _tmpbuf + char2int(_read_buf[i]) * pow(10, _encoder_count);
      }
      _tmpbuf = -_tmpbuf;
    }
  //std::cout<<"encoder_cnt : " <<_encoder_count<<std::endl;

    if(abs(_tmpbuf_old-_tmpbuf) >  _tmpbuf_old/2)   //2*pow(10,_encoder_count-1))
    {}
    else
    {
      _tmpbuf_old = _tmpbuf;
    }
  }
  //std::cout<<"tmp : " <<_tmpbuf<<std::endl;
  //std::cout<<"tmpold : " <<_tmpbuf_old<<std::endl;

    for(int i = 0; i< 255; i++)
      {
        _read_buf[i] = '\0';
      }
  tcflush(_linear_serial_port,  TCIFLUSH);//  시리얼 포트 수신 큐 초기화
  write(_linear_serial_port, _encoder, sizeof(_encoder));
  read(_linear_serial_port,  _read_buf,  255);
}


void Clinear::goalposition2Uchar(int n)
  {
  _count = 0;
  _plus_minus_state = 1;
  _t = n;
    
    if (_t<0)
    {
      _count++;
      _plus_minus_state = 0;
    }
    if(_plus_minus_state ==0)
    {
      _t = -_t;
    }
    else if (_t == 0)
    {
      _count ++;
    }
    while(_t != 0)
    {
        _t = _t/10;
        _count++;
    }
    int max_int = 0;
    //std::cout<<"_count : " << _count<<std::endl;
    //std::cout<<"goal_position : " << n <<std::endl;

switch (_count)
{
  case 1:
    _select_1[0] = 'M';
    _select_1[1] = 'A';
    _select_1[2] = ' ';
    _select_1[3] = int2Uchar(n);;
    _select_1[4] = '\r';
  //std::cout<<_select_1<<std::endl;
  write(_linear_serial_port, _select_1, sizeof(_select_1));
  //read_encoder();
  //write(serial_port, encoder, sizeof(encoder));
  break;

   case 2:
   if (_plus_minus_state == 1)
   {
    _select_2[0] = 'M';
    _select_2[1] = 'A';
    _select_2[2] = ' ';
    max_int = pow(10,_count-1);
    //std::cout<<t<<std::endl;
    //std::cout<<"count : " << count<<std::endl;
      for (int i = 0 ; i < _count ; i++)
      {
        //std::cout<<"n/t : "<<n/t<<std::endl;
       _select_2[3+i] = int2Uchar(n/max_int);
       n = n-(n/max_int)*max_int;
       max_int = max_int / 10;
       if ( i == _count-1)
       {
         _select_2[4+i] = '\r';
       }
       }
  //std::cout<<"select2 : "<<_select_2<<std::endl;
  //std::cout<<"sizeof(select2) : "<<sizeof(select2)<<std::endl;
   write(_linear_serial_port, _select_2, sizeof(_select_2));
      //read_encoder();

   //write(serial_port, encoder, sizeof(encoder));
   break;
   }
   else
   {
    _select_2[0] = 'M';
    _select_2[1] = 'A';
    _select_2[2] = ' ';
    _select_2[3] = '-';
    max_int = pow(10,_count-2);
    _select_2[4] = int2Uchar(-n);
    _select_2[5] = '\r';
 //std::cout<<"select2 : "<<_select_2<<std::endl;
  //std::cout<<"sizeof(select2) : "<<sizeof(select2)<<std::endl;
   write(_linear_serial_port, _select_2, sizeof(_select_2));
      //read_encoder();

   //write(serial_port, encoder, sizeof(encoder));
  break;
   }
  case 3:
    if (_plus_minus_state == 1)
   {
    _select_3[0] = 'M';
    _select_3[1] = 'A';
    _select_3[2] = ' ';
    max_int = pow(10,_count-1);
    //std::cout<<t<<std::endl;
    //std::cout<<"count : " << count<<std::endl;
      for (int i = 0 ; i < _count ; i++)
      {
        //std::cout<<"n/t : "<<n/t<<std::endl;
       _select_3[3+i] = int2Uchar(n/max_int);
       n = n-(n/max_int)*max_int;
       max_int = max_int / 10;
       if ( i == _count-1)
       {
         _select_3[4+i] = '\r';
       }
       }
  //std::cout<<"select3 : "<<_select_3<<std::endl;
  //std::cout<<"sizeof(select2) : "<<sizeof(select2)<<std::endl;
   write(_linear_serial_port, _select_3, sizeof(_select_3));
      //read_encoder();

   //write(serial_port, encoder, sizeof(encoder));
   break;
   }
   else
   {
    _select_3[0] = 'M';
    _select_3[1] = 'A';
    _select_3[2] = ' ';
    _select_3[3] = '-';
    max_int = pow(10,_count-2);
    n = -n;
    //std::cout<<"n : "<<n<<std::endl;
    for (int i = 0 ; i < _count-1 ; i++)
      {
          //std::cout<<"n/t : "<<n/t<<std::endl;
        _select_3[4+i] = int2Uchar(n/max_int);
        n = n-(n/max_int)*max_int;
        max_int = max_int / 10;
        if ( i == _count-1)
        {
          _select_3[5+i] = '\r';
        }
      }
  //std::cout<<"select3 : "<<_select_3<<std::endl;
  //std::cout<<"sizeof(select2) : "<<sizeof(select2)<<std::endl;
   write(_linear_serial_port, _select_4, sizeof(_select_4));
     // read_encoder();

   //write(serial_port, encoder, sizeof(encoder));
  break;
   }
  case 4:
       if (_plus_minus_state == 1)
   {
    _select_4[0] = 'M';
    _select_4[1] = 'A';
    _select_4[2] = ' ';
    max_int = pow(10,_count-1);
    //std::cout<<t<<std::endl;
    //std::cout<<"count : " << count<<std::endl;
      for (int i = 0 ; i < _count ; i++)
      {
          //std::cout<<"n/t : "<<n/t<<std::endl;
        _select_4[3+i] = int2Uchar(n/max_int);
        n = n-(n/max_int)*max_int;
        max_int = max_int / 10;
        if ( i == _count-1)
        {
          _select_4[4+i] = '\r';
        }
      }
  //std::cout<<"????????????? : "<<_select_4<<std::endl;
  //std::cout<<"sizeof(select2) : "<<sizeof(select2)<<std::endl;
   write(_linear_serial_port, _select_4, sizeof(_select_4));
     // read_encoder();

   //write(serial_port, encoder, sizeof(encoder));
   break;
   }
   else
   {
    _select_4[0] = 'M';
    _select_4[1] = 'A';
    _select_4[2] = ' ';
    _select_4[3] = '-';
    max_int = pow(10,_count-2);
    n = -n;
    //std::cout<<"n : "<<n<<std::endl;
    for (int i = 0 ; i < _count-1 ; i++)
      {
          //std::cout<<"n/t : "<<n/t<<std::endl;
        _select_4[4+i] = int2Uchar(n/max_int);
        n = n-(n/max_int)*max_int;
        max_int = max_int / 10;
        if ( i == _count-2)
        {
          _select_4[5+i] = '\r';
        }
      }
  //std::cout<<"select4 : "<<_select_4<<std::endl;
  //std::cout<<"sizeof(select2) : "<<sizeof(select2)<<std::endl;
   write(_linear_serial_port, _select_4, sizeof(_select_4));
   //read_encoder();
   //write(serial_port, encoder, sizeof(encoder));
  break;
   }
  case 5:
  if (_plus_minus_state == 1)
   {
    _select_5[0] = 'M';
    _select_5[1] = 'A';
    _select_5[2] = ' ';
    max_int = pow(10,_count-1);
    //std::cout<<t<<std::endl;
    //std::cout<<"count : " << count<<std::endl;
      for (int i = 0 ; i < _count ; i++)
      {
          //std::cout<<"n/t : "<<n/t<<std::endl;
        _select_5[3+i] = int2Uchar(n/max_int);
        n = n-(n/max_int)*max_int;
        max_int = max_int / 10;
        if ( i == _count-1)
        {
          _select_5[4+i] = '\r';
        }
      }
  //std::cout<<"select5 : "<<_select_5<<std::endl;
  //std::cout<<"sizeof(select2) : "<<sizeof(select2)<<std::endl;
   write(_linear_serial_port, _select_5, sizeof(_select_5));
   //read(_linear_serial_port, _read_buf, sizeof(_read_buf));
   //read_encoder();
  //  if(_read_buf[0] == 'M')
  //  {
  //    std::cout<<"select5 : "<<_select_5<<std::endl;
  //   read_encoder();
  //   break;  
  //  }
  //  else
  //  {
  //    break;
  //  }
   
   //write(serial_port, encoder, sizeof(encoder));
   break;
   }
   else
   {
    _select_5[0] = 'M';
    _select_5[1] = 'A';
    _select_5[2] = ' ';
    _select_5[3] = '-';
    max_int = pow(10,_count-2);
    n = -n;
    //std::cout<<"n : "<<n<<std::endl;
    for (int i = 0 ; i < _count-1 ; i++)
      {
          //std::cout<<"n/t : "<<n/t<<std::endl;
        _select_5[4+i] = int2Uchar(n/max_int);
        n = n-(n/max_int)*max_int;
        max_int = max_int / 10;
        if ( i == _count-2)
        {
          _select_5[5+i] = '\r';
        }
      }
  //std::cout<<"select5 : "<<_select_5<<std::endl;
  //std::cout<<"sizeof(select2) : "<<sizeof(select2)<<std::endl;
   write(_linear_serial_port, _select_5, sizeof(_select_5));
   //read_encoder();
   //write(serial_port, encoder, sizeof(encoder));
  break;
   }
  case 6:
    if (_plus_minus_state == 1)
   {
    _select_6[0] = 'M';
    _select_6[1] = 'A';
    _select_6[2] = ' ';
    max_int = pow(10,_count-1);
    //std::cout<<t<<std::endl;
    //std::cout<<"count : " << count<<std::endl;
      for (int i = 0 ; i < _count ; i++)
      {
          //std::cout<<"n/t : "<<n/t<<std::endl;
        _select_6[3+i] = int2Uchar(n/max_int);
        n = n-(n/max_int)*max_int;
        max_int = max_int / 10;
        if ( i == _count-1)
        {
          _select_6[4+i] = '\r';
        }
      }
  //std::cout<<"select6 : "<<_select_6<<std::endl;
  //std::cout<<"sizeof(select2) : "<<sizeof(select2)<<std::endl;
   write(_linear_serial_port, _select_6, sizeof(_select_6));
   //read_encoder();
   //write(serial_port, encoder, sizeof(encoder));
   break;
   }
   else
   {
    _select_6[0] = 'M';
    _select_6[1] = 'A';
    _select_6[2] = ' ';
    _select_6[3] = '-';
    max_int = pow(10,_count-2);
    n = -n;
    //std::cout<<"n : "<<n<<std::endl;
    for (int i = 0 ; i < _count-1 ; i++)
      {
          //std::cout<<"n/t : "<<n/t<<std::endl;
        _select_6[4+i] = int2Uchar(n/max_int);
        n = n-(n/max_int)*max_int;
        max_int = max_int / 10;

          if ( i == _count-2)
        {
          _select_6[5+i] = '\r';
        }
      }
  //std::cout<<"select6 : "<<_select_6<<std::endl;
  //std::cout<<"sizeof(select2) : "<<sizeof(select2)<<std::endl;
   write(_linear_serial_port, _select_6, sizeof(_select_6));
  // read_encoder();
   //write(serial_port, encoder, sizeof(encoder));
  break;
   }
  case 7:
       if (_plus_minus_state == 1)
   {
    _select_7[0] = 'M';
    _select_7[1] = 'A';
    _select_7[2] = ' ';
    max_int = pow(10,_count-1);
    //std::cout<<t<<std::endl;
    //std::cout<<"count : " << count<<std::endl;
      for (int i = 0 ; i < _count ; i++)
      {
          //std::cout<<"n/t : "<<n/t<<std::endl;
        _select_7[3+i] = int2Uchar(n/max_int);
        n = n-(n/max_int)*max_int;
        max_int = max_int / 10;
        if ( i == _count-1)
        {
          _select_7[4+i] = '\r';
        }
      }
  //std::cout<<"select7 : "<<_select_7<<std::endl;
  //std::cout<<"sizeof(select2) : "<<sizeof(select2)<<std::endl;
   write(_linear_serial_port, _select_7, sizeof(_select_7));
   //read_encoder();
   //write(serial_port, encoder, sizeof(encoder));
   break;
   }
   else
   {
    _select_7[0] = 'M';
    _select_7[1] = 'A';
    _select_7[2] = ' ';
    _select_7[3] = '-';
    max_int = pow(10,_count-2);
    n = -n;
    //std::cout<<"n : "<<n<<std::endl;
    for (int i = 0 ; i < _count-1 ; i++)
      {
          //std::cout<<"n/t : "<<n/t<<std::endl;
        _select_7[4+i] = int2Uchar(n/max_int);
        n = n-(n/max_int)*max_int;
        max_int = max_int / 10;

          if ( i == _count-2)
        {
          _select_7[5+i] = '\r';
        }
      }
  //std::cout<<"select7 : "<<_select_7<<std::endl;
  //std::cout<<"sizeof(select2) : "<<sizeof(select2)<<std::endl;
   write(_linear_serial_port, _select_7, sizeof(_select_7));
   //read_encoder();
   //write(serial_port, encoder, sizeof(encoder));
  break;
   }
  case 8:
    _select_8[0] = 'M';
    _select_8[1] = 'A';
    _select_8[2] = ' ';
    _select_8[3] = '-';
    max_int = pow(10,_count-2);
    n = -n;
    //std::cout<<"n : "<<n<<std::endl;
    for (int i = 0 ; i < _count-1 ; i++)
      {
          //std::cout<<"n/t : "<<n/t<<std::endl;
        _select_8[4+i] = int2Uchar(n/max_int);
        n = n-(n/max_int)*max_int;
        max_int = max_int / 10;

          if ( i == _count-2)
        {
          _select_8[5+i] = '\r';
        }
      }
  //std::cout<<"select8 : "<<_select_8<<std::endl;
  //std::cout<<"sizeof(select2) : "<<sizeof(select2)<<std::endl;
      write(_linear_serial_port, _select_8, sizeof(_select_8));
      //read_encoder();
      //write(serial_port, encoder, sizeof(encoder));
      break;
    //}
  }
  write(_linear_serial_port, _encoder, sizeof(_encoder));
}

void Clinear::goalposition2Uchar_rel(int n)
  {
  _count = 0;
  _plus_minus_state = 1;
  _t = n;
    
    if (_t<0)
    {
      _count++;
      _plus_minus_state = 0;
    }
    if(_plus_minus_state ==0)
    {
      _t = -_t;
    }
    else if (_t == 0)
    {
      _count ++;
    }
    while(_t != 0)
    {
        _t = _t/10;
        _count++;
    }
    int max_int = 0;
    //std::cout<<"_count : " << _count<<std::endl;
    //std::cout<<"goal_position : " << n <<std::endl;

switch (_count)
{
  case 1:
    _select_1[0] = 'M';
    _select_1[1] = 'R';
    _select_1[2] = ' ';
    _select_1[3] = int2Uchar(n);;
    _select_1[4] = '\r';
  //std::cout<<_select_1<<std::endl;
  write(_linear_serial_port, _select_1, sizeof(_select_1));
  //read_encoder();
  //write(serial_port, encoder, sizeof(encoder));
  break;

   case 2:
   if (_plus_minus_state == 1)
   {
    _select_2[0] = 'M';
    _select_2[1] = 'R';
    _select_2[2] = ' ';
    max_int = pow(10,_count-1);
    //std::cout<<t<<std::endl;
    //std::cout<<"count : " << count<<std::endl;
      for (int i = 0 ; i < _count ; i++)
      {
        //std::cout<<"n/t : "<<n/t<<std::endl;
       _select_2[3+i] = int2Uchar(n/max_int);
       n = n-(n/max_int)*max_int;
       max_int = max_int / 10;
       if ( i == _count-1)
       {
         _select_2[4+i] = '\r';
       }
       }
  //std::cout<<"select2 : "<<_select_2<<std::endl;
  //std::cout<<"sizeof(select2) : "<<sizeof(select2)<<std::endl;
   write(_linear_serial_port, _select_2, sizeof(_select_2));
      //read_encoder();

   //write(serial_port, encoder, sizeof(encoder));
   break;
   }
   else
   {
    _select_2[0] = 'M';
    _select_2[1] = 'R';
    _select_2[2] = ' ';
    _select_2[3] = '-';
    max_int = pow(10,_count-2);
    _select_2[4] = int2Uchar(-n);
    _select_2[5] = '\r';
 //std::cout<<"select2 : "<<_select_2<<std::endl;
  //std::cout<<"sizeof(select2) : "<<sizeof(select2)<<std::endl;
   write(_linear_serial_port, _select_2, sizeof(_select_2));
      //read_encoder();

   //write(serial_port, encoder, sizeof(encoder));
  break;
   }
  case 3:
    if (_plus_minus_state == 1)
   {
    _select_3[0] = 'M';
    _select_3[1] = 'R';
    _select_3[2] = ' ';
    max_int = pow(10,_count-1);
    //std::cout<<t<<std::endl;
    //std::cout<<"count : " << count<<std::endl;
      for (int i = 0 ; i < _count ; i++)
      {
        //std::cout<<"n/t : "<<n/t<<std::endl;
       _select_3[3+i] = int2Uchar(n/max_int);
       n = n-(n/max_int)*max_int;
       max_int = max_int / 10;
       if ( i == _count-1)
       {
         _select_3[4+i] = '\r';
       }
       }
  //std::cout<<"select3 : "<<_select_3<<std::endl;
  //std::cout<<"sizeof(select2) : "<<sizeof(select2)<<std::endl;
   write(_linear_serial_port, _select_3, sizeof(_select_3));
      //read_encoder();

   //write(serial_port, encoder, sizeof(encoder));
   break;
   }
   else
   {
    _select_3[0] = 'M';
    _select_3[1] = 'R';
    _select_3[2] = ' ';
    _select_3[3] = '-';
    max_int = pow(10,_count-2);
    n = -n;
    //std::cout<<"n : "<<n<<std::endl;
    for (int i = 0 ; i < _count-1 ; i++)
      {
          //std::cout<<"n/t : "<<n/t<<std::endl;
        _select_3[4+i] = int2Uchar(n/max_int);
        n = n-(n/max_int)*max_int;
        max_int = max_int / 10;
        if ( i == _count-1)
        {
          _select_3[5+i] = '\r';
        }
      }
  //std::cout<<"select3 : "<<_select_3<<std::endl;
  //std::cout<<"sizeof(select2) : "<<sizeof(select2)<<std::endl;
   write(_linear_serial_port, _select_4, sizeof(_select_4));
     // read_encoder();

   //write(serial_port, encoder, sizeof(encoder));
  break;
   }
  case 4:
       if (_plus_minus_state == 1)
   {
    _select_4[0] = 'M';
    _select_4[1] = 'R';
    _select_4[2] = ' ';
    max_int = pow(10,_count-1);
    //std::cout<<t<<std::endl;
    //std::cout<<"count : " << count<<std::endl;
      for (int i = 0 ; i < _count ; i++)
      {
          //std::cout<<"n/t : "<<n/t<<std::endl;
        _select_4[3+i] = int2Uchar(n/max_int);
        n = n-(n/max_int)*max_int;
        max_int = max_int / 10;
        if ( i == _count-1)
        {
          _select_4[4+i] = '\r';
        }
      }
  //std::cout<<"????????????? : "<<_select_4<<std::endl;
  //std::cout<<"sizeof(select2) : "<<sizeof(select2)<<std::endl;
   write(_linear_serial_port, _select_4, sizeof(_select_4));
     // read_encoder();

   //write(serial_port, encoder, sizeof(encoder));
   break;
   }
   else
   {
    _select_4[0] = 'M';
    _select_4[1] = 'R';
    _select_4[2] = ' ';
    _select_4[3] = '-';
    max_int = pow(10,_count-2);
    n = -n;
    //std::cout<<"n : "<<n<<std::endl;
    for (int i = 0 ; i < _count-1 ; i++)
      {
          //std::cout<<"n/t : "<<n/t<<std::endl;
        _select_4[4+i] = int2Uchar(n/max_int);
        n = n-(n/max_int)*max_int;
        max_int = max_int / 10;
        if ( i == _count-2)
        {
          _select_4[5+i] = '\r';
        }
      }
  //std::cout<<"select4 : "<<_select_4<<std::endl;
  //std::cout<<"sizeof(select2) : "<<sizeof(select2)<<std::endl;
   write(_linear_serial_port, _select_4, sizeof(_select_4));
   //read_encoder();
   //write(serial_port, encoder, sizeof(encoder));
  break;
   }
  case 5:
  if (_plus_minus_state == 1)
   {
    _select_5[0] = 'M';
    _select_5[1] = 'R';
    _select_5[2] = ' ';
    max_int = pow(10,_count-1);
    //std::cout<<t<<std::endl;
    //std::cout<<"count : " << count<<std::endl;
      for (int i = 0 ; i < _count ; i++)
      {
          //std::cout<<"n/t : "<<n/t<<std::endl;
        _select_5[3+i] = int2Uchar(n/max_int);
        n = n-(n/max_int)*max_int;
        max_int = max_int / 10;
        if ( i == _count-1)
        {
          _select_5[4+i] = '\r';
        }
      }
  //std::cout<<"select5 : "<<_select_5<<std::endl;
  //std::cout<<"sizeof(select2) : "<<sizeof(select2)<<std::endl;
   write(_linear_serial_port, _select_5, sizeof(_select_5));
   //read(_linear_serial_port, _read_buf, sizeof(_read_buf));
   //read_encoder();
  //  if(_read_buf[0] == 'M')
  //  {
  //    std::cout<<"select5 : "<<_select_5<<std::endl;
  //   read_encoder();
  //   break;  
  //  }
  //  else
  //  {
  //    break;
  //  }
   
   //write(serial_port, encoder, sizeof(encoder));
   break;
   }
   else
   {
    _select_5[0] = 'M';
    _select_5[1] = 'R';
    _select_5[2] = ' ';
    _select_5[3] = '-';
    max_int = pow(10,_count-2);
    n = -n;
    //std::cout<<"n : "<<n<<std::endl;
    for (int i = 0 ; i < _count-1 ; i++)
      {
          //std::cout<<"n/t : "<<n/t<<std::endl;
        _select_5[4+i] = int2Uchar(n/max_int);
        n = n-(n/max_int)*max_int;
        max_int = max_int / 10;
        if ( i == _count-2)
        {
          _select_5[5+i] = '\r';
        }
      }
  //std::cout<<"select5 : "<<_select_5<<std::endl;
  //std::cout<<"sizeof(select2) : "<<sizeof(select2)<<std::endl;
   write(_linear_serial_port, _select_5, sizeof(_select_5));
   //read_encoder();
   //write(serial_port, encoder, sizeof(encoder));
  break;
   }
  case 6:
    if (_plus_minus_state == 1)
   {
    _select_6[0] = 'M';
    _select_6[1] = 'R';
    _select_6[2] = ' ';
    max_int = pow(10,_count-1);
    //std::cout<<t<<std::endl;
    //std::cout<<"count : " << count<<std::endl;
      for (int i = 0 ; i < _count ; i++)
      {
          //std::cout<<"n/t : "<<n/t<<std::endl;
        _select_6[3+i] = int2Uchar(n/max_int);
        n = n-(n/max_int)*max_int;
        max_int = max_int / 10;
        if ( i == _count-1)
        {
          _select_6[4+i] = '\r';
        }
      }
  //std::cout<<"select6 : "<<_select_6<<std::endl;
  //std::cout<<"sizeof(select2) : "<<sizeof(select2)<<std::endl;
   write(_linear_serial_port, _select_6, sizeof(_select_6));
   //read_encoder();
   //write(serial_port, encoder, sizeof(encoder));
   break;
   }
   else
   {
    _select_6[0] = 'M';
    _select_6[1] = 'R';
    _select_6[2] = ' ';
    _select_6[3] = '-';
    max_int = pow(10,_count-2);
    n = -n;
    //std::cout<<"n : "<<n<<std::endl;
    for (int i = 0 ; i < _count-1 ; i++)
      {
          //std::cout<<"n/t : "<<n/t<<std::endl;
        _select_6[4+i] = int2Uchar(n/max_int);
        n = n-(n/max_int)*max_int;
        max_int = max_int / 10;

          if ( i == _count-2)
        {
          _select_6[5+i] = '\r';
        }
      }
  //std::cout<<"select6 : "<<_select_6<<std::endl;
  //std::cout<<"sizeof(select2) : "<<sizeof(select2)<<std::endl;
   write(_linear_serial_port, _select_6, sizeof(_select_6));
  // read_encoder();
   //write(serial_port, encoder, sizeof(encoder));
  break;
   }
  case 7:
       if (_plus_minus_state == 1)
   {
    _select_7[0] = 'M';
    _select_7[1] = 'R';
    _select_7[2] = ' ';
    max_int = pow(10,_count-1);
    //std::cout<<t<<std::endl;
    //std::cout<<"count : " << count<<std::endl;
      for (int i = 0 ; i < _count ; i++)
      {
          //std::cout<<"n/t : "<<n/t<<std::endl;
        _select_7[3+i] = int2Uchar(n/max_int);
        n = n-(n/max_int)*max_int;
        max_int = max_int / 10;
        if ( i == _count-1)
        {
          _select_7[4+i] = '\r';
        }
      }
  //std::cout<<"select7 : "<<_select_7<<std::endl;
  //std::cout<<"sizeof(select2) : "<<sizeof(select2)<<std::endl;
   write(_linear_serial_port, _select_7, sizeof(_select_7));
   //read_encoder();
   //write(serial_port, encoder, sizeof(encoder));
   break;
   }
   else
   {
    _select_7[0] = 'M';
    _select_7[1] = 'R';
    _select_7[2] = ' ';
    _select_7[3] = '-';
    max_int = pow(10,_count-2);
    n = -n;
    //std::cout<<"n : "<<n<<std::endl;
    for (int i = 0 ; i < _count-1 ; i++)
      {
          //std::cout<<"n/t : "<<n/t<<std::endl;
        _select_7[4+i] = int2Uchar(n/max_int);
        n = n-(n/max_int)*max_int;
        max_int = max_int / 10;

          if ( i == _count-2)
        {
          _select_7[5+i] = '\r';
        }
      }
  //std::cout<<"select7 : "<<_select_7<<std::endl;
  //std::cout<<"sizeof(select2) : "<<sizeof(select2)<<std::endl;
   write(_linear_serial_port, _select_7, sizeof(_select_7));
   //read_encoder();
   //write(serial_port, encoder, sizeof(encoder));
  break;
   }
  case 8:
    _select_8[0] = 'M';
    _select_8[1] = 'R';
    _select_8[2] = ' ';
    _select_8[3] = '-';
    max_int = pow(10,_count-2);
    n = -n;
    //std::cout<<"n : "<<n<<std::endl;
    for (int i = 0 ; i < _count-1 ; i++)
      {
          //std::cout<<"n/t : "<<n/t<<std::endl;
        _select_8[4+i] = int2Uchar(n/max_int);
        n = n-(n/max_int)*max_int;
        max_int = max_int / 10;

          if ( i == _count-2)
        {
          _select_8[5+i] = '\r';
        }
      }
  //std::cout<<"select8 : "<<_select_8<<std::endl;
  //std::cout<<"sizeof(select2) : "<<sizeof(select2)<<std::endl;
      write(_linear_serial_port, _select_8, sizeof(_select_8));
      //read_encoder();
      //write(serial_port, encoder, sizeof(encoder));
      break;
    //}
  }
}

unsigned char Clinear::int2Uchar(int k)
{
    unsigned char temp = '0';
    switch (k)
    {
      case 1:
        temp = '1';
        return temp;
        break;

        case 2:
        temp = '2';
        return temp;
        break;

        case 3:
        temp = '3';
        return temp;
        break;

        case 4:
        temp = '4';
        return temp;
        break;

        case 5:
        temp = '5';
        return temp;
        break;

        case 6:
        temp = '6';
        return temp;
        break;

        case 7:
        temp = '7';
        return temp;
        break;

        case 8:
        temp = '8';
        return temp;
        break;

        case 9:
        temp = '9';
        return temp;
        break;

        case 0:
        temp = '0';
        return temp;
        break;
    }
    return temp;
}

void Clinear::Open_linear()
{
  // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
  _linear_serial_port = open("/dev/ttyUSB0", O_RDWR|O_NOCTTY);

  // Create new termios struc, we call it 'tty' for convention
  struct termios tty;
  bzero(&tty, sizeof(tty)); 
  // Read in existing settings, and handle any error
  if(tcgetattr(_linear_serial_port, &tty) != 0) 
  {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }

  //tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  //tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  //tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
  //tty.c_cflag |= CS8; // 8 bits per byte (most common)
  //tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  //tty.c_cflag = CREAD; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
  //tty.c_cflag =  IGNPAR | ICRNL;
  //tty.c_lflag &= ~ICANON;
  //tty.c_lflag &= ~ECHO; // Disable echo
  //tty.c_lflag &= ~ECHOE; // Disable erasure
  //tty.c_lflag &= ~ECHONL; // Disable new-line echo
  //tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  //tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  //tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
  //tty.c_iflag = INPCK;
  //tty.c_oflag = 0;
  //tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  //tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  //tty.c_cc[VTIME] = 0.01;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  //tty.c_cc[VMIN] = 1;
  //tty.c_lflag = ICANON;
  
  // Set in/out baud rate to be 9600
  //cfsetispeed(&tty, B9600);

  // Set in/out baud rate to be 115200
  cfsetospeed(&tty, B115200);

  //tty.c_lflag = 0;
  bzero(tty.c_cc, NCCS);
  tty.c_cc[VTIME] = 0; 
  tty.c_cc[VMIN] = 1;  

  tty.c_cflag = CS8 | CREAD;
  tty.c_iflag = ICRNL | INPCK | IGNPAR;
  tty.c_oflag = 0;
  tty.c_lflag  =  ICANON;

  tcflush(_linear_serial_port,  TCIFLUSH);//  시리얼 포트 수신 큐 초기화tcsetattr(iDev,  TCSANOW,  &stNewState);
  tcsetattr(_linear_serial_port,  TCSANOW,  &tty);

  // Save tty settings, also checking for error
  if (tcsetattr(_linear_serial_port, TCSANOW, &tty) != 0) 
  {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }
   //write(serial_port, Enable_encoder, sizeof(Enable_encoder));

  write(_linear_serial_port, _velocity, sizeof(_velocity));
  usleep(3000);
  write(_linear_serial_port, _Echo_off, sizeof(_Echo_off));
}

int Clinear::char2int(char a)
{
  int temp = 0;
  switch (a)
  {
  case '0':
    temp  = a-'0';
    return temp;
    break;

  case '1':
    temp  = a-'0';
    return temp;
    break;

    case '2':
    temp  = a-'0';
    return temp;
    break;

    case '3':
    temp  = a-'0';
    return temp;
    break;

    case '4':
    temp  = a-'0';
    return temp;
    break;

    case '5':
    temp  = a-'0';
    return temp;
    break;

    case '6':
    temp  = a-'0';
    return temp;
    break;

    case '7':
    temp  = a-'0';
    return temp;
    break;

    case '8':
    temp  = a-'0';
    return temp;
    break;

    case '9':
    temp  = a-'0';
    return temp;
    break;
  }
  return temp;
}



