#pragma once // ?


class Clinear
{

public:
	
    Clinear();
	virtual ~Clinear(); //	

	void read_encoder();
	unsigned char int2Uchar(int k); // int to unsigned char  < k  is number > 0 ~ 9
	void goalposition2Uchar(int goal_position);
    void goalposition2Uchar_rel(int goal_position);
    void Open_linear();
    void homing();
    int char2int(char a);
    int _linear_present_position;
    int _linear_serial_port;
    char _read_buf[255];

private:
    
    unsigned char _Echo_off[5] = {'E' , 'M' , '=', '2', '\r' }; // echo_off
    unsigned char _Enable_encoder[5] = {'E' , 'E' , '=', '1', '\r' }; // encoder enable <not used>
    unsigned char _encoder[5] = {'P' , 'R' , ' ', 'P','\r'};//, '"', 'P', '\r' }; // present encoder
    unsigned char _velocity[11] = {'V' , 'M' , '=', '1','1', '0', '0', '0', '0', '0', '\r' }; // 1,000,000cnt/sec
    unsigned char _homing[10] = {'P', '=' ,'1','0','0','0','0','0', '0', '\r' }; // homing
    //unsigned char _CR[9] = {'C', 'R', '=' , '0','.','0','0','1', '\r' }; // CR

    int _t;
    int _count ;
    int _encoder_count ;
    int _temp_present_position;
    int _tmpbuf;
    int _tmpbuf_old;

	unsigned char _select_1[5] ;
    unsigned char _select_2[6] ;
    unsigned char _select_3[7] ;
    unsigned char _select_4[8] ;
    unsigned char _select_5[9] ;
    unsigned char _select_6[10] ;
    unsigned char _select_7[11] ;
    unsigned char _select_8[12] ;

    int _read_buf_int[255];
    char _temp_buf[255];
    bool _plus_minus_state; // 0 = minus , 1 = plus
    bool _encoder_plus_minus_state; // 0 = minus , 1 = plus
    bool _read_state;
};
