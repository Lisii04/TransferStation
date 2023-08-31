#include <iostream>


#include <termio.h>
#include <stdio.h>
#include <unistd.h>

using namespace std;

int scanKeyboard()
{
  //  struct termios
  //    {
  //      tcflag_t c_iflag;		/* input mode flags */
  //      tcflag_t c_oflag;		/* output mode flags */
  //      tcflag_t c_cflag;		/* control mode flags */
  //      tcflag_t c_lflag;		/* local mode flags */
  //      cc_t c_line;			/* line discipline */
  //      cc_t c_cc[NCCS];		/* control characters */
  //      speed_t c_ispeed;		/* input speed */
  //      speed_t c_ospeed;		/* output speed */
  //  #define _HAVE_STRUCT_TERMIOS_C_ISPEED 1
  //  #define _HAVE_STRUCT_TERMIOS_C_OSPEED 1
  //    };

  /*
  up 65
  down 66
  left 68
  right 67

  w 119
  a 97
  s 115
  d 100
  */
  int in;
  struct termios new_settings;
  struct termios stored_settings;
  tcgetattr(STDIN_FILENO,&stored_settings); //获得stdin 输入
  new_settings = stored_settings;           //
  new_settings.c_lflag &= (~ICANON);        //
  new_settings.c_cc[VTIME] = 0;
  tcgetattr(STDIN_FILENO,&stored_settings); //获得stdin 输入
  new_settings.c_cc[VMIN] = 1;
  tcsetattr(STDIN_FILENO,TCSANOW,&new_settings); //

  in = getchar();

  tcsetattr(STDIN_FILENO,TCSANOW,&stored_settings);
  return in;
}



int main(int argc, char *argv[]) 
{

    while(1){
        //cout << "\033c" << endl;
        int Key = scanKeyboard();

        switch (Key)
        {
        case 'w':
            {
                cout << "\033c" << endl;
                cout << "\033[47m\033[30m" << "w" << "\033[0m\n" << endl;
                break;  
            }
        case 's':
            {
                cout << "\033c" << endl;
                cout << "\033[47m\033[30m" << "s" << "\033[0m\n" << endl;
                break;  
            }
        case 'a':
            {
                cout << "\033c" << endl;
                cout << "\033[47m\033[30m" << "a" << "\033[0m\n" << endl;
                break;  
            }
        case 'd':
            {
                cout << "\033c" << endl;
                cout << "\033[47m\033[30m" << "d" << "\033[0m\n" << endl;
                break;  
            }
        default:
            break;
        }

        //printf(":%d\r\n",Key);

        
    }
    return 0;
}