#include <iostream>
#include <string>

#include <termio.h>
#include <stdio.h>
#include <unistd.h>

using namespace std;

//定义菜单链表-----

class Menu
{
    private:
        string text;

    public:
        Menu* SubMenus;

        Menu()
        {
            
        }

        Menu(string text,Menu* SubMenus = NULL)
        {
            this->text = text;
            this->SubMenus = SubMenus;
        }

        void changeText(string text)
        {
            this->text = text;
        }

        string getText()
        {
            return text;
        }


};

typedef struct Node
{
    Menu* pNodeMenu;
    struct Node* pLast;
    struct Node* pNext;
}Node;

//定义结束--


//函数定义-----

//获取键盘输入
int scanKeyboard()
{
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



//创建菜单
Node* CreatMenu()
{
    Node *head,*normal,*end;
    head = (Node*)malloc(sizeof(Node));
    head->pNodeMenu=(Menu*)malloc(sizeof(Menu));
    end=head;

    int count = 1;

    while(1)
    {
        normal=(Node*)malloc(sizeof(Node));
        normal->pNodeMenu=(Menu*)malloc(sizeof(Menu));
        string text;

        cout << "\x1b[H\x1b[2J" <<endl;
        cout << ">正在创建第" << count << "个菜单" << endl;
        count ++;

        cout <<"-输入菜单内容:";
        cin >> text;
        normal->pNodeMenu->changeText(text);
        cout << "----------------------------------" << endl;
        getchar();

        end->pNext=normal; 
        normal->pLast=end;
        end=normal;

        cout << "按任意键继续创建[按下Esc退出]" << endl;

        if(scanKeyboard() == 27)
        {
            break;
        }
    }
    end->pNext=head;
    head->pLast=end;
    return head;
}


//打印菜单
void printMenus(Node* nodefirst)
{
    if(nodefirst == NULL)
    {
        cout << "还没有任何菜单！" <<endl;
        return;
    }

    Node* pNode = nodefirst;
    Node* pEnd = nodefirst->pLast;
    while (pNode != pEnd)
    {
        cout << pNode->pNodeMenu->getText() << endl; 
        pNode = pNode->pNext;
    }
    cout << pNode->pNodeMenu->getText() << endl; 
    
}















//主菜单
void MainMenu()
{
    cout << "\x1b[H\x1b[2J" <<endl;
    cout << "------------------"<<endl;
    cout << ">1 创建菜单"<<endl;
    cout << ">2 打印所有菜单"<<endl;
    cout << ">3 开始使用"<<endl;
    cout << "|按下要进行的操作[1/2/3/Esc]";   
}
//定义结束--


//主程序-----

int main()
{
    Node* pHead = NULL;
    int Choice;
    while(1)
    {
        MainMenu();
        Choice = scanKeyboard();

        switch (Choice)
        {
        case 49:
            {
                cout << "\x1b[H\x1b[2J" <<endl;
                pHead = CreatMenu();
                break;
            }
            
        case 50:
            {
                cout << "\x1b[H\x1b[2J" <<endl;
                cout << "2222222222J" <<endl;
                //printMenus(pHead);
                break;
            }

        case 51:
            {
                cout << "\x1b[H\x1b[2J" <<endl;
                break;
            }

        default:
            break;
        }

    }


    return 0;
}
