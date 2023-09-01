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
        

        Menu()
        {
            
        }

        Menu(string text)
        {
            this->text = text;
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
    Node* SubMenus;
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
        cout << "\033[47m\033[30m" << ">正在创建第" << count << "个菜单"<< "\033[0m" << endl;
        count ++;

        cout <<"-输入菜单内容:";
        cin >> text;
        normal->pNodeMenu->changeText(text);
        normal->SubMenus = NULL;
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


void addSubmenu(Node* nodefirst)
{

    //选择菜单加入子菜单
    Node* pNode;
    int number;
    cout << "\033[2m" <<"|>请输入要加入子菜单的菜单序号：" << "\033[0m";
    cin >> number;
    pNode = nodefirst;

    for(int i = 0;i < number; i++)
    {
        pNode = pNode->pNext;
    }

    pNode->SubMenus = CreatMenu();

    cout << ">创建成功！" <<endl;

    getchar();

}


//打印和添加菜单
void printMenus(Node* nodefirst)
{
    int count = 1,IsSub = 0;

    cout << "\x1b[H\x1b[2J" <<endl;

    if(nodefirst == NULL)
    {
        cout << "\033[31m" << "还没有任何菜单！" <<endl;
        getchar();
        return;
    }

    Node* pNode = nodefirst->pNext;
    Node* pEnd = nodefirst->pLast;

    cout << "\033[47m\033[30m" << "|  所有菜单 [带下划线表示有子菜单] |" << "\033[0m\n" << endl;

    while (pNode != pEnd)
    {
        if(pNode->SubMenus != NULL)
        {
            cout << "\033[4m" << count << " " << pNode->pNodeMenu->getText() << "\033[0m" << endl;
            pNode = pNode->pNext;
            count ++;
            IsSub = 1;
        }else
        {
            cout << count << " " << pNode->pNodeMenu->getText() << endl; 
            pNode = pNode->pNext;
            count ++;
        }
    }

    if(pNode->SubMenus != NULL)
    {
        cout << "\033[4m" << count << " " << pNode->pNodeMenu->getText() << "\033[0m" << endl;
        IsSub = 1;
    }else
    {
        cout << count << " " << pNode->pNodeMenu->getText() << endl; 
    }

    cout << "\033[32m"<< "按 1 以添加子菜单|按其他键继续查看" << "\033[0m" <<endl;
    int IsAdd = scanKeyboard();

    if(IsAdd == 49)
    {
        addSubmenu(pNode);
        return;
    }

    if(IsSub == 1)
    {
        int number = 0;
        cout << "\033[2m"<< ">请输入要查看子菜单的序号[输入0以退出]：" << "\033[0m";
        cin >> number;
        pNode = nodefirst;

        if(number == 0)
        {
            return;
        }

        for(int i = 0;i < number; i++)
        {
            pNode = pNode->pNext;
        }

        if(pNode->SubMenus != NULL)
        {
            printMenus(pNode->SubMenus);
            printMenus(nodefirst);
        }else
        {
            cout << "选择的项目无子菜单！" << endl;
            getchar();
            getchar();
            printMenus(nodefirst);
        }

    }else
    {
        getchar();
        getchar();
    }  
}


void MainMenu()
{

}










//主菜单
void UI()
{
    cout << "\x1b[H\x1b[2J" <<endl;
    cout << "\033[47m\033[30m" <<">1 创建菜单               "<<endl;
    cout << ">2 显示所有菜单/添加子菜单"<<endl;
    cout << ">3 进入菜单界面           "<< "\033[0m" << endl;
    cout << "\033[32m\033[?25l" << "按下要进行的操作[1/2/3/Esc] " << "\033[0m"<< endl;
}


//定义结束--


//主程序-----

int main()
{
    Node* pHead = NULL;
    while(1)
    {
        UI();

        switch (scanKeyboard())
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
                printMenus(pHead);
                break;
            }

        case 51:
            {
                cout << "\x1b[H\x1b[2J" <<endl;
                MainMenu();
                break;
            }

        case 27:
            break;
        
        default:
            break;
        }

    }


    return 0;
}
