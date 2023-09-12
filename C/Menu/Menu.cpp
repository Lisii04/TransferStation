#include "Menu.hpp"

//函数定义-----

//读取菜单的TXT文件信息并存储在Info类型的vector内
vector<Info> read_from_file(char const *fileName)
{
    char Texts[256];
    
    int lineCount = 0;

    vector<Info> Infos;

    ifstream in;

    in.open(fileName);
    if(!in.is_open())
    {
        cout << "file not exist" << endl;
    }
    while(in.getline(Texts,256))
    {
        int i = 0,j = 0;
        Info* info = new Info;
        Infos.push_back(*info);
        char numbers[256]={};
        char content[256]={};
        string No,FartherNo,Text;

        while(Texts[i] != '\0')
        {
            if(Texts[i] == '[')
            {
                i++;
                while (Texts[i] != ']')
                {
                    numbers[i-1] = Texts[i];
                    i++;
                }
                No = numbers;
                Infos[lineCount].No = No;
                
                if(No.size() > 1)
                {
                    No.pop_back();
                    No.pop_back();
                    FartherNo = No;
                    Infos[lineCount].FartherNo = FartherNo;
                }else{
                    FartherNo = "0";
                    Infos[lineCount].FartherNo = FartherNo;
                }
            }
            i++;
            content[j] = Texts[i]; 
            j++;
        }
        Text = content;

       
        
        Infos[lineCount].Text = Text;
        lineCount ++;
    }

    return Infos;
}

//获取键盘输入并返回对应int值
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


//根据vector<Info>为每一组Info创建菜单类
vector<Menu> CreatMenu(vector<Info> Infos)
{
    vector<Menu> Menus;

    for(int i = 0;i < Infos.size();i++)
    {
        Menu* newMenu = new Menu;
        *newMenu = Menu(Infos[i].Text,Infos[i].No,Infos[i].FartherNo);
        Menus.push_back(*newMenu);
    }
    return Menus;
}

//找出当前要显示的菜单
vector<Menu> FindCurrentMenus(vector<Menu> Menus,string FatherNo,vector<Menu> CurrentMenus)
{
    vector<Menu> CurMenus;

    int j = 0,IsExist = 0;

    for (int i = 0; i < Menus.size(); i++)
    {
        if(Menus[i].getFatherNo().compare(FatherNo) == 0)
        {
            CurMenus.push_back(Menus[i]);
            j++;
            IsExist = 1;
        }
    }
    if(IsExist == 0)
    {
        return CurrentMenus;
    }
    return CurMenus;
}

//找到当前菜单的上一级菜单
vector<Menu> FatherMenus(vector<Menu> Menus,vector<Menu> CurrentMenus)
{
    vector<Menu> CurMenus;
    Menu* FatherMenu = new Menu;

    int j = 0,IsExist = 0;
    string FatherNo = CurrentMenus[0].getFatherNo();

    for (int i = 0; i < Menus.size(); i++)
    {
        if(Menus[i].getNo().compare(FatherNo) == 0)
        {
            *FatherMenu = Menus[i];
            IsExist = 1;
        }
    }
    

    if(IsExist == 0)
    {
        return CurrentMenus;
    }
    return FindCurrentMenus(Menus,FatherMenu->getFatherNo(),CurrentMenus);
}


//根据光标所在位置显示菜单
void printMenu(int choice,vector<Menu> CurrentMenus)
{
    for (int i = 0; i < CurrentMenus.size(); i++)
    {

        if(i == choice)
        {
            cout << "\033[47m\033[30m" << CurrentMenus[i].getText() << "\033[0m" << endl;
        }else
        {
            cout << CurrentMenus[i].getText() << endl;
        }

    }
    
}

//光标位置的判断以及显示菜单的主函数
void Visual(vector<Menu> Menus)
{
    vector<Menu> CurrentMenus;
    CurrentMenus = FindCurrentMenus(Menus,"0",CurrentMenus);
    int choice = 0;
    vector<int> LastChioces;

    while (1)
    {
        cout << "\033c\033[?25l" << endl;
        printMenu(choice,CurrentMenus);
        switch (scanKeyboard())
        {
        case '1':
            {
                if(choice > 0)
                    choice--;
                break;
            }
           
        case '2':
            {
                if(choice < CurrentMenus.size()-1)
                    choice++;
                break;
            }

        case '0':
            {
                CurrentMenus = FindCurrentMenus(Menus,CurrentMenus[choice].getNo(),CurrentMenus);
                LastChioces.push_back(choice);
                choice = 0;
                break;
            }

        case '9':
            {
                CurrentMenus = FatherMenus(Menus,CurrentMenus);
                choice = *(LastChioces.end()-1);
                LastChioces.pop_back();
                break;
            }

        default:
            break;
        }
    }
    
}

//定义结束--