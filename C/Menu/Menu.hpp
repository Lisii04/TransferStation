/*
创建日期：2023/8/30
最后更新：2023/9/2
开发者：Lisii
类功能：Linux下的轻量化菜单框架
*/

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <unistd.h>
#include <termio.h>

using namespace std;


#define FILENAME "./Menu.txt"
#define UP 'w'
#define DOWN 's'
#define BACK 'a'
#define DOWN 'd'


//定义菜单类-----------------

class Menu
{
    /*
    菜单信息
    -菜单内容
    -菜单编号
    -菜单的父菜单编号
    */
    private:
        string text;
        string No;
        string FatherNo;

    public:
        

        Menu()
        {
            
        }

        Menu(string text,string No,string FatherNo)
        {
            this->text = text;
            this->No = No;
            this->FatherNo = FatherNo;
        }

        void changeText(string text)
        {
            this->text = text;
        }

        void changeNo(string NO)
        {
            this->No = No;
        }

        void changeFatherNo(string FatherNo)
        {
            this->FatherNo = FatherNo;
        }

        string getText()
        {
            return text;
        }

        string getNo()
        {
            return No;
        }

        string getFatherNo()
        {
            return FatherNo;
        }


};

//定义读取信息存放的结构体
typedef struct Info
{
    string No;
    string FartherNo;
    string Text;
}Info;

//定义结束----------------------


//函数定义----------------------------

//读取菜单的TXT文件信息并存储在Info类型的vector内
vector<Info> read_from_file(char const *fileName);

//获取键盘输入并返回对应int值
int scanKeyboard();

//根据vector<Info>为每一组Info创建菜单类
vector<Menu> CreatMenu(vector<Info> Infos);

//找出当前要显示的菜单
vector<Menu> FindCurrentMenus(vector<Menu> Menus,string FatherNo,vector<Menu> CurrentMenus);

//找到当前菜单的上一级菜单
vector<Menu> FatherMenus(vector<Menu> Menus,vector<Menu> CurrentMenus);

//根据光标所在位置显示菜单
void printMenu(int choice,vector<Menu> CurrentMenus);

//光标位置的判断以及显示菜单的主函数
void Visual(vector<Menu> Menus);

//定义结束----------------------------