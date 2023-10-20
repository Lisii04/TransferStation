#include "Menu.hpp"

//主程序-----

int main()
{
    //读取文件
    char fileName[] = FILENAME;
    //从文件读取菜单信息
    vector<Info> Infos = read_from_file(fileName);
    //创建所有菜单节点
    vector<Menu> Menus = CreatMenu(Infos);
    //显示菜单
    Visual(Menus);

    return 0;
}