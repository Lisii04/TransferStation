#include <iostream>
#include <vector>
#include <python3.10/Python.h>
#include <math.h>
#include <numeric>
#include <stdlib.h>

using namespace std;

typedef struct DataPackage
{
    float K;
    float I;
    float P;
} DP;

DP interface(DP data_package)
{
    while (true)
    {

        cout << "-------------------------" << endl;
        cout << "[Datas]" << endl;
        cout << "[1]"
             << "K"
             << ":" << data_package.K << endl;
        cout << "[2]"
             << "I"
             << ":" << data_package.I << endl;
        cout << "[3]"
             << "P"
             << ":" << data_package.P << endl;
        cout << "-------------------------" << endl;
        cout << "[1.Change value|2.send data]:" << endl;
        int choice;
        float buffer;
        cin >> choice;
        if (choice == 1)
        {
            choice = 0;
            cout << "[Change value(1.K|2.I|3.P)]:" << endl;
            cin >> choice;
            switch (choice)
            {
            case 1:
                cout << "[value of K]:" << endl;
                cin >> buffer;
                data_package.K = buffer;
                break;
            case 2:
                cout << "[value of I]:" << endl;
                cin >> buffer;
                data_package.I = buffer;
                break;
            case 3:
                cout << "[value of P]:" << endl;
                cin >> buffer;
                data_package.P = buffer;
                break;

            default:
                break;
            }
        }
        else
        {
            return data_package;
        }
    }
}

/** 串口通信函数
 *  @param TURN_DIRECTION (0不转弯  1左转  2右转)
 *  @param FLAG_SLOW (0不减速 1减速)
 *  @param FLAG_STOP (0不停止 1停止)
 *  @param DEVIATION (偏移量)
 *  @retval 1 或 0
 */
int uart_send(DP data_package)
{
    // 初始化python接口
    Py_Initialize();
    if (!Py_IsInitialized())
    {
        std::cout << "python init fail" << std::endl;
        return 0;
    }
    // 初始化使用的变量
    PyObject *pModule = NULL;
    PyObject *pFunc = NULL;
    PyObject *pName = NULL;

    // 初始化python系统文件路径，保证可以访问到 .py文件
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append('..')");

    // 调用python文件名。
    pModule = PyImport_ImportModule("uart");

    // 调用函数
    pFunc = PyObject_GetAttrString(pModule, "uart_send");

    // 给python传参数
    PyObject *pArgs = PyTuple_New(1);

    // 0：第一个参数，传入 float 类型的值 2.242
    PyTuple_SetItem(pArgs, 0, Py_BuildValue("[f,f,f]", data_package.K, data_package.I, data_package.P));

    // 使用C++的python接口调用该函数
    PyObject *pReturn = PyEval_CallObject(pFunc, pArgs);

    // 结束python接口初始化
    Py_Finalize();
    return 1;
}

int main()
{
    DP data_package;

    uart_send(interface(data_package));
    return 0;
}
