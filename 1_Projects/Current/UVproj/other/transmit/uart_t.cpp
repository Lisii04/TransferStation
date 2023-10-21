#include <python3.10/Python.h>
#include <iostream>

int uart_send(int IF_READY, int IF_SLOW, int IF_STOP, float delta_x)
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
    PyObject *pArgs = PyTuple_New(4);

    // 0：第一个参数，传入 float 类型的值 2.242
    PyTuple_SetItem(pArgs, 0, Py_BuildValue("f", delta_x));
    PyTuple_SetItem(pArgs, 1, Py_BuildValue("i", IF_READY));
    PyTuple_SetItem(pArgs, 2, Py_BuildValue("i", IF_SLOW));
    PyTuple_SetItem(pArgs, 3, Py_BuildValue("i", IF_STOP));

    // 使用C++的python接口调用该函数
    PyObject *pReturn = PyEval_CallObject(pFunc, pArgs);

    // 结束python接口初始化
    Py_Finalize();
    return 1;
}

int main()
{
    uart_send(1, 0, 0, 2.2412);
    return 0;
}