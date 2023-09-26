from django.shortcuts import render, redirect
from login import models
from django.contrib import messages


# Create your views here.
def login(request):
    if request.method == "POST":
        get_studentID = request.POST.get('studentID')
        get_password = request.POST.get('password')
        print(get_studentID)
        try:
            password = models.UserInfo.objects.get(studentID=get_studentID).password
            print(password)
            Isfound = 1
        except:
            Isfound = 0

        if Isfound == 0:
            messages.error(request, "该用户不存在！")
            return render(request, 'login.html')
        else:
            if password == get_password:
                return redirect('../transfer')
            else:
                messages.error(request, "密码错误！")
                return render(request, 'login.html')

    return render(request, 'login.html')


def home(request):
    return redirect('../login')


def register(request):
    # models.UserInfo.objects.create()
    if request.method == 'POST':
        get_username = request.POST.get('username')
        get_password = request.POST.get('password')
        get_repassword = request.POST.get('repassword')
        get_studentID = request.POST.get('studentID')
        get_group = request.POST.get('group')
        get_grade = request.POST.get('grade')

        if not(get_password and get_repassword and get_grade and get_group and get_username and get_studentID):
            messages.error(request, "请填写所有项目！")
            return render(request, 'register.html')

        if get_password != get_repassword:
            messages.error(request, "两次输入的密码不一致！")
            return render(request, 'register.html')

        try:
            result = models.UserInfo.objects.get(studentID=get_studentID)
            Isfound = 1
        except:
            Isfound = 0

        if(Isfound == 0):
            models.UserInfo.objects.create(username=get_username, password=get_password, studentID=get_studentID,
                                       group=get_group, grade=get_grade)
            print("Success!")
        else:
            messages.error(request,"该用户已存在！")

        return render(request, 'register.html')
    return render(request, 'register.html')


def register_return(request):
    return redirect('../../login')

def transfer(request):
    return render(request, 'transfer.html')