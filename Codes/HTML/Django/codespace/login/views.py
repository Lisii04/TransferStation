# from django.contrib import messages
from django.shortcuts import render
from login import models
from django.http import JsonResponse

from django.views.decorators.csrf import csrf_exempt


# Create your views here.
@csrf_exempt
def login(request):
    if request.method == 'POST':
        username = request.POST.get('username')
        password = request.POST.get('password')

        IsFind = 0


        if username != "":
            for name in models.UserInfo.objects.all():
                if username == name.username:
                    if password == name.password:
                        # success
                        data = {
                            'status': 200
                        }
                        return render(request, 'transfer.html')

                    IsFind = 1
                    # password wrong
                    data = {
                        'status': -1
                    }
                    # messages.error(request, '用户名或密码不正确')
                    return JsonResponse(data)

            if IsFind == 0:
                # Not Found
                data = {
                    'status': 404
                }
                # messages.error(request, '用户不存在')
                return JsonResponse(data)

    return render(request, "login.html")


def register(request):
    return
