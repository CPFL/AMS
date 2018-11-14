from django.conf.urls import url
from django.urls import re_path
from . import views

app_name = "route_code_editor"

urlpatterns = [
    re_path(r'.*', views.index, name='index'),
]
