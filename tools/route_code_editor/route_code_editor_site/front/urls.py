from django.conf.urls import url
from django.urls import re_path
from . import views

app_name = "ams_management_site"

urlpatterns = [
    re_path(r'.*', views.app, name='app'),
]
