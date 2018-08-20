from kvs_client import get_ams_kvs_client_class
from time import sleep

import redis

DC = get_ams_kvs_client_class(redis)
d = DC()
d.set_args_of_ConnectionPool(host="127.0.0.1")
d.set_args_of_StrictRedis()

d.connect()

while True:
    sleep(2)
    # d.set("src", {"test": "src"})
    src_data = d.get("src")
    d.set("dst", src_data, "src")
