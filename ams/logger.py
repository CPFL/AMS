#!/usr/bin/env python
# coding: utf-8

import traceback

from logging import config, getLogger
from pprint import PrettyPrinter, pformat

default_config = {
    "version": 1,
    "disable_existing_loggers": False,
    "formatters": {
        "simple": {
            "format": "%(name)s %(asctime)s %(levelname)s pid %(process)d line %(lineno)d " +
                      "in %(module)s/%(funcName)s: %(message)s"
        },
    },
    "handlers": {
        "console": {
            "class": "logging.StreamHandler",
            "level": "WARNING",
            "formatter": "simple",
            "stream": "ext://sys.stdout"
        },
        "log_file": {
            "class": "logging.handlers.RotatingFileHandler",
            "level": "INFO",
            "formatter": "simple",
            "filename": "./ams.log",
            "maxBytes": 10485760,
            "backupCount": 2,
            "encoding": "utf8"
        },
    },
    "root": {
        "level": "INFO",
    },
    "loggers": {
        "ams": {
            "handlers": ["console", "log_file"],
            "qualname": "ams",
            "propagate": True
        }
    }
}

try:
    config.dictConfig(default_config)
except OSError:
    pass
except ValueError:
    pass
except:
    traceback.print_exc()

logger = getLogger("ams")
logger.pp = PrettyPrinter(indent=2).pprint
logger.pformat = lambda x: "\n"+pformat(x)
logger.deprecation_warning_message = lambda new, old: "Use {} instead of {}".format(new, old)
