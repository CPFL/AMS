#!/usr/bin/env python
# coding: utf-8

from logging import config, getLogger
from pprint import PrettyPrinter


config.dictConfig({
    "version": 1,
    "disable_existing_loggers": False,
    "formatters": {
        "simple": {
            "format": "%(name)s %(asctime)s %(levelname)s pid %(process)d line %(lineno)d in %(funcName)s: %(message)s"
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
            "propagate": True}}})

logger = getLogger("ams")
logger.pp = PrettyPrinter(indent=2).pprint
logger.deprecation_warning_message = lambda new, old: "Use {} instead of {}".format(new, old)
